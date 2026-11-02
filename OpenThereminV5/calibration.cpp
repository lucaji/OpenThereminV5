// OpenThereminV5 firmware development (2026): Luca Cipressi (milletgrain).

#include "calibration.h"
#include "SPImcpDAC.h"
#include "timer.h"
#include "ihandlers.h"
#include "EEPROM.h"
#include "build.h"
#include "hw.h"

#include <util/atomic.h>

static const uint32_t PITCH_FIXED_OSCILLATOR_FREQUENCY = 500000UL;
static const uint32_t VOLUME_FIXED_OSCILLATOR_FREQUENCY = 460800UL;
static const uint16_t CALIBRATION_BASE_MAX_DRIFT = 150;
static const uint16_t CALIBRATION_TOLERANCE = 15;
static const uint16_t PITCH_BEAT_FREQUENCY = 700;
static const uint16_t VOLUME_BEAT_FREQUENCY = 700;
static const uint16_t DAC_12BIT_MAX = 4095;
static const uint16_t SAFE_DAC_VALUE = 2048;
// Provisional instrument-level limits: qualify on real antennas before release.
// Volume periods include foreground-independent PCINT timestamp jitter, so their
// individual spread limit is deliberately wider than the mean-drift limit.
static const uint8_t CALIBRATION_WINDOW_SAMPLES = 32;
static const uint16_t CALIBRATION_PERIOD_SPREAD = 1024;
static const uint16_t CALIBRATION_MEAN_DRIFT = 128;
static const uint16_t CALIBRATION_FINAL_BEAT_DRIFT = 100;
static const uint16_t CALIBRATION_WINDOW_TIMEOUT = 3125; // Timer2: 200 ms.
static const uint16_t CALIBRATION_SAMPLE_TIMEOUT = 157; // Timer2: ~10 ms.

int32_t pitchCalibrationBase = F_CPU / PITCH_BEAT_FREQUENCY;
int32_t volCalibrationBase = F_CPU / VOLUME_BEAT_FREQUENCY;
static uint16_t currentPitchDac = SAFE_DAC_VALUE;
static uint16_t currentVolumeDac = SAFE_DAC_VALUE;
static bool haveStoredCalibration = false;

// Validate a captured Timer1 period without division, narrowing, or zero divisors.
static bool validCalibrationBase(int32_t period, uint16_t beatHz) {
    return period > 0 && period <= UINT16_MAX &&
        (uint32_t)period * (beatHz - CALIBRATION_BASE_MAX_DRIFT) < F_CPU &&
        (uint32_t)period * (beatHz + CALIBRATION_BASE_MAX_DRIFT) > F_CPU;
}

// Versioned records occupy two 24-byte slots, leaving V4's bytes 0..11 intact.
// All fields are encoded explicitly: host/AVR padding and endianness cannot
// change the persistent format. The old slot survives every partial new write.
static const uint16_t CALIBRATION_SLOT_ADDRESS[2] = {32, 64};
static const uint8_t CALIBRATION_RECORD_SIZE = 24;
static const uint8_t CALIBRATION_RECORD_VERSION = 1;
static const uint8_t CALIBRATION_COMMIT_OFFSET = 23;
static const uint8_t CALIBRATION_COMMITTED = 0xa5;

struct CalibrationValues {
    uint16_t pitchDac;
    uint16_t volumeDac;
    int32_t pitchBase;
    int32_t volumeBase;
};

static bool validCalibrationValues(const CalibrationValues& values) {
    return values.pitchDac <= DAC_12BIT_MAX && values.volumeDac <= DAC_12BIT_MAX &&
        validCalibrationBase(values.pitchBase, PITCH_BEAT_FREQUENCY) &&
        validCalibrationBase(values.volumeBase, VOLUME_BEAT_FREQUENCY);
}

static uint32_t decodeCalibrationField(const uint8_t* bytes, uint8_t count) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < count; ++i) value |= static_cast<uint32_t>(bytes[i]) << (8 * i);
    return value;
}

static void encodeCalibrationField(uint8_t* bytes, uint32_t value, uint8_t count) {
    for (uint8_t i = 0; i < count; ++i) {
        bytes[i] = static_cast<uint8_t>(value);
        value >>= 8;
    }
}

static uint16_t calibrationCrc(const uint8_t* bytes, uint8_t count) {
    // CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xffff.
    uint16_t crc = 0xffff;
    for (uint8_t i = 0; i < count; ++i) {
        crc ^= static_cast<uint16_t>(bytes[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = crc & 0x8000 ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                               : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

static bool readCalibrationSlot(uint8_t slot, CalibrationValues& values, uint32_t& sequence) {
    uint8_t bytes[CALIBRATION_RECORD_SIZE];
    for (uint8_t i = 0; i < CALIBRATION_RECORD_SIZE; ++i)
        bytes[i] = EEPROM.read(CALIBRATION_SLOT_ADDRESS[slot] + i);
    if (bytes[0] != 'T' || bytes[1] != '5' ||
        bytes[2] != CALIBRATION_RECORD_VERSION || bytes[3] != CALIBRATION_RECORD_SIZE ||
        bytes[22] != 0 || bytes[CALIBRATION_COMMIT_OFFSET] != CALIBRATION_COMMITTED ||
        calibrationCrc(bytes, 20) != decodeCalibrationField(bytes + 20, 2)) return false;
    CalibrationValues candidate;
    candidate.pitchDac = decodeCalibrationField(bytes + 8, 2);
    candidate.volumeDac = decodeCalibrationField(bytes + 10, 2);
    candidate.pitchBase = decodeCalibrationField(bytes + 12, 4);
    candidate.volumeBase = decodeCalibrationField(bytes + 16, 4);
    if (!validCalibrationValues(candidate)) return false;
    values = candidate;
    sequence = decodeCalibrationField(bytes + 4, 4);
    return true;
}

static bool newestCalibration(CalibrationValues& values, uint32_t& sequence, uint8_t& slot) {
    bool found = false;
    for (uint8_t index = 0; index < 2; ++index) {
        CalibrationValues candidate;
        uint32_t candidateSequence;
        if (!readCalibrationSlot(index, candidate, candidateSequence)) continue;
        const uint32_t difference = found ? candidateSequence - sequence : 0;
        if (!found || (difference != 0 && difference < 0x80000000UL)) {
            values = candidate;
            sequence = candidateSequence;
            slot = index;
            found = true;
        }
    }
    return found;
}

static bool storeCalibration(const CalibrationValues& values) {
    if (!validCalibrationValues(values)) return false;
    CalibrationValues previous;
    uint32_t sequence = 0;
    uint8_t previousSlot = 0;
    const bool havePrevious = newestCalibration(previous, sequence, previousSlot);
    const uint8_t slot = havePrevious ? previousSlot ^ 1 : 0;
    const uint16_t address = CALIBRATION_SLOT_ADDRESS[slot];
    ++sequence;
    uint8_t bytes[CALIBRATION_RECORD_SIZE] = {};
    bytes[0] = 'T';
    bytes[1] = '5';
    bytes[2] = CALIBRATION_RECORD_VERSION;
    bytes[3] = CALIBRATION_RECORD_SIZE;
    encodeCalibrationField(bytes + 4, sequence, 4);
    encodeCalibrationField(bytes + 8, values.pitchDac, 2);
    encodeCalibrationField(bytes + 10, values.volumeDac, 2);
    encodeCalibrationField(bytes + 12, values.pitchBase, 4);
    encodeCalibrationField(bytes + 16, values.volumeBase, 4);
    encodeCalibrationField(bytes + 20, calibrationCrc(bytes, 20), 2);
    bytes[CALIBRATION_COMMIT_OFFSET] = CALIBRATION_COMMITTED;

    // Invalidate the destination before touching its payload. EEPROM.update
    // waits for any preceding byte; readback also waits for the final write.
    EEPROM.update(address + CALIBRATION_COMMIT_OFFSET, 0);
    for (uint8_t i = 0; i < CALIBRATION_COMMIT_OFFSET; ++i) EEPROM.update(address + i, bytes[i]);
    for (uint8_t i = 0; i < CALIBRATION_COMMIT_OFFSET; ++i)
        if (EEPROM.read(address + i) != bytes[i]) return false;
    EEPROM.update(address + CALIBRATION_COMMIT_OFFSET, CALIBRATION_COMMITTED);
    CalibrationValues verified;
    uint32_t verifiedSequence;
    if (!readCalibrationSlot(slot, verified, verifiedSequence) || verifiedSequence != sequence ||
        verified.pitchDac != values.pitchDac || verified.volumeDac != values.volumeDac ||
        verified.pitchBase != values.pitchBase || verified.volumeBase != values.volumeBase) {
        EEPROM.update(address + CALIBRATION_COMMIT_OFFSET, 0);
        return false;
    }
    return true;
}

// Timer1 counts VO_PITCH rising edges on T1 / PD5 for approximately one second.
// Timer0 remains the Arduino timebase in this mode; ICP1 / PB0 is not used here.
static uint32_t GetPitchMeasurement() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR1B = 0;
        TCNT1 = 0;
        timer_overflow_counter = 0;
        TIFR1 = _BV(TOV1);
        TCCR1B = _BV(CS12) | _BV(CS11) | _BV(CS10);
    }
    delay(1000);

    uint16_t count;
    uint16_t overflows;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR1B = 0;
        count = TCNT1;
        overflows = timer_overflow_counter;
        // Include a final overflow whose interrupt has not run yet.
        if (TIFR1 & _BV(TOV1)) ++overflows;
        TIFR1 = _BV(TOV1);
    }
    return ((uint32_t)overflows << 16) | count;
}

// Timer0 counts VO_VOL rising edges on T0 / PD4; Timer1 gates a one-second window.
// OCR0A=255 in normal mode sets OCF0A on the following timer clock (the wrap),
// so each serviced compare interrupt represents exactly 256 input edges.
static uint32_t GetVolumeMeasurement() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR0B = 0;
        TCCR1B = 0;
        TCNT0 = 0;
        TCNT1 = (uint16_t)(65536UL - F_CPU / 1024UL);
        timer_overflow_counter = 0;
        TIFR0 = _BV(OCF0A) | _BV(TOV0);
        TIFR1 = _BV(TOV1);
        // Start with a known /1024 phase; its free-running first interval could
        // otherwise shorten the gate by almost 64 us (about 29 volume counts).
        GTCCR = _BV(PSRSYNC);
        TCCR1B = _BV(CS12) | _BV(CS10);
        TCCR0B = _BV(CS02) | _BV(CS01) | _BV(CS00);
    }
    while (!(TIFR1 & _BV(TOV1))) {}

    uint8_t count;
    uint16_t overflows;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR0B = 0;
        TCCR1B = 0;
        count = TCNT0;
        overflows = timer_overflow_counter;
        if (TIFR0 & _BV(OCF0A)) ++overflows;
        TIFR0 = _BV(OCF0A) | _BV(TOV0);
        TIFR1 = _BV(TOV1);
    }
    return ((uint32_t)overflows << 8) | count;
}

static bool validFrequency(uint32_t frequency) {
    // Zero means no oscillator input. External timer clocks cannot exceed F_CPU/2.
    return frequency > 0 && frequency <= F_CPU / 2;
}

static bool frequencyAtTarget(uint32_t frequency, uint32_t target) {
    const uint32_t error = frequency > target ? frequency - target : target - frequency;
    return error <= CALIBRATION_TOLERANCE;
}

template<typename Measure>
static bool confirmCalibrationDac(uint32_t target, uint16_t candidate,
                                  uint32_t first, Measure measure) {
    uint32_t minimum = first;
    uint32_t maximum = first;
    // A transient target-looking reading must not commit a rail or a wrong
    // bracket. Three independent gates must agree, with no extra ISR work.
    for (uint8_t repeat = 0; repeat < 2; ++repeat) {
        const uint32_t frequency = measure(candidate);
        if (!validFrequency(frequency) || !frequencyAtTarget(frequency, target)) return false;
        if (frequency < minimum) minimum = frequency;
        if (frequency > maximum) maximum = frequency;
    }
    return maximum - minimum <= CALIBRATION_TOLERANCE;
}

// A 12-bit bracketed search cannot leave the DAC range or divide by an equal
// measurement pair. Success always identifies the most recently applied DAC
// value, measured within tolerance of the target (not merely two similar reads).
template<typename Measure>
static bool findCalibrationDac(uint32_t target, uint16_t& dac, Measure measure,
                               uint16_t previousDac = UINT16_MAX) {
    if (previousDac <= DAC_12BIT_MAX) {
        const uint32_t frequency = measure(previousDac);
        if (validFrequency(frequency) && frequencyAtTarget(frequency, target)) {
            if (!confirmCalibrationDac(target, previousDac, frequency, measure)) return false;
            dac = previousDac;
            return true;
        }
    }
    uint16_t low = 0;
    uint16_t high = DAC_12BIT_MAX;
    const uint32_t lowFrequency = measure(low);
    if (!validFrequency(lowFrequency)) return false;
    if (frequencyAtTarget(lowFrequency, target)) {
        if (!confirmCalibrationDac(target, low, lowFrequency, measure)) return false;
        dac = low;
        return true;
    }
    const uint32_t highFrequency = measure(high);
    if (!validFrequency(highFrequency)) return false;
    if (frequencyAtTarget(highFrequency, target)) {
        if (!confirmCalibrationDac(target, high, highFrequency, measure)) return false;
        dac = high;
        return true;
    }
    if ((lowFrequency < target) == (highFrequency < target)) return false;
    const bool increasing = highFrequency > lowFrequency;

    for (uint8_t iteration = 0; iteration < 12 && high - low > 1; ++iteration) {
        const uint16_t candidate = low + (high - low) / 2;
        const uint32_t frequency = measure(candidate);
        if (!validFrequency(frequency)) return false;
        if (frequencyAtTarget(frequency, target)) {
            if (!confirmCalibrationDac(target, candidate, frequency, measure)) return false;
            dac = candidate;
            return true;
        }
        if ((frequency < target) == increasing) low = candidate;
        else high = candidate;
        HW_LED_BLUE_TOGGLE;
    }
    return false;
}

static uint32_t measureAtDac(bool volume, uint16_t dac) {
    if (volume) {
        SPImcpDAC2Bsend(dac);
        // Timer0 is unavailable to delay() during volume calibration. Use its
        // timer-independent microsecond counterpart for a 100 ms settling time.
        for (uint8_t i = 0; i < 100; ++i) delayMicroseconds(1000);
        return GetVolumeMeasurement();
    }
    SPImcpDAC2Asend(dac);
    delay(100);
    return GetPitchMeasurement();
}

static bool calibrateOscillator(bool volume, uint16_t& dac) {
    const uint32_t target = volume
        ? VOLUME_FIXED_OSCILLATOR_FREQUENCY - VOLUME_BEAT_FREQUENCY
        : PITCH_FIXED_OSCILLATOR_FREQUENCY - PITCH_BEAT_FREQUENCY;
    DEBUG_PRINT(F("Calibration target: ")); DEBUG_PRINTLN(target);
    return findCalibrationDac(target, dac, [volume](uint16_t candidate) {
        return measureAtDac(volume, candidate);
    }, haveStoredCalibration ? dac : UINT16_MAX);
}

struct CalibrationWindow {
    uint32_t sum = 0;
    uint16_t minimum = UINT16_MAX;
    uint16_t maximum = 0;
    uint16_t lastSample = 0;
    uint8_t count = 0;

    bool add(uint16_t period, uint16_t elapsed, uint16_t beatHz) {
        if (!validCalibrationBase(period, beatHz)) return false;
        if (period < minimum) minimum = period;
        if (period > maximum) maximum = period;
        if (maximum - minimum > CALIBRATION_PERIOD_SPREAD) return false;
        // Keep validating the faster antenna until its partner also finishes,
        // without changing this window's fixed 32-period mean.
        if (count < CALIBRATION_WINDOW_SAMPLES) {
            sum += period;
            ++count;
        }
        lastSample = elapsed;
        return true;
    }

    uint16_t mean() const {
        return static_cast<uint16_t>((sum + CALIBRATION_WINDOW_SAMPLES / 2) /
                                     CALIBRATION_WINDOW_SAMPLES);
    }
};

static bool collectCalibrationWindow(uint16_t& pitchMean, uint16_t& volumeMean,
                                     bool allowStartupRestart) {
    CalibrationWindow pitch;
    CalibrationWindow volume;
    uint16_t elapsed = 0;
    uint8_t previous = TCNT2;
    discardMeasurements();
    // Poll both mailboxes together. Sequential per-channel waits would discard
    // intervening periods and lose the cancellation of volume timestamp jitter.
    while (elapsed < CALIBRATION_WINDOW_TIMEOUT) {
        const uint8_t now = TCNT2;
        elapsed += static_cast<uint8_t>(now - previous);
        previous = now;
        uint16_t period;
        bool restarted;
        if (takePitchMeasurement(period, restarted)) {
            // The initial pair after restoring capture is expected to restart.
            // Any later restart proves a rejected pulse or expired reference;
            // matching periods afterward cannot make that window continuous.
            if (restarted && (!allowStartupRestart || pitch.count != 0)) return false;
            if (!pitch.add(period, elapsed, PITCH_BEAT_FREQUENCY)) return false;
        }
        if (takeVolumeMeasurement(period, restarted)) {
            if (restarted && (!allowStartupRestart || volume.count != 0)) return false;
            if (!volume.add(period, elapsed, VOLUME_BEAT_FREQUENCY)) return false;
        }
        // Completing a window does not excuse a channel that then disappears
        // while the other antenna is still filling its window.
        if (elapsed - pitch.lastSample >= CALIBRATION_SAMPLE_TIMEOUT ||
            elapsed - volume.lastSample >= CALIBRATION_SAMPLE_TIMEOUT) return false;
        if (pitch.count == CALIBRATION_WINDOW_SAMPLES &&
            volume.count == CALIBRATION_WINDOW_SAMPLES) {
            // A raw expiry/rejected pulse can latch a restart before a new
            // complete period exists, so mailbox reads alone cannot detect it.
            if (captureRestartPending()) return false;
            pitchMean = pitch.mean();
            volumeMean = volume.mean();
            return true;
        }
        serialOutputService();
    }
    return false;
}

static bool stableCalibrationMean(uint16_t first, uint16_t second, uint16_t beatHz) {
    const uint16_t difference = first > second ? first - second : second - first;
    const uint32_t mean = (static_cast<uint32_t>(first) + second + 1) / 2;
    return difference <= CALIBRATION_MEAN_DRIFT &&
        mean * (beatHz - CALIBRATION_FINAL_BEAT_DRIFT) <= F_CPU &&
        mean * (beatHz + CALIBRATION_FINAL_BEAT_DRIFT) >= F_CPU;
}

static bool calibration_finalize(int32_t& pitchBase, int32_t& volumeBase) {
    uint16_t firstPitch, firstVolume, secondPitch, secondVolume;
    if (!collectCalibrationWindow(firstPitch, firstVolume, true) ||
        !collectCalibrationWindow(secondPitch, secondVolume, false) ||
        !stableCalibrationMean(firstPitch, secondPitch, PITCH_BEAT_FREQUENCY) ||
        !stableCalibrationMean(firstVolume, secondVolume, VOLUME_BEAT_FREQUENCY)) return false;
    // Publish neither baseline until both antennas pass with their final DACs.
    pitchBase = (static_cast<uint32_t>(firstPitch) + secondPitch + 1) / 2;
    volumeBase = (static_cast<uint32_t>(firstVolume) + secondVolume + 1) / 2;
    return true;
}

bool calibration_start() {
    uint16_t pitchDac = currentPitchDac;
    uint16_t volumeDac = currentVolumeDac;
    int32_t pitchBase = pitchCalibrationBase;
    int32_t volumeBase = volCalibrationBase;

    // The application has already muted DDS. Disable its interrupts before any
    // foreground SPI access; the same rule applies when rolling back later.
    ihInitialisePitchMeasurement();
    SPImcpDACsend(2048);
    SPImcpDAClatch(); // Park audio even if DDS has not yet published its muted frame.
    SPImcpDAC2Bsend(1600); // Retain the original volume bias during pitch tuning.
    bool success = calibrateOscillator(false, pitchDac);
    if (success) {
        ihInitialiseVolumeMeasurement();
        success = calibrateOscillator(true, volumeDac);
    }

    if (success) {
        // Restore the regular sensor/timing mode to validate fresh beat periods.
        ihInitialiseTimer();
        ihInitialiseInterrupts();
        independentDelay(100);
        success = calibration_finalize(pitchBase, volumeBase);
    }

    if (!success) {
        ihInitialisePitchMeasurement();
        SPImcpDAC2Asend(currentPitchDac);
        SPImcpDAC2Bsend(currentVolumeDac);
        ihInitialiseTimer();
        ihInitialiseInterrupts();
        return false;
    }

    // Commit only a complete, validated result. Keep the previous EEPROM slot,
    // RAM values and DACs recoverable until the new record passes readback.
    // EEPROM reads/writes can stall the AVR CPU. Keep the already-muted DDS
    // stopped for this storage phase, as it is during the oscillator search.
    ihInitialisePitchMeasurement();
    const CalibrationValues values = {pitchDac, volumeDac, pitchBase, volumeBase};
    if (!storeCalibration(values)) {
        SPImcpDAC2Asend(currentPitchDac);
        SPImcpDAC2Bsend(currentVolumeDac);
        ihInitialiseTimer();
        ihInitialiseInterrupts();
        return false;
    }
    currentPitchDac = pitchDac;
    currentVolumeDac = volumeDac;
    haveStoredCalibration = true;
    pitchCalibrationBase = pitchBase;
    volCalibrationBase = volumeBase;
    ihInitialiseTimer();
    ihInitialiseInterrupts();
    return true;
}

bool calibration_read() {
    CalibrationValues values;
    uint32_t sequence = 0;
    uint8_t slot = 0;
    bool valid = newestCalibration(values, sequence, slot);
    if (!valid) {
        // Read-only migration fallback. Legacy data has range checks but no
        // checksum; if both V5 slots fail this can recover an older V4 tuning.
        EEPROM.get(EEPROM_PITCH_DAC_VOLTAGE_ADDRESS, values.pitchDac);
        EEPROM.get(EEPROM_VOLUME_DAC_VOLTAGE_ADDRESS, values.volumeDac);
        EEPROM.get(EEPROM_PITCH_DAC_CALIBRATION_BASE_ADDRESS, values.pitchBase);
        EEPROM.get(EEPROM_VOLUME_DAC_CALIBRATION_BASE_ADDRESS, values.volumeBase);
        valid = validCalibrationValues(values);
    }

    currentPitchDac = valid ? values.pitchDac : SAFE_DAC_VALUE;
    currentVolumeDac = valid ? values.volumeDac : SAFE_DAC_VALUE;
    haveStoredCalibration = valid;
    pitchCalibrationBase = valid ? values.pitchBase : F_CPU / PITCH_BEAT_FREQUENCY;
    volCalibrationBase = valid ? values.volumeBase : F_CPU / VOLUME_BEAT_FREQUENCY;
    SPImcpDACinit();
    SPImcpDACsend(2048);
    SPImcpDAClatch();
    SPImcpDAC2Asend(currentPitchDac);
    SPImcpDAC2Bsend(currentVolumeDac);
    DEBUG_PRINTLN(valid ? F("Stored calibration loaded") : F("Calibration required"));
    return valid;
}
