// V5 firmware development (2026): Luca Cipressi (milletgrain).

#include "application.h"
#include "calibration.h"
#include "hw.h"
#include "ihandlers.h"
#include "timer.h"

namespace {

const uint16_t MAX_VOLUME = 4095;
const uint16_t BUTTON_HOLD_TICKS = millisToTicks(1920);
// Provisional control feel: verify these margins/times with the physical rig.
const uint16_t BUTTON_QUALIFY_TICKS = millisToTicks(8);
const uint16_t POT_POLL_TICKS = millisToTicks(2);
const uint16_t PITCH_SIGNAL_TIMEOUT_TICKS = millisToTicks(10);
const uint16_t VOLUME_SIGNAL_TIMEOUT_TICKS = millisToTicks(10);
const int16_t FIELD_POT_NOISE_MARGIN = 2;
const uint8_t SELECTOR_MARGIN = 6;
const int32_t FIELD_POT_MAX_STEP_Q8 = 32L * 256;

bool calibrationValid = false;
bool audioEnabled = false;
bool buttonWasPressed = false;
bool buttonRawPressed = false;
bool buttonUnmutePending = false;
bool calibrationArmed = false;
uint16_t buttonStarted = 0;
uint16_t buttonCandidateSince = 0;
uint16_t lastPotRead = 0;
uint8_t nextPot = 0;

int16_t pitchPotValue = 0;
int16_t volumePotValue = 0;
int16_t pitchPotTarget = 0;
int16_t volumePotTarget = 0;
uint8_t pitchPotFraction = 0;
uint8_t volumePotFraction = 0;
uint8_t registerValue = 2;
uint8_t waveValue = 0;

bool havePitch = false;
bool haveVolume = false;
int32_t filteredPitch = 0;
uint8_t filteredPitchFraction = 0; // Q8 remainder retained by the pitch filter.
int32_t filteredVolume = 0;
uint8_t filteredVolumeFraction = 0;
uint16_t lastPitchMeasurement = 0;
uint16_t lastVolumeMeasurement = 0;
uint32_t lastIncrement = 0; // Q8 increment for the 24-bit DDS.
uint16_t lastGain = 0;

#if ((SERIAL_PORT_MODE) & SERIAL_PORT_MODE_MIDI)
uint16_t lastRawPitch = 0;
uint16_t lastMidiSend = 0;
#endif

#if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
uint16_t lastPitchCV = UINT16_MAX;
uint16_t lastVolumeCV = UINT16_MAX;
bool gateHigh = false;
#endif

uint16_t clampValue(int32_t value, uint16_t maximum) {
    if (value < 0) return 0;
    if (value > maximum) return maximum;
    return static_cast<uint16_t>(value);
}

inline uint16_t gainFromVolume(uint16_t volume) {
    // Unsigned arithmetic is required: the maximum product is 65535.
    return volume * (volume + 2U);
}

uint16_t gainFromLevel(uint16_t level) {
    // Preserve the old square-law values at every 16-code anchor, while
    // retaining the intermediate 12-bit volume levels. The final anchor is full.
    if (level >= 4080) return UINT16_MAX;
    return (static_cast<uint32_t>(level) * (level + 32U)) >> 8;
}

uint8_t selectorForAdc(uint16_t value, uint8_t current, uint8_t positions, bool force) {
    const uint16_t scaled = value * positions;
    if (force) return scaled >> 10;
    const uint16_t margin = SELECTOR_MARGIN * positions;
    while (current + 1 < positions && scaled >= (current + 1U) * 1024 + margin) ++current;
    while (current > 0 && scaled < current * 1024U - margin) --current;
    return current;
}

bool readFieldPot(uint8_t pin, int16_t& target, int16_t& value, uint8_t& fraction, bool force) {
    const int16_t raw = analogRead(pin);
    if (force) {
        target = value = raw;
        fraction = 0;
        return true;
    }
    // Endpoint access must not depend on the previous accepted ADC value.
    if (raw == 0 || raw == 1023 || abs(raw - target) > FIELD_POT_NOISE_MARGIN) target = raw;
    return false; // Smoothing publishes the value at the regular UI slot below.
}

bool smoothFieldPot(int16_t target, int16_t& value, uint8_t& fraction) {
    const int32_t previous = static_cast<int32_t>(value) * 256 + fraction;
    const int32_t delta = static_cast<int32_t>(target) * 256 - previous;
    if (delta == 0) return false;
    // Gentle fine adjustment, faster intentional movement, and a finite slew.
    int32_t step = delta / (abs(delta) > 16L * 256 ? 2 : 4);
    if (step > FIELD_POT_MAX_STEP_Q8) step = FIELD_POT_MAX_STEP_Q8;
    if (step < -FIELD_POT_MAX_STEP_Q8) step = -FIELD_POT_MAX_STEP_Q8;
    if (step == 0) step = delta > 0 ? 1 : -1;
    const int32_t next = previous + step;
    value = next >> 8;
    fraction = static_cast<uint8_t>(next);
    return true;
}

uint8_t filterCoefficient(uint16_t ticks) {
    // round(256 * (1 - exp(-t / 5 ms))) at 8-tick (256 us) intervals.
    // Linear interpolation avoids divisions/floating point and rate-dependent
    // per-event smoothing. Clamp at the 10 ms dropout boundary's upper neighbor.
    static const uint16_t coefficients[] PROGMEM = {
        0, 13, 25, 36, 47, 58, 68, 77, 86, 95, 103, 110, 118, 124,
        131, 137, 143, 149, 154, 159, 164, 169, 173, 177, 181, 185,
        188, 192, 195, 198, 201, 204, 206, 209, 211, 213, 215, 217,
        219, 221, 223
    };
    if (ticks >= 320) return 223;
    const uint8_t index = ticks >> 3;
    const uint8_t a = pgm_read_word_near(coefficients + index);
    const uint8_t b = pgm_read_word_near(coefficients + index + 1);
    return a + ((static_cast<uint16_t>(b - a) * (ticks & 7) + 4) >> 3);
}

int32_t filterPeriod(int32_t previous, uint16_t measurement, uint16_t elapsed, bool closing) {
    // Faster volume closing retains crisp articulation. This is foreground
    // work only; no smoothing or new state is added to the sample ISR.
    const uint16_t responseTicks = closing
        ? (elapsed > 128 ? 320 : elapsed * 2 + elapsed / 2) : elapsed;
    const uint8_t alpha = filterCoefficient(responseTicks);
    const int32_t delta = static_cast<int32_t>(measurement) * 256 - previous;
    // Split before multiplication so even the full 16-bit period span cannot
    // overflow a signed 32-bit intermediate. Exact floor(delta * alpha / 256).
    return previous + (delta >> 8) * alpha +
        ((static_cast<uint16_t>(static_cast<uint8_t>(delta)) * alpha) >> 8);
}

void showPlayingState() {
    if (audioEnabled) {
        HW_LED_BLUE_ON;
        HW_LED_RED_OFF;
    } else {
        HW_LED_BLUE_OFF;
        HW_LED_RED_ON;
    }
}

bool readPot(uint8_t pot, bool force = false) {
    switch (pot) {
    case 0:
        return readFieldPot(PITCH_POT, pitchPotTarget, pitchPotValue, pitchPotFraction, force);
    case 1:
        return readFieldPot(VOLUME_POT, volumePotTarget, volumePotValue, volumePotFraction, force);
    case 2: {
        const int16_t value = analogRead(REGISTER_SELECT_POT);
        const uint8_t selected = 3 - selectorForAdc(value, 3 - registerValue, 3, force);
        if (!force && selected == registerValue) return false;
        // Left = -1 octave, center = 0, right = +1 octave.
        registerValue = selected;
        DEBUG_PRINT(F("OCT register="));
        DEBUG_PRINTLN(registerValue);
        return true;
    }
    default: {
        const int16_t value = analogRead(WAVE_SELECT_POT);
        const uint8_t selected = selectorForAdc(value, waveValue, num_wavetables, force);
        if (!force && selected == waveValue) return false;
        waveValue = selected;
        setWavetable(selected);
        DEBUG_PRINT(F("WAV="));
        DEBUG_PRINTLN(selected);
        return true;
    }
    }
}

void updateOutputs() {
    // Clamp before narrowing: a negative 32-bit result must never wrap to a
    // large positive int16_t and become maximum pitch.
    int32_t pitchQ8 = havePitch
        ? (pitchCalibrationBase - filteredPitch + 2048L -
           (static_cast<int32_t>(pitchPotValue) << 2)) * 256 - filteredPitchFraction -
           static_cast<uint16_t>(pitchPotFraction) * 4
        : 0;
    if (pitchQ8 < 0) pitchQ8 = 0;
    if (pitchQ8 > 16383L * 256) pitchQ8 = 16383L * 256;
    const uint32_t increment = static_cast<uint32_t>(pitchQ8) >> registerValue;
    const int32_t volumeQ8 = (MAX_VOLUME - 1024L) * 256 -
        ((volCalibrationBase - filteredVolume) * 256 - filteredVolumeFraction) / 2 +
        (static_cast<int32_t>(volumePotValue) * 256 + volumePotFraction) * 4;
    const uint16_t volume = audioEnabled && havePitch && haveVolume
        ? clampValue(volumeQ8 >> 8, MAX_VOLUME) : 0;
    const uint16_t gain = gainFromLevel(volume);

    if (increment != lastIncrement || gain != lastGain) {
        setAudioParametersFine(increment, gain);
        lastIncrement = increment;
        lastGain = gain;
    }

#if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
    const uint16_t pitchValue = pitchQ8 >> 8;
    // Hold pitch CV while muted; volume CV and gate explicitly become zero.
    if (audioEnabled && havePitch) {
        uint16_t cv;
#if CV_OUTPUT_MODE == CV_OUTPUT_MODE_LOG
        uint16_t octave = 0;
        uint16_t mantissa = pitchValue;
        while (mantissa > 1023) {
            octave += 819;
            mantissa >>= 1;
        }
        const uint16_t fraction = mantissa > 512 ? mantissa - 512 : 0;
        const int32_t value = octave +
            static_cast<int32_t>((static_cast<uint32_t>(fraction) * 819) >> 9) - 48L;
        cv = clampValue(value, 4095);
#else
        cv = pitchValue >> 2;
#endif
        if (cv != lastPitchCV) {
            publishPitchCV(cv);
            lastPitchCV = cv;
        }
    }
    if (volume != lastVolumeCV) {
        publishVolumeCV(volume);
        lastVolumeCV = volume;
    }

    const uint8_t gateLevel = volume >> 5;
    if (!gateHigh && audioEnabled && havePitch && haveVolume &&
        gateLevel >= GATE_ON_THRESHOLD_LEVEL) {
        GATE_PULLUP;
        if (GATE_SENSE) {
            GATE_DRIVE_HIGH;
            gateHigh = true;
        }
    } else if (!audioEnabled || gateLevel <= GATE_OFF_THRESHOLD_LEVEL) {
        // Failed sensing leaves a pull-up enabled even though gateHigh is false.
        GATE_DRIVE_LOW;
        gateHigh = false;
    }
#endif
}

#if AUDIO_FEEDBACK_MODE == AUDIO_FEEDBACK_ON
bool playTone(float hz, uint16_t milliseconds, uint8_t volume) {
    uint32_t oldIncrement;
    uint16_t oldGain;
    getAudioParametersFine(oldIncrement, oldGain);
    // 2^24 phase units per cycle / 31250 samples per second.
    setAudioParametersFine(static_cast<uint32_t>(hz * (16777216.0f / 31250.0f)),
                       gainFromVolume(volume));
    const bool completed = millitimer(milliseconds);
    setAudioParametersFine(oldIncrement, completed ? oldGain : 0);
    return completed;
}
#endif

void finishCalibrationUi(bool resumeAudio, bool clockFailed = false) {
    audioEnabled = resumeAudio;
    discardMeasurements();
    havePitch = false;
    haveVolume = false;
    filteredPitchFraction = filteredVolumeFraction = 0;
    updateOutputs();
    if (clockFailed) parkAudioAfterClockFailure();
    showPlayingState();
}

void runCalibration() {
    // Calibration is modal and repurposes both hardware timers. Silence the
    // actual DDS gain before any blocking animation or measurement.
    audioEnabled = false;
    updateOutputs();
    for (uint8_t i = 0; i < 10; ++i) {
        if (!millitimer(200 - i * 10)) {
            finishCalibrationUi(false, true);
            return;
        }
        HW_LED_BLUE_TOGGLE;
        HW_LED_RED_TOGGLE;
    }
    HW_LED_BLUE_ON;
    HW_LED_RED_ON;

#if AUDIO_FEEDBACK_MODE == AUDIO_FEEDBACK_ON
    const float middleC = 261.6f;
    if (!playTone(middleC, 150, 25) || !playTone(middleC * 2, 150, 25) ||
        !playTone(middleC * 4, 150, 25)) {
        finishCalibrationUi(false, true);
        return;
    }
#endif

    const bool success = calibration_start();
    if (success) {
        calibrationValid = true;
#if AUDIO_FEEDBACK_MODE == AUDIO_FEEDBACK_ON
        if (!playTone(middleC * 2, 150, 25) || !playTone(middleC * 2, 150, 25)) {
            finishCalibrationUi(false, true);
            return;
        }
#endif
    } else {
        HW_LED_BLUE_OFF;
        for (uint8_t i = 0; i < 10; ++i) {
            if (!millitimer(200 - i * 10)) {
                finishCalibrationUi(false, true);
                return;
            }
            HW_LED_RED_TOGGLE;
        }
#if AUDIO_FEEDBACK_MODE == AUDIO_FEEDBACK_ON
        if (!playTone(middleC * 4, 150, 25) || !playTone(middleC, 150, 25)) {
            finishCalibrationUi(false, true);
            return;
        }
#endif
    }

    // Start the filters from fresh post-calibration periods on either path.
    finishCalibrationUi(success);
}

} // namespace

void theremin_setup() {
    serialOutputBegin();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
#if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
    GATE_DRIVE_LOW;
#endif
    showPlayingState();
    for (uint8_t pot = 0; pot < 4; ++pot) readPot(pot, true);
    calibrationValid = calibration_read();
    ihInitialiseTimer();
    ihInitialiseInterrupts();
    updateOutputs();
    lastPotRead = readTimerTicks();
#if ((SERIAL_PORT_MODE) & SERIAL_PORT_MODE_MIDI)
    lastMidiSend = lastPotRead;
#endif
}

void theremin_loop() {
    uint16_t now = readTimerTicks();
    bool changed = false;
    const bool pressed = HW_BUTTON_PRESSED;
    if (pressed != buttonRawPressed) {
        buttonRawPressed = pressed;
        buttonCandidateSince = now;
    }
    // The board uses a TTP223 touch controller. Brief stable-state qualification
    // rejects glitches without imposing mechanical-switch debounce timings.
    if (pressed != buttonWasPressed &&
        static_cast<uint16_t>(now - buttonCandidateSince) >= BUTTON_QUALIFY_TICKS) {
        buttonWasPressed = pressed;
        if (pressed) {
            buttonStarted = buttonCandidateSince;
            calibrationArmed = true;
            buttonUnmutePending = !audioEnabled;
            audioEnabled = false; // Muting is immediate after qualification.
        } else {
            // Unmute only on a short release; holding from mute never sounds
            // a note before calibration begins.
            if (calibrationArmed && buttonUnmutePending && calibrationValid) audioEnabled = true;
            calibrationArmed = false;
            buttonUnmutePending = false;
        }
        changed = true;
        showPlayingState();
    }

    if (pressed && buttonWasPressed && calibrationArmed &&
        static_cast<uint16_t>(now - buttonStarted) >= BUTTON_HOLD_TICKS) {
        // One calibration per press, even if the button remains held afterward.
        calibrationArmed = false;
        buttonUnmutePending = false;
        runCalibration();
        now = readTimerTicks();
        lastPotRead = now;
#if ((SERIAL_PORT_MODE) & SERIAL_PORT_MODE_MIDI)
        lastMidiSend = now;
#endif
        changed = true;
    }

    uint16_t measurement;
    bool pitchRestarted;
    const uint16_t pitchElapsed = now - lastPitchMeasurement;
    if (takePitchMeasurement(measurement, pitchRestarted)) {
        const int32_t previous = filteredPitch * 256 + filteredPitchFraction;
        const int32_t target = static_cast<int32_t>(measurement) * 256;
        const int32_t next = havePitch && !pitchRestarted && pitchElapsed < PITCH_SIGNAL_TIMEOUT_TICKS
            ? filterPeriod(previous, measurement, pitchElapsed, false) : target;
        filteredPitch = next >> 8;
        filteredPitchFraction = static_cast<uint8_t>(next);
        havePitch = true;
        lastPitchMeasurement = now;
#if ((SERIAL_PORT_MODE) & SERIAL_PORT_MODE_MIDI)
        lastRawPitch = measurement;
#endif
        changed = true;
    } else if (havePitch && (!pitchCaptureFresh() || pitchElapsed >= PITCH_SIGNAL_TIMEOUT_TICKS)) {
        // Raw-edge expiry and confirmed-period expiry are separate safeguards.
        // The next valid pair seeds the filter instead of gliding from stale data.
        havePitch = false;
        filteredPitchFraction = 0;
        changed = true;
    }
    bool volumeRestarted;
    if (takeVolumeMeasurement(measurement, volumeRestarted)) {
        const uint16_t period = measurement < 5000 ? 5000 : measurement;
        const uint16_t elapsed = now - lastVolumeMeasurement;
        const int32_t previous = filteredVolume * 256 + filteredVolumeFraction;
        const int32_t target = static_cast<int32_t>(period) * 256;
        const int32_t next = haveVolume && !volumeRestarted && elapsed < VOLUME_SIGNAL_TIMEOUT_TICKS
            ? filterPeriod(previous, period, elapsed, target < previous) : target;
        filteredVolume = next >> 8;
        filteredVolumeFraction = static_cast<uint8_t>(next);
        haveVolume = true;
        lastVolumeMeasurement = now;
        changed = true;
    } else if (haveVolume &&
               static_cast<uint16_t>(now - lastVolumeMeasurement) >= VOLUME_SIGNAL_TIMEOUT_TICKS) {
        // Missing or persistently unconfirmed pulses must silence audio, CV
        // and gate. A fresh period will seed the filter again on recovery.
        haveVolume = false;
        filteredVolumeFraction = 0;
        changed = true;
    }

    // One blocking ADC conversion per UI slot, all four controls about every 8ms.
    if (static_cast<uint16_t>(now - lastPotRead) >= POT_POLL_TICKS) {
        lastPotRead = now;
        changed = readPot(nextPot) || changed;
        nextPot = (nextPot + 1) & 3;
        changed = smoothFieldPot(pitchPotTarget, pitchPotValue, pitchPotFraction) || changed;
        changed = smoothFieldPot(volumePotTarget, volumePotValue, volumePotFraction) || changed;
    }
    if (changed) updateOutputs();

#if ((SERIAL_PORT_MODE) & SERIAL_PORT_MODE_MIDI)
    // Legacy two-byte pitch telemetry, not MIDI note-on/note-off messages.
    if (static_cast<uint16_t>(now - lastMidiSend) >= millisToTicks(100)) {
        lastMidiSend = now;
        serialOutputTelemetry(lastRawPitch);
    }
#endif
    serialOutputService();
}
