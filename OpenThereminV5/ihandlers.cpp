// OpenThereminV5 firmware development (2026): Luca Cipressi (milletgrain).

#include "ihandlers.h"
#include "SPImcpDAC.h"
#include "timer.h"
#include "build.h"
#include "hw.h"
#include "audio_arithmetic.h"

#include <util/atomic.h>

#include "wavetables_generated.h"

const uint8_t num_wavetables = WAVE_VOICE_COUNT;
static const uint16_t MCP_DAC_BASE = 2048;

static volatile uint16_t vScaledVolume = 0;
static volatile uint16_t vPointerIncrement = 0;
static volatile uint8_t vIncrementFraction = 0;
// Foreground owns voice/band selection; ISR only loads a complete published pointer.
static uint8_t selectedVoice = 0;
static uint8_t selectedBand = 0;
static const int16_t* volatile requestedWave = wave_sine_b32;
static const int16_t* volatile activeWave = wave_sine_b32;
static uint16_t pointer = 0; // DDS high 16 bits: 8 index + 8 interpolation bits.
static uint8_t phaseFraction = 0; // Eight additional phase bits for smooth bass pitch.

static inline __attribute__((always_inline)) int16_t interpolateWave(
        const int16_t* table, uint16_t phase) {
    const uint8_t offset = phase >> 8;
    const uint8_t fraction = phase;
    const int16_t* at = table + offset;
    const int16_t a = pgm_read_word_near(at);
    const int16_t b = pgm_read_word_near(at + 1);
    return a + interpolateDelta(b - a, fraction);
}

static uint8_t bandForIncrement(uint16_t increment) {
    // Upward transitions have 6.25% margin below each Nyquist limit.
    // Small downward hysteresis avoids chatter without retaining unsafe harmonics.
    static const uint16_t upper[] = {960, 1920, 3840};
    uint8_t band = selectedBand;
    while (band < WAVE_BAND_COUNT - 1 && increment >= upper[band]) ++band;
    while (band > 0 && increment < upper[band - 1] - 32) --band;
    return band;
}

static volatile uint16_t pitchMeasurement = 0;
static volatile uint16_t volumeMeasurement = 0;
static volatile bool pitchValueAvailable = false;
static volatile bool volumeValueAvailable = false;
static volatile bool volumeRestarted = true;

#if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
static volatile uint16_t pitchCV = 0;
static volatile uint16_t volCV = 0;
static volatile bool pitchCVAvailable = false;
static volatile bool volumeCVAvailable = false;
#endif

volatile uint16_t timer_overflow_counter = 0;

// Edge ISRs arm their channel at zero. Saturating at five makes acceptance
// one-shot and leaves a channel disarmed until a new hardware edge arrives.
static const uint8_t DEBOUNCE_COMPLETE = 5;
// 3.072 ms exceeds the slowest calibrated beat period (1 / 550 Hz), but
// expires before Timer1's 4.096 ms wrap can alias a missing edge into a period.
static const uint8_t VOLUME_EDGE_TIMEOUT_TICKS = 96;
static const uint8_t PITCH_EDGE_TIMEOUT_TICKS = 96;
static volatile uint8_t pitchEdgeAge = PITCH_EDGE_TIMEOUT_TICKS;
static volatile bool pitchRestarted = true;
static volatile uint8_t debounce_p = DEBOUNCE_COMPLETE;
static volatile uint8_t debounce_v = DEBOUNCE_COMPLETE;
static volatile uint8_t volumeEdgeAge = VOLUME_EDGE_TIMEOUT_TICKS;
static uint16_t pitchCandidate = 0;
static volatile uint16_t volumeCandidate = 0;
static volatile bool volumeCandidateValid = false;
static uint16_t previousPitchCounter = 0;
static uint16_t previousVolumeCounter = 0;
static bool havePreviousPitch = false;
static volatile bool havePreviousVolume = false;

bool takePitchMeasurement(uint16_t& value, bool& restarted) {
    bool available;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        available = pitchValueAvailable;
        if (available) {
            value = pitchMeasurement;
            restarted = pitchRestarted;
            pitchRestarted = false;
            pitchValueAvailable = false;
        }
    }
    return available;
}

bool takePitchMeasurement(uint16_t& value) {
    bool restarted;
    return takePitchMeasurement(value, restarted);
}

bool pitchCaptureFresh() {
    // An 8-bit read is atomic on AVR; publication still uses the mailbox above.
    return pitchEdgeAge < PITCH_EDGE_TIMEOUT_TICKS;
}

bool captureRestartPending() {
    bool pending;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Read both flags in one bounded snapshot; do not add branch paths to
        // the interrupt-masked interval with a short-circuit expression.
        pending = pitchRestarted | volumeRestarted;
    }
    return pending;
}

bool takeVolumeMeasurement(uint16_t& value, bool& restarted) {
    bool available;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        available = volumeValueAvailable;
        if (available) {
            value = volumeMeasurement;
            restarted = volumeRestarted;
            volumeRestarted = false;
            volumeValueAvailable = false;
        }
    }
    return available;
}

bool takeVolumeMeasurement(uint16_t& value) {
    bool restarted;
    return takeVolumeMeasurement(value, restarted);
}

void discardMeasurements() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pitchValueAvailable = false;
        volumeValueAvailable = false;
    }
}

void parkAudioAfterClockFailure() {
    uint8_t savedMask;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        savedMask = EIMSK;
        EIMSK = savedMask & ~_BV(INT1);
        vScaledVolume = 0;
    }
    // Only audio owns SPI during playback. Masking its source lets foreground
    // park the DAC while capture interrupts remain enabled. This is a fault-mode
    // latch, not an additional latch in the normal per-sample output path.
    SPImcpDACsend(MCP_DAC_BASE);
    SPImcpDAClatch();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        EIFR = _BV(INTF1); // Discard a clock edge that returned during parking.
        EIMSK = savedMask;
    }
}

void setAudioParameters(uint16_t increment, uint16_t gain) {
    setAudioParametersFine(static_cast<uint32_t>(increment) << 8, gain);
}

template<bool AdoptNow>
static inline __attribute__((always_inline)) void publishAudioControls(
        uint32_t incrementQ8, uint16_t gain, const int16_t* table) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (AdoptNow) activeWave = table;
        vPointerIncrement = incrementQ8 >> 8;
        vIncrementFraction = static_cast<uint8_t>(incrementQ8);
        vScaledVolume = gain;
        requestedWave = table;
    }
}

void setAudioParametersFine(uint32_t incrementQ8, uint16_t gain) {
    const uint32_t maximum = 8191UL * 256 + 128; // 16383 / 2, highest register.
    if (incrementQ8 > maximum) incrementQ8 = maximum;
    const uint16_t increment = incrementQ8 >> 8;
    selectedBand = bandForIncrement(increment);
    const int16_t* table = waveTableFor(selectedVoice, selectedBand);
    // Only foreground writes these controls, so the decision can precede CLI.
    // Keep branching and loads outside the short, coherent publication window.
    if (vScaledVolume == 0 || incrementQ8 == 0)
        publishAudioControls<true>(incrementQ8, gain, table);
    else
        publishAudioControls<false>(incrementQ8, gain, table);
}

void getAudioParameters(uint16_t& increment, uint16_t& gain) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        increment = vPointerIncrement;
        gain = vScaledVolume;
    }
}

void getAudioParametersFine(uint32_t& incrementQ8, uint16_t& gain) {
    uint16_t increment;
    uint8_t fraction;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        increment = vPointerIncrement;
        fraction = vIncrementFraction;
        gain = vScaledVolume;
    }
    incrementQ8 = (static_cast<uint32_t>(increment) << 8) | fraction;
}

void setWavetable(uint8_t selector) {
    if (selector < num_wavetables) {
        selectedVoice = selector;
        const int16_t* table = waveTableFor(selectedVoice, selectedBand);
        if (vScaledVolume == 0 || (vPointerIncrement == 0 && vIncrementFraction == 0)) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                activeWave = table;
                requestedWave = table;
            }
        } else {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { requestedWave = table; }
        }
    }
}

void publishPitchCV(uint16_t value) {
#if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
    if (value > 4095) value = 4095;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pitchCV = value;
        pitchCVAvailable = true;
    }
#else
    (void)value;
#endif
}

void publishVolumeCV(uint16_t value) {
#if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
    if (value > 4095) value = 4095;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        volCV = value;
        volumeCVAvailable = true;
    }
#else
    (void)value;
#endif
}

// Called with interrupts masked while switching back from calibration.
static void resetCaptureState() {
    debounce_p = DEBOUNCE_COMPLETE;
    pitchEdgeAge = PITCH_EDGE_TIMEOUT_TICKS;
    pitchRestarted = true;
    debounce_v = DEBOUNCE_COMPLETE;
    volumeEdgeAge = VOLUME_EDGE_TIMEOUT_TICKS;
    volumeRestarted = true;
    volumeCandidateValid = false;
    havePreviousPitch = false;
    havePreviousVolume = false;
    pitchValueAvailable = false;
    volumeValueAvailable = false;
}

void ihInitialiseTimer() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        EIMSK = 0;
        PCICR = 0;
        TIMSK1 = 0;
        TCCR1B = 0;
        TCCR1A = 0;
        TCCR1C = 0;
        TCNT1 = 0;
        TIFR1 = (1 << ICF1) | (1 << OCF1A) | (1 << OCF1B) | (1 << TOV1);
        resetCaptureState();

        // Timer1 timestamps F_PITCH at ICP1/PB0 using the 16 MHz CPU clock.
        TCCR1B = (1 << ICES1) | (1 << CS10);
        TIMSK1 = (1 << ICIE1);

        // Timer0 overflow interrupts are only used with DDS disabled during
        // pitch calibration. Playback timing uses SAMPLE_CLK and polled Timer2.
        TIMSK0 = 0;
        TCCR0B = 0;
        TCCR0A = (1 << WGM01) | (1 << WGM00);
        TCNT0 = 0;
        TIFR0 = (1 << OCF0A) | (1 << OCF0B) | (1 << TOV0);
        TCCR0B = (1 << CS01) | (1 << CS00);
    }
    initialiseIndependentTimer();
}

void ihInitialiseInterrupts() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // INT1 is the highest-priority enabled playback interrupt. Volume uses
        // PCINT18 on the same PD2 pin, below INT1 in the dispatch order.
        EIMSK = 0;
        PCICR = 0;
        PCMSK2 = _BV(PCINT18);
        PCIFR = _BV(PCIF2);
        // INT0 stays masked, but its rising-edge flag records even a short
        // pulse that has already fallen by the time PCINT2 is dispatched.
        EICRA = (1 << ISC00) | (1 << ISC01) | (1 << ISC10) | (1 << ISC11);
        EIFR = (1 << INTF0) | (1 << INTF1);
        PCICR = _BV(PCIE2);
        EIMSK = (1 << INT1);
    }
}

void ihInitialisePitchMeasurement() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        EIMSK = 0;
        PCICR = 0;
        TIMSK1 = 0;
        TCCR1B = 0;
        TCCR1A = 0;
        TCCR1C = 0;
        TIFR1 = (1 << ICF1) | (1 << OCF1A) | (1 << OCF1B) | (1 << TOV1);
        EIFR = (1 << INTF0) | (1 << INTF1);
        TIMSK1 = (1 << TOIE1);
        // Calibration starts Timer1's external T1/PD5 clock for each gate.
        // Restore Arduino delay() only after all sample/volume IRQs are off.
        TIMSK0 = 0;
        TCCR0B = 0;
        TCCR0A = _BV(WGM01) | _BV(WGM00);
        TCNT0 = 0;
        TIFR0 = _BV(OCF0A) | _BV(OCF0B) | _BV(TOV0);
        TCCR0B = _BV(CS01) | _BV(CS00);
        TIMSK0 = _BV(TOIE0);
    }
}

// Keep mode changes identifiable in the linked timing audit (DDS is off).
void __attribute__((noinline)) ihInitialiseVolumeMeasurement() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        EIMSK = 0;
        PCICR = 0;
        TIMSK0 = 0;
        TIMSK1 = 0;
        TCCR0B = 0;
        TCCR1B = 0;
        TCCR0A = 0;
        TCCR1A = 0;
        TCCR1C = 0;
        OCR0A = 0xff;
        TIFR0 = (1 << OCF0A) | (1 << OCF0B) | (1 << TOV0);
        TIFR1 = (1 << ICF1) | (1 << OCF1A) | (1 << OCF1B) | (1 << TOV1);
        EIFR = (1 << INTF0) | (1 << INTF1);
        TIMSK0 = (1 << OCIE0A);
        // Calibration starts external T0/PD4 counting and Timer1's /1024
        // gate clock together for each measurement; neither runs yet.
    }
}

// Capture publication and CV transfer are deliberately different paths. A busy
// confirmation/timeout frame defers CV to a subsequent sample, keeping the
// sample ISR bounded without dropping sensor work or changing LDAC ownership.
template<bool AllowCv>
static inline __attribute__((always_inline)) void serviceVolumeAndCv() {
    const uint8_t age = volumeEdgeAge;
    if (age < VOLUME_EDGE_TIMEOUT_TICKS) {
        const uint8_t nextAge = age + 1;
        volumeEdgeAge = nextAge;
        if (nextAge == VOLUME_EDGE_TIMEOUT_TICKS) {
            havePreviousVolume = false;
            volumeRestarted = true;
            volumeCandidateValid = false;
            volumeValueAvailable = false;
            debounce_v = DEBOUNCE_COMPLETE;
            return;
        }
    }

    // INT1 can run before the pending pin-change ISR has armed the new pulse.
    // The masked INT0 latch identifies that rising edge even after it fell.
    if (F_VOL_PIN && !(EIFR & _BV(INTF0))) {
        const uint8_t count = debounce_v;
        if (count < DEBOUNCE_COMPLETE) {
            const uint8_t next = count + 1;
            debounce_v = next;
            if (next == DEBOUNCE_COMPLETE && volumeCandidateValid) {
                volumeMeasurement = volumeCandidate;
                volumeValueAvailable = true;
            }
            // Keep all debounce frames free of CV, not just publication.
            return;
        }
    }

#if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
    if (AllowCv) {
        if (pitchCVAvailable) {
            SPImcpDAC3Asend(pitchCV);
            pitchCVAvailable = false;
        } else if (volumeCVAvailable) {
            SPImcpDAC3Bsend(volCV);
            volumeCVAvailable = false;
        }
    }
#endif
}

// 31.25 kHz external sample clock: 512 CPU cycles / 32 us per frame.
// Normal non-nested ISRs. Only an already-running PCINT2 or TIMER1_CAPT can
// delay dispatch; Timer0 and UART interrupts are disabled during playback.
ISR(INT1_vect) {
#ifdef TH_DEBUG
    HW_LED_RED_ON;
#endif

    // All DACs share LDAC. Audio and pending CV data queued during the prior
    // frame become visible together; there are no other runtime latch pulses.
    SPImcpDAClatch();
    const uint16_t phase = pointer;
    // Audible table changes are adopted only at a real phase wrap below.
    const int16_t waveSample = interpolateWave(activeWave, phase);
    const int16_t scaledSample = scaleWaveExact(waveSample, vScaledVolume);
    SPImcpDACsend(scaledSample + MCP_DAC_BASE);
    const uint16_t finePhase = static_cast<uint16_t>(phaseFraction) + vIncrementFraction;
    phaseFraction = static_cast<uint8_t>(finePhase);
    const uint16_t next = phase + vPointerIncrement + (finePhase >> 8);
    pointer = next;
    if (next < phase) activeWave = requestedWave;
    incrementTimer();

    // Expire before Timer1 wraps (4.096 ms). A returning edge starts a new
    // pair, never an interval against an old modulo-16-bit timestamp.
    const uint8_t pitchAge = pitchEdgeAge;
    if (pitchAge < PITCH_EDGE_TIMEOUT_TICKS) {
        pitchEdgeAge = pitchAge + 1;
        if (pitchAge + 1 == PITCH_EDGE_TIMEOUT_TICKS) {
            havePreviousPitch = false;
            pitchValueAvailable = false;
            pitchRestarted = true;
            debounce_p = DEBOUNCE_COMPLETE;
        }
    }

    // INT1 can dispatch ahead of a pending input-capture ISR. Do not confirm
    // the previous candidate using a new pulse before that ISR resets debounce.
    if (F_PITCH_PIN && !(TIFR1 & _BV(ICF1))) {
        const uint8_t count = debounce_p;
        if (count < DEBOUNCE_COMPLETE) {
            const uint8_t next = count + 1;
            debounce_p = next;
            if (next == 3) {
                pitchCandidate = ICR1;
            } else if (next == DEBOUNCE_COMPLETE) {
                if (havePreviousPitch) {
                    pitchMeasurement = pitchCandidate - previousPitchCounter;
                    pitchValueAvailable = true;
                }
                previousPitchCounter = pitchCandidate;
                havePreviousPitch = true;
            }
            serviceVolumeAndCv<false>();
            goto sampleComplete;
        }
    }

    serviceVolumeAndCv<true>();

sampleComplete:
#ifdef TH_DEBUG
    HW_LED_RED_OFF;
#endif
    ; // The label is also valid when the timing marker is compiled away.
}

ISR(PCINT2_vect) {
    // Pin-change triggers on both edges. Use the masked INT0 rising-edge latch,
    // not the current pin level: a complete short pulse may coalesce while DDS
    // runs. It must still advance the raw reference to prevent false long periods.
    if (!(EIFR & _BV(INTF0))) return;
    EIFR = _BV(INTF0);
    // Keep the raw reference on every edge, but tell calibration/filters when
    // an intervening high pulse never reached confirmation.
    if (debounce_v < DEBOUNCE_COMPLETE) {
        volumeRestarted = true;
        // A pre-rejection mailbox value must not consume the restart flag
        // before a new confirmed raw-edge period has actually been published.
        volumeValueAvailable = false;
    }
    // Finish the flag handoff before the 16-bit arithmetic, keeping AVR
    // register lifetimes (and the compiler's ISR save/restore set) short.
    volumeCandidateValid = havePreviousVolume;
    havePreviousVolume = true;
    const uint16_t counter = TCNT1;
    // Always advance on a raw edge, even if its high pulse is too short for
    // confirmation. Measuring between accepted pulses would count skipped
    // cycles as one long period and incorrectly turn near-antenna mute loud.
    const uint16_t previous = previousVolumeCounter;
    previousVolumeCounter = counter;
    // Store the new reference before subtraction so it need not stay live
    // in another register pair throughout the candidate calculation.
    volumeCandidate = counter - previous;
    volumeEdgeAge = 0;
    debounce_v = 0;
}

ISR(TIMER1_CAPT_vect) {
    // A new edge before confirmation means the preceding pulse was rejected.
    // Break the accepted pair so skipped cycles cannot masquerade as one period.
    if (debounce_p < DEBOUNCE_COMPLETE) {
        havePreviousPitch = false;
        pitchValueAvailable = false;
        pitchRestarted = true;
    }
    pitchEdgeAge = 0;
    debounce_p = 0;
}

// Extend Timer0's external volume pulse count during calibration.
ISR(TIMER0_COMPA_vect) {
    ++timer_overflow_counter;
}

// Same counter extension during pitch calibration. An explicit vector alias
// avoids AVR GCC folding identical ISR bodies into a CALL to another RETI,
// which would re-enable interrupts inside the caller's epilogue.
ISR(TIMER1_OVF_vect, ISR_ALIASOF(TIMER0_COMPA_vect));
