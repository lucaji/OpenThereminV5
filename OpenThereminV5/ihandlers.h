// OpenThereminV5 firmware development (2026): Luca Cipressi (milletgrain).

#ifndef _IHANDLERS_H
#define _IHANDLERS_H

#include <Arduino.h>

extern const uint8_t num_wavetables;

// Calibration accesses this only while the measurement clock is stopped and
// interrupts are masked, accounting for any pending hardware overflow flag.
extern volatile uint16_t timer_overflow_counter;

// Each take operation copies one complete sample and clears its ready flag.
// A slow consumer receives the latest sample; samples are not queued.
bool takePitchMeasurement(uint16_t& value);
// The restart flag remains latched until consumed, even if foreground missed
// the brief expired state. Seed the pitch filter instead of smoothing old data.
bool takePitchMeasurement(uint16_t& value, bool& restarted);
bool pitchCaptureFresh();
// True while either channel has an unconsumed restart marker.
// Calibration uses this to detect expiry/rejection even without a new sample.
bool captureRestartPending();
bool takeVolumeMeasurement(uint16_t& value);
// Also reports expiry or an intervening rejected pulse, without changing the
// raw-edge period calculation. Calibration can require uninterrupted windows.
bool takeVolumeMeasurement(uint16_t& value, bool& restarted);
void discardMeasurements();
// Fault-mode foreground operation: park the physical audio DAC when SAMPLE_CLK
// cannot deliver a queued mute. Runtime CV is disabled in this V5 milestone.
void parkAudioAfterClockFailure();

// Publish/read a coherent DDS control pair on the 8-bit MCU.
void setAudioParameters(uint16_t increment, uint16_t gain);
void getAudioParameters(uint16_t& increment, uint16_t& gain);
// V5 fine pitch uses Q8 units of the original 16-bit increment (24-bit DDS).
// Inputs above the supported top register are clamped before publication.
void setAudioParametersFine(uint32_t incrementQ8, uint16_t gain);
void getAudioParametersFine(uint32_t& incrementQ8, uint16_t& gain);
void setWavetable(uint8_t selector);

// Publish a complete CV value, replacing an older pending update.
void publishPitchCV(uint16_t value);
void publishVolumeCV(uint16_t value);

void ihInitialiseTimer();
void ihInitialiseInterrupts();
void ihInitialisePitchMeasurement();
void ihInitialiseVolumeMeasurement();

#endif // _IHANDLERS_H
