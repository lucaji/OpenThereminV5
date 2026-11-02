// OpenThereminV5 firmware development (2026): Luca Cipressi (milletgrain).

#ifndef _TIMER_H
#define _TIMER_H

#include <Arduino.h>

// One DDS tick is 32 us. Direct conversions support 0..2097 ms; use
// millitimer() for longer waits so the 16-bit tick count cannot overflow.
constexpr uint16_t millisToTicks(uint16_t milliseconds) {
    return static_cast<uint16_t>((static_cast<uint32_t>(milliseconds) * 125U) / 4U);
}

// The counter advances only while the DDS interrupt is enabled, and wraps
// every 2.097152 seconds. Foreground clients keep their own start timestamps.
uint16_t readTimerTicks();
void incrementTimer(); // Called only by the DDS ISR.

inline bool timerElapsed(uint16_t start, uint16_t ticks) {
    return static_cast<uint16_t>(readTimerTicks() - start) >= ticks;
}

// Wait on sample time, but fail if the independent clock exceeds the requested
// duration by 10 ms. False means SAMPLE_CLK stalled or fell substantially behind;
// callers performing modal feedback must leave audio muted on that path.
bool ticktimer(uint16_t ticks);
bool millitimer(uint16_t milliseconds);

// Timer2 /1024: 64 us per tick, no interrupts, independent of SAMPLE_CLK.
// Callers must poll within its 16.384 ms wrap interval.
void initialiseIndependentTimer();
void independentDelay(uint16_t milliseconds);

#endif // _TIMER_H
