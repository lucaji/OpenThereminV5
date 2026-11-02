// OpenThereminV5 firmware development (2026): Luca Cipressi (milletgrain).

#include "timer.h"
#include "serial_output.h"

#include <util/atomic.h>

static volatile uint16_t timerTicks = 0;

uint16_t readTimerTicks() {
    uint16_t value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = timerTicks;
    }
    return value;
}

void incrementTimer() {
    ++timerTicks;
}

bool ticktimer(uint16_t ticks) {
    // Each chunk is below both the sample-clock wrap and the accumulated
    // Timer2 deadline. Polling must remain faster than Timer2's 16.384 ms wrap.
    const uint16_t maxChunk = millisToTicks(1000);
    while (ticks != 0) {
        const uint16_t chunk = ticks > maxChunk ? maxChunk : ticks;
        const uint16_t start = readTimerTicks();
        const uint16_t wallLimit = (chunk + 1U) / 2U + 157U; // 64 us ticks + 10 ms.
        uint16_t elapsed = 0;
        uint8_t previous = TCNT2;
        while (!timerElapsed(start, chunk)) {
            serialOutputService();
            const uint8_t now = TCNT2;
            elapsed += static_cast<uint8_t>(now - previous);
            previous = now;
            if (elapsed >= wallLimit) return false;
        }
        ticks -= chunk;
    }
    return true;
}

void initialiseIndependentTimer() {
    TIMSK2 = 0;
    TCCR2B = 0;
    TCCR2A = 0;
    TCNT2 = 0;
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
}

void independentDelay(uint16_t milliseconds) {
    uint32_t remaining = (static_cast<uint32_t>(milliseconds) * 125 + 7) / 8;
    uint8_t previous = TCNT2;
    while (remaining != 0) {
        serialOutputService();
        const uint8_t now = TCNT2;
        const uint8_t elapsed = now - previous;
        if (elapsed >= remaining) break;
        remaining -= elapsed;
        previous = now;
    }
}

bool millitimer(uint16_t milliseconds) {
    while (milliseconds != 0) {
        const uint16_t chunk = milliseconds > 1000 ? 1000 : milliseconds;
        if (!ticktimer(millisToTicks(chunk))) return false;
        milliseconds -= chunk;
    }
    return true;
}
