// V5 firmware development (2026): Luca Cipressi (milletgrain).

#include "serial_output.h"
#include "build.h"

PolledSerialOutput SerialOutput;

#if SERIAL_PORT_MODE != SERIAL_PORT_MODE_OFF
namespace {
constexpr uint8_t QUEUE_SIZE = 64;
constexpr uint8_t QUEUE_MASK = QUEUE_SIZE - 1;
uint8_t queue[QUEUE_SIZE];
uint8_t head = 0, tail = 0;
uint16_t dropped = 0;

uint8_t freeBytes() { return (tail - head - 1) & QUEUE_MASK; }
void recordDrop(uint8_t count) {
    dropped = dropped > UINT16_MAX - count ? UINT16_MAX : dropped + count;
}
void enqueue(uint8_t value) {
    queue[head] = value;
    head = (head + 1) & QUEUE_MASK;
}
}
#endif

void serialOutputBegin() {
    // Also explicitly disable the receiver: unsolicited RX traffic has no ISR.
    UCSR0B = 0;
#if SERIAL_PORT_MODE != SERIAL_PORT_MODE_OFF
    head = tail = 0;
    dropped = 0;
#if (SERIAL_PORT_MODE & SERIAL_PORT_MODE_MIDI)
    constexpr uint16_t divisor = F_CPU / 8UL / SERIAL_SPEED_MIDI - 1;
    UCSR0A = _BV(U2X0);
#else
    // Match Arduino's 16 MHz / 57600 exception (normal speed, -0.8% error).
    constexpr uint16_t divisor = F_CPU / 16UL / SERIAL_SPEED_DEBUG - 1;
    UCSR0A = 0;
#endif
    UBRR0H = divisor >> 8;
    UBRR0L = divisor;
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8N1
    UCSR0B = _BV(TXEN0);
#endif
}

void serialOutputService() {
#if SERIAL_PORT_MODE != SERIAL_PORT_MODE_OFF
    if (head != tail && (UCSR0A & _BV(UDRE0))) {
        UDR0 = queue[tail];
        tail = (tail + 1) & QUEUE_MASK;
    }
#endif
}

size_t PolledSerialOutput::write(uint8_t value) {
#if SERIAL_PORT_MODE != SERIAL_PORT_MODE_OFF
    if (freeBytes() == 0) { recordDrop(1); return 0; }
    enqueue(value);
    return 1;
#else
    (void)value;
    return 0;
#endif
}

bool serialOutputTelemetry(uint16_t value) {
#if SERIAL_PORT_MODE != SERIAL_PORT_MODE_OFF
    if (freeBytes() < 2) { recordDrop(2); return false; }
    enqueue(value);
    enqueue(value >> 8);
    return true;
#else
    (void)value;
    return false;
#endif
}

uint16_t serialOutputDroppedBytes() {
#if SERIAL_PORT_MODE != SERIAL_PORT_MODE_OFF
    return dropped;
#else
    return 0;
#endif
}
