// V5 firmware development (2026): Luca Cipressi (milletgrain).

#ifndef SERIAL_OUTPUT_H
#define SERIAL_OUTPUT_H

#include <Arduino.h>
#include <Print.h>

// Foreground-only, bounded TX. No receive or transmit interrupts are enabled.
// A full queue drops new bytes; it never waits for the UART or masks interrupts.
class PolledSerialOutput : public Print {
public:
    using Print::write;
    size_t write(uint8_t value) override;
};
extern PolledSerialOutput SerialOutput;
void serialOutputBegin();
void serialOutputService(); // At most one byte per call, only when UDRE0 is set.
bool serialOutputTelemetry(uint16_t value); // Enqueue both bytes or neither.
uint16_t serialOutputDroppedBytes(); // Saturates; foreground diagnostics only.

#endif
