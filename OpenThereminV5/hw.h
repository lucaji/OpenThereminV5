// OpenThereminV5 firmware development (2026): Luca Cipressi (milletgrain).

#include <Arduino.h>

#ifndef _HW_H_
#define _HW_H_

#define F_VOL_PIN               (PIND & (1 << PORTD2))
#define F_PITCH_PIN             (PINB & (1 << PORTB0))

#define BUTTON_PIN              6
#define HW_BUTTON_STATE         (PIND & (1<<PORTD6))
#define HW_BUTTON_PRESSED       (HW_BUTTON_STATE == LOW)

#define LED_BLUE_PIN            18
#define HW_LED_BLUE_ON          (PORTC |= (1<<PORTC4))
#define HW_LED_BLUE_OFF         (PORTC &= ~(1<<PORTC4))
#define HW_LED_BLUE_TOGGLE      (PORTC = PORTC ^ (1<<PORTC4))

#define LED_RED_PIN             19
#define HW_LED_RED_ON           (PORTC |= (1<<PORTC5))
#define HW_LED_RED_OFF          (PORTC &= ~(1<<PORTC5))
#define HW_LED_RED_TOGGLE       (PORTC = PORTC ^ (1<<PORTC5))

#define GATE_PULLUP             (DDRC &= ~(1<<PORTC2));(PORTC |= (1<<PORTC2))
#define GATE_SENSE              (PINC & (1<<PORTC2))
#define GATE_DRIVE_HIGH         (DDRC |= (1<<PORTC2))
#define GATE_DRIVE_LOW          (PORTC &= ~(1<<PORTC2));(DDRC |= (1<<PORTC2))

#define PITCH_POT               0
#define VOLUME_POT              1
#define REGISTER_SELECT_POT     6
#define WAVE_SELECT_POT         7

#define EEPROM_PITCH_DAC_VOLTAGE_ADDRESS            0
#define EEPROM_PITCH_DAC_CALIBRATION_BASE_ADDRESS   4
#define EEPROM_VOLUME_DAC_VOLTAGE_ADDRESS           2
#define EEPROM_VOLUME_DAC_CALIBRATION_BASE_ADDRESS  8

// HARDWARE ISR FREQUENCY DEBUG MACRO
// Pulse the red LED around the DDS ISR body. This excludes compiler
// prologue/epilogue and time waiting for another ISR. Measure SAMPLE_CLK
// and DAC_LATCH as well to assess complete sample latency and jitter.
// #define TH_DEBUG  // Added by ThF 20200419

#endif // _HW_H_
