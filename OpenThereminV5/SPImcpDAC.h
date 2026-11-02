// OpenThereminV5 firmware development (2026): Luca Cipressi (milletgrain).

/* Control the mcp 4921/4922 DACs with hardware SPI of the Arduino UNO
 * ...without all the overhead of the Arduino SPI lib...
 * Just the needed functions in a runtime optimized way by "Theremingenieur" Thierry Frenkel
 * This file is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <Arduino.h>


#ifndef _DAC_H_
#define _DAC_H_


// Data direction & Port register & Bit number for DAC Latch:
#define MCP_DAC_LDAC_DDR 	DDRD
#define MCP_DAC_LDAC_PORT 	PORTD
#define MCP_DAC_LDAC_BIT 	7
// Data direction & Port register & Bit number for DAC CS
#define MCP_DAC_CS_DDR 		DDRB
#define MCP_DAC_CS_PORT 	PORTB
#define MCP_DAC_CS_BIT 		2
// Data direction & Port register & Bit number for DAC2 CS
#define MCP_DAC2_CS_DDR 	DDRB
#define MCP_DAC2_CS_PORT 	PORTB
#define MCP_DAC2_CS_BIT 	1
// Data direction & Port register & Bit number for DAC3 CS
#define MCP_DAC3_CS_DDR 	DDRC
#define MCP_DAC3_CS_PORT 	PORTC
#define MCP_DAC3_CS_BIT 	3
// Data direction & Port registers & Bit numbers for Hardware SPI
#define HW_SPI_DDR 			DDRB
#define HW_SPI_SCK_BIT 		5
#define HW_SPI_MISO_BIT 	4 // unused in this configuration
#define HW_SPI_MOSI_BIT 	3

static inline void SPImcpDACinit() {
	// initialize the latch pin:
	MCP_DAC_LDAC_DDR |= _BV(MCP_DAC_LDAC_BIT);
	MCP_DAC_LDAC_PORT |= _BV(MCP_DAC_LDAC_BIT);
	// initialize the CS pins:
	MCP_DAC_CS_DDR   |= _BV(MCP_DAC_CS_BIT);
	MCP_DAC_CS_PORT  |= _BV(MCP_DAC_CS_BIT);
	MCP_DAC2_CS_DDR  |= _BV(MCP_DAC2_CS_BIT);
	MCP_DAC2_CS_PORT |= _BV(MCP_DAC2_CS_BIT);
  	MCP_DAC3_CS_DDR  |= _BV(MCP_DAC3_CS_BIT);
  	MCP_DAC3_CS_PORT |= _BV(MCP_DAC3_CS_BIT);
	// initialize the hardware SPI pins:
	HW_SPI_DDR |= _BV(HW_SPI_SCK_BIT);
	HW_SPI_DDR |= _BV(HW_SPI_MOSI_BIT);
	// initialize the hardware SPI registers
	SPCR = _BV(SPE) | _BV(MSTR);	// SPI master, mode 0, MSB first, no SPI interrupt.
	SPSR = _BV(SPI2X);  			// f_osc/2 = 8 MHz: 2 us wire time per 16-bit frame, plus software overhead.
}

static inline __attribute__((always_inline)) void SPImcpDACtransmit(uint16_t data) {
	// High byte
    SPDR = (uint8_t)(data >> 8);
    while (!(SPSR & _BV(SPIF))) {}
    (void)SPDR; // clear SPIF properly
    // Low byte
    SPDR = (uint8_t)(data & 0xFF);
    while (!(SPSR & _BV(SPIF))) {}
    (void)SPDR; // clear SPIF properly
}

static inline __attribute__((always_inline)) void SPImcpDAClatch() {
	MCP_DAC_LDAC_PORT &= ~_BV(MCP_DAC_LDAC_BIT);
	MCP_DAC_LDAC_PORT |= _BV(MCP_DAC_LDAC_BIT);
}

static inline __attribute__((always_inline)) void SPImcpDACsend(uint16_t data12) {
    // data12 is assumed 0..4095
    uint16_t frame = (data12 & 0x0FFF) | 0x7000; // BUF=1, GA=1x, SHDN=1, A/B depends on part
    MCP_DAC_CS_PORT &= ~_BV(MCP_DAC_CS_BIT);
    SPImcpDACtransmit(frame);
    MCP_DAC_CS_PORT |=  _BV(MCP_DAC_CS_BIT);
    // The next DDS frame's leading LDAC pulse applies this sample.
}

// Calibration sends apply immediately. Call these only while DDS is disabled,
// because all three DAC devices share the same LDAC signal.
static inline __attribute__((always_inline)) void SPImcpDAC2Asend(uint16_t data) {
	MCP_DAC2_CS_PORT &= ~_BV(MCP_DAC2_CS_BIT);
	// Sanitize input data and add DAC config MSBs
	data &= 0x0FFF;
	data |= 0x7000;
	SPImcpDACtransmit(data);
	MCP_DAC2_CS_PORT |= _BV(MCP_DAC2_CS_BIT);
	SPImcpDAClatch();
}

static inline __attribute__((always_inline)) void SPImcpDAC2Bsend(uint16_t data) {
	MCP_DAC2_CS_PORT &= ~_BV(MCP_DAC2_CS_BIT);
	// Sanitize input data and add DAC config MSBs
	data &= 0x0FFF;
	data |= 0xF000;
	SPImcpDACtransmit(data);
	MCP_DAC2_CS_PORT |= _BV(MCP_DAC2_CS_BIT);
	SPImcpDAClatch();
}

// Runtime CV sends queue only. DDS latches CV and audio together once per frame.
static inline __attribute__((always_inline)) void SPImcpDAC3Asend(uint16_t data) {
  MCP_DAC3_CS_PORT &= ~_BV(MCP_DAC3_CS_BIT);
  // Sanitize input data and add DAC config MSBs
  data &= 0x0FFF;
  data |= 0x7000;
  SPImcpDACtransmit(data);
  MCP_DAC3_CS_PORT |= _BV(MCP_DAC3_CS_BIT);
}

static inline __attribute__((always_inline)) void SPImcpDAC3Bsend(uint16_t data) {
  MCP_DAC3_CS_PORT &= ~_BV(MCP_DAC3_CS_BIT);
  // Sanitize input data and add DAC config MSBs
  data &= 0x0FFF;
  data |= 0xF000;
  SPImcpDACtransmit(data);
  MCP_DAC3_CS_PORT |= _BV(MCP_DAC3_CS_BIT);
}

#endif
