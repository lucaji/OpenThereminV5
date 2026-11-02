// OpenThereminV5 firmware development (2026): Luca Cipressi (milletgrain).

/**
 * @brief BUILD OPTIONS for OpenTheremin V5
 * 
 * This file contains macros that will activate customizable
 * options for the CV/GATE support and debug messages on the final build (firmware).
 * 
 * Disabling unnecessary options will slim down the firmware size and ease the CPU load.
 * 
 * Please refer to each option's description.
 */
#ifndef _BUILD_H
#define _BUILD_H

/*
 * AUDIO_FEEDBACK_MODE
 * 
 * - AUDIO_FEEDBACK_ON
 *   enables short audio feedbacks (beeps) during calibration
 *   along the visual indication of the current mode.
 * 
 * - AUDIO_FEEDBACK_OFF
 *   disables the audio feedback (silent), keeping the visual
 *   indication of the current mode.
 * 
 */
#define AUDIO_FEEDBACK_ON   0
#define AUDIO_FEEDBACK_OFF  1
#ifndef AUDIO_FEEDBACK_MODE
#define AUDIO_FEEDBACK_MODE AUDIO_FEEDBACK_ON
#endif

/*
 * CV support is deliberately unavailable in this V5 milestone. The CV/GATE
 * scheduling and voltage contract still require their own review and hardware tests.
 */
#define CV_OUTPUT_MODE_OFF      0
#define CV_OUTPUT_MODE_LOG      1
#define CV_OUTPUT_MODE_LINEAR   2
#ifndef CV_SUPPORT_ENABLED
#define CV_SUPPORT_ENABLED      0
#endif
#ifndef CV_OUTPUT_MODE
#define CV_OUTPUT_MODE          CV_OUTPUT_MODE_OFF
#endif
#if CV_SUPPORT_ENABLED != 0 || CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
#error "V5 CV/GATE support is deferred: CV_SUPPORT_ENABLED and CV_OUTPUT_MODE must be 0"
#endif

// Set the trigger levels for the Gate signal
// Making both values equal risk the gate signal to bounce, leave at least 4 (hysteresis) between both.
// Set both to 128 to disable the Gate Signal.
#define GATE_ON_THRESHOLD_LEVEL     24 // That's the level which will drive the Gate high when volume increases from lower
#define GATE_OFF_THRESHOLD_LEVEL    16 // That's the level which will drive the Gate low when volume decreases from higher


/**
 * SERIAL_PORT_MODE
 * 
 * This macro selects the serial port (UART) mode
 * for external communication (legacy pitch telemetry)
 * and/or to emit debug messages
 * 
 * - SERIAL_PORT_MODE_OFF
 *   disables completely the use of the serial port
 *   reduces code size and CPU overhead
 * 
 * - SERIAL_PORT_MODE_DEBUG
 *   will emit debug information that can be read on
 *   the Serial Monitor (or any serial terminal software)
 *   The serial speed for this mode is set by the
 *   SERIAL_SPEED_DEBUG macro defaulted at 57600 bps
 *   V5 transmits from a bounded foreground queue. No UART ISR runs.
 *   Saturation drops new debug bytes; RX is unused and disabled.
 * 
 * - SERIAL_PORT_MODE_MIDI
 *   retains the legacy mode name and sends the raw pitch period as two
 *   bytes (low byte first) every 100ms at 31250bps. These are not MIDI
 *   note-on/note-off messages; a receiver must understand this protocol.
 *   For an actual MIDI interface, refer to the MIDI expansion:
 *   https://www.gaudi.ch/OpenTheremin/index.php/opentheremin-v4/midi-interface
 * 
 * NOTE: during debug procedures it is possible to enable
 * both telemetry + DEBUG messages (at fixed speed of 31250 bps).
 * The resulting mixed text/binary stream is only for diagnostics.
 * with a bit-wise OR mask on both constants as follows:
 * #define SERIAL_PORT_MODE (SERIAL_PORT_MODE_DEBUG | SERIAL_PORT_MODE_MIDI)
 * 
 * TO DISABLE THE SERIAL PORT COMPLETELY, JUST SET
 * #define SERIAL_PORT_MODE SERIAL_PORT_MODE_OFF
 */
#define SERIAL_PORT_MODE_OFF        0x00
#define SERIAL_PORT_MODE_DEBUG      0x01
#define SERIAL_PORT_MODE_MIDI       0x02
#ifndef SERIAL_PORT_MODE
#define SERIAL_PORT_MODE            SERIAL_PORT_MODE_OFF
#endif

#define SERIAL_SPEED_DEBUG          57600 // serial com speed
#define SERIAL_SPEED_MIDI           31250 // standard MIDI speed


/**
 * Serial debug macros
 * 
 * The following macros compile away if the serial debug mode
 * is being disabled - this will reduce the code size and CPU load automatically.
 */
#if ((SERIAL_PORT_MODE) & SERIAL_PORT_MODE_DEBUG)
    #define DEBUG_PRINT(x)     SerialOutput.print(x)
    #define DEBUG_PRINTLN(x)   SerialOutput.println(x)
#else
    #define DEBUG_PRINT(x)     ((void)0)
    #define DEBUG_PRINTLN(x)   ((void)0)
#endif

#if AUDIO_FEEDBACK_MODE != AUDIO_FEEDBACK_ON && AUDIO_FEEDBACK_MODE != AUDIO_FEEDBACK_OFF
#error "Invalid AUDIO_FEEDBACK_MODE"
#endif
#if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF && CV_OUTPUT_MODE != CV_OUTPUT_MODE_LOG && CV_OUTPUT_MODE != CV_OUTPUT_MODE_LINEAR
#error "Invalid CV_OUTPUT_MODE"
#endif
#if ((SERIAL_PORT_MODE) & ~(SERIAL_PORT_MODE_DEBUG | SERIAL_PORT_MODE_MIDI))
#error "Invalid SERIAL_PORT_MODE"
#endif

#include "serial_output.h"
#endif // _BUILD_H
