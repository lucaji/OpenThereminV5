// OpenThereminV5 firmware development (2026): Luca Cipressi (milletgrain).

#include <Arduino.h>

#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

extern int32_t pitchCalibrationBase;
extern int32_t volCalibrationBase;

// Load the newest CRC-checked V5 slot, otherwise a plausible legacy V4 record,
// or initialize safe RAM defaults. Boot never writes/migrates EEPROM; the next
// successful calibration saves a V5 slot while preserving legacy bytes.
bool calibration_read();
bool calibration_start();

#endif // _CALIBRATION_H_
