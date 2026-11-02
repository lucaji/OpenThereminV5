# OpenThereminV5 Firmware

Open Source Theremin Instrument

## V5 firmware development

`Open_Theremin_V5/Open_Theremin_V5.ino` is the new V5 development
sketch for the existing V4.x hardware. V5 adds eight generated voices, pitch-dependent harmonic limits,
interpolated playback, 24-bit phase precision, stable calibration, and smoother
foreground controls. CV/GATE support is explicitly disabled pending a dedicated
contract; enabling it is rejected at build time.

## Credits and contributions

- Urs Gaudenz — original Open.Theremin project and firmware.
- David Harvey, Michael Margolis, and "Theremingenieur" Thierry Frenkel —
  contributions to the original firmware.
- Luca Cipressi (milletgrain) — V5 firmware development (2026), including the
  new voice bank, playback improvements, calibration and control refinements,
  and verification tooling.

Schematics, printed circuit board (PCB) design, bill of materials (BOM) and Arduino UNO compatible software for the OpenTheremin V4.

## Open Source Theremin based on the Arduino Platform

OpenTheremin is an arduino based real Theremin. The legendary music instrument was invented by Leon Theremin back in 1920. The theremin is played with two antennas, one to control the pitch and one for volume. The electronic shield with two ports to connect those antennas comprises two heterodyne oscillators to measure the distance of the hand to the antenna when playing the instrument. The resulting signal is fed into a AtMega328P microcontroller. After linearization and filtering the circuit generates the instruments sound that is then played through a high quality digital analog audio converter on the board. The characteristics of the sound can be determined by a wave table on the arduino.

For more info on the open source project and on availability of ready made shield see:

http://www.gaudi.ch/OpenTheremin/

## Installation

1. Open up the Arduino IDE
2. Open the File "Open_Theremin_V5.ino"
3. Selecting the correct usb port on Tools -> Serial Port
4. Select the correct arduino board from Tools -> Board
5. Upload the code by clicking on the upload button.

## V5 controls, capture and calibration

V5 preserves the eight-voice bank and three audio registers. Selectors use fixed
boundaries with small hysteresis; pitch/volume field-pot offsets retain fractional
values and move smoothly. Antenna filters account for elapsed time. A short
release unmutes a muted instrument; a calibration hold never briefly unmutes it.
ADC hardware settings remain unchanged pending physical measurements.

Both antenna paths reject stale references. Pitch recovery requires a fresh
confirmed pair and reseeds its filter. Audio owns the normal shared DAC latch;
clock failure during modal feedback uses a separate, guarded physical mute.
Timer0/UART interrupts remain disabled during playback, and serial transmission
uses a bounded foreground queue. The legacy MIDI-named mode remains raw two-byte
pitch telemetry, not MIDI note messages.

Calibration confirms candidate DAC settings repeatedly and validates both
antennas across two measurement windows. Failure restores previous settings.
Two versioned EEPROM slots use checksums, commit-last writes and readback; V4's
legacy bytes remain readable and unchanged. The control and stability limits are
provisional settings for a repeatable hardware test rig.

