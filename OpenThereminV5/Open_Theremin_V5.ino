/*
 *  Open.Theremin control software for Arduino UNO
 *  Version 5.0 development — for Open.Theremin V4.x hardware
 *  (c) 2026 Luca Cipressi - milletgrain
 *
 *  Open.Theremin control software is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Open.Theremin control software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with
 *  the Open.Theremin control software. If not, see <http://www.gnu.org/licenses/>.
 *
 *  Original V4 firmware
 *  Copyright (C) 2010-2025 by Urs Gaudenz
 *
 *  With important contributions by:
 *  David Harvey
 *  Michael Margolis
 *  "Theremingenieur" Thierry Frenkel
 *  Luca Cipressi (milletgrain)
 */
 
/**
Building the code
=================
build.h contains #defines that control the compilation of the code
please refer to the comments in the actual `build.h` for detailed description.

           
Structure of the code
=====================
** Open_Theremin_V5.ino **
This file. Calls theremin_setup() and theremin_loop() from the standard
Arduino setup() and loop() entry points.

** application.h/application.cpp **
Main application code. Holds the state of the app (playing, calibrating), deals
with initialisation and the app main loop, consumes atomic pitch/volume
measurements and publishes DDS controls. Potentiometer reads are staggered.
The external sample-clock interrupt sends audio and CV data to the DACs.

** calibration.h/calibration.cpp **
Self-calibration routines

** SPImcpDAC.h **
Internal library for communicating with the MCP492x DAC chips (audio, calibration and CV outputs).

** build.h **
Preprocessor definitions for customizable build options (see above).

** hw.h **
Definitions for hardware button and LED.

** ihandlers.h/ihandlers.cpp **
Interrupt handlers, private shared state, and atomic control/measurement APIs
implementing the DDS (Direct Digital Synthesis) wave generator.

** wavetables_generated.h **
Eight original voices, pitch-dependent harmonic limits, 256-point interpolated
cycles. Reproduce with Software/tools/generate_wavetables.py. Timbre selects a voice.

** timer.h/timer.cpp **
Monotonic sample ticks, atomic clock snapshots, and independent elapsed-time waits.

** EEPROM.h **
Interface library for persistent storage of validated oscillator calibration.

*/

#include "application.h"


void setup() {
  theremin_setup();
}

void loop() {
  theremin_loop();
}
