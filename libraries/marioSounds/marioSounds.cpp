/* marioSounds -- Play sounds on boot, shutdown and low voltage in the future
 *
 * Copyright 2016-2020 David HASKO
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "marioSounds.h"
#include <toneAC.h>
#include <VarioSettings.h>

void MarioSounds::bootUp(){
  toneAC(660, 10, 100, true);
  delay(150);
  toneAC(660, 10, 100, true);
  delay(300);
  toneAC(660, 10, 100, true);
  delay(300);
  toneAC(510, 10, 100, true);
  delay(100);
  toneAC(660, 10, 100, true);
  delay(300);
  toneAC(770, 10, 100, true);
  delay(550);
  toneAC(380, 10, 100, true);
  delay(100);
  
  if(VARIOMETER_POWER_ON_DELAY>1800) delay(VARIOMETER_POWER_ON_DELAY-1800);
}

void MarioSounds::shutDown(){
  for(uint8_t i=0; i<3; i++){
	  toneAC(400, 10, 100, true);
	  delay(200);
  }
}

void MarioSounds::lowVoltage(){
  toneAC(400, 10, 100, true);
}

void MarioSounds::debugBeep() {
  toneAC(500);
  delay(200);
  noToneAC();
  delay(200);
}
