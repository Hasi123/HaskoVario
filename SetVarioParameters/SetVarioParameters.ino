/* SetVarioParameters -- Record settings in EEPROM
 *
 * Copyright 2016-2019 Baptiste PELLEGRIN
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
#include <VarioSettings.h>
#include <beeper.h>
#include <toneAC.h>
#include <avr/pgmspace.h>
#include <digit.h>
#include <IGCSentence.h>
#include <marioSounds.h>
#include <varioPower.h>

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!            !!! WARNING  !!!              !!*/
/*!! Before building check :                  !!*/
/*!! libraries/VarioSettings/VarioSettings.h  !!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
VarioPower varioPower;

#define BEEP_FREQ 800
const char model[] PROGMEM = VARIOMETER_MODEL;
const char pilot[] PROGMEM = VARIOMETER_PILOT_NAME;
const char glider[] PROGMEM = VARIOMETER_GLIDER_NAME;

IGCHeader header;

void setup() {
  //init varioPower
  varioPower.init();
  
   /* launch firmware update if needed */
  delay(VARIOMETER_POWER_ON_DELAY);
  varioPower.updateFW();
  
  /* save params to EEPROM */
  boolean state = header.saveParams(model, pilot, glider);

  /* if OK signal */
  if( state ) {
    delay(1000);
    
    for( int i = 0; i<3; i++) {
      toneAC(BEEP_FREQ);
      delay(200);
      toneAC(0);
      delay(200);
    }
  }
  varioPower.sleep();
}

void loop() {
  
  
}
