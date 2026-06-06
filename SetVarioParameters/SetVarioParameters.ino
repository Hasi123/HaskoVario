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
 *
 * NOTE: The IGC header (pilot name, glider type, etc.) is now stored in
 * PROGMEM directly via the compile-time string in IGCSentence.cpp.
 * The values are configured in VarioSettings.h. This sketch is kept
 * only for legacy EEPROM parameter setup (no longer required).
 */

#include <Arduino.h>
#include <VarioSettings.h>
#include <varioPower.h>

VarioPower varioPower;

void setup() {
  varioPower.init();

  delay(VARIOMETER_POWER_ON_DELAY);
  varioPower.updateFW();

  /* IGC header is now in PROGMEM — no EEPROM save needed */

  delay(500);
  varioPower.sleep();
}

void loop() {
}
