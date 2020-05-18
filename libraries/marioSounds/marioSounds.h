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

#ifndef marioSounds_h
#define marioSounds_h

#include <Arduino.h>

class MarioSounds
{
  public:
    void bootUp();
	void shutDown();
	void lowVoltage();
	void debugBeep();
};

extern MarioSounds marioSounds;

#endif
