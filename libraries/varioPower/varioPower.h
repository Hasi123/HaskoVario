/* varioPower -- Turns the vario off by disabling LDO and putting the MCU to sleep 
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

#ifndef varioPower_h
#define varioPower_h

#include <Arduino.h>

#define INTPIN 3

class VarioPower
{
  public:
    void sleep();
    void init();
    void update();
};

extern VarioPower varioPower;

#endif