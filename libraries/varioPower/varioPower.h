/* varioPower -- Turns the vario off by disabling LDO and putting the MCU to sleep

   Copyright 2016-2020 David HASKO

   This file is part of GNUVario.

   GNUVario is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   GNUVario is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef varioPower_h
#define varioPower_h

#include <Arduino.h>

#define INTPIN 3

#if INTPIN == 2
#elif INTPIN == 3
#else
#error Either choose pin 2 or 3 for interrupt
#endif

class VarioPower
{
  public:
    void sleep(void);
    void init(void);
	void updateFW(void);
    void update(void);
  private:
	uint8_t beepStatus;
    uint32_t nextEvent;
};

#endif
