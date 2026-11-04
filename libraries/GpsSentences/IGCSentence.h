/* GPSSentences -- Generate some standard GPS sentences 
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

#ifndef IGC_SENTENCE_H
#define IGC_SENTENCE_H

#include <Arduino.h>

class IGCHeader {
 public :
  bool saveParams(const char* model, const char* pilot, const char* glider);
  int16_t begin(void);
  bool available(void);
  uint8_t get(void);

 private :
  uint16_t size;
  int addr;
  static const char headerData[] PROGMEM;
};

/****************************************************/
/* B record : B HHMMSS DDMMmmmN DDDMMmmmE V PPPPP GGGGG */
/* built from a GGA sentence into the caller buffer */
/****************************************************/
#define IGC_B_TIME_POS 1
#define IGC_B_LAT_POS 7
#define IGC_B_NS_POS 14
#define IGC_B_LONG_POS 15
#define IGC_B_EW_POS 23
#define IGC_B_VALIDITY_POS 24
#define IGC_B_PRESS_POS 25
#define IGC_B_GNSS_POS 30
#define IGC_B_SIZE 37  //with CRLF

class IGCSentence {
 public:
  void begin(double baroAlti, uint8_t* record);  //record : IGC_B_SIZE bytes
  void feed(uint8_t c);  //GGA characters after the tag, up to '*'
  uint8_t end(void);     //record length, 0 if there is no valid position

 private:
  uint8_t* rec;
  long gnssAlti;
  uint8_t field;
  uint8_t digits;
  uint8_t parts;
};

#endif
