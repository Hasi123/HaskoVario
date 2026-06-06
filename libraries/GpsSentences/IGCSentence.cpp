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

#include <IGCSentence.h>

#include <Arduino.h>
#include <VarioSettings.h>

/**************/
/* IGC header */
/**************/

#define IGC_HEADER_PROGMEM_STRING \
  "AXYYHASKO1\r\n" \
  "HFDTEDATE:000000" \
  "HFPLTPILOTINCHARGE:" VARIOMETER_PILOT_NAME "\r\n" \
  "HFCM2CREW2:\r\n" \
  "HFGTYGLIDERTYPE:" VARIOMETER_GLIDER_NAME "\r\n" \
  "HFGIDGLIDERID:" VARIOMETER_GLIDER_ID "\r\n" \
  "HFDTMGPSDATUM:WGS84\r\n" \
  "HFALG:GEO\r\n" \
  "HFALP:ISA\r\n" \
  "HFRFWFIRMWAREVERSION:" VARIOMETER_FIRMWARE_VERSION "\r\n" \
  "HFRHWHARDWAREVERSION:" VARIOMETER_HARDWARE_VERSION "\r\n" \
  "HFFTYFRTYPE:XYY,HaskoVarioGPS\r\n" \
  "HFGPSRECEIVER:Quectel,L80,22,9999\r\n" \
  "HFPRSPRESSALTSENSOR:TE,MS5611,9999\r\n" \
  "HFFRSSECURITYOK\r\n"

const char IGCHeader::headerData[] PROGMEM = IGC_HEADER_PROGMEM_STRING;


int16_t IGCHeader::begin(void) {

  addr = 0;
  size = sizeof(headerData) - 1;  // exclude null terminator

  /* find date position: search for "\r\nHFDTEDATE:" and return position right after */
  for (int16_t i = 0; i < size - 12; i++) {
    if (pgm_read_byte_near(headerData + i) == '\r' &&
        pgm_read_byte_near(headerData + i + 1) == '\n' &&
        pgm_read_byte_near(headerData + i + 2) == 'H' &&
        pgm_read_byte_near(headerData + i + 3) == 'F' &&
        pgm_read_byte_near(headerData + i + 4) == 'D' &&
        pgm_read_byte_near(headerData + i + 5) == 'T' &&
        pgm_read_byte_near(headerData + i + 6) == 'E' &&
        pgm_read_byte_near(headerData + i + 7) == 'D' &&
        pgm_read_byte_near(headerData + i + 8) == 'A' &&
        pgm_read_byte_near(headerData + i + 9) == 'T' &&
        pgm_read_byte_near(headerData + i + 10) == 'E' &&
        pgm_read_byte_near(headerData + i + 11) == ':') {
      return i + 12;
    }
  }

  return -1;  // should never happen
}


bool IGCHeader::available(void) {

  return (bool)size;
}


uint8_t IGCHeader::get(void) {

  uint8_t c = pgm_read_byte_near(headerData + addr);
  addr++;
  size--;

  return c;
}


bool IGCHeader::saveParams(const char* model, const char* pilot, const char* glider) {

  return true;
}


/********************/
/* IGC sentence "B" */
/********************/
uint8_t IGCSentence::begin(double baroAlti) {

  /* clear variables */
  outc = 0;
  commaCount = 1;  //TAG not feeded
  digitCount = IGC_SENTENCE_TIME_SIZE; //we start immediately with time

  /* no negative alti */
  uint16_t intAlti;
  if( baroAlti >= 0.0 ) {
    intAlti = (uint16_t)baroAlti;
  } else {
    intAlti = 0;
  }

  /* fill in reverse order */
  for( int i = 0; i<IGC_SENTENCE_ALTI_SIZE; i++ ) {
    digitBuffer[i] = '0' + (intAlti % 10);
    intAlti /= 10;
  }

  return 'B';
}

void IGCSentence::feed(uint8_t c) {

  outc = 0;
  
  /* when receive commma, reinit vars */
  if( c == ',' ) {
    commaCount++;
        
    if( commaCount == 2 ) {
      digitCount = IGC_SENTENCE_LAT_SIZE; //latitude
    }

    else if( commaCount == 3 || commaCount == 5 ) {
      digitCount = IGC_SENTENCE_CARDINAL_SIZE; //N,S,E,O
    }

    else if( commaCount == 4 ) {
      digitCount = IGC_SENTENCE_LONG_SIZE; //longitude
    }

    else if( commaCount == 7 ) {
      outc = 'A'; //we have alti
    }

    else if( commaCount == 8 || commaCount == 10 ) {
      outc = digitBuffer[IGC_SENTENCE_ALTI_SIZE - 1]; //first alti digit
      digitCount = IGC_SENTENCE_ALTI_SIZE - 2;  //next digits (used as pointers)
    }

    else if( commaCount == 9 ) {
      digitCount = IGC_SENTENCE_ALTI_SIZE - 1; //we read gps alti in reverse order
    }

    else if( commaCount == 11 ) {
      outc = '\r';
    }

    return;
  }

  /*********************/
  /* there is two case */
  /*********************/
  
  /* -> we are reading GPS alti */
  if( commaCount == 9 ) {
    if( digitCount < IGC_SENTENCE_ALTI_SIZE ) {
      /* if dot was not found get the digit */
      if( c != '.' ) {
	digitBuffer[digitCount] = c;
	digitCount--;
      }

      /* build alti digits */
      else {
	uint8_t i = 0;
	
	/* translate digits */
	digitCount++;
	while( digitCount < IGC_SENTENCE_ALTI_SIZE ) {
	  digitBuffer[i] = digitBuffer[digitCount];
	  i++;
	  digitCount++;
	}

	/* fill with 0 */
	while( i < IGC_SENTENCE_ALTI_SIZE ) {
	  digitBuffer[i] = '0';
	  i++;
	}
      }
    }
  }
  
  /* -> we are outputting digits */
  else {
    if( commaCount < 8 && c != '.' && digitCount ) {
	digitCount--;
	outc = c;
    }
  }
}
 
bool IGCSentence::available() {

  return (bool)outc;
}

uint8_t IGCSentence::get() {

  /* get last char */
  uint8_t retChar = outc;
  outc = 0;

  /* check if we need to send next char immediately  */
  if( digitCount >= 0 ) {
    if( commaCount == 8 || commaCount == 10 ) {  //we outputing alti 
      outc = digitBuffer[digitCount];
      digitCount--;
    }
  }

  else {
    digitCount = 0; //don't stay with digitCount = -1 
  }

  /* new line */
  if( retChar == '\r' ) {
    outc = '\n';
  }

  /* send */
  return retChar;
}

    

  
