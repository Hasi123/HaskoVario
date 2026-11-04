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
  "HFCM2CREW2:NIL\r\n" \
  "HFGTYGLIDERTYPE:" VARIOMETER_GLIDER_NAME "\r\n" \
  "HFGIDGLIDERID:" VARIOMETER_GLIDER_ID "\r\n" \
  "HFDTMGPSDATUM:WGS84\r\n" \
  "HFRFWFIRMWAREVERSION:" VARIOMETER_FIRMWARE_VERSION "\r\n" \
  "HFRHWHARDWAREVERSION:" VARIOMETER_HARDWARE_VERSION "\r\n" \
  "HFFTYFRTYPE:" VARIOMETER_MANUFACTURER "," VARIOMETER_MODEL "\r\n" \
  "HFGPSRECEIVER:Quectel,L80,22,9999\r\n" \
  "HFPRSPRESSALTSENSOR:TE,MS5611,9999\r\n" \
  "HFFRSSECURITYOK\r\n" \
  "HFALGALTGPS:GEO\r\n" \
  "HFALPALTPRESSURE:ISA\r\n"

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

/* GGA fields, counted after the tag */
#define GGA_TIME_FIELD 1
#define GGA_LAT_FIELD 2
#define GGA_NS_FIELD 3
#define GGA_LONG_FIELD 4
#define GGA_EW_FIELD 5
#define GGA_QUALITY_FIELD 6
#define GGA_ALTI_FIELD 9

/* received parts */
#define PART_TIME 0x01
#define PART_LAT 0x02
#define PART_NS 0x04
#define PART_LONG 0x08
#define PART_EW 0x10
#define PART_FIX 0x20
#define PART_ALTI 0x40
#define PART_ALTI_NEG 0x80
#define PARTS_POSITION (PART_TIME | PART_LAT | PART_NS | PART_LONG | PART_EW | PART_FIX)

#define DIGITS_DONE 0xFF  //ignore the remaining digits of the field

/* 5 characters, negative values with sign instead of leading zero */
static void writeAlti(uint8_t* p, long alti) {
  uint8_t n = 5;
  if (alti < 0) {
    *p++ = '-';
    alti = -alti;
    n = 4;
    if (alti > 9999) alti = 9999;
  } else if (alti > 99999) {
    alti = 99999;
  }
  while (n) {
    n--;
    p[n] = '0' + (uint8_t)(alti % 10);
    alti /= 10;
  }
}

void IGCSentence::begin(double baroAlti, uint8_t* record) {
  rec = record;
  rec[0] = 'B';
  writeAlti(&rec[IGC_B_PRESS_POS], (long)(baroAlti + ((baroAlti < 0.0) ? -0.5 : 0.5)));
  gnssAlti = 0;
  field = GGA_TIME_FIELD;  //TAG not feeded
  digits = 0;
  parts = 0;
}

/* store a fixed size digit field, without the dot */
static bool storeDigit(uint8_t* p, uint8_t& digits, uint8_t size, uint8_t c) {
  if (c < '0' || c > '9' || digits >= size) return false;
  p[digits++] = c;
  return digits == size;
}

/* lat/long with less decimals : pad with 0 if at least degrees and minutes */
static bool padDigits(uint8_t* p, uint8_t& digits, uint8_t size) {
  if (digits < size - 3) return false;
  while (digits < size) p[digits++] = '0';
  return true;
}

void IGCSentence::feed(uint8_t c) {
  if (c == ',') {
    if (field == GGA_LAT_FIELD && padDigits(&rec[IGC_B_LAT_POS], digits, 7)) parts |= PART_LAT;
    if (field == GGA_LONG_FIELD && padDigits(&rec[IGC_B_LONG_POS], digits, 8)) parts |= PART_LONG;
    field++;
    digits = 0;
    return;
  }

  switch (field) {
    case GGA_TIME_FIELD:
      if (storeDigit(&rec[IGC_B_TIME_POS], digits, 6, c)) parts |= PART_TIME;
      break;

    case GGA_LAT_FIELD:
      if (storeDigit(&rec[IGC_B_LAT_POS], digits, 7, c)) parts |= PART_LAT;
      break;

    case GGA_NS_FIELD:
      if (c == 'N' || c == 'S') {
        rec[IGC_B_NS_POS] = c;
        parts |= PART_NS;
      }
      break;

    case GGA_LONG_FIELD:
      if (storeDigit(&rec[IGC_B_LONG_POS], digits, 8, c)) parts |= PART_LONG;
      break;

    case GGA_EW_FIELD:
      if (c == 'E' || c == 'W') {
        rec[IGC_B_EW_POS] = c;
        parts |= PART_EW;
      }
      break;

    case GGA_QUALITY_FIELD:
      /* 1-5 are real fixes, 0 is none and 6 is dead reckoning */
      if (c >= '1' && c <= '5') parts |= PART_FIX;
      break;

    case GGA_ALTI_FIELD:
      /* meters, decimals dropped */
      if (c == '-') {
        parts |= PART_ALTI_NEG;
      } else if (c == '.') {
        digits = DIGITS_DONE;
      } else if (c >= '0' && c <= '9' && digits != DIGITS_DONE) {
        gnssAlti = gnssAlti * 10 + (c - '0');
        parts |= PART_ALTI;
      }
      break;
  }
}

uint8_t IGCSentence::end(void) {
  /* no record without a valid position */
  if ((parts & PARTS_POSITION) != PARTS_POSITION) {
    return 0;
  }

  /* no GNSS altitude : 2D fix, V and 00000 */
  if (parts & PART_ALTI) {
    rec[IGC_B_VALIDITY_POS] = 'A';
    writeAlti(&rec[IGC_B_GNSS_POS], (parts & PART_ALTI_NEG) ? -gnssAlti : gnssAlti);
  } else {
    rec[IGC_B_VALIDITY_POS] = 'V';
    writeAlti(&rec[IGC_B_GNSS_POS], 0);
  }

  rec[IGC_B_SIZE - 2] = '\r';
  rec[IGC_B_SIZE - 1] = '\n';
  return IGC_B_SIZE;
}
