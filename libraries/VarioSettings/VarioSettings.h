/* VarioSettings -- Main configuration file
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

#ifndef VARIO_SETTINGS_H
#define VARIO_SETTINGS_H

/*----------------------------*/
/*          SOFTWARE          */
/*      Vario parameters      */
/*                            */
/*----------------------------*/

/* Set your personnal info here.                         */
/* Compilation is sufficient after changing any of these. */

#define VARIOMETER_MODEL "HaskoVarioGPS"
#define VARIOMETER_PILOT_NAME "David Hasko"
#define VARIOMETER_GLIDER_NAME "OZONE Zeolite 2"
#define VARIOMETER_GLIDER_ID "D-HASK"
#define VARIOMETER_FIRMWARE_VERSION "1.3"
#define VARIOMETER_HARDWARE_VERSION "0.2"


/* time zone relative to UTC */
#define VARIOMETER_TIME_ZONE (+2) 

/*********/
/* Beeps */
/*********/

/* The volume of the beeps, max = 10 */
#define VARIOMETER_BEEP_VOLUME 10

/* The variometer react like this according to vertical speed in m/s :        */
/* (near climbing beep is not enabled by default)                             */
/*                                                                            */
/* <--LOW-BEEP--|------SILENT------|--NEAR-CLIMBING-BEEP--|--CLIMBING-BEEP--> */
/*              |                  |                      |                   */
/*           SINKING         CLIMBING-SENSITIVITY      CLIMBING               */
#define VARIOMETER_SINKING_THRESHOLD -2.5
#define VARIOMETER_CLIMBING_THRESHOLD 0.2
#define VARIOMETER_NEAR_CLIMBING_SENSITIVITY 0.5

/* The near climbing alarm : signal that you enter or exit the near climbing zone */
/* The near climbing beep : beep when you are in near climbing zone               */
//#define VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
//#define VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP


/********************/
/* Measure behavior */
/********************/

/* Flight start detection conditions :                      */
/* -> Minimum time after poweron in milliseconds            */
/* -> Minimum vertical velocity in m/s (low/high threshold) */
/* -> Minimum ground speed in km/h                          */
#define FLIGHT_START_MIN_TIMESTAMP 15000
#define FLIGHT_START_VARIO_LOW_THRESHOLD (-0.5)
#define FLIGHT_START_VARIO_HIGH_THRESHOLD 0.5
#define FLIGHT_START_MIN_SPEED 10.0

/* Set the GPS precision needed to use the GPS altitude value  */
/* to calibrate the barometric altitude.                       */
/*      !!! the best possible precision is 100 !!!             */ 
#define VARIOMETER_GPS_ALTI_CALIBRATION_PRECISION_THRESHOLD 350


/*****************************/
/* SDCard/Bluetooth behavior */
/*****************************/

/* What type of barometric altitude is sent :           */
/* -> Based on international standard atmosphere        */
/* -> Calibrated with GPS altitude                      */
//#define VARIOMETER_SDCARD_SEND_CALIBRATED_ALTITUDE
//#define VARIOMETER_BLUETOOTH_SEND_CALIBRATED_ALTITUDE

/* GPS track recording on SD card starting condition :  */ 
/* -> As soon as possible (GPS fix)                     */
/* -> When flight start is detected                     */
//#define VARIOMETER_RECORD_WHEN_FLIGHT_START

/* What type of vario NMEA sentence is sent by bluetooth. */
/* Possible values are :                                  */
/*  - VARIOMETER_SENT_LXNAV_SENTENCE                      */
/*  - VARIOMETER_SENT_LK8000_SENTENCE                     */
#define VARIOMETER_SENT_LXNAV_SENTENCE

/* When there is no GPS to sync variometer bluetooth sentences */
/* set the delay between sendings in milliseconds.             */ 
#define VARIOMETER_SENTENCE_DELAY 550


/*----------------------------*/
/*          HARDWARE          */
/*      Vario parameters      */
/*                            */
/*----------------------------*/

/* Comment or uncomment according to  */
/* what you embed in the variometer   */ 
#define HAVE_SPEAKER
#define HAVE_ACCELEROMETER
#define HAVE_GPS
#define HAVE_SDCARD
#define HAVE_BLUETOOTH
//#define HAVE_VOLTAGE_DIVISOR

/* IGC file security (G record with HMAC-SHA256)                    */
/* When enabled, the IGC file gets a G record with the HMAC-SHA256  */
/* signature over all protected records.                             */
#define HAVE_IGC_SECURITY

/* HMAC-SHA256 key for IGC G record (32 bytes = 256 bits)           */
/* Shared per model. Change to invalidate all existing flight logs.  */
#define VARIOMETER_HMAC_KEY { 0x48, 0x61, 0x73, 0x6B, 0x6F, 0x56, 0x61, 0x72, \
                              0x69, 0x6F, 0x48, 0x4D, 0x41, 0x43, 0x4B, 0x65, \
                              0x79, 0x32, 0x35, 0x36, 0x42, 0x69, 0x74, 0x73, \
                              0x53, 0x68, 0x61, 0x72, 0x65, 0x64, 0x21, 0x21 }

/* Set the pin used for SD card module */
#define SDCARD_CS_PIN 8

/* time needed to power on all the devices */
#define VARIOMETER_POWER_ON_DELAY 2000

/* The bauds rate used by the GPS and Bluetooth modules. */
/* GPS and bluetooth need to have the same bauds rate.   */
#define GPS_BLUETOOTH_BAUDS 9600

/* The GPS period in ms                             */
/* use the gps-time-analysis sketch to determine it */
#define GPS_PERIOD 1000

/* The GPS RMC and GGA sentences parameters */
/* Check the SD card ouput or bluetooth output of gps-time-analysis */
#define NMEA_TAG_SIZE 5
#define NMEA_RMC_TAG "GPRMC"
#define NMEA_GGA_TAG "GPGGA"

/* precision = 10^(number of digit after dot in gps ouput) */
/* check the gps-time-analysis output to check the precision */
/* RMC speed is at 8th position */
/* GGA time is at 2th position */
/* GGA Alti is at 10th position */
/* be carefull, time precision is an int */
#define NMEA_RMC_SPEED_PRECISION 1000.0
#define NMEA_GGA_TIME_PRECISION 100
#define NMEA_GGA_ALTI_PRECISION 10.0

/*********************/
/* TWO WIRE settings */
/*********************/

/* Set the freq */
#define VARIO_TW_FREQ 400000UL

#endif
