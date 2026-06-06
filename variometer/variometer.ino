/* variometer -- The GNUVario embedded code

   Copyright 2016-2019 Baptiste PELLEGRIN
   Modified 2021 by David Hasko

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

#include <Arduino.h>
#include <SPI.h>
#include <VarioSettings.h>
#include <I2CHelper.h>
#include <MPU6050.h>
#include <ms5611.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <toneAC.h>
#include <avr/pgmspace.h>
#include <LightSdCard.h>
#include <LightFat16.h>
#include <SerialNmea.h>
#include <NmeaParser.h>
#include <LxnavSentence.h>
#include <LK8Sentence.h>
#include <IGCSentence.h>
#include <marioSounds.h>
#include <varioPower.h>
#ifdef HAVE_IGC_SECURITY
#include <hmac_sha256.h>
#include <igc_key.h>
#endif

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!            !!! WARNING  !!!              !!*/
/*!! Before building check :                  !!*/
/*!! libraries/VarioSettings/VarioSettings.h  !!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/*******************/
/* Custom objects  */
/*******************/
VarioPower varioPower;
kalmanvert kalmanvert;

/*******************/
/* General objects */
/*******************/
#define VARIOMETER_STATE_INITIAL 0
#define VARIOMETER_STATE_DATE_RECORDED 1
#define VARIOMETER_STATE_CALIBRATED 2
#define VARIOMETER_STATE_FLIGHT_STARTED 3

#ifdef HAVE_GPS
uint8_t variometerState = VARIOMETER_STATE_INITIAL;
#else
uint8_t variometerState = VARIOMETER_STATE_CALIBRATED;
#endif  //HAVE_GPS





/***************/
/* gps objects */
/***************/
#ifdef HAVE_GPS

NmeaParser nmeaParser;

#ifdef HAVE_BLUETOOTH
boolean lastSentence = false;
#endif  //HAVE_BLUETOOTH

#ifdef HAVE_SDCARD
lightfat16 file;
IGCHeader header;
IGCSentence igc;

#define SDCARD_STATE_INITIAL 0
#define SDCARD_STATE_INITIALIZED 1
#define SDCARD_STATE_READY 2
#define SDCARD_STATE_ERROR -1
int8_t sdcardState = SDCARD_STATE_INITIAL;

#ifdef HAVE_IGC_SECURITY
HMACSha256 igcHmac;
bool igcHmacActive = false;
#endif

#endif  //HAVE_SDCARD

#endif  //HAVE_GPS

/*********************/
/* bluetooth objects */
/*********************/
#ifdef HAVE_BLUETOOTH
#if defined(VARIOMETER_SENT_LXNAV_SENTENCE)
LxnavSentence bluetoothNMEA;
#elif defined(VARIOMETER_SENT_LK8000_SENTENCE)
LK8Sentence bluetoothNMEA;
#else
#error No bluetooth sentence type specified !
#endif

unsigned long lastVarioSentenceTimestamp = 0;
#endif  //HAVE_BLUETOOTH


/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {
  //init varioPower
  varioPower.init();

  /************/
  /* init SPI */
  /************/

  /* set all SPI CS lines before talking to devices */
#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
  file.enableSPI();
#endif  //defined(HAVE_SDCARD) && defined(HAVE_GPS)

  /****************/
  /* init SD Card */
  /****************/
#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
  if (file.init() >= 0) {
    sdcardState = SDCARD_STATE_INITIALIZED;  //useless to set error
  }
#endif  //defined(HAVE_SDCARD) && defined(HAVE_GPS)

  /**************************/
  /* init gps and bluetooth */
  /**************************/
#if defined(HAVE_BLUETOOTH) || defined(HAVE_GPS)
#ifdef HAVE_GPS
  serialNmea.begin(GPS_BLUETOOTH_BAUDS, true);
#else
  serialNmea.begin(GPS_BLUETOOTH_BAUDS, false);
#endif  //HAVE_GPS
#endif  //defined(HAVE_BLUETOOTH) || defined(HAVE_GPS)

  /**************************/
  /* init Two Wires devices */
  /**************************/
  I2C::begin();

  //ms5611
  ms.init();

  //MPU6050
  mpu.init();  // load dmp and setup for normal use
  attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), getSensors, RISING);

  //play sound and check if need to update
  marioSounds.bootUp();
  varioPower.updateFW();
  if (mpu.calibrate())   //run calibration if up side down
    varioPower.reset();  //reset to load calibration data and dmp again

  //init kalman filter
  I2C::newData = 0;
  while (!I2C::newData)
    ;  //wait for fresh data
  ms.update();
  float firstAlti = ms.getAltitude();
  kalmanvert.init(firstAlti,
                  0.0,
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());
}

#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
void createSDCardTrackFile(void);
#ifdef HAVE_IGC_SECURITY
void finalizeIGCFile(void);
#endif

static void writeFeed(uint8_t b) {
  file.write(b);
#ifdef HAVE_IGC_SECURITY
  if (igcHmacActive) igcHmac.write(b);
#endif
}

#endif  //defined(HAVE_SDCARD) && defined(HAVE_GPS)
void enableflightStartComponents(void);

/*----------------*/
/*      LOOP      */
/*----------------*/
void loop() {
  static float alt, vertAccel;

  //new sensor data ready
  switch (I2C::newData) {

    case -1:
      mpu.resetFIFO();
      I2C::newData++;
      break;

    case 1:
      ms.update();
      I2C::newData++;
      break;

    case 2:
      alt = ms.getAltitude();
      I2C::newData++;
      break;

    case 3:
      vertAccel = mpu.getVertaccel();
      I2C::newData++;
      break;

    case 4:
      kalmanvert.update1(vertAccel, millis());
      I2C::newData++;
      break;

    case 5:
      kalmanvert.update2(alt);
      /* set beeper */
#ifdef HAVE_SPEAKER
      beeper.setVelocity(kalmanvert.getVelocity());
#endif  //HAVE_SPEAKER
      I2C::newData = 0;
  }

  /*********************/
  /* update varioPower */
  /*********************/
  varioPower.update();

  /********************/
  /* check for shutdown */
  /********************/
  if (varioPower.isShutdownPending()) {
#if defined(HAVE_SDCARD) && defined(HAVE_GPS) && defined(HAVE_IGC_SECURITY)
    finalizeIGCFile();
#endif
    varioPower.completeShutdown();
  }

  /*****************/
  /* update beeper */
  /*****************/
#ifdef HAVE_SPEAKER
  beeper.update();
#endif  //HAVE_SPEAKER

/********************/
/* update bluetooth */
/********************/
#ifdef HAVE_BLUETOOTH
  /* check the last vario nmea sentence */
  if (millis() - lastVarioSentenceTimestamp > VARIOMETER_SENTENCE_DELAY) {
    lastVarioSentenceTimestamp = millis();
#ifdef VARIOMETER_BLUETOOTH_SEND_CALIBRATED_ALTITUDE
    bluetoothNMEA.begin(kalmanvert.getCalibratedPosition(), kalmanvert.getVelocity());
#else
    bluetoothNMEA.begin(kalmanvert.getPosition(), kalmanvert.getVelocity());
#endif
    serialNmea.lock();  //will be writed at next loop
  }

#ifdef HAVE_GPS
  /* in priority send vario nmea sentence */
  if (bluetoothNMEA.available()) {
    lastVarioSentenceTimestamp = millis();
    while (bluetoothNMEA.available()) {
      serialNmea.write(bluetoothNMEA.get());
    }
    serialNmea.release();
  }
#endif  //!HAVE_GPS
#endif  //HAVE_BLUETOOTH

  /**************/
  /* update GPS */
  /**************/
#ifdef HAVE_GPS
#ifdef HAVE_BLUETOOTH
  /* else try to parse GPS nmea */
  else {
#endif  //HAVE_BLUETOOTH

    /* try to lock sentences */
    if (serialNmea.lockRMC()) {
      nmeaParser.beginRMC();
    } else if (serialNmea.lockGGA()) {
      nmeaParser.beginGGA();
#ifdef HAVE_BLUETOOTH
      lastSentence = true;
#endif  //HAVE_BLUETOOTH
    }

    /* parse if needed */
    if (nmeaParser.isParsing()) {
#ifdef HAVE_SDCARD
      /* buffer B record data, write to SD only if GPS fix is valid */
      uint8_t bRecBuf[48];
      uint8_t bRecLen = 0;
      if (sdcardState == SDCARD_STATE_READY && nmeaParser.isParsingGGA()) {
        bRecBuf[bRecLen++] = igc.begin(kalmanvert.getPosition());
      }
#endif
      while (nmeaParser.isParsing()) {
        uint8_t c = serialNmea.read();

        /* parse sentence */
        nmeaParser.feed(c);

#ifdef HAVE_SDCARD
        /* if GGA, buffer IGC output */
        if (sdcardState == SDCARD_STATE_READY && nmeaParser.isParsingGGA()) {
          igc.feed(c);
          while (igc.available()) {
            uint8_t bc = igc.get();
            if (bRecLen < sizeof(bRecBuf)) {
              bRecBuf[bRecLen++] = bc;
            }
          }
        }
#endif  //HAVE_SDCARD
      }
      serialNmea.release();

#ifdef HAVE_SDCARD
      /* flush B record to SD only if GPS has valid position */
      if (sdcardState == SDCARD_STATE_READY && nmeaParser.satelliteCount > 0) {
        for (uint8_t i = 0; i < bRecLen; i++) {
          writeFeed(bRecBuf[i]);
        }
      }
#endif

#ifdef HAVE_BLUETOOTH
      /* if this is the last GPS sentence */
      /* we can send our sentences */
      if (lastSentence) {
        lastSentence = false;
#ifdef VARIOMETER_BLUETOOTH_SEND_CALIBRATED_ALTITUDE
        bluetoothNMEA.begin(kalmanvert.getCalibratedPosition(), kalmanvert.getVelocity());
#else
        bluetoothNMEA.begin(kalmanvert.getPosition(), kalmanvert.getVelocity());
#endif
        serialNmea.lock();  //will be writed at next loop
      }
#endif  //HAVE_BLUETOOTH
    }


    /***************************/
    /* update variometer state */
    /*    (after parsing)      */
    /***************************/
    if (variometerState < VARIOMETER_STATE_FLIGHT_STARTED) {

      /* if initial state check if date is recorded  */
      if (variometerState == VARIOMETER_STATE_INITIAL) {
        if (nmeaParser.haveDate()) {
          variometerState = VARIOMETER_STATE_DATE_RECORDED;
        }
      }

      /* check if we need to calibrate the altimeter */
      else if (variometerState == VARIOMETER_STATE_DATE_RECORDED) {

        /* we need a good quality value */
        if (nmeaParser.haveNewAltiValue() && nmeaParser.precision < VARIOMETER_GPS_ALTI_CALIBRATION_PRECISION_THRESHOLD) {

          /* calibrate */
          double gpsAlti = nmeaParser.getAlti();
          kalmanvert.calibratePosition(gpsAlti);

          variometerState = VARIOMETER_STATE_CALIBRATED;
#if defined(HAVE_SDCARD) && !defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
          createSDCardTrackFile();
#endif  //defined(HAVE_SDCARD) && ! defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
        }
      }

      /* else check if the flight have started */
      else {  //variometerState == VARIOMETER_STATE_CALIBRATED

        /* check flight start condition */
        if ((millis() > FLIGHT_START_MIN_TIMESTAMP) && (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD) && (nmeaParser.getSpeed() > FLIGHT_START_MIN_SPEED)) {
          variometerState = VARIOMETER_STATE_FLIGHT_STARTED;
          enableflightStartComponents();
        }
      }
    }
#ifdef HAVE_BLUETOOTH
  }
#endif  //HAVE_BLUETOOTH
#endif  //HAVE_GPS

  /* if no GPS, we can't calibrate, and we have juste to check flight start */
#ifndef HAVE_GPS
  if (variometerState == VARIOMETER_STATE_CALIBRATED) {  //already calibrated at start
    if ((millis() > FLIGHT_START_MIN_TIMESTAMP) && (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD)) {
      variometerState = VARIOMETER_STATE_FLIGHT_STARTED;
      enableflightStartComponents();
    }
  }
#endif  // !HAVE_GPS


}



#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
void createSDCardTrackFile(void) {
  /* start the sdcard record */
  if (sdcardState == SDCARD_STATE_INITIALIZED) {

    /* some cards doesn't like delays between init and write, so reinit */
    file.init();

    /* build date : convert from DDMMYY to YYMMDD */
    uint8_t dateChar[8] = {0};  //last 2 bytes are for increment number, filled by begin()
    uint8_t* dateCharP = dateChar;
    uint32_t date = nmeaParser.date;
    for (uint8_t i = 0; i < 3; i++) {
      uint8_t num = ((uint8_t)(date % 100));
      dateCharP[0] = (num / 10) + '0';
      dateCharP[1] = (num % 10) + '0';
      dateCharP += 2;
      date /= 100;
    }

    /* create file */
    if (file.begin((char*)dateChar, 8) >= 0) {
      sdcardState = SDCARD_STATE_READY;

#ifdef HAVE_IGC_SECURITY
      uint8_t keyBuf[32];
      memcpy_P(keyBuf, igcHmacKey, 32);
      igcHmac.begin(keyBuf, 32);
      igcHmacActive = true;
#endif

      /* write the header */
      int16_t datePos = header.begin();
      if (datePos >= 0) {
        while (datePos) {
          writeFeed(header.get());
          datePos--;
        }

        /* write date : DDMMYY */
        uint8_t* dateCharP = &dateChar[4];
        for (int i = 0; i < 3; i++) {
          writeFeed(dateCharP[0]);
          writeFeed(dateCharP[1]);
          header.get();
          header.get();
          dateCharP -= 2;
        }

        /* write flight number from filename (dateChar[6..7]) */
        writeFeed(',');
        writeFeed(dateChar[6]);
        writeFeed(dateChar[7]);
        writeFeed('\r');
        writeFeed('\n');

        while (header.available()) {
          writeFeed(header.get());
        }
      }
    } else {
      sdcardState = SDCARD_STATE_ERROR;  //avoid retry
#ifdef HAVE_IGC_SECURITY
      igcHmacActive = false;
#endif
    }
  }
}
#endif  //defined(HAVE_SDCARD) && defined(HAVE_GPS)


#if defined(HAVE_SDCARD) && defined(HAVE_GPS) && defined(HAVE_IGC_SECURITY)

static void writeHexByte(uint8_t b) {
  uint8_t nib = b >> 4;
  file.write((uint8_t)((nib < 10) ? ('0' + nib) : ('A' + nib - 10)));
  nib = b & 0x0F;
  file.write((uint8_t)((nib < 10) ? ('0' + nib) : ('A' + nib - 10)));
}

void finalizeIGCFile(void) {
  if (!igcHmacActive) return;
  igcHmac.finalize();
  const uint8_t* d = igcHmac.digest();

  for (uint8_t line = 0; line < 2; line++) {
    file.write('G');
    for (uint8_t i = 0; i < 16; i++) {
      writeHexByte(d[line * 16 + i]);
    }
    file.write('\r');
    file.write('\n');
  }

  file.sync();
  igcHmacActive = false;
}
#endif


void enableflightStartComponents(void) {
  /* enable near climbing */
#ifdef HAVE_SPEAKER
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
  beeper.setGlidingAlarmState(true);
#endif
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP
  beeper.setGlidingBeepState(true);
#endif
#endif  //HAVE_SPEAKER

#if defined(HAVE_SDCARD) && defined(HAVE_GPS) && defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
  createSDCardTrackFile();
#endif  // defined(HAVE_SDCARD) && defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
}

void getSensors() {
  I2C::intHandler();
}
