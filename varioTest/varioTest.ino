#define DEBUGSD

#include <I2CHelper.h>
#include <MPU6050.h>
#include <ms5611.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <marioSounds.h>
#include <varioPower.h>
#ifdef DEBUGSD
#include <SPI.h>
#include <SD.h>

const int chipSelect = 8;
#endif

VarioPower varioPower;
kalmanvert kalmanvert;


void setup() {
  //init varioPower
  varioPower.init();

  //Serial.begin(57600);
  //Serial.println("Start");

#ifdef DEBUGSD
  SD.begin(chipSelect);
  //SD.remove("calib.txt");
  SD.remove("datalog.txt");
#endif

  /**************************/
  /* init Two Wires devices */
  /**************************/
  I2C::begin();

  //ms5611
  ms.init();

  //MPU6050
  mpu.init(); // load dmp and setup for normal use
  attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), getSensors, RISING);

  //play sound and check if need to update
  marioSounds.bootUp();
  varioPower.updateFW();
  if (mpu.calibrate()) { //run calibration if up side down
#ifdef DEBUGSD
    File dataFile = SD.open("calib.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(mpu.debugString);
      dataFile.close();
    }
#endif
    varioPower.reset(); //reset to load calibration data and dmp again
  }

  //init kalman filter
  I2C::newData = 0;
  while (!I2C::newData); //wait for fresh data
  ms.update();
  float firstAlti = ms.getAltitude();
  Serial.println(firstAlti);
  kalmanvert.init(firstAlti,
                  0.0,
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());
}

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
      I2C::newData++;
      break;

    case 6:
      beeper::setVelocity(kalmanvert.getVelocity());

#ifdef DEBUGSD
      String dataString = "";
      dataString += String(mpu.accelData[0]);
      dataString += ",";
      dataString += String(mpu.accelData[1]);
      dataString += ","; //'\n';
      dataString += String(mpu.accelData[2]);

      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      // if the file is available, write to it:
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
      }
#endif

      I2C::newData = 0;
  }

  varioPower.update();
  beeper::update();

}

void getSensors() {
  I2C::intHandler();
}
