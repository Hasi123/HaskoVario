
#include <I2CHelper.h>
#include <MPU6050.h>
#include <ms5611.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <marioSounds.h>
#include <varioPower.h>


VarioPower varioPower;
kalmanvert kalmanvert;
beeper beeper;
MPU6050 mpu;
ms5611 ms;
short newgyro[3], newaccel[3];
long newquat[4];


void setup() {
  //init varioPower
  varioPower.init();

  //Serial.begin(57600);
  //Serial.println("Start");

  /**************************/
  /* init Two Wires devices */
  /**************************/
  I2C::begin();

  //MPU6050 calibration
  mpu.calibrate(); //run calibration if up side down

  //ms5611
  ms.init();
  for (uint8_t i = 0; i < (MS5611_TEMP_EVERY + 1); i++) {
    ms.getMeasure();
    ms.startMeasure();
    delay(MS5611_CONV_DELAY);
  }

  //init kalman filter
  ms.update();
  float firstAlti = ms.getAltitude();
  kalmanvert.init(firstAlti,
                  0.0,
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());
  //delay(MS5611_CONV_DELAY); //not needed since mpu.init() takes >50 ms

  mpu.init(); // load dmp and setup for normal use
}

void loop() {
  //new DMP packet ready
  if (mpu.newDmp()) { // read interrupt status register
    //static unsigned long lastTime;
    //unsigned long now = micros();
    //Serial.print(now - lastTime); Serial.print("\t");
    //lastTime = now;


    ms.update();
    float alt = ms.getAltitude();
    //Serial.print(alt); Serial.print("\t");


    mpu.getFIFO(newgyro, newaccel, newquat);
    float vertAccel = mpu.getVertaccel(newaccel, newquat);
    //Serial.print(vertAccel, 5); Serial.print(" \t");
    //Serial.print(newaccel[0]); Serial.print("\t");
    //Serial.print(newaccel[1]); Serial.print("\t");
    //Serial.print(newaccel[2]); Serial.print("\t");

    if (!isnan(alt) && !isnan(vertAccel))
      kalmanvert.update(alt, vertAccel, millis());
    //Serial.print(kalmanvert.getVelocity()); Serial.print("\t");


    // set beeper
    beeper.setVelocity(kalmanvert.getVelocity());

    //Serial.println();
  }

  /*
    ms.compute();
    float alt = ms.getAltitude();
    ms.startMeasure();
    Serial.println(alt);
    delay(MS5611_CONV_DELAY);
  */

  varioPower.update();
  beeper.update();

}
