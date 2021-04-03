#include <I2CHelper.h>
#include <MPU6050.h>
#include <ms5611.h>
#include <kalmanvert.h>

kalmanvert kalmanvert;
MPU6050 mpu;
ms5611 ms;

short gyro[3], accel[3];
long quat[4];

void setup() {
  Serial.begin(57600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println("Start");

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
  Serial.println(firstAlti);
  kalmanvert.init(firstAlti,
                  0.0,
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());
  //delay(MS5611_CONV_DELAY); //not needed since mpuInit() takes >50 ms

  mpu.init(); // load dmp and setup for normal use
}

void loop() {

  if (mpu.newDmp()) {
    ms.update();
    float alt = ms.getAltitude();
    mpu.getFIFO(gyro, accel, quat);
    float vertAccel = mpu.getVertaccel(accel, quat);
    kalmanvert.update( alt,
                       vertAccel,
                       millis() );

    Serial.print(alt, 5); Serial.print("\t");
    Serial.print(vertAccel, 5); Serial.print(" \t");
    Serial.println(kalmanvert.getVelocity());
  }
}
