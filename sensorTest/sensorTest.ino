#include <I2CHelper.h>
#include <MPU6050.h>
#include <ms5611.h>
#include <kalmanvert.h>

kalmanvert kalmanvert;
MPU6050 mpu;

byte newData;

void setup() {
  Serial.begin(57600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println("Start");

  I2C::begin();

  //ms5611
  ms.init();

  //MPU6050
  mpu.init(); // load dmp and setup for normal use
  attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), getSensors, RISING);

  delay(2000); //let alt stabilize
  if (mpu.calibrate()) //run calibration if up side down
    setup();

  //init kalman filter
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

  if (newData) {
    newData = false;
    ms.update();
    float alt = ms.getAltitude();
    float vertAccel = mpu.getVertaccel();
    kalmanvert.update(alt, vertAccel, millis());

    Serial.print(alt, 5); Serial.print("\t");
    Serial.print(vertAccel, 5); Serial.print(" \t");
    Serial.println(kalmanvert.getVelocity());
  }
}

void getSensors() {
  ms.getMeasure();
  ms.startMeasure();
  mpu.getFIFO();
  newData = true;
}
