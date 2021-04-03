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
  ms.compute();
  float firstAlti = ms.getAltitude();
  Serial.println(firstAlti);
  ms.startMeasure(); //get measurement for loop()
  kalmanvert.init(firstAlti,
                  0.0,
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());
  //delay(MS5611_CONV_DELAY); //not needed since mpuInit() takes >50 ms

  //start MPU
#ifdef MPU6050_INTERRUPT_PIN
  attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), mpuInterrupt, RISING);
#endif

  mpu.init(); // load dmp and setup for normal use
}

void loop() {

  //new DMP packet ready
#ifdef MPU6050_INTERRUPT_PIN
  if (mpu.newDMP) { //hardware interrupt triggered
    mpu.newDMP = 0;
#else
  if (mpu.newDmp()) { // read interrupt status register
#endif
    ms.compute();
    float alt = ms.getAltitude();
    ms.startMeasure();
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

//ISR must be defined here
#ifdef MPU6050_INTERRUPT_PIN
void mpuInterrupt() {
  mpu.newDMP = 1;
}
#endif
