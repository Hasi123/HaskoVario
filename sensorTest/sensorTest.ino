#include <I2CHelper.h>
#include <MPU6050.h>
#include <ms5611.h>
#include <kalmanvert.h>

kalmanvert kalmanvert;

unsigned long now;
unsigned long start, finish;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(57600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println("Start");

  I2C::begin();

  //ms5611
  ms.init();
  Serial.println("ms Initialized");

  //MPU6050
  mpu.init(); // load dmp and setup for normal use
  attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), getSensors, RISING);
  Serial.println("MPU Initialized");

  delay(2000); //let alt stabilize

  if (mpu.calibrate()) { //run calibration if up side down
    Serial.println("MPU calibration entered");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }

  Serial.println("Past MPU calibration");

  //init kalman filter
  I2C::newData = 0;
  while (!I2C::newData); //wait for fresh data
  ms.update();
  float firstAlti = ms.getAltitude();
  Serial.print("firstAlti: ");
  Serial.println(firstAlti);
  kalmanvert.init(firstAlti,
                  0.0,
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());
  Serial.println("Kalman Initialized");
}

void printdur() {
  static unsigned long prev;
  now = micros();
  Serial.print(now - prev); Serial.print("\t");
  prev = now;
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
      //printdur();
      ms.update();
      I2C::newData++;
      break;

    case 2:
      //printdur();
      alt = ms.getAltitude();
      I2C::newData++;
      break;

    case 3:
      //printdur();
      vertAccel = mpu.getVertaccel();
      I2C::newData++;
      break;

    case 4:
      //printdur();
      kalmanvert.update1(vertAccel, millis());
      I2C::newData++;
      break;

    case 5:
      //printdur();
      kalmanvert.update2(alt);
      I2C::newData++;
      break;

    case 6:
      //printdur();
      Serial.print(alt, 5); Serial.print("\t");
      Serial.print(vertAccel, 5); Serial.print(" \t");
      Serial.print(kalmanvert.getVelocity());
      Serial.println();

      I2C::newData = 0;
  }
}

void getSensors() {
  I2C::intHandler();
}
