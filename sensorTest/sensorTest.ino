#include <I2CHelper.h>
#include <MPU6050.h>
#include <ms5611.h>
#include <kalmanvert.h>

kalmanvert kalmanvert;
MPU6050 mpu;

unsigned long now;
unsigned long start, finish;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println("Start");

  I2C::begin();

  //ms5611
  ms.init();

  //MPU6050
  mpu.init(); // load dmp and setup for normal use
  attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), getSensors, RISING);

  delay(2000); //let alt stabilize
  if (mpu.calibrate()) { //run calibration if up side down
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
}

void printdur() {
  static unsigned long prev;
  now = micros();
  Serial.print(now - prev); Serial.print("\t");
  prev = now;
}

void loop() {
  static unsigned long times[5];

  static float alt, vertAccel;

  //new sensor data ready
  switch (mpu.newData) {

    case 1:
      //printdur();
      ms.update();
      mpu.newData++;
      break;

    case 2:
      //printdur();
      alt = ms.getAltitude();
      mpu.newData++;
      break;

    case 3:
      //printdur();
      vertAccel = mpu.getVertaccel();
      mpu.newData++;
      break;

    case 4:
      //printdur();
      kalmanvert.update1(vertAccel, millis());
      mpu.newData++;
      break;

    case 5:
      //printdur();
      kalmanvert.update2(alt);
      mpu.newData++;
      break;

    case 6:
      //printdur();
      Serial.print(finish - start); Serial.print("\t");
      Serial.print(alt, 5); Serial.print("\t");
      Serial.print(vertAccel, 5); Serial.print(" \t");
      Serial.println(kalmanvert.getVelocity());

      mpu.newData = 0;
  }

  /* if (mpu.newData) {
     mpu.newData = false;

     times[0] = micros();
     ms.update();
     times[1] = micros();
     float alt = ms.getAltitude();
     times[2] = micros();
     float vertAccel = mpu.getVertaccel();
     times[3] = micros();
     kalmanvert.update(alt, vertAccel, millis());
     times[4] = micros();


     for (byte i = 0; i < 4; i++) {
       Serial.print(times[i + 1] - times[i]); Serial.print("\t");
     }
     Serial.println();

       Serial.print(alt, 5); Serial.print("\t");
       Serial.print(vertAccel, 5); Serial.print(" \t");
       Serial.println(kalmanvert.getVelocity());

    }*/
}

void getSensors() {
  //start = micros();
  ms.getMeasure();
  ms.startMeasure();
  //finish = micros();
  start = micros();
  mpu.getFIFO();
  finish = micros();
  mpu.newData = 1;
}
