#include <Arduino.h>
#include "ms5611Helper.h"
#include "I2CHelper.h"

//internal variables
const unsigned char msAddr = MS5611_DEFAULT_ADDRESS;
float temperature, pressure;
bool msCurrentType;
uint32_t d1, d2;
float msCoeffs[6] = {32768L, 65536L, 3.90625E-3, 7.8125E-3, 256, 1.1920928955E-7};

//issue command to start measurement
void msStartMeasure(void) {
  static uint8_t counter = 0;
  if (counter) { //get pressure
    I2C::sendCMD(msAddr, MS5611_CONV_D1);
    msCurrentType = 1;
  }
  else { //get temperature
    I2C::sendCMD(msAddr, MS5611_CONV_D2);
    msCurrentType = 0;
    counter = MS5611_TEMP_EVERY;
  }
  counter--;
}

//get measurement
void msGetMeasure(void) {
  if (msCurrentType) { //get pressure
    d1 = I2C::read24(msAddr, MS5611_ADC_READ);
  }
  else { //get temperature
    d2 = I2C::read24(msAddr, MS5611_ADC_READ);
  }
}

void msInit(void) {
  /* reset */
  I2C::sendCMD(msAddr, MS5611_RESET);
  delay(MS5611_RESET_DELAY);

  //read factory calibrations from PROM
  //multiply with constant values from datasheet
  for (uint8_t reg = 0; reg < 6; reg++) {
    msCoeffs[reg] *= (uint16_t)I2C::readWord(msAddr, MS5611_READ_PROM + (reg * 2));
  }

  /* get first data */
  msStartMeasure(); //temp
  delay(MS5611_CONV_DELAY);
  msGetMeasure();
  msStartMeasure(); //pressure
  delay(MS5611_CONV_DELAY);
}

void computeFloat() {
  // ALL MAGIC NUMBERS ARE FROM DATASHEET

  // TEMP & PRESS MATH - PAGE 7/20
  float dT = d2 - msCoeffs[4];
  temperature = 2000 + dT * msCoeffs[5];

  float offset =  msCoeffs[1] + dT * msCoeffs[3];
  float sens = msCoeffs[0] + dT * msCoeffs[2];

  // SECOND ORDER COMPENSATION - PAGE 8/20
  // COMMENT OUT < 2000 CORRECTION IF NOT NEEDED
  // NOTE TEMPERATURE IS IN 0.01 C
  if (temperature < 2000) {
    float T2 = dT * dT * 4.6566128731E-10;
    float t = temperature - 2000;
    t = t * t;
    float offset2 = 2.5 * t;
    float sens2 = 1.25 * t;
    // COMMENT OUT < -1500 CORRECTION IF NOT NEEDED
    if (temperature < -1500) {
      t = temperature + 1500;
      t = t * t;
      offset2 += 7 * t;
      sens2 += 5.5 * t;
    }
    temperature -= T2;
    offset -= offset2;
    sens -= sens2;
  }
  // END SECOND ORDER COMPENSATION

  pressure = (d1 * sens * 4.76837158205E-7 - offset) * 3.051757813E-7;
}

float msComputeAltitude(void) {
  msGetMeasure();
  computeFloat();
  float alti;
  alti = pow((pressure / (MS5611_BASE_SEA_PRESSURE)), 0.1902664357); //could approximate with taylor series to save ~1kb
  alti = (1 - alti) * 44330;
  return alti;
}

float msComputePressure(void) {
  msGetMeasure();
  computeFloat();
  return pressure;
}

float msComputeTemperature(void) {
  msGetMeasure();
  computeFloat();
  temperature *= 0.01;
  return temperature;
}
