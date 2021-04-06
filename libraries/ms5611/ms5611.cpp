#include <Arduino.h>
#include "ms5611.h"
#include <I2CHelper.h>

float ms5611::temperature;
float ms5611::pressure;
uint8_t volatile ms5611::msCurrentType = 0;
//uint32_t volatile ms5611::msMeasure;
uint32_t volatile ms5611::d1;
uint32_t volatile ms5611::d2;
float ms5611::msCoeffs[6] = {32768L, 65536L, 3.90625E-3, 7.8125E-3, 256, 1.1920928955E-7};
//bool ms5611::msReady = 0;

//issue command to start measurement
void ms5611::startMeasure(void) {
  if (msCurrentType) { //get pressure
    I2C::sendCMD(msAddr, MS5611_CONV_D1);
  }
  else { //get temperature
    I2C::sendCMD(msAddr, MS5611_CONV_D2);
    msCurrentType = MS5611_TEMP_EVERY;
  }
  msCurrentType--;
}

//get measurement
void ms5611::getMeasure(void) {
  //msMeasure = I2C::read24(msAddr, MS5611_ADC_READ);
  if (msCurrentType == (MS5611_TEMP_EVERY - 1)) { //get temperature
    d2 = I2C::read24(msAddr, MS5611_ADC_READ);
  }
  else { //get pressure
    d1 = I2C::read24(msAddr, MS5611_ADC_READ);
  }
  //msReady = true;
}

void ms5611::init(void) {
  /* reset */
  I2C::sendCMD(msAddr, MS5611_RESET);
  delay(MS5611_RESET_DELAY);

  //read factory calibrations from PROM
  //multiply with constant values from datasheet
  for (uint8_t reg = 0; reg < 6; reg++) {
    msCoeffs[reg] *= (uint16_t)I2C::readWord(msAddr, MS5611_READ_PROM + (reg * 2));
  }

  //setup timer 2 interrupt
#ifdef MS5611_USE_TIMER
  cli();
  TCCR2A = 0b00000010; //CTC MODE
  TCCR2B = 0b00000111; //1024 prescaler
  TIMSK2 = 0b00000010; //enable CompA
  TCNT2  = 0; //reset timer
  OCR2A  = MS5611_INTERRUPT_COMPARE; //set compare register
  sei();
#endif
}

void ms5611::stopTimer(void) {
TIMSK2 = 0;
}

void ms5611::update(void) {
  //msReady = false;
  
  /*
  cli();
  if (msCurrentType == (MS5611_TEMP_EVERY - 2)) //get temperature
    d2 = msMeasure;
  else //get pressure
    d1 = msMeasure;
  sei();
  */

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

float ms5611::getAltitude(void) {
  float alti;
  alti = pow((pressure / (MS5611_BASE_SEA_PRESSURE)), 0.1902664357); //could approximate with taylor series to save ~1kb
  alti = (1 - alti) * 44330;
  return alti;
}

float ms5611::getPressure(void) {
  return pressure;
}

float ms5611::getTemperature(void) {
  return (temperature * 0.01);
}
/*
bool ms5611::ready(void) {
  return msReady;
}
*/
//get and start measures
#ifdef MS5611_USE_TIMER
ISR (TIMER2_COMPA_vect) {
  ms5611::getMeasure();
  ms5611::startMeasure();
}
#endif
