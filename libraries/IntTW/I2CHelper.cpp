#include <Arduino.h>
#include "I2CHelper.h"

#include "IntTW.h"
#include <avr/pgmspace.h>

#include <ms5611.h>
#include <MPU6050.h>

int8_t volatile I2C::newData = 0;
uint8_t volatile I2C::msData[3];
uint8_t volatile I2C::msCurrentType = 0;
uint8_t volatile I2C::fifoData[MPU6050_FIFO_LENGTH];

void I2C::begin(void) {
  intTW.begin();
}

unsigned char I2C::readByte(unsigned char devAddr, unsigned char regAddr) {
  unsigned char buff;
  intTW.readBytes(devAddr, regAddr, 1, &buff);
  return buff;
}

bool I2C::writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char data) {
  return intTW.writeBytes(devAddr, regAddr, 1, &data);
}

short I2C::readWord(unsigned char devAddr, unsigned char regAddr) {
  uint8_t buff[2];
  intTW.readBytes(devAddr, regAddr, 2, buff);
  return ((short)(buff[0] << 8) | (short)buff[1]);
}

bool I2C::writeWord(unsigned char devAddr, unsigned char regAddr, short data) {
  uint8_t buff[2];
  buff[0] = (uint8_t)(data >> 8);
  buff[1] = (uint8_t)data;
  return intTW.writeBytes(devAddr, regAddr, 2, buff);
}

char I2C::readBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data) {
  return intTW.readBytes(devAddr, regAddr, length, data);
}

bool I2C::writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data) {
  return intTW.writeBytes(devAddr, regAddr, length, data);
}

bool I2C::writeWords(unsigned char devAddr, unsigned char regAddr, unsigned char length, short *data) {
  uint8_t buff[6];
  for (uint8_t i = 0; i < 3; i++) {
    buff[i*2] = (uint8_t)(data[i] >> 8);
    buff[i*2+1] = (uint8_t)data[i];
  }
  return intTW.writeBytes(devAddr, regAddr, length*2, buff);
}

bool I2C::sendCMD(unsigned char devAddr, unsigned char cmd) {
  return intTW.writeBytes(devAddr, cmd, 0, NULL);
}

uint32_t I2C::read24(unsigned char devAddr, unsigned char regAddr) {
  uint8_t buff[3];
  intTW.readBytes(devAddr, regAddr, 3, buff);
  uint32_t msVal = ((uint32_t)buff[0] << 16) | ((uint32_t)buff[1] << 8) | buff[2];
  return msVal;
}

static const uint8_t msStartTemp[] PROGMEM = { INTTW_ACTION(MS5611_DEFAULT_ADDRESS, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       MS5611_ADC_READ,
					       INTTW_ACTION(MS5611_DEFAULT_ADDRESS, INTTW_READ),
					       INTTW_DEST(3, INTTW_AT_POINTER),
					       INTTW_ACTION(MS5611_DEFAULT_ADDRESS, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       MS5611_CONV_D2 };

static const uint8_t msStartPres[] PROGMEM = { INTTW_ACTION(MS5611_DEFAULT_ADDRESS, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       MS5611_ADC_READ,
					       INTTW_ACTION(MS5611_DEFAULT_ADDRESS, INTTW_READ),
					       INTTW_DEST(3, INTTW_AT_POINTER),
					       INTTW_ACTION(MS5611_DEFAULT_ADDRESS, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       MS5611_CONV_D1 };
						   
static const uint8_t mpuReadFifoCount[] PROGMEM = { INTTW_ACTION(MPU6050_DEFAULT_ADDRESS, INTTW_WRITE),
						    INTTW_DEST(1, INTTW_IN_CMD),
						    MPU6050_RA_FIFO_COUNTH,
						    INTTW_ACTION(MPU6050_DEFAULT_ADDRESS, INTTW_READ),
						    INTTW_DEST(2, INTTW_AT_POINTER) };


static const uint8_t mpuReadFifo[] PROGMEM = { INTTW_ACTION(MPU6050_DEFAULT_ADDRESS, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       MPU6050_RA_FIFO_R_W,
					       INTTW_ACTION(MPU6050_DEFAULT_ADDRESS, INTTW_READ),
					       INTTW_DEST(MPU6050_FIFO_LENGTH, INTTW_AT_POINTER) };

/* get sensor data interrupt handler
 1: getSensors() calls intHandler()
 2: (alternating) ms measure
 2: on ready trigger get fifo count -> error handling
 3: on ready trigger get fifo
 4: on ready parse and release new data
*/
void I2C::intHandler(void){
  intTW.setRxBuffer(msData);
  if (msCurrentType) { //pressure
    intTW.start(msStartPres, sizeof(msStartPres), INTTW_USE_PROGMEM, mpuGetFIFOcount);
  }
  else { //temperature
    intTW.start(msStartTemp, sizeof(msStartTemp), INTTW_USE_PROGMEM, mpuGetFIFOcount);
    msCurrentType = MS5611_TEMP_EVERY;
  }
}

void I2C::mpuGetFIFOcount(void){
  intTW.setRxBuffer(fifoData);
  intTW.start(mpuReadFifoCount, sizeof(mpuReadFifoCount), INTTW_USE_PROGMEM | INTTW_KEEP_BUS, mpuCheckFIFOcount);
}

void I2C::mpuCheckFIFOcount(void){
  //TODO: check if fifo size as expected
  uint16_t fifo_count = ((uint16_t)fifoData[0] << 8) | (uint16_t)fifoData[1];
  if (fifo_count != MPU6050_FIFO_LENGTH) { //reset FIFO if not exactly 1 packet
    intTW.stop();
    newData = -1;
  }
  else {
    mpuGetFIFO();
  }
}

void I2C::mpuGetFIFO(void){
  intTW.setRxBuffer(fifoData);
  intTW.start(mpuReadFifo, sizeof(mpuReadFifo), INTTW_USE_PROGMEM, dataReady);
}

void I2C::dataReady(void){
  ms5611::parseMeasure(msCurrentType == (MS5611_TEMP_EVERY - 1), msData);
  msCurrentType--;
  mpu.parseFIFO(fifoData);
  newData = 1;
}
