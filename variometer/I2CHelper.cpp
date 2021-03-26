#include <Arduino.h>
#include "I2CHelper.h"
#include "I2Cdev.h"

void I2Cbegin(void) {
  Fastwire::setup(400, true);
}

bool writeBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char data) {
  return I2Cdev::writeBits(devAddr, regAddr, bitStart, length, data);
}

unsigned char readByte(unsigned char devAddr, unsigned char regAddr) {
  unsigned char buff;
  I2Cdev::readByte(devAddr, regAddr, &buff);
  return buff;
}

bool writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char buff) {
  return I2Cdev::writeByte(devAddr, regAddr, buff);
}

short readWord(unsigned char devAddr, unsigned char regAddr) {
  uint16_t buff;
  I2Cdev::readWord(devAddr, regAddr, &buff);
  return buff;
}

bool writeWord(unsigned char devAddr, unsigned char regAddr, short buff) {
  return I2Cdev::writeWord(devAddr, regAddr, buff);
}

char readBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data) {
  return I2Cdev::readBytes(devAddr, regAddr, length, data);
}

bool writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data) {
  return I2Cdev::writeBytes(devAddr, regAddr, length, data);
}

bool writeWords(unsigned char devAddr, unsigned char regAddr, unsigned char length, short *data) {
  return I2Cdev::writeWords(devAddr, regAddr, length, (uint16_t*)data);
}

bool sendCMD(unsigned char devAddr, unsigned char cmd) {
  return I2Cdev::writeBytes(devAddr, cmd, 0, NULL);
}

uint32_t read24(unsigned char devAddr, unsigned char regAddr) {
  uint8_t buff[3];
  I2Cdev::readBytes(devAddr, regAddr, 3, buff);
  uint32_t msVal = ((uint32_t)buff[0] << 16) | ((uint32_t)buff[1] << 8) | buff[2];
  return msVal;
}
