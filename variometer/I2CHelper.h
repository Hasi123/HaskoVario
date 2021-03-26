#ifndef I2CHELPER_H
#define I2CHELPER_H

#include <Arduino.h>

void I2Cbegin(void);
bool writeBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char data);
unsigned char readByte(unsigned char devAddr, unsigned char regAddr);
bool writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char buff);
short readWord(unsigned char devAddr, unsigned char regAddr);
bool writeWord(unsigned char devAddr, unsigned char regAddr, short buff);
char readBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data);
bool writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data);
bool writeWords(unsigned char devAddr, unsigned char regAddr, unsigned char length, short *data);
bool sendCMD(unsigned char devAddr, unsigned char cmd);
uint32_t read24(unsigned char devAddr, unsigned char regAddr);

#endif
