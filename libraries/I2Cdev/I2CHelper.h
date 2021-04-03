#ifndef I2CHELPER_H
#define I2CHELPER_H

#include <Arduino.h>

class I2C {
  public:
    I2C(void);
    static void begin(void);
    static bool writeBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char data);
    static unsigned char readByte(unsigned char devAddr, unsigned char regAddr);
    static bool writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char buff);
    static short readWord(unsigned char devAddr, unsigned char regAddr);
    static bool writeWord(unsigned char devAddr, unsigned char regAddr, short buff);
    static char readBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data);
    static bool writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data);
    static bool writeWords(unsigned char devAddr, unsigned char regAddr, unsigned char length, short *data);
    static bool sendCMD(unsigned char devAddr, unsigned char cmd);
    static uint32_t read24(unsigned char devAddr, unsigned char regAddr);
};

#endif
