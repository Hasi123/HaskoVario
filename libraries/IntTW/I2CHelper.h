#ifndef I2CHELPER_H
#define I2CHELPER_H

#include <Arduino.h>

#include <ms5611.h>
#include <MPU6050.h>

class I2C {
  public:
    //I2C(void);
    static void begin(void);
    static unsigned char readByte(unsigned char devAddr, unsigned char regAddr);
    static bool writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char data);
    static short readWord(unsigned char devAddr, unsigned char regAddr);
    static bool writeWord(unsigned char devAddr, unsigned char regAddr, short data);
    static char readBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data);
    static bool writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data);
    static bool writeWords(unsigned char devAddr, unsigned char regAddr, unsigned char length, short *data);
    static bool sendCMD(unsigned char devAddr, unsigned char cmd);
    static uint32_t read24(unsigned char devAddr, unsigned char regAddr);
    static void intHandler(void);
    static void kick(void);
	
	static int8_t volatile newData;

  private:
    static uint8_t volatile msData[3];
	static uint8_t volatile msCurrentType;
	static uint8_t volatile pendingInt;
	static uint8_t volatile fifoPackets;
    static uint8_t volatile fifoData[MPU6050_FIFO_LENGTH];
	static void mpuGetFIFOcount(void);
	static void mpuCheckFIFOcount(void);
	static void mpuGetFIFO(void);
	static void mpuFIFOread(void);
	static void mpuResetFIFO(void);
	static void mpuEnableFIFO(void);
	static void chainEnd(void);
	static void dataReady(void);
};

#endif
