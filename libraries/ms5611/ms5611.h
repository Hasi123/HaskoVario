/*
  How to use:
  ms5611 ms; //create object
  
  void setup() {
	ms.init(); //Init once in setup()
  }

  ms.update(); // will read temperature every MS5611_TEMP_EVERY readings
  delay(MS5611_CONV_DELAY); //update() can be called at most every MS5611_CONV_DELAY milliseconds
  float alt = ms.getAltitude(); //or ms.getPressure() or ms.getTemperature() depending on what you need
*/

#ifndef _MS5611_H
#define _MS5611_H

#include <Arduino.h>

#define MS5611_ADDRESS_CSB_LOW     0x77 //address pin low (GND)
#define MS5611_ADDRESS_CSB_HIGH    0x76 //address pin high (VCC)
#ifndef MS5611_DEFAULT_ADDRESS
#define MS5611_DEFAULT_ADDRESS     MS5611_ADDRESS_CSB_LOW
#endif

#define MS5611_RESET        0x1E
#define MS5611_READ_PROM    0xA2
#define MS5611_CONV_D1      0x48 //OSR=4096
#define MS5611_CONV_D2      0x58 //OSR=4096
#define MS5611_ADC_READ     0x00

#define MS5611_RESET_DELAY  3
#define MS5611_CONV_DELAY   10

#define MS5611_TEMP_EVERY 50 //how often should we update temperature?

//values for calculations
#define MS5611_BASE_SEA_PRESSURE 1013.25

class ms5611 {
  public:
    ms5611(unsigned char Addr = MS5611_DEFAULT_ADDRESS);
    void init(void);
    void startMeasure(void);
    void getMeasure(void);
    void update(void);
    float getAltitude(void);
    float getPressure(void);
    float getTemperature(void);

  private:
    unsigned char msAddr;
    float temperature, pressure;
    bool msCurrentType;
    uint32_t d1, d2;
    float msCoeffs[6] = {32768L, 65536L, 3.90625E-3, 7.8125E-3, 256, 1.1920928955E-7};
};

#endif
