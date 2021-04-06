/*  How to use
  void setup() {
    ms.init(); //Init once in setup()
  }

  void loop() {
    if (ms.ready())
      ms.update();

    float alt = ms.getAltitude(); //or ms.getPressure() or ms.getTemperature() depending on what you need
}
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
#define MS5611_CONV_DELAY   10 //max 9.04ms

#define MS5611_TEMP_EVERY 20 //how often should we update temperature?

//values for calculations
#define MS5611_BASE_SEA_PRESSURE 1013.25

//timer 2 defines
//#define MS5611_USE_TIMER
#ifdef MS5611_USE_TIMER
//startMeasure() and getMeasure() take max 0.488ms on 16MHz and 0.976ms on 8MHz
#if F_CPU == 16000000L
#define MS5611_INTERRUPT_COMPARE 160 //10.048 ms
#elif F_CPU == 8000000L
#define MS5611_INTERRUPT_COMPARE 81 //10.496 ms
#else
#error Interrupt compare value not defined for this CPU frequency.
#endif //F_CPU
#endif //MS5611_USE_TIMER

class ms5611 {
  public:
    static void init(void);
	static void stopTimer(void);
    static void startMeasure(void);
    static void getMeasure(void);
    static void update(void);
    static float getAltitude(void);
    static float getPressure(void);
    static float getTemperature(void);
#ifdef MS5611_USE_TIMER
	static bool ready(void);
#endif

  private:
    ms5611(void) {} //disable creating other instances
    static const unsigned char msAddr = MS5611_DEFAULT_ADDRESS;
    static float temperature, pressure;
    static uint8_t volatile msCurrentType;
    //static uint32_t volatile msMeasure;
	static uint32_t volatile d1, d2;
    static float msCoeffs[6];
#ifdef MS5611_USE_TIMER
	static bool msReady;
#endif
};

extern ms5611 ms;

#endif
