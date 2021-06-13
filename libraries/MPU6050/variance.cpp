#include <Arduino.h>
#include "variance.h"

void variance::reset(void) {
  for (unsigned char i = 0; i < 3; i++) {
    sum[i] = 0;
    sqsum[i] = 0;
  }
  dataPoints = 0;
}
void variance::update(short* readings) {
  dataPoints++;
  for (unsigned char i = 0; i < 3; i++) { //get all axis
    sum[i] += readings[i];
    sqsum[i] += readings[i] * readings[i];
  }
}
unsigned long variance::getSum(void) {
  for (unsigned char i = 0; i < 3; i++) {
    var[i] = sqsum[i] - (sum[i] * sum[i] / dataPoints);
    //sd[i] = sqrt(var[i]);
  }
  return (var[0] + var[1] + var[2]); // < threshold * dataPoints);
}
