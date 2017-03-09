#include "temperature.h"
#include <avr/pgmspace.h>

void temperatureSensorClass::setup(uint8_t pin) {
  //init and first meas, set average
  this->pin=pin;

  analogRead(this->pin);
  this->oAverageFilter.setAverage(analogRead(this->pin));

  
}

void temperatureSensorClass::triggerTemperatureMeasurement() {
  //return read value
  int rawtemp = analogRead(this->pin);
  
  double current_celsius = 0;

  byte i;
  for (i = 1; i < (TEMPTABLE_ITEMS-1); i++) {
    if (temptable[i][0] > rawtemp) {
      double realtemp  = temptable[i - 1][1] + (rawtemp - temptable[i - 1][0]) * (temptable[i][1] - temptable[i - 1][1]) / (temptable[i][0] - temptable[i - 1][0]);
      
    /*if (pgm_read_word(temptable[i][0]) > rawtemp) {
      double realtemp  = (short)pgm_read_word(&(temptable[i - 1][1])) + (rawtemp - (short)pgm_read_word(&(temptable[i - 1][0]))) * (short)(pgm_read_word(&(temptable[i][1])) - (short)pgm_read_word(&(temptable[i - 1][1]))) / (short)(pgm_read_word(&(temptable[i][0])) - (short)pgm_read_word(&(temptable[i - 1][0])));
      */
      
      if (realtemp > 300)
        realtemp = 300;

      current_celsius = realtemp;

      break;
    }
  }

  // Overflow: We just clamp to 0 degrees celsius
  if (i == (TEMPTABLE_ITEMS-1))
    current_celsius = 0;

  this->oAverageFilter.AddToFloatAverage(current_celsius);
  this->temperature=this->oAverageFilter.getAverage();

}

double temperatureSensorClass::getTemperature() {
  return this->temperature;
}

uint8_t temperatureSensorClass::getStatus() {
  //no error: return 0
  return 0;
}

