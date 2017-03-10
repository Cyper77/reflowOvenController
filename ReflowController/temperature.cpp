#include "temperature.h"
#include <avr/pgmspace.h>

void temperatureSensorClass::setup(uint8_t pin) {
  
  //store pin local
  this->pin=pin;
  
  //dummy readout->throw away
  analogRead(this->pin);

  //init filter
  this->oAverageFilter.setAverage(this->readTemperatureCelsius());
  
}


void temperatureSensorClass::triggerTemperatureMeasurement() {
  double celsius;

  //read temp and convert to celsius
  celsius=this->readTemperatureCelsius();
  
  //add new value to rolling average filter
  this->oAverageFilter.AddToFloatAverage(celsius);

  //calculate new averaged result and store in cache for further use in main program
  this->temperature=this->oAverageFilter.getAverage();
  
}

double temperatureSensorClass::readTemperatureCelsius() {
  //get raw value
  int rawtemp = analogRead(this->pin);
  
  double celsius;

  byte i;
  // Derived from MarlinFW
  short(*tt)[][2] = (short(*)[][2])(temptable);
  for (i = 1; i < (TEMPTABLE_ITEMS-1); i++) {
    if (PGM_RD_W((*tt)[i][0]) > rawtemp) {
       celsius = PGM_RD_W((*tt)[i - 1][1]) +
                (rawtemp - PGM_RD_W((*tt)[i - 1][0])) *
                (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i - 1][1])) /
                (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i - 1][0]));
      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == (TEMPTABLE_ITEMS-1))
    celsius = PGM_RD_W((*tt)[(TEMPTABLE_ITEMS-1)][1]);

  return celsius;
}

double temperatureSensorClass::getTemperature() {
  return this->temperature;
}

uint8_t temperatureSensorClass::getStatus() {
  //no error: return 0
  return 0;
}

