#include "temperature.h"
#include <avr/pgmspace.h>

void temperatureSensorClass::setup(uint8_t pin) {
  
  //store pin local
  this->pin=pin;
  
  //dummy readout->throw away
  analogRead(this->pin);

  //init filter
  this->oAverageFilter.setAverage(analogRead(this->pin));
  
}

// Derived in parts from MarlinFW
void temperatureSensorClass::triggerTemperatureMeasurement() {
  //get raw value
  int rawtemp = analogRead(this->pin);
  
  double celsius;

  
  byte i;
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

  //add new value to rolling average filter
  this->oAverageFilter.AddToFloatAverage(celsius);

  //calculate new averaged result and store in cache for further use in main program
  this->temperature=this->oAverageFilter.getAverage();
  
}

void temperatureSensorClass::triggerTemperatureMeasurement_old() {
  //return read value
  int rawtemp = analogRead(this->pin);
  
  double current_celsius = 0;

  byte i;
  for (i = 1; i < (TEMPTABLE_ITEMS-1); i++) {
    if (temptable[i][0] > rawtemp) {
      double realtemp  = temptable[i - 1][1] + (rawtemp - temptable[i - 1][0]) * (temptable[i][1] - temptable[i - 1][1]) / (temptable[i][0] - temptable[i - 1][0]);
      
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

