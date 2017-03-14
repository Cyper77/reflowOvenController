#include "temperature.h"
#include <avr/pgmspace.h>

void temperatureSensorClass::setup(uint8_t pin) {
  
  //store pin local
  this->pin=pin;

  //status (0=ok, otherwise error)
  this->status=0;
  
  //dummy readout->throw away
  analogRead(this->pin);

  //init filter
  this->oAverageFilter.setAverage(this->readTemperatureRaw());
  
}


void temperatureSensorClass::triggerTemperatureMeasurement() {
  uint16_t rawtemp;

  //read temp and convert to celsius
  rawtemp=this->readTemperatureRaw();
  
  //add new value to rolling average filter
  this->oAverageFilter.AddToFloatAverage(rawtemp);

  //calculate new averaged result and store in cache for further use in main program
  this->temperatureRaw=this->oAverageFilter.getAverage();
  
}

uint16_t temperatureSensorClass::readTemperatureRaw() {
  //get raw value
  uint16_t rawtemp = analogRead(this->pin);

  //sanity check
  if(rawtemp<=5 || rawtemp>1018) {
    this->status=1;
    //go on but set error flag for signaling
  }
  
  return rawtemp;
}

double temperatureSensorClass::getTemperatureCelsius() {
  uint16_t rawtemp = this->temperatureRaw;
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

uint8_t temperatureSensorClass::getStatus() {
  return this->status;
}

