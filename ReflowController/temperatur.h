#ifndef TEMPERATUR_H
#define TEMPERATUR_H

#include <Arduino.h>

//temperature table
const int temp_table[] = {
  
};



void setupTemperatureSensors(void);
float readTemperature(uint8_t sensor);


#endif
