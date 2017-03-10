#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <Arduino.h>
#include "config.h"
#include "floatAverage.h"

#define PGM_RD_W(x) (short)pgm_read_word(&x)

//temperature table
#define TEMPTABLE_ITEMS 64
const short temptable[TEMPTABLE_ITEMS][2] PROGMEM = {
  {   23, 300 },
  {   25, 295 },
  {   27, 290 },
  {   28, 285 },
  {   31, 280 },
  {   33, 275 },
  {   35, 270 },
  {   38, 265 },
  {   41, 260 },
  {   44, 255 },
  {   48, 250 },
  {   52, 245 },
  {   56, 240 },
  {   61, 235 },
  {   66, 230 },
  {   71, 225 },
  {   78, 220 },
  {   84, 215 },
  {   92, 210 },
  {  100, 205 },
  {  109, 200 },
  {  120, 195 },
  {  131, 190 },
  {  143, 185 },
  {  156, 180 },
  {  171, 175 },
  {  187, 170 },
  {  205, 165 },
  {  224, 160 },
  {  245, 155 },
  {  268, 150 },
  {  293, 145 },
  {  320, 140 },
  {  348, 135 },
  {  379, 130 },
  {  411, 125 },
  {  445, 120 },
  {  480, 115 },
  {  516, 110 },
  {  553, 105 },
  {  591, 100 },
  {  628,  95 },
  {  665,  90 },
  {  702,  85 },
  {  737,  80 },
  {  770,  75 },
  {  801,  70 },
  {  830,  65 },
  {  857,  60 },
  {  881,  55 },
  {  903,  50 },
  {  922,  45 },
  {  939,  40 },
  {  954,  35 },
  {  966,  30 },
  {  977,  25 },
  {  985,  20 },
  {  993,  15 },
  {  999,  10 },
  { 1004,   5 },
  { 1008,   0 },
  { 1012,  -5 },
  { 1016, -10 },
  { 1020, -15 }
};



class temperatureSensorClass {
  private:
    //have current temperature (averaged) in cache
    double temperature;

    uint8_t status;
  
  public:

    //ADC-Pin to read from
    uint8_t pin;

    //object to hold the average-filter
    floatAverageClass oAverageFilter;

    //initialize
    void setup(uint8_t pin);

    //organizes temperature readings and internal updates. to be triggered by main program
    void triggerTemperatureMeasurement();

    //read from ADC convert to celsius and return value
    double readTemperatureCelsius();

    //return cached temperature
    double getTemperature();

    //TODO: implement some checks if readings are illegal
    uint8_t getStatus();

    
    
};

#endif
