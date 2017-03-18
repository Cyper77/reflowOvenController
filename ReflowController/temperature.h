#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <Arduino.h>
#include "config.h"
#include "floatAverage.h"

#define OVERSAMPLENR 8      // choose max that OVERSAMPLENR * 1023 is below 2^10 to prevent overflow

#define PGM_RD_W(x) (short)pgm_read_word(&x)

//temperature table
//thermistor 1 in marlinfw, 100k NTC
#define TEMPTABLE_ITEMS 64
const short temptable[TEMPTABLE_ITEMS][2] PROGMEM = {
  {   23 * OVERSAMPLENR, 300 },
  {   25 * OVERSAMPLENR, 295 },
  {   27 * OVERSAMPLENR, 290 },
  {   28 * OVERSAMPLENR, 285 },
  {   31 * OVERSAMPLENR, 280 },
  {   33 * OVERSAMPLENR, 275 },
  {   35 * OVERSAMPLENR, 270 },
  {   38 * OVERSAMPLENR, 265 },
  {   41 * OVERSAMPLENR, 260 },
  {   44 * OVERSAMPLENR, 255 },
  {   48 * OVERSAMPLENR, 250 },
  {   52 * OVERSAMPLENR, 245 },
  {   56 * OVERSAMPLENR, 240 },
  {   61 * OVERSAMPLENR, 235 },
  {   66 * OVERSAMPLENR, 230 },
  {   71 * OVERSAMPLENR, 225 },
  {   78 * OVERSAMPLENR, 220 },
  {   84 * OVERSAMPLENR, 215 },
  {   92 * OVERSAMPLENR, 210 },
  {  100 * OVERSAMPLENR, 205 },
  {  109 * OVERSAMPLENR, 200 },
  {  120 * OVERSAMPLENR, 195 },
  {  131 * OVERSAMPLENR, 190 },
  {  143 * OVERSAMPLENR, 185 },
  {  156 * OVERSAMPLENR, 180 },
  {  171 * OVERSAMPLENR, 175 },
  {  187 * OVERSAMPLENR, 170 },
  {  205 * OVERSAMPLENR, 165 },
  {  224 * OVERSAMPLENR, 160 },
  {  245 * OVERSAMPLENR, 155 },
  {  268 * OVERSAMPLENR, 150 },
  {  293 * OVERSAMPLENR, 145 },
  {  320 * OVERSAMPLENR, 140 },
  {  348 * OVERSAMPLENR, 135 },
  {  379 * OVERSAMPLENR, 130 },
  {  411 * OVERSAMPLENR, 125 },
  {  445 * OVERSAMPLENR, 120 },
  {  480 * OVERSAMPLENR, 115 },
  {  516 * OVERSAMPLENR, 110 },
  {  553 * OVERSAMPLENR, 105 },
  {  591 * OVERSAMPLENR, 100 },
  {  628 * OVERSAMPLENR,  95 },
  {  665 * OVERSAMPLENR,  90 },
  {  702 * OVERSAMPLENR,  85 },
  {  737 * OVERSAMPLENR,  80 },
  {  770 * OVERSAMPLENR,  75 },
  {  801 * OVERSAMPLENR,  70 },
  {  830 * OVERSAMPLENR,  65 },
  {  857 * OVERSAMPLENR,  60 },
  {  881 * OVERSAMPLENR,  55 },
  {  903 * OVERSAMPLENR,  50 },
  {  922 * OVERSAMPLENR,  45 },
  {  939 * OVERSAMPLENR,  40 },
  {  954 * OVERSAMPLENR,  35 },
  {  966 * OVERSAMPLENR,  30 },
  {  977 * OVERSAMPLENR,  25 },
  {  985 * OVERSAMPLENR,  20 },
  {  993 * OVERSAMPLENR,  15 },
  {  999 * OVERSAMPLENR,  10 },
  { 1004 * OVERSAMPLENR,   5 },
  { 1008 * OVERSAMPLENR,   0 },
  { 1012 * OVERSAMPLENR,  -5 },
  { 1016 * OVERSAMPLENR, -10 },
  { 1020 * OVERSAMPLENR, -15 }
};


class temperatureSensorClass {
  private:
    //ADC-Pin to read from
    uint8_t pin;
    
    //have current temperature (averaged) in cache
    double temperature;

    //holds information about status of sensor
    uint8_t status;
    
    //read from ADC convert to celsius and return value
    uint16_t readTemperatureRaw();
    
    //convert raw temp to celsius
    double convertTemperature(uint16_t rawtemp);
    
    //object to hold the average-filter
    floatAverageClass oAverageFilter;
    
  public:
    //initialize
    void setup(uint8_t pin);

    //organizes temperature readings and internal updates. to be triggered by main program
    void triggerTemperatureMeasurement();

    //calculates the delta in rise of temp
    double getTemperatureRampPerDatastep();
    
    //return cached temperature
    double getTemperatureCelsius();

    //some checks to check for illegal readings are performed while operating. returns 0: OK otherwise errror occured
    uint8_t getStatus();

    
    
};

#endif
