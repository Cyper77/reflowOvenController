#include <Arduino.h>

//#define DEBUG
#define SERIAL_BAUD 57600

// -------------------- Profile
#define PROFILE_DEFAULT_soakTemp      130;
#define PROFILE_DEFAULT_soakDuration  30;
#define PROFILE_DEFAULT_peakTemp      220;
#define PROFILE_DEFAULT_peakDuration  10;
#define PROFILE_DEFAULT_rampUpRate    0.80;
#define PROFILE_DEFAULT_rampDownRate  3.0;
#define FAN_DEFAULT_SPEED 33

#define idleTemp        50 // the temperature at which to consider the oven safe to leave to cool naturally


// -------------------- Control
#define WindowSize  100    //in ms

#define HEATER_Kp   4.00
#define HEATER_Ki   0.05
#define HEATER_Kd   2.00
#define FAN_Kp      1.00
#define FAN_Ki      0.03
#define FAN_Kd      10.0

// -------------------- Actuators, Heater, Fans
#define SSR1_PIN    12
#define SSR2_PIN    11

// -------------------- Sensors, Buttons, Encoders
#define TEMP0_ADC 0
#define TEMP1_ADC 1
#define TEMP2_ADC 2

#define stopKeyInputPin     7
#define ENCODER_A_PIN       2
#define ENCODER_B_PIN       3
#define ENCODER_BUTTON_PIN  4


// -------------------- LCD
#define LCD_RS      5
#define LCD_ENABLE  6
#define LCD_D0      7
#define LCD_D1      8
#define LCD_D2      9
#define LCD_D3      10


// -------------------- EEPROM
#define offsetFanSpeed 481 // 30 * 16 + 1 one byte wide
#define offsetProfileNum 482 // 30 * 16 + 2 one byte wide

