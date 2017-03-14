#include <Arduino.h>

//#define DEBUG
#define SERIAL_BAUD 115200
#define ENABLE_SERIAL_MENU 0

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
#define SIZE_OF_AVG 16    // over how many values the temperature is averaged

#define HEATER_Kp   4.00
#define HEATER_Ki   0.05
#define HEATER_Kd   2.00
#define FAN_Kp      1.00
#define FAN_Ki      0.03
#define FAN_Kd      10.0

// -------------------- Actuators, Heater, Fans
#define HEATING_PIN     6
#define FAN_PIN         5

// -------------------- Sensors, Buttons, Encoders
#define TEMP0_ADC 0
#define TEMP1_ADC 1

#define STOP_SWITCH_PIN     0
#define ENCODER_A_PIN       2
#define ENCODER_B_PIN       3
#define ENCODER_BUTTON_PIN  4


// -------------------- LCD
#define LCD_COLS 20
#define LCD_ROWS 4

#define LCD_RS      8
#define LCD_ENABLE  9
#define LCD_D0      10
#define LCD_D1      11
#define LCD_D2      12
#define LCD_D3      13


// -------------------- EEPROM
#define NUMBER_OF_STORED_PROFILES 10
// don't change the following defines
#define AMOUNT_EEPROM_PER_PROFILE 16
#define offsetFanSpeed    NUMBER_OF_STORED_PROFILES*AMOUNT_EEPROM_PER_PROFILE+1 // 30 * 16 + 1 one byte wide
#define offsetProfileNum  NUMBER_OF_STORED_PROFILES*AMOUNT_EEPROM_PER_PROFILE+2 // 30 * 16 + 2 one byte wide

