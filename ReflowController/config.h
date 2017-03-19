#include <Arduino.h>

//#define DEBUG
#define SERIAL_BAUD 115200
#define ENABLE_SERIAL_MENU 0

// -------------------- Profile
// this is the default profile, special profiles see in profiles.h
#define PROFILE_DEFAULT_soakTemp      165;
#define PROFILE_DEFAULT_soakDuration  90;
#define PROFILE_DEFAULT_peakTemp      249;
#define PROFILE_DEFAULT_peakDuration  20;
#define PROFILE_DEFAULT_rampUpRate    1.0;
#define PROFILE_DEFAULT_rampDownRate  3.0;

#define IDLETEMP        50 // the temperature at which to consider the oven safe to leave to cool naturally


// -------------------- Control
#define MODULATION_SWITCH_PERIOD 1000 //ms
#define MODULATION_WINDOWSIZE  100    //in ms
                                      // Zero Crossing (and so switching current on/off by ssr with zero cross detection) ist 100Hz=10ms in 50Hz AC net.
                                      // if window is 100ms and on-time any between 0...100ms the max resolution in switching is 10. thats way enough for heaters
#define SIZE_OF_AVG 16    // over how many values the temperature is averaged

//conservative PID parameters
#define HEATER_CONSERVATIVE_Kp     4.4//2.2//4.4
#define HEATER_CONSERVATIVE_Ki    0.08//0.03//0.12
#define HEATER_CONSERVATIVE_Kd   30//20.0//39.6

//aggressive PID parameters (switched automatically to, if ramps are driven)
#define THRESHOLD_RETURN_TO_CONSERVATIVE_PID_PARAMETERS 20    //degrees, return to conservative parameters if X before target
#define HEATER_RAMPS_Kp 17.0
#define HEATER_RAMPS_Ki  0.0
#define HEATER_RAMPS_Kd 20.0

// -------------------- Actuators, Heater, Fans
#define HEATING_PIN     6		//SSR1
#define FAN_PIN         5		//SSR2
#define BUZZER_PIN		  7
#define BUZZER_DEFAULT	1		//1 on 0 off

// -------------------- Sensors, Buttons, Encoders
#define TEMP0_ADC 0			//Analog ADC0
#define TEMP1_ADC 1			//Analog ADC1

#define STOP_SWITCH_PIN     0
#define ENCODER_A_PIN       A8  // if using encoderIn.h as input pay attention on following advice:
#define ENCODER_B_PIN       A9  // EncoderA/B-Pins have to feature PCINT. Thats true for 2/3 on UNO and A8/A9 on MEGA2560
#define ENCODER_BUTTON_PIN  4   // alternatively use ClickEncoderIn.h


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
#define EEPROM_OFFSET_FAN_SPEED			NUMBER_OF_STORED_PROFILES*AMOUNT_EEPROM_PER_PROFILE+ 1 // one byte wide
#define EEPROM_OFFSET_PROFILE_NUMBER    NUMBER_OF_STORED_PROFILES*AMOUNT_EEPROM_PER_PROFILE+ 2 // one byte wide
#define EEPROM_OFFSET_SETTINGS			NUMBER_OF_STORED_PROFILES*AMOUNT_EEPROM_PER_PROFILE+ 3 // multiple bytes wide

