

//#define DEBUG

// -------------------- Control
#define WindowSize 100

#define HEATER_Kp   4.00
#define HEATER_Ki   0.05
#define HEATER_Kd   2.00
#define FAN_Kp      1.00
#define FAN_Ki      0.03
#define FAN_Kd      10.0


// -------------------- Actuators, Heater, Fans
#define fanOutPin 8
#define heaterOutPin 9

#define idleTemp 50 // the temperature at which to consider the oven safe to leave to cool naturally
#define FAN_DEFAULT_SPEED 33



// -------------------- Sensors, Buttons, Encoders
#define TEMP0_ADC 0
#define TEMP1_ADC 1
#define TEMP2_ADC 2

#define stopKeyInputPin 7


// -------------------- LCD


// -------------------- EEPROM
#define offsetFanSpeed 481 // 30 * 16 + 1 one byte wide
#define offsetProfileNum 482 // 30 * 16 + 2 one byte wide

