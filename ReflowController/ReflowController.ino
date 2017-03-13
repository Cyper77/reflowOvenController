/*
  ESTechnical Reflow Oven Controller

  Ed Simmons 2012-2015
  Michael Groene 2017

  bases on v2.7 (checkout XXX) of ESTechnical

  Changes
  -------
  v0.2
    - Optimized for Arduino Uno
    - UI via Encoder and Pushbutton
    - use FilterAverage-Lib
    - Use 100k NTC as temperature sensor (standard in 3D-printing)
    - Safety-Check for illegal temperature reading
    - RampRate math fixed
  BUGS/TODOS:
    - PID-Autotune (TODO)
    - BEEPER (TODO)
    - adapt output and switching freq to the zero crossing freq (100Hz), http://playground.arduino.cc/Code/PIDLibraryRelayOutputExample

  http://www.estechnical.co.uk
  http://www.estechnical.co.uk/reflow-controllers/t962a-reflow-oven-controller-upgrade
  http://www.estechnical.co.uk/reflow-ovens/estechnical-reflow-oven
*/

#include <Arduino.h>
#include "config.h"

String ver = "base2.7_v0.2"; // bump minor version number on small changes, major on large changes, eg when eeprom layout changes

// ------------------- some declarations/prototypes
void updateDisplay(boolean fullUpdate=false);

#include "temperature.h"
temperatureSensorClass therm0;
temperatureSensorClass therm1;

// data type for the values used in the reflow profile
struct profileValues {
  int soakTemp;
  int soakDuration;
  int peakTemp;
  int peakDuration;
  double rampUpRate;
  double rampDownRate;
} activeProfile;
int profileNumber = 0;

int fanAssistSpeed = FAN_DEFAULT_SPEED; // default fan speed
int dummyVar;

#include <EEPROM.h>
#include <PID_v1.h>

#include <menu.h>
#include <menuIO/liquidCrystalOut.h>
#include <menuIO/serialOut.h>
#include <menuIO/encoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
using namespace Menu;

boolean menuSuspended=true;
LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D0, LCD_D1, LCD_D2, LCD_D3);


encoderIn<ENCODER_A_PIN, ENCODER_B_PIN> encoder;                      //simple quad encoder driver
encoderInStream<ENCODER_A_PIN, ENCODER_B_PIN> encStream(encoder, 4);  //simple quad encoder fake Stream

//a keyboard with only one key as the encoder button
keyMap encBtn_map[] = {{ -ENCODER_BUTTON_PIN, options->getCmdChar(enterCmd)}}; //negative pin numbers use internal pull-up, this is on when low
keyIn<1> encButton(encBtn_map);//1 is the number of keys

//input from the encoder + encoder button + serial
Stream* inputsList[] = {&encStream, &encButton, &Serial};
chainStream<3> in(inputsList);//3 is the number of inputs

result showEvent(eventMask e, navNode& nav, prompt& item) {
  Serial.print("event: ");
  Serial.println(e);
  return proceed;
}

// ----------- MENU setup
MENU(menuEditProfile, "Edit current Profile", showEvent, anyEvent, noStyle
     , EXIT(" ^-")
     , FIELD(dummyVar, "Ramp up", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
     , FIELD(dummyVar, "Soak Temp", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
     , FIELD(dummyVar, "Soak Time", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
     , FIELD(dummyVar, "Peak Temp", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
     , FIELD(dummyVar, "Peak Time", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
     , FIELD(dummyVar, "Ramp Down", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
    );

MENU(menuSettings, "Settings", showEvent, anyEvent, noStyle
     , EXIT(" ^-")
     , FIELD(dummyVar, "Heater kP", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
     , FIELD(dummyVar, "Heater kI", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
     , FIELD(dummyVar, "Heater kD", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
     , OP("AutoTune", showEvent, anyEvent)
     , OP("Buzzer On/Off", showEvent, anyEvent)
    );

MENU(menuManualMode, "Manual Control", showEvent, anyEvent, noStyle
     , EXIT(" ^-")
     , OP("All Off", showEvent, enterEvent )
     , OP("Set Temperature", showEvent, anyEvent)
     , OP("Set SSR1 Duty", showEvent, anyEvent)
     , OP("SSR2 On/Off", showEvent, anyEvent)
     , OP("SSR3 On/Off", showEvent, anyEvent)
    );

MENU(menuFactoryReset, "Factory Reset", showEvent, anyEvent, noStyle
     , EXIT(" ^-")
     , EXIT("No")
     , OP("Yes", showEvent, anyEvent)
     , EXIT("No")
    );

MENU(mainMenu, "Main menu", doNothing, noEvent, wrapStyle
     , EXIT(" ^-Status Screen")
     , OP("Start Profile", cycleStart, enterEvent )
     , SUBMENU(menuEditProfile)
     , OP("Save Profile", showEvent, enterEvent )
     , OP("Load Profile", showEvent, enterEvent )
     , SUBMENU(menuSettings)
     , SUBMENU(menuManualMode)
     , SUBMENU(menuFactoryReset)
    );

#define MAX_DEPTH 2
MENU_OUTPUTS(out, MAX_DEPTH
             , LIQUIDCRYSTAL_OUT(lcd, {0, 0, LCD_COLS, LCD_ROWS})
             , NONE
            );
NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);     //the navigation root object

result idle(menuOut& o, idleEvent e) {
  switch (e) {
    case idleStart: /* suspending menu */ break;
    case idling: /* suspended... */ menuSuspended=true; updateDisplay(true); break;
    case idleEnd: /* resuming menu */ menuSuspended=false; break;
  }
  return proceed;
}

// timer related
unsigned long windowStartTime;
unsigned long startTime, stateChangedTime = 0, lastUpdate = 0, lastDisplayUpdate = 0, lastSerialOutput = 0; // a handful of timer variables

// PID variables
double controlSetpoint, controlInput, controlOutput;

//Define the PID tuning parameters
double   heaterKp = HEATER_Kp,	heaterKi = HEATER_Ki,    heaterKd = HEATER_Kd;
double		fanKp = FAN_Kp,		   fanKi = FAN_Ki,			fanKd = FAN_Kd;

//Specify the links and initial tuning parameters
PID PID(&controlInput, &controlOutput, &controlSetpoint, heaterKp, heaterKi, heaterKd, DIRECT);
unsigned int fanValue, heaterValue;


// state machine bits
enum state {
  sIDLE,
  sRAMPTOSOAK,
  sSOAK,
  sRAMPTOPEAK,
  sPEAK,
  sRAMPDOWM,
  sCOOLDOWN
} currentState = sIDLE, lastState = sIDLE;
boolean stateChanged = false;

double rampRate = 0;				//for calculated ramp rate
boolean lastStopPinState = true;	//for debouncing

void abortWithError(int error) {
  // set outputs off for safety.
  digitalWrite(HEATING_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);

  lcd.clear();

  Serial.print(F("abortWithError No: "));
  Serial.println(error);

  switch (error) {
    case 1:
      lcd.print(F("Temperature"));
      lcd.setCursor(0, 1);
      lcd.print(F("following error"));
      lcd.setCursor(0, 2);
      lcd.print(F("during heating"));
      break;
    case 3:
      lcd.print(F("Thermoelement input"));
      lcd.setCursor(0, 1);
      lcd.print(F("open circuit"));
      lcd.setCursor(0, 2);
      lcd.print(F("Power off &"));
      lcd.setCursor(0, 3);
      lcd.print(F("check connections"));
      break;
  }
  while (1) {
    /* and stop forever... */
  }

}

void displayTemperature(double val) {
  char tempStr[7];
  dtostrf(val, 5, 1, tempStr);
  displayPaddedString(tempStr, 5);
  lcd.print((char)223);// degrees symbol!
  lcd.print(F("C"));
}

void displayOutputPower(unsigned int val) {
  if (val <= 99) lcd.print(" ");
  if (val <= 9) lcd.print(" ");
  lcd.print(val);
  lcd.print(F("%"));
}

void displayRampRate(double val) {
  char tempStr[6];
  dtostrf(val, 4, 1, tempStr);
  lcd.write((uint8_t)0); // delta symbol
  lcd.print(F("Temp "));
  displayPaddedString(tempStr, 5);
  lcd.print((char)223);// degrees symbol!
  lcd.print(F("C/S"));
}

// prints a string of the defined length
// any space left after the string has been added to the string to print is filled with spaces
void displayPaddedString(char *str, uint8_t length) {
  uint8_t srcLength = strlen(str);
  uint8_t spaces = length - srcLength;
  lcd.print(str);
  for (uint8_t i = 0; i < spaces; i++) {
    lcd.print(" ");
  }
}

void displayState() {
  switch (currentState) {
    case sIDLE:
      displayPaddedString(("Idle"), 9);
      break;
    case sRAMPTOSOAK:
      displayPaddedString(("Ramp Up"), 9);
      break;
    case sSOAK:
      displayPaddedString(("Soak"), 9);
      break;
    case sRAMPTOPEAK:
      displayPaddedString(("Ramp Up"), 9);
      break;
    case sPEAK:
      displayPaddedString(("Peak"), 9);
      break;
    case sRAMPDOWM:
      displayPaddedString(("Ramp Down"), 9);
      break;
    case sCOOLDOWN:
      displayPaddedString(("Cool Down"), 9);
      break;
  }
}

void displayCycleDuration() {
	char buf[5];
	if(currentState!=sIDLE) {
		sprintf(buf, "%ds", (millis() - startTime) / 1000);
		lcd.print(buf);
	} else {
		lcd.print("--");
	}
}

void updateDisplay(boolean fullUpdate=false) {
  if (fullUpdate) {
    lcd.clear();

    // draw all the static bits now
    lcd.setCursor(10, 1);
    lcd.print(F("Sp"));
  
    lcd.setCursor(0, 2);
    lcd.print(F("Heat"));
  
    lcd.setCursor(10, 2);
    lcd.print(F("Fan"));
  }
  
  lcd.setCursor(0, 0);
  displayTemperature(therm0.getTemperature());

  lcd.setCursor(8, 0);
  if (therm1.getStatus() == 0) {
    displayTemperature(therm1.getTemperature());
  } else {
    lcd.print(F(" ---"));
  }

  lcd.setCursor(16, 0);
  displayCycleDuration();

  lcd.setCursor(0, 1);
  displayState();

  lcd.setCursor(12, 1);
  displayTemperature(controlSetpoint);

  lcd.setCursor(5, 2);
  displayOutputPower((unsigned int)heaterValue);

  lcd.setCursor(14, 2);
  displayOutputPower((unsigned int)fanValue);

  lcd.setCursor(0, 3);
  displayRampRate(rampRate);
}

void setup() {
  // ---------------- setup IO
  pinMode(STOP_SWITCH_PIN, INPUT);
  digitalWrite(STOP_SWITCH_PIN, HIGH);

  /* ENC_A and B are managed by encoder-lib */
  pinMode(ENCODER_BUTTON_PIN, INPUT);
  digitalWrite(ENCODER_BUTTON_PIN, HIGH);

  pinMode(FAN_PIN, OUTPUT);
  pinMode(HEATING_PIN, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  while (!Serial);
  Serial.println(F("Reflow Controller starting...")); Serial.flush();

  // ---------------- Setup menu
  encoder.begin();
  lcd.begin(LCD_COLS, LCD_ROWS);
  nav.idleTask = idle;            //point a function to be used when menu is suspended
  /*mainMenu[1].enabled=disabledStatus; */
  nav.showTitle = false;
  byte deltaChar[8] = {
    0b00000,
    0b00100,
    0b00100,
    0b01010,
    0b01010,
    0b10001,
    0b11111,
    0b00000
  };
  lcd.createChar(0, deltaChar);
  lcd.clear();

  //detect first-run
  if (firstRun()) {
    factoryReset();
    loadParameters(0);
  } else {
    loadLastUsedProfile();
  }

  loadFanSpeed();

  //init temp-measurement and set averaging filter to first measured value
  therm0.setup(TEMP0_ADC);
  therm1.setup(TEMP1_ADC);

  // ---------------- PID setup
  PID.SetOutputLimits(0, WindowSize);
  PID.SetSampleTime(100);

  //turn the PID on
  //PID.SetMode(AUTOMATIC);

  if (therm0.getStatus() != 0) {
    abortWithError(3);
  }

  startTime = millis();

  //the menu will start on idle state, press select to enter menu
  nav.idleOn(idle);
}


void loop() {

  // every 100ms
  if (millis() - lastUpdate >= 100) {
    stateChanged = false;
    lastUpdate = millis();

    //read temperatures from ADC
    therm0.triggerTemperatureMeasurement();
    therm1.triggerTemperatureMeasurement();

    if (therm0.getStatus() != 0) {
      abortWithError(3);
    }

    rampRate = (therm0.oAverageFilter.getLeastAddedValue() - therm0.oAverageFilter.getOldestAddedValue()) * 10 / (SIZE_OF_AVG - 1); // subtract earliest reading from the current one
    // this gives us the rate of rise in degrees per second

    controlInput = therm0.getTemperature(); // update the variable the PID reads
    //Serial.print("Temp1= ");
    //Serial.println(readings[index]);

    // if the state has changed, set the flags and update the time of state change
    if (currentState != lastState) {
      lastState = currentState;
      stateChanged = true;
      stateChangedTime = millis();
    }

    //do display-work
    nav.poll();

    //status screen is shown if menu is in suspended mode
    if(menuSuspended==true) {
      if (millis() - lastDisplayUpdate > 250) { // 4hz display during reflow cycle
        lastDisplayUpdate = millis();
        updateDisplay();
      }
    }
    
    // check for the stop key being pressed
    boolean stopPin = digitalRead(STOP_SWITCH_PIN); // check the state of the stop key
    if (stopPin == LOW && lastStopPinState != stopPin) { // if the state has just changed
      //debounced keypress
      if (currentState == sCOOLDOWN) {
        currentState = sIDLE;
      } else if (currentState != sIDLE) {
        currentState = sCOOLDOWN;
      }
    }
    lastStopPinState = stopPin;


    // ----------------- run the state machine
    switch (currentState) {
      case sIDLE:
        //Serial.println("Idle");
        break;

      case sRAMPTOSOAK:
        //Serial.println("ramp");
        if (stateChanged) {
          PID.SetMode(MANUAL);
          controlOutput = 80;
          PID.SetMode(AUTOMATIC);
          PID.SetControllerDirection(DIRECT);
          PID.SetTunings(heaterKp, heaterKi, heaterKd);
          controlSetpoint = therm0.getTemperature();    //start at current measured temeprature in the moment of changing state
          stateChanged = false;
        }
        controlSetpoint += (activeProfile.rampUpRate / 10); // target set ramp up rate

        if (controlSetpoint >= activeProfile.soakTemp - 1) {
          currentState = sSOAK;
        }
        break;

      case sSOAK:
        if (stateChanged) {
          controlSetpoint = activeProfile.soakTemp;
          stateChanged = false;
        }
        if (millis() - stateChangedTime >= (unsigned long) activeProfile.soakDuration * 1000) {
          currentState = sRAMPTOPEAK;
        }
        break;

      case sRAMPTOPEAK:
        if (stateChanged) {
          stateChanged = false;
        }

        controlSetpoint += (activeProfile.rampUpRate / 10); // target set ramp up rate

        if (controlSetpoint >= activeProfile.peakTemp - 1) { // seems to take arodun 8 degrees rise to tail off to 0 rise
          controlSetpoint = activeProfile.peakTemp;
          currentState = sPEAK;
        }
        break;

      case sPEAK:
        if (stateChanged) {
          controlSetpoint = activeProfile.peakTemp;
          stateChanged = false;
        }

        if (millis() - stateChangedTime >= (unsigned long) activeProfile.peakDuration * 1000) {
          currentState = sRAMPDOWM;
        }
        break;

      case sRAMPDOWM:
        if (stateChanged) {
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanKp, fanKi, fanKd);
          stateChanged = false;
          controlSetpoint = activeProfile.peakTemp - 15; // get it all going with a bit of a kick! v sluggish here otherwise, too hot too long
        }

        controlSetpoint -= (activeProfile.rampDownRate / 10);

        if (controlSetpoint <= idleTemp) {
          currentState = sCOOLDOWN;
        }
        break;

      case sCOOLDOWN:
        if (stateChanged) {
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanKp, fanKi, fanKd);
          controlSetpoint = idleTemp;
        }
        if (controlInput < (idleTemp + 5)) {
          currentState = sIDLE;
          PID.SetMode(MANUAL);
          controlOutput = 0;
        }
        break;
    }
  }

  // safety check that we're not doing something stupid.
  // if the thermocouple is wired backwards, temp goes DOWN when it increases
  // during cooling, the t962a lags a long way behind, hence the hugely lenient cooling allowance.

  // both of these errors are blocking and do not exit!
  if (controlSetpoint > controlInput + 50) abortWithError(1); // if we're 50 degree cooler than setpoint, abort
  //if(Input > Setpoint + 50) abortWithError(2);// or 50 degrees hotter, also abort

  PID.Compute();

  if (currentState == sIDLE) {
    //all off in idle mode
    heaterValue = 0;
    fanValue = 0;
  } else if (currentState == sRAMPDOWM || currentState == sCOOLDOWN) {
    // in cooling phases turn heater off hard and control fan instead
    heaterValue = 0;
    fanValue = controlOutput;
  } else {
    // other phases are non-idle or heating-phases
    // heater is controlled by PID and fan is on assisting speed
    heaterValue = controlOutput;
    fanValue = fanAssistSpeed;
  }

  if ( (millis() - windowStartTime) > WindowSize) {
    //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  if ( heaterValue < (millis() - windowStartTime) ) {
    digitalWrite(HEATING_PIN, LOW);
  } else {
    digitalWrite(HEATING_PIN, HIGH);
  }

  if (    fanValue < (millis() - windowStartTime) ) {
    digitalWrite(FAN_PIN, LOW);
  } else {
    digitalWrite(FAN_PIN, HIGH);
  }


  //after everything is done... every 250ms
  if ( (millis() - lastSerialOutput) > 250) {
    lastSerialOutput = millis();
    sendSerialUpdate();
  }

}


//start a cycle
void cycleStart() {

  currentState = sRAMPTOSOAK;
  
  lcd.clear();
  lcd.print("Starting cycle ");
  lcd.print(profileNumber);
  delay(1000);

  //time the cycle started
  startTime = millis();
  
  //suspend menu go to status screen
  nav.idleOn(idle);
}

void saveProfile(unsigned int targetProfile) {
  profileNumber = targetProfile;
  lcd.clear();
  lcd.print("Saving profile ");
  lcd.print(profileNumber);

#ifdef DEBUG

  Serial.println("Check parameters:");
  Serial.print("idleTemp ");
  Serial.println(idleTemp);
  Serial.print("ramp Up rate ");
  Serial.println(activeProfile.rampUpRate);
  Serial.print("soakTemp ");
  Serial.println(activeProfile.soakTemp);
  Serial.print("soakDuration ");
  Serial.println(activeProfile.soakDuration);
  Serial.print("peakTemp ");
  Serial.println(activeProfile.peakTemp);
  Serial.print("peakDuration ");
  Serial.println(activeProfile.peakDuration);
  Serial.print("rampDownRate ");
  Serial.println(activeProfile.rampDownRate);
  Serial.println("About to save parameters");
#endif

  saveParameters(profileNumber); // profileNumber is modified by the menu code directly, this method is called by a menu action

  delay(500);
}

void loadProfile(unsigned int targetProfile) {
  // We may be able to do-away with profileNumber entirely now the selection is done in-function.
  profileNumber = targetProfile;
  lcd.clear();
  lcd.print("Loading profile ");
  lcd.print(profileNumber);
  saveLastUsedProfile();

#ifdef DEBUG

  Serial.println("Check parameters:");
  Serial.print("idleTemp ");
  Serial.println(idleTemp);
  Serial.print("ramp Up rate ");
  Serial.println(activeProfile.rampUpRate);
  Serial.print("soakTemp ");
  Serial.println(activeProfile.soakTemp);
  Serial.print("soakDuration ");
  Serial.println(activeProfile.soakDuration);
  Serial.print("peakTemp ");
  Serial.println(activeProfile.peakTemp);
  Serial.print("peakDuration ");
  Serial.println(activeProfile.peakDuration);
  Serial.print("rampDownRate ");
  Serial.println(activeProfile.rampDownRate);
  Serial.println("About to load parameters");
#endif

  loadParameters(profileNumber);

#ifdef DEBUG

  Serial.println("Check parameters:");
  Serial.print("idleTemp ");
  Serial.println(idleTemp);
  Serial.print("ramp Up rate ");
  Serial.println(activeProfile.rampUpRate);
  Serial.print("soakTemp ");
  Serial.println(activeProfile.soakTemp);
  Serial.print("soakDuration ");
  Serial.println(activeProfile.soakDuration);
  Serial.print("peakTemp ");
  Serial.println(activeProfile.peakTemp);
  Serial.print("peakDuration ");
  Serial.println(activeProfile.peakDuration);
  Serial.print("rampDownRate ");
  Serial.println(activeProfile.rampDownRate);
  Serial.println("after loading parameters");
#endif

  delay(500);
}


void saveParameters(unsigned int profile) {

  unsigned int offset = 0;
  if (profile != 0) offset = profile * 16;


  EEPROM.write(offset++, lowByte(activeProfile.soakTemp));
  EEPROM.write(offset++, highByte(activeProfile.soakTemp));

  EEPROM.write(offset++, lowByte(activeProfile.soakDuration));
  EEPROM.write(offset++, highByte(activeProfile.soakDuration));

  EEPROM.write(offset++, lowByte(activeProfile.peakTemp));
  EEPROM.write(offset++, highByte(activeProfile.peakTemp));

  EEPROM.write(offset++, lowByte(activeProfile.peakDuration));
  EEPROM.write(offset++, highByte(activeProfile.peakDuration));

  int temp = activeProfile.rampUpRate * 10;
  EEPROM.write(offset++, (temp & 255));
  EEPROM.write(offset++, (temp >> 8) & 255);

  temp = activeProfile.rampDownRate * 10;
  EEPROM.write(offset++, (temp & 255));
  EEPROM.write(offset++, (temp >> 8) & 255);

}

void loadParameters(unsigned int profile) {
  unsigned int offset = 0;
  if (profile != 0) offset = profile * 16;

  activeProfile.soakTemp = EEPROM.read(offset++);
  activeProfile.soakTemp |= EEPROM.read(offset++) << 8;

  activeProfile.soakDuration = EEPROM.read(offset++);
  activeProfile.soakDuration |= EEPROM.read(offset++) << 8;

  activeProfile.peakTemp = EEPROM.read(offset++);
  activeProfile.peakTemp |= EEPROM.read(offset++) << 8;

  activeProfile.peakDuration = EEPROM.read(offset++);
  activeProfile.peakDuration |= EEPROM.read(offset++) << 8;

  int temp = EEPROM.read(offset++);
  temp |= EEPROM.read(offset++) << 8;
  activeProfile.rampUpRate = ((double)temp / 10);

  temp = EEPROM.read(offset++);
  temp |= EEPROM.read(offset++) << 8;
  activeProfile.rampDownRate = ((double)temp / 10);

}


boolean firstRun() { // we check the whole of the space of the 16th profile, if all bytes are 255, we are doing the very first run
  unsigned int offset = 16;
  for (unsigned int i = offset * 15; i < (offset * 15) + 16; i++) {
    if (EEPROM.read(i) != 255) return false;
  }
  lcd.clear();
  lcd.print(F("First run..."));
  delay(500);
  return true;
}

void factoryReset() {
  // clear any adjusted settings first, just to be sure...
  activeProfile.soakTemp      = PROFILE_DEFAULT_soakTemp;
  activeProfile.soakDuration  = PROFILE_DEFAULT_soakDuration;
  activeProfile.peakTemp      = PROFILE_DEFAULT_peakTemp;
  activeProfile.peakDuration  = PROFILE_DEFAULT_peakDuration;
  activeProfile.rampUpRate    = PROFILE_DEFAULT_rampUpRate;
  activeProfile.rampDownRate  = PROFILE_DEFAULT_rampDownRate;
  lcd.clear();
  lcd.print("Resetting...");

  // then save the same profile settings into all slots
  for (int i = 0; i < 30; i++) {
    saveParameters(i);
  }
  fanAssistSpeed = FAN_DEFAULT_SPEED;
  saveFanSpeed();
  profileNumber = 0;
  saveLastUsedProfile();

  delay(100);
}

void saveFanSpeed() {
  unsigned int temp = (unsigned int) fanAssistSpeed;
  EEPROM.write(offsetFanSpeed, (temp & 255));
  //Serial.print("Saving fan speed :");
  //Serial.println(temp);
  lcd.clear();
  lcd.print("Saving...");
  delay(250);

}

void loadFanSpeed() {
  unsigned int temp = 0;
  temp = EEPROM.read(offsetFanSpeed);
  fanAssistSpeed = (int) temp;
  //Serial.print("Loaded fan speed :");
  //Serial.println(fanAssistSpeed);
}

void saveLastUsedProfile() {
  unsigned int temp = (unsigned int) profileNumber;
  EEPROM.write(offsetProfileNum, (temp & 255));
  //Serial.print("Saving active profile number :");
  //Serial.println(temp);

}

void loadLastUsedProfile() {
  unsigned int temp = 0;
  temp = EEPROM.read(offsetProfileNum);
  profileNumber = (int) temp;
  //Serial.print("Loaded last used profile number :");
  //Serial.println(temp);
  loadParameters(profileNumber);
}

void sendSerialUpdate()
{

  if (currentState == sIDLE) {
    Serial.print(F("0,0,0,0,0,"));
    Serial.print(therm0.getTemperature());
    Serial.print(",");
    if (therm1.getStatus() == 0) {
      Serial.print(therm1.getTemperature());
    } else {
      Serial.print("999");
    }
    Serial.print(",");
    Serial.print(rampRate);
    Serial.println();
  } else {

    Serial.print(millis() - startTime);
    Serial.print(",");
    Serial.print((int)currentState);
    Serial.print(",");
    Serial.print(controlSetpoint);
    Serial.print(",");
    Serial.print(heaterValue);
    Serial.print(",");
    Serial.print(fanValue);
    Serial.print(",");
    Serial.print(therm0.getTemperature());
    Serial.print(",");
    if (therm1.getStatus() == 0) {
      Serial.print(therm1.getTemperature());
    } else {
      Serial.print("999");
    }
    Serial.print(",");
    Serial.print(rampRate);
    Serial.println();
  }
}
  