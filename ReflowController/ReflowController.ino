/*
Reflow Oven Controller

Ed Simmons 2012-2015
Michael Groene 2017

based on v2.7 (checkout XXX) of ESTechnical

Changes
-------
v0.3
- Suited for Arduinos with 64kb and above flash memory (e.g. Arduino Mega)
- UI via Encoder and Pushbutton
- use FilterAverage-Lib
- Use 100k NTC as temperature sensor (standard in 3D-printing)
- Safety-Check for illegal temperature reading
- RampRate via linear regression
- BEEPER
- PID-Autotune (doesnt work currently)
- EEPROMex with Double, etc.
*/

#include <Arduino.h>
#include "config.h"

String ver = "base2.7_v0.3"; // bump minor version number on small changes, major on large changes, eg when eeprom layout changes

#include "temperature.h"
temperatureSensorClass therm0;
temperatureSensorClass therm1;

// data type for the values used in the reflow profile
struct profileValues {
	uint16_t soakTemp;
	uint16_t soakDuration;
	uint16_t peakTemp;
	uint16_t peakDuration;
	double rampUpRate;
	double rampDownRate;
} activeProfile;
uint8_t profileNumber = 0;

#include <EEPROMex.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// PID variables
double controlSetpoint, controlInput, controlOutput;
struct autotuneParameter {
	uint16_t targetTemperature=220;
	uint16_t startOutput=80;
	double noiseBand=1;
	double tuneStep=1;
} autotune;
//Define the PID tuning parameters
struct PIDparameter {
	double kP;
	double kI;
	double kD;
} heaterPID_conservative;

uint16_t manualModeTemperatureSetpoint=0;
uint16_t manualModeSSR1Duty=0;

boolean buzzerOn=BUZZER_DEFAULT;

// state machine bits
enum state {
	sIDLE,
	sRAMPTOSOAK,
	sSOAK,
	sRAMPTOPEAK,
	sPEAK,
	sRAMPDOWM,
	sCOOLDOWN,
	sAUTOTUNE,
	sMANUAL_TEMPCONTROLLED,
	sMANUAL_DUTYFIXED,
} currentState = sIDLE, lastState = sIDLE;
boolean stateChanged = false;

#include <menu.h>
#include <menuIO/liquidCrystalOut.h>
#if ENABLE_SERIAL_MENU == 1
	#include <menuIO/serialOut.h>
#endif
#include <menuIO/encoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
using namespace Menu;


// ------------------- some declarations/prototypes for menu
void updateDisplay(boolean fullUpdate=false);
void cycleStart(void);
void autotuneStart(void);
void changeProfile(eventMask e);
void saveProfile(eventMask e);
void saveSettings();
void allOff();
void manualModeDutyFixedStart();
void manualModeTempControlledStart();
void factoryReset();

boolean menuSuspended=true;
//config menuOptions('>','-',false,false,defaultNavCodes,true);
LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D0, LCD_D1, LCD_D2, LCD_D3);

encoderIn<ENCODER_A_PIN, ENCODER_B_PIN> encoder;                      //simple quad encoder driver
encoderInStream<ENCODER_A_PIN, ENCODER_B_PIN> encStream(encoder, 4);  //simple quad encoder fake Stream

//a keyboard with only one key as the encoder button
keyMap encBtn_map[] = {{ -ENCODER_BUTTON_PIN, options->getCmdChar(enterCmd)}}; //negative pin numbers use internal pull-up, this is on when low
keyIn<1> encButton(encBtn_map);         //1 is the number of keys


//todo: use instead disabling the start cycle menu: https://github.com/neu-rah/ArduinoMenu/issues/15
//mainMenu.data[2]->text="Some new text";//change third option text
//mainMenu.redraw(lcd,allIn);

#if ENABLE_SERIAL_MENU > 0
	//input from the encoder + encoder button + serial
	Stream* inputsList[] = {&encStream, &encButton, &Serial};
	chainStream<3> in(inputsList);        //3 is the number of inputs
#else
	//input from the encoder + encoder button
	Stream* inputsList[] = {&encStream, &encButton};
	chainStream<2> in(inputsList);        //2 is the number of inputs
#endif


// ----------- MENU setup
MENU(menuEditProfile, "Edit Profile", doNothing, noEvent, noStyle
	, OP(" ^.. & save", saveProfile, enterEvent )
	, FIELD(activeProfile.rampUpRate,    "  Up rate",     " C/s", 0.1,      5,   0.1, 0,  doNothing, noEvent, noStyle)
	, FIELD(activeProfile.soakTemp,      "Soak temp",       " C",    50,    180,   5, 1,    doNothing, noEvent, noStyle)
	, FIELD(activeProfile.soakDuration,  "Soak time",       " s",    10,    300,   5, 1,    doNothing, noEvent, noStyle)
	, FIELD(activeProfile.peakTemp,      "Peak temp",       " C",   100,    300,   5, 1,    doNothing, noEvent, noStyle)
	, FIELD(activeProfile.peakDuration,  "Peak time",       " s",     5,    120,   5, 1,    doNothing, noEvent, noStyle)
	, FIELD(activeProfile.rampDownRate,  "Down rate",     " C/s", 0.1,       10, 0.1, 0,  doNothing, noEvent, noStyle)
);

TOGGLE(buzzerOn,buzzerCtrl,"Buzzer: ",doNothing,noEvent,noStyle
	,VALUE("On",true,doNothing,noEvent)
	,VALUE("Off",false,doNothing,noEvent)
);

MENU(menuSettings, "Settings", doNothing, noEvent, noStyle
	, OP(" ^.. & save", saveSettings, enterEvent )
	, FIELD(heaterPID_conservative.kP, "Heater kP", "", 0, 100, 0.1, 0.01, doNothing, noEvent, noStyle)
	, FIELD(heaterPID_conservative.kI, "Heater kI", "", 0, 100, 0.1, 0.01, doNothing, noEvent, noStyle)
	, FIELD(heaterPID_conservative.kD, "Heater kD", "", 0, 100, 0.1, 0.01, doNothing, noEvent, noStyle)
	, SUBMENU(buzzerCtrl)
);

MENU(menuManualMode, "Manual Control", doNothing, anyEvent, noStyle
	, EXIT(" ^-")
	, OP("All Off", allOff, enterEvent )
	, FIELD(manualModeTemperatureSetpoint,      "Set Temp.",       " C",   50,    300,   5, 1,    manualModeTempControlledStart, exitEvent, noStyle)
	, FIELD(manualModeSSR1Duty,      "Set SSR1 Duty",       " %",   0,    1000,   20, 5,    manualModeDutyFixedStart, exitEvent, noStyle)
	//, OP("SSR2 On/Off", doNothing, enterEvent)
);

MENU(menuAutotune, "PID Autotune", doNothing, anyEvent, noStyle
	, EXIT(" ^-")
	, FIELD(autotune.targetTemperature,    "Target Temp.",     " C", 0,      280,   5, 1,  doNothing, noEvent, noStyle)
	, FIELD(autotune.startOutput,    "Start Output",     " %", 0,      100,   5, 1,  doNothing, noEvent, noStyle)
	, FIELD(autotune.noiseBand,    "Noiseband",     " C", 0.1,      10,   0.1, 0,  doNothing, noEvent, noStyle)
	, FIELD(autotune.tuneStep,    "Tunestep",     "", 0.1,      100,   1, 0.1,  doNothing, noEvent, noStyle)
	, OP("Start Autotune", autotuneStart, enterEvent)
);

MENU(menuFactoryReset, "Factory Reset", doNothing, noEvent, noStyle
	, EXIT(" ^-")
	, EXIT("No")
	, OP("Yes", factoryReset, enterEvent)
	, EXIT("No")
);


MENU(mainMenu, "Main menu", doNothing, noEvent, noStyle
	, EXIT(" ^-Status Screen")
	, OP("Start Cycle", cycleStart, enterEvent )
	, FIELD(profileNumber,"Active Profile","",0,NUMBER_OF_STORED_PROFILES-1,1,0,changeProfile,exitEvent,noStyle)
	, SUBMENU(menuEditProfile)
	, SUBMENU(menuSettings)
	, SUBMENU(menuManualMode)
	, SUBMENU(menuAutotune)
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
		case idleStart:
		/* suspending menu */
		break;
		
		case idling:
		/* suspended... */
		menuSuspended=true;
		updateDisplay(true);
		
		break;
		
		case idleEnd:
		/* resuming menu */
		menuSuspended=false;
		if(currentState!=sIDLE) {
			// if resuming main-menu in state!=idle, disable the start-option
			mainMenu[1].disable();  //disable item no. 2 in mainMenu
			//nav.doNav(navCmd(idxCmd,0));
			//nav.idleOn(idle);
			//menuSuspended=true;
			//updateDisplay(true);
		} else {
			//enable start-cycle item again
			mainMenu[1].enable();
		}
		break;
	}
	return proceed;
}

// timer related
unsigned long windowStartTime;
unsigned long startTime, stateChangedTime = 0, lastUpdate = 0, lastDisplayUpdate = 0, lastSerialOutput = 0; // a handful of timer variables

//Specify the links and initial tuning parameters
PID heaterPID(&controlInput, &controlOutput, &controlSetpoint, heaterPID_conservative.kP, heaterPID_conservative.kI, heaterPID_conservative.kD, DIRECT);
PID_ATune HeaterPIDautotune(&controlInput, &controlOutput);
int heaterValue;

double rampRate = 0;				//for calculated ramp rate
boolean lastStopPinState = true;	//for debouncing

void allOff() {
	//this function should bring the oven in a safe operating mode
	controlSetpoint=0;
	
	//turn off everything
	digitalWrite(HEATING_PIN, LOW);
	digitalWrite(FAN_PIN, LOW);
	
	//turn autotuner off if it was running
	HeaterPIDautotune.Cancel();
	
}

void abortWithError(uint8_t error) {
	// set outputs off for safety.
	allOff();
	
	if(buzzerOn)
		tone(BUZZER_PIN,1760,1000);  //Error Beep
	
	lcd.clear();

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

void displayOutputPower(int val) {
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
			displayPaddedString(("Ramp Soak"), 9);
		break;
		case sSOAK:
			displayPaddedString(("Soak"), 9);
		break;
		case sRAMPTOPEAK:
			displayPaddedString(("Ramp Peak"), 9);
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
		case sAUTOTUNE:
			displayPaddedString(("Autotune"), 9);
		break;
		case sMANUAL_TEMPCONTROLLED:
			displayPaddedString(("man.TEMP!"), 9);
		break;
		case sMANUAL_DUTYFIXED:
			displayPaddedString(("man.DUTY!"), 9);
		break;
	}
}

void displayCycleDuration() {
	char buf[5];
	if(currentState!=sIDLE) {
		sprintf(buf, "%ds", (millis() - startTime) / 1000);
		lcd.print(buf);
	} else {
		lcd.print(F("    "));
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
	}
	
	lcd.setCursor(0, 0);
	displayTemperature(therm0.getTemperatureCelsius());

	lcd.setCursor(8, 0);
	if (therm1.getStatus() == 0) {
		displayTemperature(therm1.getTemperatureCelsius());
	} else {
		lcd.print(F("     "));
	}

	lcd.setCursor(16, 0);
	displayCycleDuration();

	lcd.setCursor(0, 1);
	displayState();

	lcd.setCursor(12, 1);
	displayTemperature(controlSetpoint);

	lcd.setCursor(5, 2);
	displayOutputPower(heaterValue);

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

	pinMode(HEATING_PIN, OUTPUT);
	//pinMode(FAN_PIN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);
	
	Serial.begin(SERIAL_BAUD);
	while (!Serial);
	Serial.println(F("Reflow Controller starting...")); Serial.flush();

	// ---------------- Setup menu
	//options=&menuOptions;
	encoder.begin();
	lcd.begin(LCD_COLS, LCD_ROWS);
	nav.idleTask = idle;            //point a function to be used when menu is suspended
	//nav.sleepTask = idle;
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
	if (firstRun())
		factoryReset();
	
	loadLastUsedProfile();
	loadSettings();
	
	//init temp-measurement and set averaging filter to first measured value
	therm0.setup(TEMP0_ADC);
	therm1.setup(TEMP1_ADC);

	// ---------------- PID setup
	heaterPID.SetMode(AUTOMATIC);
	heaterPID.SetOutputLimits(0, MODULATION_WINDOWSIZE);
	heaterPID.SetSampleTime(100);

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
		
		//check for error on therm0
		if (therm0.getStatus() != 0) {
			abortWithError(3);
		}
		
		// this gives us the rate of rise in degrees per second
		rampRate = therm0.getTemperatureRampPerDatastep() * 10; 
		
		// update the variable the PID reads
		controlInput = therm0.getTemperatureCelsius();

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

			allOff();
			
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
				
				if (stateChanged) {
					
					//enable menu item for next cycle again after cooldown
					mainMenu[1].enable();
					
					stateChanged=false;
				}
			
			break;

			case sRAMPTOSOAK:
				
				if (stateChanged) {
					if(buzzerOn)
						tone(BUZZER_PIN,1760,50);

					//aggressive ramp-mode
					heaterPID.SetTunings(HEATER_RAMPS_Kp, HEATER_RAMPS_Ki, HEATER_RAMPS_Kd);
					
					//start at current measured temperature in the moment of changing state
					controlSetpoint = therm0.getTemperatureCelsius();

					stateChanged = false;
				}
				
				// update controlSetpoint
				if(controlSetpoint<activeProfile.soakTemp) {
					//if below soakTemp adding ramp up rate
					controlSetpoint += (activeProfile.rampUpRate / 10); 
				} else {
					//stop adding ramp rate if soakTemp is set
					controlSetpoint = activeProfile.soakTemp;
				}
				
				//check whether to return to conservative heating parameters
				if (controlInput >= activeProfile.soakTemp - THRESHOLD_RETURN_TO_CONSERVATIVE_PID_PARAMETERS ) {
					heaterPID.SetTunings(heaterPID_conservative.kP,heaterPID_conservative.kI,heaterPID_conservative.kD);
				}
				
				//if soakTemp was hit once, switch to sSOAK
				if (controlInput >= activeProfile.soakTemp - 5) {
					currentState = sSOAK;
				}
				
			break;

			case sSOAK:
				if (stateChanged) {
					controlSetpoint = activeProfile.soakTemp;
					
					if(buzzerOn)
						tone(BUZZER_PIN,1760,50);
					
					stateChanged = false;
				}
				
				//this statechange is time-driven: wait until soakDuration is over
				if (millis() - stateChangedTime >= (unsigned long) activeProfile.soakDuration * 1000) {
					currentState = sRAMPTOPEAK;
				}
			break;

			case sRAMPTOPEAK:
				if (stateChanged) {
					if(buzzerOn)
						tone(BUZZER_PIN,1760,50);

					//aggressive ramp-mode
					heaterPID.SetTunings(HEATER_RAMPS_Kp, HEATER_RAMPS_Ki, HEATER_RAMPS_Kd);
				
					stateChanged = false;
				}
				
				// update controlSetpoint
				if(controlSetpoint<activeProfile.peakTemp) {
					//if below peakTemp adding ramp up rate
					controlSetpoint += (activeProfile.rampUpRate / 10);
				} else {
					//stop adding ramp rate if peakTemp is set
					controlSetpoint = activeProfile.peakTemp;
				}
				
				//return to conservative heating
				if (controlInput >= activeProfile.peakTemp - THRESHOLD_RETURN_TO_CONSERVATIVE_PID_PARAMETERS) {
					heaterPID.SetTunings(heaterPID_conservative.kP,heaterPID_conservative.kI,heaterPID_conservative.kD);
				}
			
				//statechange temperature-driven: if peakTemp was hit once, switch to sPEAK
				if (controlInput >= activeProfile.peakTemp - 5) {
					currentState = sPEAK;
				}
				
			break;

			case sPEAK:
				if (stateChanged) {
					
					controlSetpoint = activeProfile.peakTemp;
					
					if(buzzerOn)
						tone(BUZZER_PIN,1760,80);
					
					stateChanged = false;
				}
				
				//this statechange is time-driven: wait until peakDuration is over
				if (millis() - stateChangedTime >= (unsigned long) activeProfile.peakDuration * 1000) {
					currentState = sRAMPDOWM;
				}
			break;

			case sRAMPDOWM:
				
				if (stateChanged) {
					stateChanged = false;
					
					if(buzzerOn)
						tone(BUZZER_PIN,1760,250);
					
					controlSetpoint = activeProfile.peakTemp - 15; // get it all going with a bit of a kick! v sluggish here otherwise, too hot too long
				} else {
					//do some annoying buzzing, to get the user open the door
					if(buzzerOn)
						tone(BUZZER_PIN,1760,25);
				}
				
				controlSetpoint -= (activeProfile.rampDownRate / 10);
				
				if (controlSetpoint <= IDLETEMP) {
					currentState = sCOOLDOWN;
				}
			break;

			case sCOOLDOWN:
				if (stateChanged) {
					stateChanged = false;
					if(buzzerOn)
						tone(BUZZER_PIN,1760,50);
					controlSetpoint = IDLETEMP;
				}
				
				if (controlInput < (IDLETEMP + 5)) {
					currentState = sIDLE;
					if(buzzerOn)
						tone(BUZZER_PIN,1760,100);
					controlOutput = 0;
				}
			break;

			case sAUTOTUNE:
				if (stateChanged) {
					stateChanged = false;

					//init tuning
					controlSetpoint = autotune.targetTemperature;
					controlOutput=autotune.startOutput;
					HeaterPIDautotune.SetNoiseBand(autotune.noiseBand); //noise: how much noise to ignore. Try to make this as small as possible, while still preventing output chatter.
					HeaterPIDautotune.SetOutputStep(autotune.tuneStep);
					HeaterPIDautotune.SetLookbackSec((int)20);  //seconds: integer. think about how far apart the peaks are. 1/4-1/2 of this distance is a good value
					HeaterPIDautotune.SetControlType(1);        // 0: PI 1: PID
				
				}

			
				if (HeaterPIDautotune.Runtime() != 0) {
					//tuning finished: runtime returns 1
					currentState = sCOOLDOWN;

					heaterPID_conservative.kP = HeaterPIDautotune.GetKp();
					heaterPID_conservative.kI = HeaterPIDautotune.GetKi();
					heaterPID_conservative.kD = HeaterPIDautotune.GetKd();
					heaterPID.SetTunings(heaterPID_conservative.kP,heaterPID_conservative.kI,heaterPID_conservative.kD);
					saveSettings();



					lcd.clear();
					lcd.print(F("Tuning finished, new parameters saved to EEPROM"));
					delay(1500);
				}

			break;
			
			case sMANUAL_TEMPCONTROLLED:
				if (stateChanged) {
					stateChanged = false;

					//init manual mode
					heaterPID.SetTunings(heaterPID_conservative.kP, heaterPID_conservative.kI, heaterPID_conservative.kD);
				
				}
				
			break;
			
			case sMANUAL_DUTYFIXED:
				if (stateChanged) {
					stateChanged = false;

					//init manual mode

				
				}

			

			break;
		}
	}


	
	if (currentState != sMANUAL_TEMPCONTROLLED && currentState != sAUTOTUNE) {
		//check for errors only in automatic modes
		
		// both of these errors are blocking and do not exit!
		if (controlSetpoint > controlInput + 50) abortWithError(1); // if we're 50 degree cooler than setpoint, abort
		//if(Input > Setpoint + 50) abortWithError(2);// or 50 degrees hotter, also abort
	}

	if(currentState!=sAUTOTUNE)
	heaterPID.Compute();

	if (currentState == sIDLE) {
		//all off in idle mode
		heaterValue = 0;
		//fanValue = 0;
	} else if (currentState == sRAMPDOWM || currentState == sCOOLDOWN) {
		// in cooling phases turn heater off hard and control fan instead
		heaterValue = 0;
		//fanValue = controlOutput;
	} else if (currentState == sMANUAL_DUTYFIXED) {
		// leave as is... new value is passed via menu and its callback-function
	} else {
		// other phases are non-idle or heating-phases
		// heater is controlled by PID and fan is on assisting speed
		heaterValue = controlOutput;

		if(heaterValue>MODULATION_WINDOWSIZE)
			heaterValue=MODULATION_WINDOWSIZE;
		
		//fanValue = fanAssistSpeed;
	}

	if ( (millis() - windowStartTime) > MODULATION_SWITCH_PERIOD) {
		//time to shift the Relay Window
		windowStartTime += MODULATION_SWITCH_PERIOD;
	}

	if ( heaterValue*(MODULATION_SWITCH_PERIOD/MODULATION_WINDOWSIZE) < (millis() - windowStartTime) ) {
		digitalWrite(HEATING_PIN, LOW);
		} else {
		digitalWrite(HEATING_PIN, HIGH);
	}
	/*
	if (    fanValue < (millis() - windowStartTime) ) {
	digitalWrite(FAN_PIN, LOW);
	} else {
	digitalWrite(FAN_PIN, HIGH);
	}
	*/

	//after everything is done... every 250ms
	if ( (millis() - lastSerialOutput) > 250) {
		lastSerialOutput = millis();
		sendSerialUpdate();
	}

}

void manualModeTempControlledStart() {

	currentState = sMANUAL_TEMPCONTROLLED;

	controlSetpoint=manualModeTemperatureSetpoint;
	//set current Tuning parameters
	heaterPID.SetTunings(heaterPID_conservative.kP, heaterPID_conservative.kI, heaterPID_conservative.kD);
	
	
	lcd.clear();
	lcd.print("Starting manual mode");
	delay(1000);

	//time the cycle started
	startTime = millis();
	
	//suspend menu go to status screen
	nav.idleOn(idle);
}

void manualModeDutyFixedStart() {

	currentState = sMANUAL_DUTYFIXED;

	heaterValue=manualModeSSR1Duty;
	
	lcd.clear();
	lcd.print("Starting manual mode");
	delay(1000);

	//time the cycle started
	startTime = millis();
	
	//suspend menu go to status screen
	nav.idleOn(idle);
}


//start a cycle
void cycleStart() {
	
	if(therm0.getTemperatureCelsius()>IDLETEMP) {
		//don't start if temp too high
		lcd.clear();
		lcd.print("Temp too high. Wait");
		lcd.setCursor(0, 1);
		lcd.print("until below ");
		lcd.print(IDLETEMP);
		lcd.print((char)223);
		lcd.print("C");
		delay(1000);
		
		nav.idleOn(idle);
		
	} else {
		//okay, start...
		
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
	
	
}

//start a cycle
void autotuneStart() {

	currentState = sAUTOTUNE;

	//set current Tuning parameters
	heaterPID.SetTunings(heaterPID_conservative.kP, heaterPID_conservative.kI, heaterPID_conservative.kD);
	
	lcd.clear();
	lcd.print("Starting Autotune");
	delay(1000);

	//time the cycle started
	startTime = millis();
	
	//suspend menu go to status screen
	nav.idleOn(idle);
}

void changeProfile(eventMask e) {
	//called after menu has set the new profileNumber... time to load the values
	//Serial.println("Loading Profile");
	
	#ifdef DEBUG
	Serial.println("Check parameters:");
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

}

void loadParameters(uint8_t profile) {
	uint16_t offset = 0;
	offset = profile * AMOUNT_EEPROM_PER_PROFILE;

	activeProfile.soakTemp = EEPROM.readInt(offset);  offset+=2;
	activeProfile.soakDuration = EEPROM.readInt(offset);  offset+=2;
	activeProfile.peakTemp = EEPROM.readInt(offset);  offset+=2;
	activeProfile.peakDuration = EEPROM.readInt(offset);  offset+=2;
	activeProfile.rampUpRate = EEPROM.readDouble(offset);  offset+=4;
	activeProfile.rampDownRate = EEPROM.readDouble(offset);  offset+=4;

}


void saveProfile(eventMask e) {
	//Serial.println(e);

	//leave menu
	nav.doNav(escCmd);

	saveLastUsedProfile();
	
	//Serial.println("Saving Profile");
	#ifdef DEBUG
	Serial.println("Check parameters:");
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

	saveParameters(profileNumber);
	return proceed;
}

void saveParameters(uint8_t profile) {
	uint16_t offset = 0;
	offset = profile * AMOUNT_EEPROM_PER_PROFILE;

	EEPROM.writeInt(offset,activeProfile.soakTemp);  offset+=2;
	EEPROM.writeInt(offset,activeProfile.soakDuration);  offset+=2;
	EEPROM.writeInt(offset,activeProfile.peakTemp);  offset+=2;
	EEPROM.writeInt(offset,activeProfile.peakDuration);  offset+=2;
	EEPROM.writeDouble(offset,activeProfile.rampUpRate);  offset+=4;
	EEPROM.writeDouble(offset,activeProfile.rampDownRate);  offset+=4;

}

void loadSettings() {
	uint16_t offset = EEPROM_OFFSET_SETTINGS;
	
	heaterPID_conservative.kP = EEPROM.readDouble(offset);  offset+=4;
	heaterPID_conservative.kI = EEPROM.readDouble(offset);  offset+=4;
	heaterPID_conservative.kD = EEPROM.readDouble(offset);  offset+=4;
	
	buzzerOn = EEPROM.readByte(offset++);
	
}

void saveSettings() {
	uint16_t offset = EEPROM_OFFSET_SETTINGS;

	//leave menu
	nav.doNav(escCmd);
	
	EEPROM.writeDouble(offset,heaterPID_conservative.kP);  offset+=4;
	EEPROM.writeDouble(offset,heaterPID_conservative.kI);  offset+=4;
	EEPROM.writeDouble(offset,heaterPID_conservative.kD);  offset+=4;
	
	EEPROM.writeByte(offset++,buzzerOn);
	
}

boolean firstRun() {
	// check the the space of the first profile. if all bytes are 255, it's the very first run
	for (uint8_t i = 0; i < (AMOUNT_EEPROM_PER_PROFILE-1); i++) {
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
	for (uint8_t i = 0; i < NUMBER_OF_STORED_PROFILES; i++) {
		saveParameters(i);
	}

	heaterPID_conservative.kP=HEATER_CONSERVATIVE_Kp;
	heaterPID_conservative.kI=HEATER_CONSERVATIVE_Ki;
	heaterPID_conservative.kD=HEATER_CONSERVATIVE_Kd;
	buzzerOn=BUZZER_DEFAULT;
	saveSettings();
	
	//fanAssistSpeed = FAN_DEFAULT_SPEED;
	//saveFanSpeed();
	profileNumber = 0;
	saveLastUsedProfile();


	loadLastUsedProfile();
	loadSettings();
	
	delay(100);
}

void saveLastUsedProfile() {
	EEPROM.write(EEPROM_OFFSET_PROFILE_NUMBER, profileNumber);
}

void loadLastUsedProfile() {
	profileNumber = EEPROM.read(EEPROM_OFFSET_PROFILE_NUMBER);
	loadParameters(profileNumber);
}

void sendSerialUpdate() {
	char floatBuffer[20];
	if (currentState == sIDLE) {
		Serial.print(F("0,0,0,0,0"));
	} else {
		Serial.print(millis() - startTime);
		Serial.print(",");
		Serial.print((int)currentState);
		Serial.print(",");
		Serial.print(controlSetpoint);
		Serial.print(",");
		Serial.print(heaterValue);
		Serial.print(",");
		Serial.print(/*fanValue*/0);
	}
	
	Serial.print(",");
	Serial.print(therm0.getTemperatureCelsius());
	Serial.print(",");
	if (therm1.getStatus() == 0) {
		Serial.print(therm1.getTemperatureCelsius());
		} else {
		Serial.print("999");
	}
	Serial.print(",");
	Serial.print(rampRate);
	
	
	Serial.print(",");
	Serial.print(dtostrf(heaterPID.GetKp(), 6+3, 6, floatBuffer));
	Serial.print(",");
	Serial.print(dtostrf(heaterPID.GetKi(), 6+3, 6, floatBuffer));
	Serial.print(",");
	Serial.print(dtostrf(heaterPID.GetKd(), 6+3, 6, floatBuffer));
	/*
	Serial.print(",");
	Serial.print(dtostrf(controlInput, 6+3, 6, floatBuffer));
	Serial.print(",");
	Serial.println(dtostrf(controlOutput, 6+3, 6, floatBuffer));
	*/
	Serial.println("");
}
  
