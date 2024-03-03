
/*
  Code to open and close vents on kiln
*/

/*
	Need to add sanitization start time in millis and RTC
	Need to add logic to track sanitization time based on temp
	Need to add sanitization time in logic to determine vent state
	Need to set LED outputs
*/

/******************************************************************************
* Includes
******************************************************************************/
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <RTClib.h>
#include <SD.h>
#include <EEPROM.h>

/******************************************************************************
* Defines
******************************************************************************/
//Use these for state definitions
#define OFF 0
#define ON 1
#define RUN 1
#define CLOSE 0
#define OPEN 1

//Use these for debugging
#define USE_SERIAL 0
#define USE_SERIAL_SHT31 0
#define USE_SERIAL_RTC 0
#define USE_SERIAL_SD 0

#define USE_SHT31 1
#define USE_RTC 1
#define USE_SD 1

/******************************************************************************
* Instances
******************************************************************************/
Servo UprRH;
Servo UprLH;
Servo LwrRH;
Servo LwrLH;

//Temp and Humidity Sensor
#if USE_SHT31
	Adafruit_SHT31 sht31 = Adafruit_SHT31();
#endif

//Real Time Clock
#if USE_RTC
	RTC_PCF8523 rtc;
#endif

// set up variables using the SD utility library functions:
#if USE_SD
	File dataFile;
#endif

/******************************************************************************
* Constants
******************************************************************************/
//Digital Pins
//-----------------
//const int  = 13;								//TBD, used by shield
//const int  = 12;								//TBD, used by shield
//const int  = 11;								//TBD, used by shield
const int chipSelect = 10;				//SD card pin
const int pinServoUprRH = 9;			//Servo PPM for vent, Upr RH
const int pinFanLH = 8;						//Relay for fan, LH
const int pinFanCTR = 7;					//Relay for fan, Ctr
const int pinServoLwrRH = 6;			//Servo PPM for vent, Lwr RH
const int pinServoLwrLH = 5;			//Servo PPM for vent, Lwr LH
const int pinLEDSanitize = 4;			//LED light to indicate sanitize complete
const int pinServoUprLH = 3;			//Servo PPM for vent, Upr LH
const int pinFanRH = 2;						//Relay for fan, RH
const int pinLEDTBD = 1;					//LED light to indicate TBD
const int pinCool = 0;						//Relay for electronic cooling fans

//Analog Pins
//-----------------
const int pinSolarVoltage = A0;		//Input, solar panel voltage sense
const int pinOnOff = A1;					//Input, Logic on/off switch
const int pinSanitize = A2;				//INPUT, Sanitize on/off switch
//const int  = 17;
//const int  = A4;
//const int  = A5;

//Other
//-----------------
const int angleCloseLwrLH = 45;
const int angleOpenLwrLH = 135;

const int angleCloseLwrRH = 45;
const int angleOpenLwrRH = 135;

const int angleCloseUprRH = 50;
const int angleOpenUprRH = 135;

const int angleCloseUprLH = 40;
const int angleOpenUprLH = 125;

const float					voltageMapMax						= 5.0;
const float					inputMapMax							= 1023;

const float 				rl                      = 100000000; //load resistance arduino analog INPUT

const float 				r1Solar                 = 150000; //voltage divider r1Solar
const float 				r2Solar                 = 47000; //voltage divider r2Solar
const float         voltageSolarMinOff      = 18; //minimum voltage to turn on
const float 				voltageSolarMinOn				= 16; //minimum voltage while running
const float					voltageSolarMinAbs			= 12; //minimum voltage to be considered day, close/open vents
const float         rDivideSolar            = (r2Solar*rl)/(r2Solar+rl); //voltage Divider for panel sense
const float         voltageSolarMinSenseOff	= voltageSolarMinOff*rDivideSolar/(r1Solar + rDivideSolar); //minimum voltage as sensed by Arduino
const float         voltageSolarMinSenseOn  = voltageSolarMinOn*rDivideSolar/(r1Solar + rDivideSolar); //minimum voltage as sensed by Arduino
const float         voltageSolarMinSenseAbs = voltageSolarMinAbs*rDivideSolar/(r1Solar + rDivideSolar); //minimum voltage as sensed by Arduino
const int           inputSolarMinSenseOff   = voltageSolarMinSenseOff/voltageMapMax*inputMapMax; //min operating value read by analog input for solar voltage
const int           inputSolarMinSenseOn	  = voltageSolarMinSenseOn/voltageMapMax*inputMapMax; //min operating value read by analog input for solar voltage
const int           inputSolarMinSenseAbs	  = voltageSolarMinSenseAbs/voltageMapMax*inputMapMax; //min operating value read by analog input for solar voltage

const float 				r1Battery                	= 100000; //voltage divider r1 Battery
const float 				r2Battery                 = 47000; //voltage divider r2 Battery
const float         voltageBatteryMinOff      = 13.6; //minimum voltage to turn on
const float 				voltageBatteryMinOn				= 12.6; //minimum voltage while running
const float         rDivideBattery          	= (r2Battery*rl)/(r2Battery+rl); //voltage Divider for battery sense
const float         voltageBatteryMinSenseOff	= voltageBatteryMinOff*rDivideBattery/(r1Battery + rDivideBattery); //minimum voltage as sensed by Arduino
const float         voltageBatteryMinSenseOn  = voltageBatteryMinOn*rDivideBattery/(r1Battery + rDivideBattery); //minimum voltage as sensed by Arduino
const int           inputBatteryMinSenseOff   = voltageBatteryMinSenseOff/voltageMapMax*inputMapMax; //min operating value read by analog input for battery voltage
const int           inputBatteryMinSenseOn	  = voltageBatteryMinSenseOn/voltageMapMax*inputMapMax; //min operating value read by analog input for battery voltage

const int           timeRequireSanitizeHours	= 4; //minimum number of hours to hold at sanitization temp
const int						tempSanitizeMin						= 60; //140F minimum temperature for sanitization
const int						tempSanitizeMax						= 71; //160F maximum temperature for sanitization

const int						numVoltageRead						= 5; //voltage sensing smoothing function

//const byte					fileNumber 								= (byte)0;
const byte					fileNumber 								= EEPROM.read(0) + (byte)1;
const int 					timeSaveIntervalMins 			= 10;

/******************************************************************************
* Variables
******************************************************************************/
int             inputSolarSense[numVoltageRead]; //input value read for solar panel voltage
int							inputSolarSenseAvg = 0; //average of smoothed solar voltage readings
long 						inputSolarSenseTotal = 0; //total of voltage reading matrix
int             inputBatterySense[numVoltageRead]; //input value read for solar panel voltage
int							inputBatterySenseAvg = 0; //average of smoothed solar voltage readings
long 						inputBatterySenseTotal = 0; //total of voltage reading matrix
int             inputOnOff = 0; //on or off
int 						inputSanitize = 0; //sanitize cycle on or off
int 						completeSanitize = 0; //sanitize cycle complete
float   				timeRunningMillis = 0; //millis() since button press
float				    timeStartMillis = 0; //millis() when button pressed
int             timeRunningDays = 0; //days since turned on
float   				timeRunningSanitizeMillis = 0; //millis() since sanitize temp reached
float				    timeStartSanitizeMillis = 0; //millis() when sanitize temp reached
int             timeRunningSanitizeHours = 0; //hours since sanitize temp reached
float						temp = 0;
float						humid = 0;
float						lastHeat = 0;

long 						readCount = 0;
int 						fullSmooth = 0;

volatile int    stateOnOff = OFF;
volatile int    stateLEDSanitize = OFF;
volatile int    stateVent = CLOSE;
volatile int    stateFan = OFF;
volatile int 		fanAltLH = HIGH;
volatile int 		fanAltRH = HIGH;
volatile int    stateCool = LOW;

char 						daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
DateTime 				now;
unsigned long 	nowUnix;
DateTime				startedKiln;
unsigned long 	startedKilnUnix;
DateTime				startedSanitize;
unsigned long 	startedSanitizeUnix;

char 						fileName[20];
byte 						saveData = 1;
int 						stateFanSaved = stateFan;
float 					timeSaveMillis = 0;

/******************************************************************************
* Code
******************************************************************************/
void setup() {
  // put your setup code here, to run once:
	#if USE_SERIAL || USE_SERIAL_SHT31 || USE_SERIAL_RTC || USE_SERIAL_SD
		Serial.begin(57600);
	#endif

	#ifndef ESP8266
	  //while (!Serial); // wait for serial port to connect. Needed for native USB
	#endif

  //----------------------------------------
	//Declare Pins
	//----------------------------------------

	//Digital Pins (Analog out, PPM, does not need declared)
	//13
	//12
  //11
	pinMode(SS, OUTPUT); //10
	pinMode(pinServoUprRH,OUTPUT); //9
	pinMode(pinFanLH,OUTPUT); //8
	pinMode(pinFanCTR,OUTPUT); //7
	pinMode(pinServoLwrRH,OUTPUT); //6
	pinMode(pinServoLwrLH,OUTPUT); //5
	pinMode(pinLEDSanitize,OUTPUT); //4
	pinMode(pinServoUprLH,OUTPUT); //3
	pinMode(pinFanRH,OUTPUT); //2
	#if !(USE_SERIAL || USE_SERIAL_SHT31 || USE_SERIAL_RTC || USE_SERIAL_SD)
		pinMode(pinLEDTBD,OUTPUT); //1
		pinMode(pinCool,OUTPUT); //0
	#endif

	//Analog Pins
  pinMode(pinSolarVoltage,INPUT); //14 (A0)
  pinMode(pinOnOff,INPUT); //15 (A1)
  pinMode(pinSanitize,INPUT); //16 (A2)
	//17 (A3)
  //18 (A4)
  //19 (A5)

  UprRH.attach(pinServoUprRH);
  UprLH.attach(pinServoUprLH);
  LwrRH.attach(pinServoLwrRH);
  LwrLH.attach(pinServoLwrLH);

	#if USE_SERIAL
		Serial.println();
	#endif

	#if USE_SHT31
		if (! sht31.begin(0x44))   // Set to 0x45 for alternate i2c addr
		{
			#if USE_SERIAL_SHT31
				Serial.println(F("Couldn't find SHT31"));
			#endif
	    //while (1) delay(1);
	  }
	#endif

  //----------------------------------------
	//Set Initial Output State
	//----------------------------------------
  //Fans
  digitalWrite(pinFanCTR,LOW);
  digitalWrite(pinFanLH,LOW);
  digitalWrite(pinFanRH,LOW);
  digitalWrite(pinCool,LOW);
  //Servos
  UprRH.write(angleCloseUprRH);
  UprLH.write(angleCloseUprLH);
  LwrRH.write(angleCloseLwrRH);
  LwrLH.write(angleCloseLwrLH);
	//LEDs
	digitalWrite(pinLEDSanitize,LOW);
	digitalWrite(pinLEDTBD,LOW);

	//----------------------------------------
	//Real Time Clock
	//----------------------------------------
	#if USE_RTC
	  if (! rtc.begin()) {
			#if USE_SERIAL_RTC
				Serial.println(F("Couldn't find RTC"));
		    Serial.flush();
			#endif
	    //while (1) delay(10);
	  }

	  if (! rtc.initialized() || rtc.lostPower()) {
			#if USE_SERIAL_RTC
				Serial.println(F("RTC is NOT initialized, let's set the time!"));
			#endif
	    // When time needs to be set on a new device, or after a power loss, the
	    // following line sets the RTC to the date & time this sketch was compiled
	    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	    // This line sets the RTC with an explicit date & time, for example to set
	    // January 21, 2014 at 3am you would call:
	    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
	    //
	    // Note: allow 2 seconds after inserting battery or applying external power
	    // without battery before calling adjust(). This gives the PCF8523's
	    // crystal oscillator time to stabilize. If you call adjust() very quickly
	    // after the RTC is powered, lostPower() may still return true.
	  }

	  // When time needs to be re-set on a previously configured device, the
	  // following line sets the RTC to the date & time this sketch was compiled
	  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	  // This line sets the RTC with an explicit date & time, for example to set
	  // January 21, 2014 at 3am you would call:
	  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

	  // When the RTC was stopped and stays connected to the battery, it has
	  // to be restarted by clearing the STOP bit. Let's do this to ensure
	  // the RTC is running.
	  rtc.start();

	   // The PCF8523 can be calibrated for:
	  //        - Aging adjustment
	  //        - Temperature compensation
	  //        - Accuracy tuning
	  // The offset mode to use, once every two hours or once every minute.
	  // The offset Offset value from -64 to +63. See the Application Note for calculation of offset values.
	  // https://www.nxp.com/docs/en/application-note/AN11247.pdf
	  // The deviation in parts per million can be calculated over a period of observation. Both the drift (which can be negative)
	  // and the observation period must be in seconds. For accuracy the variation should be observed over about 1 week.
	  // Note: any previous calibration should cancelled prior to any new observation period.
	  // Example - RTC gaining 43 seconds in 1 week
	  float drift = 43; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
	  float period_sec = (7 * 86400);  // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
	  float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
	  float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
	  // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
	  int offset = round(deviation_ppm / drift_unit);
	  // rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
	  // rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration
		#if USE_SERIAL_RTC
	  	Serial.print(F("Offset is ")); Serial.println(offset); // Print to control offset
		#endif

		startedKiln = rtc.now();
		startedKilnUnix = startedKiln.unixtime();
		startedSanitize = rtc.now();
		startedSanitizeUnix = startedSanitize.unixtime();

	#endif

	//----------------------------------------
	//SD Card Initialization
	//----------------------------------------
	//store current file number in EEPROM
	EEPROM.write(0,fileNumber);
	sprintf(fileName, "log%03d.csv", (int)fileNumber);
	#if USE_SERIAL
		Serial.print(F("File Name: "));
		Serial.println(fileName);
	#endif

	#if USE_SD
		#if USE_SERIAL_SD
			Serial.print(F("Initializing SD card..."));
		#endif

		// see if the card is present and can be initialized:
	  if (!SD.begin(chipSelect)) {
	    Serial.println(F("Card failed, or not present"));
	    // don't do anything more:
	    //while (1) ;
	  }
	  Serial.println(F("card initialized."));

	  // Open up the file we're going to log to!
	  dataFile = SD.open(fileName, FILE_WRITE);
	  if (! dataFile) {
	    Serial.println(F("error opening file"));
	    // Wait forever since we cant write data
	    //while (1) ;
	  }

		dataFile.println(
											#if USE_RTC
											"Year,Month,Day,Hour,Minute,Second,"
											#endif
											"Min Solar Voltage Integer,Min Battery Voltage Integer,Actual Solar Voltage Integer,Actual Battery Voltage Integer,"
											#if USE_SHT31
											"Temp (C),Humidity (%),"
											#endif
											"Runtime (days),Sanitize Time (hrs),"
											"On/Off State,Fan State,Vent State,"
											#if USE_SHT31
											"Sensor Heater State,"
											#endif
											"Sanitize Input,"
											"Sanitize Complete Status,"
											"Sanitize LED State,"
											);
	#endif
}

void loop() {
  // put your main code here, to run repeatedly:

  //----------------------------------------
  //Read Inputs
  //----------------------------------------

	//Erase smoothing function values from previous loopthru
	if(fullSmooth)
	{
		inputSolarSenseTotal = inputSolarSenseTotal - inputSolarSense[readCount];
		inputBatterySenseTotal = inputBatterySenseTotal - inputBatterySense[readCount];
	}

	//Read inputs
  inputSolarSense[readCount] = analogRead(pinSolarVoltage);
	inputBatterySense[readCount] = analogRead(pinOnOff);
	inputSanitize = analogRead(pinSanitize);

	//Calculate Inputs
	inputOnOff = (inputBatterySense[readCount] / inputMapMax) + 0.5; //0 or 1
	inputSanitize = (inputSanitize / inputMapMax) + 0.5; //0 or 1

	//Read Temp Humidity
	#if USE_SHT31
		temp = sht31.readTemperature();
	  humid = sht31.readHumidity();
	#endif

	//Read Real Time Clock
	#if USE_RTC
		now = rtc.now();
		nowUnix = now.unixtime();
	#endif

	//Log date and time
	#if USE_SD && USE_RTC
		if (saveData)
		{
			dataFile.print(now.year(), DEC);
			dataFile.print(",");
			dataFile.print(now.month(), DEC);
			dataFile.print(",");
			dataFile.print(now.day(), DEC);
			dataFile.print(",");
			dataFile.print(now.hour(), DEC);
			dataFile.print(",");
			dataFile.print(now.minute(), DEC);
			dataFile.print(",");
			dataFile.print(now.second(), DEC);
			dataFile.print(",");
		}
	#endif

	//Smoothing Function
	inputSolarSenseTotal = inputSolarSenseTotal + inputSolarSense[readCount];
	inputSolarSenseAvg = inputSolarSenseTotal / numVoltageRead;

	inputBatterySenseTotal = inputBatterySenseTotal + inputBatterySense[readCount];
	inputBatterySenseAvg = inputBatterySenseTotal / numVoltageRead;

	#if USE_SERIAL
		if (stateFan)
		{
			Serial.print(F("Minimum Solar Voltage Integer = "));
			Serial.println(inputSolarMinSenseOn);
			Serial.print(F("Minimum Battery Voltage Integer = "));
			Serial.println(inputBatteryMinSenseOn);
		}
		else
		{
			Serial.print(F("Minimum Solar Voltage Integer = "));
			Serial.println(inputSolarMinSenseOff);
			Serial.print(F("Minimum Battery Voltage Integer = "));
			Serial.println(inputBatteryMinSenseOff);
		}
		Serial.print(F("Actual Solar Voltage Integer Value = "));
		Serial.println(inputSolarSenseAvg);
		Serial.print(F("Actual Battery Voltage Integer Value = "));
		Serial.println(inputBatterySenseAvg);
	#endif

	#if USE_SD
		if (saveData)
		{
			if (stateFan)
			{
				dataFile.print(inputSolarMinSenseOn);
				dataFile.print(",");
				dataFile.print(inputBatteryMinSenseOn);
				dataFile.print(",");
			}
			else
			{
				dataFile.print(inputSolarMinSenseOff);
				dataFile.print(",");
				dataFile.print(inputBatteryMinSenseOff);
				dataFile.print(",");
			}
			dataFile.print(inputSolarSenseAvg);
			dataFile.print(",");
			dataFile.print(inputBatterySenseAvg);
			dataFile.print(",");
		}
	#endif

	#if USE_SHT31
		if (! isnan(temp))  // check if 'is not a number'
		{
			#if USE_SERIAL_SHT31
			Serial.print(F("Temp *C = "));
			Serial.println(temp);
			#endif

			#if USE_SD
				if (saveData)
				{
					dataFile.print(temp);
					dataFile.print(",");
				}
			#endif
		}
		else
		{
			#if USE_SERIAL_SHT31
	    Serial.println("Failed to read temperature");
			#endif

			#if USE_SD
				if (saveData)
				{
			    dataFile.print("#N/A");
					dataFile.print(",");
				}
			#endif
	  }

	  if (! isnan(humid))  // check if 'is not a number'
		{
			#if USE_SERIAL_SHT31
			Serial.print(F("Hum. % = "));
			Serial.println(humid);
			#endif

			#if USE_SD
				if (saveData)
				{
					dataFile.print(humid);
					dataFile.print(",");
				}
			#endif
		}
	  else
		{
			#if USE_SERIAL_SHT31
			Serial.println("Failed to read humidity");
			#endif

			#if USE_SD
				if (saveData)
				{
			    dataFile.print("#N/A");
					dataFile.print(",");
				}
			#endif
	  }
	#endif

  //----------------------------------------
  //Logic - Calculations
  //----------------------------------------

  // Calculate Runtime
  timeRunningMillis = millis() - timeStartMillis;
	timeRunningDays = timeRunningMillis/1000/60/60/24;

	//Determine if sanitize condition
	if (temp <= tempSanitizeMin) //sanitize condition not currently met
		timeStartSanitizeMillis = millis();
		#if USE_RTC
			startedSanitizeUnix = nowUnix;
		#endif

	//Calculate Sanitize time
	timeRunningSanitizeMillis = millis() - timeStartSanitizeMillis;
	timeRunningSanitizeHours = timeRunningSanitizeMillis/1000/60/60;

	//Use RTC if present, overwrite runtime variables
	#if USE_RTC
		timeRunningDays = (nowUnix - startedKilnUnix)/60/60/24;
		timeRunningSanitizeHours = (nowUnix - startedSanitizeUnix)/60/60;
	#endif

	//Determine if sanitize complete
	if (timeRunningSanitizeHours >= timeRequireSanitizeHours)
		completeSanitize = 1;

	#if USE_SERIAL
		Serial.print(F("Run Time: "));
		Serial.print(timeRunningMillis);
		Serial.print(F(" milliseconds, "));
		Serial.print(timeRunningDays);
		Serial.println(F(" days"));

		Serial.print(F("Sanitize Time: "));
		Serial.print(timeRunningSanitizeMillis);
		Serial.print(F(" milliseconds, "));
		Serial.print(timeRunningSanitizeHours);
		Serial.println(F(" hours"));
	#endif

	#if USE_SD
		if (saveData)
		{
			dataFile.print(timeRunningDays);
			dataFile.print(",");
			dataFile.print(timeRunningSanitizeHours);
			dataFile.print(",");
		}
	#endif

  //----------------------------------------
  //Logic - Determine State
  //----------------------------------------
	if (fullSmooth) //skip logic until smoothing array full
	{
		// LOGIC STATE
		if (inputOnOff)
	    stateOnOff = RUN;
	  else
	    stateOnOff = OFF;

		// FAN STATE
		if (((stateFan) && (inputBatterySenseAvg >= inputBatteryMinSenseOn))
				|| ((!stateFan) && (inputBatterySenseAvg >= inputBatteryMinSenseOff)))
	    stateFan = RUN;
	  else
	    stateFan = OFF;

		// VENT, COOLING STATE
		if (inputSolarSenseAvg >= inputSolarMinSenseAbs)
		{
			stateVent = OPEN;
			stateCool = RUN;
		}
		else
		{
			stateVent = CLOSE;
			stateCool = OFF;
		}

		// SANITIZE STATE, OVERWRITE VENT STATE TO CLOSE
		if (inputSanitize)
		{
			if (completeSanitize)
				stateLEDSanitize = ON;
			else
			{
				if (temp <= tempSanitizeMax)
					stateVent = CLOSE;
			}
		}
	}

	#if USE_SERIAL
		Serial.print(F("On/Off State: "));
		Serial.println(stateOnOff);
		Serial.print(F("Fan State: "));
		Serial.println(stateFan);
		Serial.print(F("Vent State: "));
		Serial.println(stateVent);
	#endif

	#if USE_SD
		if (saveData)
		{
			dataFile.print(stateOnOff);
			dataFile.print(",");
			dataFile.print(stateFan);
			dataFile.print(",");
			dataFile.print(stateVent);
			dataFile.print(",");
		}
	#endif

  //----------------------------------------
  //State Definitions
  //----------------------------------------

  switch(stateOnOff)
  {
    case OFF:
    {
      stateVent = CLOSE;
      stateFan = OFF;

			timeStartMillis = millis();
			timeStartSanitizeMillis = millis();
			#if USE_RTC
				startedKiln = rtc.now();
				startedKilnUnix = startedKiln.unixtime();
				startedSanitize = rtc.now();
				startedSanitizeUnix = startedSanitize.unixtime();
			#endif
      break;
    }
  }

  switch(stateVent)
  {
    case CLOSE:
    {
      UprRH.write(angleCloseUprRH);
      UprLH.write(angleCloseUprLH);
      LwrRH.write(angleCloseLwrRH);
      LwrLH.write(angleCloseLwrLH);
      break;
    }
    case OPEN:
    {
      UprRH.write(angleOpenUprRH);
      UprLH.write(angleOpenUprLH);
      LwrRH.write(angleOpenLwrRH);
      LwrLH.write(angleOpenLwrLH);
      break;
    }
  }

  switch(stateFan)
  {
    case OFF:
    {
      digitalWrite(pinFanCTR,LOW);
      digitalWrite(pinFanLH,LOW);
      digitalWrite(pinFanRH,LOW);
      break;
    }
    case RUN:
    {
			digitalWrite(pinFanCTR,LOW);
      delay(3000);
      digitalWrite(pinFanLH,fanAltLH);
      delay(3000);
      digitalWrite(pinFanRH,fanAltRH);
      delay(3000);
      break;
    }
  }

  #if !(USE_SERIAL || USE_SERIAL_SHT31 || USE_SERIAL_RTC || USE_SERIAL_SD)
		switch(stateCool)
		{
			case OFF:
			{
				digitalWrite(pinCool,LOW);
				break;
			}
			case RUN:
			{
				digitalWrite(pinCool,HIGH);
				break;
			}
		}
  #endif

	switch(stateLEDSanitize)
	{
		case OFF:
		{
			digitalWrite(pinLEDSanitize,LOW);
			break;
		}
		case ON:
		{
			digitalWrite(pinLEDSanitize,HIGH);
			break;
		}
	}
	//Temp and Humidity Heater Function
	#if USE_SHT31
		if (millis() >= lastHeat + 60000) //every minute
		{
			lastHeat = millis();
			sht31.heater(true);
		}
		else
			sht31.heater(false);

		#if USE_SERIAL_SHT31
			Serial.print(F("Heater Enabled State: "));
	    if (sht31.isHeaterEnabled())
	      Serial.println(F("ENABLED"));
	    else
	      Serial.println(F("DISABLED"));
		#endif

		#if USE_SD
			if (saveData)
			{
		    if (sht31.isHeaterEnabled())
		      dataFile.print("ENABLED");
		    else
		      dataFile.print("DISABLED");
				dataFile.print(",");
			}
		#endif
	#endif

	#if USE_SERIAL
		Serial.print(F("Cooling Fan: "));
		Serial.println(stateCool);
		Serial.print(F("Sanitize Input: "));
		Serial.println(inputSanitize);
		Serial.print(F("Sanitize Complete: "));
		Serial.println(completeSanitize);
		Serial.print(F("Sanitize LED State: "));
		Serial.println(stateLEDSanitize);
	#endif

	#if USE_SD
		if (saveData)
		{
			dataFile.print(inputSanitize);
			dataFile.print(",");
			dataFile.print(completeSanitize);
			dataFile.print(",");
			dataFile.print(stateLEDSanitize);
			dataFile.print(",");
		}
	#endif

	//Real Time Clock runtime
	#if USE_SERIAL_RTC && USE_RTC
		//Now
		Serial.print(F("Current Time and Date: "));
		Serial.print(now.year(), DEC);
		Serial.print('/');
		Serial.print(now.month(), DEC);
		Serial.print('/');
		Serial.print(now.day(), DEC);
		Serial.print(F(" ("));
		Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
		Serial.print(F(") "));
		Serial.print(now.hour(), DEC);
		Serial.print(':');
		Serial.print(now.minute(), DEC);
		Serial.print(':');
		Serial.println(now.second(), DEC);

		//Start
		Serial.print(F("Start Time and Date: "));
		Serial.print(startedKiln.year(), DEC);
		Serial.print('/');
		Serial.print(startedKiln.month(), DEC);
		Serial.print('/');
		Serial.print(startedKiln.day(), DEC);
		Serial.print(F(" ("));
		Serial.print(daysOfTheWeek[startedKiln.dayOfTheWeek()]);
		Serial.print(F(") "));
		Serial.print(startedKiln.hour(), DEC);
		Serial.print(':');
		Serial.print(startedKiln.minute(), DEC);
		Serial.print(':');
		Serial.println(startedKiln.second(), DEC);

		Serial.print(F("Runtime: "));
	  Serial.print(timeRunningDays);
		Serial.println(F(" Days"));
	#endif

	//Loop variables
	if(readCount >= numVoltageRead - 1)
	{
		fullSmooth = 1;
		readCount = 0;		//readCount goes back to 0 and isnt used again unless fullSmooth is set back to 0
	}
	else
	{
		readCount += 1;
	}

	#if USE_SERIAL
		Serial.print(F("Number of Readings: "));
		Serial.println(readCount);
		Serial.print(F("Full Smoothing? "));
		if (fullSmooth)
			Serial.println(F("YES"));
		else
			Serial.println(F("NO"));
	#endif

	#if USE_SD
		if (saveData)
		{
			// The following line will 'save' the file to the SD card after every
		  // line of data - this will use more power and slow down how much data
		  // you can read but it's safer!
		  // If you want to speed up the system, remove the call to flush() and it
		  // will save the file only every 512 bytes - every time a sector on the
		  // SD card is filled with data.
			dataFile.println();
			dataFile.flush();
		}
	#endif

	if 	((millis() - timeSaveMillis > (float)timeSaveIntervalMins*60*1000) ||
			(stateFanSaved != stateFan))
	{
		//alternate RH and LH fan on using only two fans
		(fanAltLH ^= fanAltRH), (fanAltRH ^= fanAltLH), (fanAltLH ^= fanAltRH);

		saveData = 1;
		stateFanSaved = stateFan;
		timeSaveMillis = millis();
	}
	else
		saveData = 0;

  delay(10000);
}
