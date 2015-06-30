#ifndef SCREEN_H_
#define SCREEN_H_
#include "Arduino.h"
#include "Config.h"

#ifdef FEATURE_TEMPERATURE
	void displayTemperature(int16_t temp) ;
#endif
	void displayMode(void);
	void displayArmed(void);
	void displayCallsign(void);
	void displayHorizon(int16_t rollAngle, int16_t pitchAngle);
	void displayVoltage(void);
#ifdef FEATURE_VID_VOLTAGE
	void displayVidVoltage(void);
#endif
	void displayCurrentThrottle(void);
	void displayTime(void);
#ifdef FEATURE_AMPERAGE
	void displayAmperage(void);
#endif
	void displaypMeterSum(void);
	void displayRSSI(void);
	void displayHeading(void);
	void displayHeadingGraph(void);
	void displayIntro(char position);
	void displayFontScreen(void);
	void displayGPSPosition(void);
	void displayGPS_altitude (void);
	void displayNumberOfSat(void);
	void displayDebug1(int16_t x);
	void displayGPS_speed(void);
	void displayAltitude(void);
	void displayClimbRate(void);
	void displayDistanceToHome(void);
	void displayAngleToHome(void);
	void displayDirectionToHome(void);
	void displayCursor(void);
	void displayConfigScreen(void);
	void displaySensor(void);
	void displayGPSMode(void); 
	void displayautoPilot(void);
		
#endif /* SCREEN_H_ */
