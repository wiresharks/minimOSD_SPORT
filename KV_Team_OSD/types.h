#ifndef TYPES_H_
#define TYPES_H_

#include "Arduino.h"
#include "Config.h"


typedef struct {
	uint16_t cycleTime;
	int16_t I2CError;
	uint16_t  sensorPresent;
	uint32_t  sensorActive;
	uint8_t version;
} MW_status_t;


typedef struct {
	uint8_t fix;
	uint8_t numSat;
	int32_t latitude;
	int32_t longitude;
	int16_t altitude;
	uint16_t speed;
	int16_t ground_course;
	//int32_t distanceToHome;
	int16_t distanceToHome;
	int16_t directionToHome;
} GPS_t;

typedef struct {
	int16_t Angle[2];
	int16_t Heading;
} MW_ATTITUDE_t;

typedef struct {
	int16_t  accSmooth[3];
	int16_t  gyroData[3];
	int16_t  magADC[3];
	//int16_t  gyroADC[3];
	//int16_t  accADC[3];
} MW_imu_t;

typedef struct {
   int32_t Altitude;
   int16_t Vario;
   int16_t SonarAlt;
} MW_ALTTITUDE_t;


typedef struct {
	uint8_t VBat;
	uint16_t pMeterSum;
	uint16_t Rssi;
	uint16_t Amperage;
} MW_ANALOG_t;


struct pid_ {
	uint8_t P8;
	uint8_t I8;
	uint8_t D8;
};

#if defined(CLEANFLIGHT)
struct TRCRates_t {
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollRate8;
  uint8_t pitchRate8;
  uint8_t yawRate8;
  uint8_t dynThrPID8;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  uint16_t tpaBreakpoint16;
  uint8_t rcYawExpo8;
} __attribute__((__packed__ ));
#else
struct TRCRates_t {
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
} __attribute__((__packed__ ));
#endif


typedef struct {
	pid_    pid[PIDITEMS];
	TRCRates_t rates;
}conf_t;


// Settings Locations
enum Setting_ {
  S_CHECK_,					// 0 used for check
  S_RSSIMIN,    			// 1
  S_RSSIMAX,    			// 2
  S_RSSI_ALARM,				// 3
  S_MWRSSI, 				// 4
  S_PWMRSSI,				// 5
  S_PWMRSSIDIVIDER,			// 6
  S_VOLTAGEMIN,				// 7
  S_BATCELLS,				// 8
  S_DIVIDERRATIO, 			// 9
  S_MAINVOLTAGE_VBAT, 		// 10
  S_VIDDIVIDERRATIO,		// 11
  S_VIDVOLTAGE_VBAT,		// 12
  //S_TEMPERATUREMAX,		// 14
  S_BOARDTYPE, 				// 13
  S_DISPLAYGPS,				// 14
  S_COORDINATES,			// 15
  S_HEADING360,				// 16
  S_UNITSYSTEM,				// 17
  S_VIDEOSIGNALTYPE, 		// 18
  S_RESETSTATISTICS, 		// 19
  S_ENABLEADC,				// 20
  S_BLINKINGHZ,    			// 21 selectable alarm blink freq
  S_MWAMPERAGE,				// 22
  S_CURRSENSSENSITIVITY, 	// 23
  S_CURRSENSOFFSET_H,		// 24
  S_CURRSENSOFFSET_L,		// 25
  S_CLIMB_RATE_ALARM,		// 26
  S_VOLUME_DIST_MAX, 		// 27
  S_VOLUME_ALT_MAX, 		// 28
  S_VOLUME_ALT_MIN,			// 29
  S_VIDVOLTAGEMIN,			// 30
  S_PITCH_WARNING,			// 31
  S_CALLSIGN,				// 32
  S_CS0,					// 33 - 10 callsign char locations
  S_CS1,					// 34
  S_CS2,					// 35
  S_CS3,					// 36
  S_CS4,					// 37
  S_CS5, 					// 38
  S_CS6,					// 39
  S_CS7,					// 40
  S_CS8,					// 41
  S_CS9,					// 42
  // EEPROM_SETTINGS must be last for H/W settings!
  EEPROM_SETTINGS,  		//43
  
// Screen item Locations
// ********* EEProm enum data position and display On/Off option for all items on screen ****************
// Enum valid for both PAL/NTSC  
  L_GPS_NUMSATPOSITIONROW,    		//44
  L_GPS_NUMSATPOSITIONCOL,			//45	
  L_GPS_NUMSATPOSITIONDSPL,
  L_GPS_DIRECTIONTOHOMEPOSROW,
  L_GPS_DIRECTIONTOHOMEPOSCOL, 
  L_GPS_DIRECTIONTOHOMEPOSDSPL,		 
  L_GPS_DISTANCETOHOMEPOSROW,		//50
  L_GPS_DISTANCETOHOMEPOSCOL,
  L_GPS_DISTANCETOHOMEPOSDSPL,
  L_SPEEDPOSITIONROW,
  L_SPEEDPOSITIONCOL,				
  L_SPEEDPOSITIONDSPL,				// 55
  L_GPS_ANGLETOHOMEPOSROW,
  L_GPS_ANGLETOHOMEPOSCOL,
  L_GPS_ANGLETOHOMEPOSDSPL, 
  /*L_MW_GPS_ALTPOSITIONROW,          // Do not remove yet
  L_MW_GPS_ALTPOSITIONCOL,
  L_MW_GPS_ALTPOSITIONDSPL,*/
  L_SENSORPOSITIONROW,				
  L_SENSORPOSITIONCOL,				// 60
  L_SENSORPOSITIONDSPL,
  L_MODEPOSITIONROW,
  L_MODEPOSITIONCOL, 
  L_MODEPOSITIONDSPL,
  L_MW_HEADINGPOSITIONROW,
  L_MW_HEADINGPOSITIONCOL,			// 65
  L_MW_HEADINGPOSITIONDSPL,
  L_MW_HEADINGGRAPHPOSROW,
  L_MW_HEADINGGRAPHPOSCOL,
  L_MW_HEADINGGRAPHPOSDSPL,			// 70
 /* L_TEMPERATUREPOSROW,              // Do not remove yet
  L_TEMPERATUREPOSCOL,
  L_TEMPERATUREPOSDSPL,*/

  L_MW_ALTITUDEPOSITIONROW,
  L_MW_ALTITUDEPOSITIONCOL,
  L_MW_ALTITUDEPOSITIONDSPL,
  L_CLIMBRATEPOSITIONROW,
  L_CLIMBRATEPOSITIONCOL,			// 75
  L_CLIMBRATEPOSITIONDSPL,
  L_HORIZONPOSITIONROW,
  L_HORIZONPOSITIONCOL,
  L_HORIZONPOSITIONDSPL,
  L_HORIZONSIDEREFROW,			// 80
  L_HORIZONSIDEREFCOL,
  L_HORIZONSIDEREFDSPL,
  L_HORIZONCENTERREFROW,
  L_HORIZONCENTERREFCOL,
  L_HORIZONCENTERREFDSPL,			// 85
  L_CURRENTTHROTTLEPOSITIONROW,
  L_CURRENTTHROTTLEPOSITIONCOL,
  L_CURRENTTHROTTLEPOSITIONDSPL,
  L_FLYTIMEPOSITIONROW,
  L_FLYTIMEPOSITIONCOL,				// 90
  L_FLYTIMEPOSITIONDSPL,
  L_ONTIMEPOSITIONROW,
  L_ONTIMEPOSITIONCOL,
  L_ONTIMEPOSITIONDSPL,
  L_MOTORARMEDPOSITIONROW,			// 95
  L_MOTORARMEDPOSITIONCOL,
  L_MOTORARMEDPOSITIONDSPL,
  L_MW_GPS_LATPOSITIONROW,
  L_MW_GPS_LATPOSITIONCOL,
  L_MW_GPS_LATPOSITIONDSPL,			// 100
  L_MW_GPS_LONPOSITIONROW,
  L_MW_GPS_LONPOSITIONCOL,
  L_MW_GPS_LONPOSITIONDSPL,
  L_RSSIPOSITIONROW,
  L_RSSIPOSITIONCOL,				// 105
  L_RSSIPOSITIONDSPL,
  L_VOLTAGEPOSITIONROW,
  L_VOLTAGEPOSITIONCOL,
  L_VOLTAGEPOSITIONDSPL,
  L_MAINBATLEVEVOLUTIONROW,			// 110
  L_MAINBATLEVEVOLUTIONCOL,
  L_MAINBATLEVEVOLUTIONDSPL,  
  L_VIDVOLTAGEPOSITIONROW,
  L_VIDVOLTAGEPOSITIONCOL,
  L_VIDVOLTAGEPOSITIONDSPL,			// 115
  L_AMPERAGEPOSITIONROW,
  L_AMPERAGEPOSITIONCOL,
  L_AMPERAGEPOSITIONDSPL,
  L_PMETERSUMPOSITIONROW,
  L_PMETERSUMPOSITIONCOL,			// 120
  L_PMETERSUMPOSITIONDSPL,
  L_CALLSIGNPOSITIONROW,
  L_CALLSIGNPOSITIONCOL,
  L_CALLSIGNPOSITIONDSPL,			// 124
  
  // EEPROM_ITEM_LOCATION must be last for Items location!
  EEPROM_ITEM_LOCATION				// 125
};

#endif /* TYPES_H_ */
