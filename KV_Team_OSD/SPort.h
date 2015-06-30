#pragma once
#include "Config.h"

#ifdef SPORT

#define FCS_CURR_DATA_ID      (0x0200U) //< current
#define FCS_VOLT_DATA_ID      (0x0210U) //< voltage
#define GPS_LAT_LON_DATA_ID   (0x0800U)  //< gps latitude
#define GPS_ALT_DATA_ID       (0x0820U)  //< gps altitude
#define GPS_SPEED_DATA_ID     (0x0830U)  //< gps speed
#define GPS_COG_DATA_ID       (0x0840U)  //< gps course on ground
#define GPS_DATE_TIME_DATA_ID (0x0850U)  //< gps date time
#define ACCX_FIRST_ID         (0x0700U)  //< acc X
#define ACCY_FIRST_ID         (0x0710U)  //< acc Y
#define ACCZ_FIRST_ID         (0x0720U)  //< acc Z

void sport_init(void);
void sport_check(void);
#endif

