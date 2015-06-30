#include "Arduino.h"
#include "types.h"
#include "Serial.h"
#include "SoftwareSerial.h"
#include "SPort.h"

#ifdef SPORT

/*!
 * SPort specific data bytes
 */
#define START_STOP_BYTE         0x7e
#define XOR_BYTE_FLAG           0x7d
#define XOR_MASK                0x20
#define DATA_FRAME_BYTE         0x10

/*!
 * Sensor handler definition
 */
typedef void (*FSensorHandler)(void);
typedef struct TSensor {
    uint8_t id; // id of sensor
    FSensorHandler handler; // handler function
} Sensor_t;

/*!
 * \brief States for receive state machine.
 */
enum EReceive {
  RECEIVE_IDLE = 0, RECEIVE_SENSOR_REQUEST
};

/*!
 * \brief Serial modes.
 */
enum ESerialMode {
  SERIAL_RX = 0, SERIAL_TX
};

/*
 * s.port functions
 */
static void sport_byte_received(uint8_t data);
static void sport_handle_sensor(uint8_t id);
static void sport_send_data(uint16_t type, uint32_t data);
static uint8_t sport_send_byte(uint8_t byte, uint16_t crc);
static void sport_set_serial(ESerialMode mode);

/*
 * Sensor handler declarations
 */
#if defined(SPORT_VBAT)
static void sensor_handler_fcs(void);
#endif

#ifdef SPORT_GPS
static void sensor_handler_gps(void);
#endif

#ifdef SPORT_ACC
static void sensor_handler_acc(void);
#endif

/*
 * Sensor/handler definition
 */
static const Sensor_t s_Sensors[] = {
#if defined(SPORT_VBAT)
  { SPORT_VBAT_SENSOR_ID, sensor_handler_fcs },
#endif
#ifdef SPORT_GPS
  { SPORT_GPS_SENSOR_ID, sensor_handler_gps},
#endif
#ifdef SPORT_ACC
  { SPORT_ACC_SENSOR_ID, sensor_handler_acc},
#endif
};

/*!
 * \brief local static variables
 */
static SoftwareSerial sportSerial(SPORT_PIN, SPORT_PIN, true); //< soft serial

static EReceive s_receiveSate = RECEIVE_IDLE;
static ESerialMode s_mode = SERIAL_RX;

#ifdef SPORT_DEBUG
static void WriteDebug(const char * str) {
  size_t bytes = strlen(str);
  for(size_t i = 0; i < bytes; i++) {
    SerialWrite(0, str[i]);
  }
}
#endif

static void Log ( const char * format, ... )
{
#ifdef SPORT_DEBUG
  char buffer[64];
  va_list args;
  va_start (args, format);
  vsprintf (buffer,format, args);
  WriteDebug (buffer);
  va_end (args);
#endif
}

/*!
 * function definitions
 */
void sport_init(void)
{
  sportSerial.begin(SPORT_BAUD);
}

void sport_check(void)
{
  if (s_mode == SERIAL_RX) {
    while (sportSerial.available() && (s_mode == SERIAL_RX)) {
      const int data = sportSerial.read();
      if (data < 0) {
        break;
      }
      sport_byte_received(static_cast<uint8_t>(data));
    }
  }
  else {
    sport_set_serial(SERIAL_RX);
  }
}

static void sport_set_serial(ESerialMode mode)
{
  s_mode = mode;
  if (s_mode == SERIAL_TX) {
    pinMode(SPORT_PIN, OUTPUT);
  }
  else if (mode == SERIAL_RX) {
    pinMode(SPORT_PIN, INPUT);
  }
  else {
    // Should not happen
  }
}

static void sport_byte_received(uint8_t data)
{
  switch (s_receiveSate) {
    case RECEIVE_SENSOR_REQUEST: {
      sport_handle_sensor(data);
      s_receiveSate = RECEIVE_IDLE; // 1 sensor per request ?
      break;
    }

    case RECEIVE_IDLE:
    default: {
      if (data == START_STOP_BYTE) {
        s_receiveSate = RECEIVE_SENSOR_REQUEST;
      }
      break;
    }
  }
}


static void sport_send_data(uint16_t type, uint32_t data)
{
  uint16_t crc = 0;
  crc = sport_send_byte(DATA_FRAME_BYTE, crc);
  crc = sport_send_byte(type, crc);
  crc = sport_send_byte(type >> 8, crc);
  crc = sport_send_byte(data, crc);
  crc = sport_send_byte(data >> 8, crc);
  crc = sport_send_byte(data >> 16, crc);
  crc = sport_send_byte(data >> 24, crc);
  crc = 0xFF - crc;
  (void) sport_send_byte(crc, crc);
}

static uint8_t sport_send_byte(uint8_t byte, uint16_t crc)
{
  if (byte == START_STOP_BYTE) {
    (void) sportSerial.write(XOR_BYTE_FLAG);
    (void) sportSerial.write(START_STOP_BYTE ^ XOR_MASK); // 0x7E xor 0x20
  }
  else if (byte == XOR_BYTE_FLAG) {
    (void) sportSerial.write(XOR_BYTE_FLAG);
    (void) sportSerial.write(XOR_BYTE_FLAG ^ XOR_MASK); // 0x7D xor 0x20
  }
  else {
    (void) sportSerial.write(byte);
  }

  crc += byte;
  crc += (crc >> 8);
  crc &= 0x00ff;
  crc += (crc >> 8);
  crc &= 0x00ff;

  return crc;
}

static void sport_handle_sensor(uint8_t id)
{
  for (int i = 0; i < (sizeof(s_Sensors) / sizeof(s_Sensors[0])); i ++) {
    if (s_Sensors[i].id == id) {
      sport_set_serial(SERIAL_TX);
      s_Sensors[i].handler();
      sport_set_serial(SERIAL_RX);
      break;
    }
  }
}

#ifdef SPORT_VBAT
void sensor_handler_fcs(void)
{
  static int dataIndex = 0;
  static const uint8_t DATA_COUNT = 6;

  switch (dataIndex) {
    case 0: {
      sport_send_data(FCS_CURR_DATA_ID,
#ifdef SPORT_DEBUG_DATA
        400
#else
        0
#endif
        );
      break;
    }

    case 1: {
      extern uint16_t voltage;
      sport_send_data(FCS_VOLT_DATA_ID,
#ifdef SPORT_DEBUG_DATA
        1100
#else
        voltage
#endif
        );
      break;
    }

    default: {
      break;
    }
  }

  if(++dataIndex > DATA_COUNT) {
    dataIndex = 0;
  }
}
#endif

#ifdef SPORT_GPS
void sensor_handler_gps(void)
{
  static int dataIndex = 0;
  static const uint8_t DATA_COUNT = 6;
  extern GPS_t GPS;
  const bool fix =
#ifdef SPORT_DEBUG_DATA
    true
#else
    GPS.fix
#endif
    ;

#define CONVERT_LL(x) ((((x > 0 ? x : -x) * 60 * 10000) & 0x3FFFFFFF) | (x < 0 ? 0x40000000 : 0))
#define CONVERT_DT(x,y,z) ((((uint32_t)x << 24) & 0xFF000000LU) | (((uint32_t)y << 16) & 0x00FF0000LU) | (((uint32_t)z << 8) & 0x0000FF00LU))

  switch(dataIndex) {
    case 0: {
      sport_send_data(GPS_LAT_LON_DATA_ID, (fix ? CONVERT_LL(
#ifdef SPORT_DEBUG_DATA
        12345678
#else
        GPS.latitude
#endif
        ) : 0) | 0x80000000);
      break;
    }

    case 1: {
      sport_send_data(GPS_LAT_LON_DATA_ID, (fix ? CONVERT_LL(
#ifdef SPORT_DEBUG_DATA
        1020304
#else
        GPS.longitude
#endif
        ) : 0));
      break;
    }

    case 2: {
      sport_send_data(GPS_ALT_DATA_ID, (fix ? (
#ifdef SPORT_DEBUG_DATA
        80
#else
        GPS.altitude
#endif
        * 100) : 0));
      break;
    }

    case 3: {
      sport_send_data(GPS_SPEED_DATA_ID, (fix ? ((
#ifdef SPORT_DEBUG_DATA
        100
#else
        GPS.speed
#endif
        * 1944) / 100) : 0));
      break;
    }

    case 4: {
      sport_send_data(GPS_COG_DATA_ID, (fix ? (
#ifdef SPORT_DEBUG_DATA
        333
#else
        GPS.ground_course
#endif
        * 100) : 0 ));
      break;
    }

    case 5: {
      sport_send_data(GPS_DATE_TIME_DATA_ID, CONVERT_DT(15, 1, 1)|0xFF);
      break;
    }

    case 6: {
      sport_send_data(GPS_DATE_TIME_DATA_ID, CONVERT_DT(0, 0, 0));
      break;
    }

    default: {
      break;
    }
  }

  if(++dataIndex > DATA_COUNT) {
    dataIndex = 0;
  }
}
#endif

#ifdef SPORT_ACC
static void sensor_handler_acc(void)
{
  static int dataIndex = 0;
  static const uint8_t DATA_COUNT = 3;
  extern MW_imu_t MW_IMU;

  switch(dataIndex) {
    case 0: {
      sport_send_data(ACCX_FIRST_ID,
#ifdef SPORT_DEBUG_DATA
        12300
#else
        MW_IMU.accSmooth[0]
#endif
        );
      break;
    }

    case 1: {
      sport_send_data(ACCY_FIRST_ID,
#ifdef SPORT_DEBUG_DATA
        34500
#else
        MW_IMU.accSmooth[1]
#endif
        );
      break;
    }

    case 2: {
      sport_send_data(ACCZ_FIRST_ID,
#ifdef SPORT_DEBUG_DATA
        56700
#else
        MW_IMU.accSmooth[2]
#endif
        );
      break;
    }

    default: {
      break;
    }
  }

  if (++ dataIndex > DATA_COUNT) {
    dataIndex = 0;
  }
}
#endif

#endif

