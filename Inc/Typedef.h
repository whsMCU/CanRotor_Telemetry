#pragma once

enum rc {
  ROLL = 0,
  PITCH,
  YAW,
  THROTTLE,
  GEAR,
  AUX1,
  AUX2
};

typedef struct pid_t{
float kp[3];
float ki[3];
float kd[3];
} pid_t;

typedef struct button_t{
	uint8_t page;
  uint8_t	pos;
  uint8_t previous_page;
	uint8_t page_pressed;
	
	uint8_t parm_setting;
	uint8_t pos_flag;
	uint8_t send_flag;
} button_t;

typedef struct flags_t {
    uint8_t OK_TO_ARM;
    uint8_t ARMED;
    uint8_t ACC_CALIBRATED;
    uint8_t ANGLE_MODE;
    uint8_t HORIZON_MODE;
    uint8_t MAG_MODE;
    uint8_t BARO_MODE;
	  uint8_t User_MODE;
    uint8_t GPS_HOME_MODE;
    uint8_t GPS_HOLD_MODE;
    uint8_t HEADFREE_MODE;
    uint8_t PASSTHRU_MODE;
    uint8_t GPS_FIX;
    uint8_t GPS_FIX_HOME;
    uint8_t SMALL_ANGLE;
    uint8_t CALIBRATE_MAG;
    uint8_t VARIO_MODE;
    uint8_t FIXED_WING;                     // set when in flying_wing or airplane mode. currently used by althold selection code
    uint8_t MOTORS_STOPPED;
    uint8_t FW_FAILSAFE_RTH_ENABLE;
    uint8_t CLIMBOUT_FW;
    uint8_t CRUISE_MODE;
} flags_t;

typedef struct {
  uint32_t  capture_rise[8];
  uint32_t  capture_fall[8];
  int16_t  rcADC[8];
  int16_t  rcCommand[7];
} rc;

typedef struct {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
	int16_t  Temp[1];
  int16_t  gyroADC[3];
  int16_t  accADC[3];

  int16_t  acctemp[3];
  float  accRaw[3];
  float  gyroRaw[3];
  float  magRaw[3];
  float  gyrotemp[3][2];
  float  gyroaver[3];
  float    accx;
  float    accy;
	float  angle[3];
	
	float  heading;
	
} imu_t;

typedef struct {
	uint32_t armedTime;
	uint16_t cycleTime;
	uint8_t mode, error, errors_count;
} st_t;

typedef struct _TM_AHRSIMU_t {
    float Roll;             /*!< Roll angle value. This parameter is in units of degrees */
    float Pitch;            /*!< Pitch angle value. This parameter is in units of degrees */
    float Yaw;              /*!< Yaw angle value. This parameter is in units of degrees */
    float Inclination;      /*!< Inclination in units of degrees */

    float _beta;
    float _q0, _q1, _q2, _q3;
    float _sampleRate;
} TM_AHRSIMU_t;

typedef struct gps_t {
  char GPS[120];

  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
  uint8_t year;
  uint8_t month;
  uint8_t day;

  uint16_t milliseconds;

  float latitude;
  float longitude;

  uint32_t latitude_fixed;
  uint32_t longitude_fixed;

  float latitudeDegrees;
  float longitudeDegrees;

  float geoidheight;
  float altitude;

  float speed;
  float angle;
  float magvariation;
  float HDOP;

  char lat;
  char lon;
  char mag;

  bool fix;

  uint8_t fixquality;
  uint8_t satellites;

  uint8_t lineidx;
  uint8_t recvdflag;

  uint32_t error;

} gps_t;

typedef struct {
  uint16_t fc[6];
  uint8_t ct;
  uint8_t uosr;
  int32_t TEMP2;
  int32_t dT, dT_C5;
  int64_t OFF, SENS;
  int64_t OFF1, SENS1;
  int64_t OFF2, SENS2;

  uint32_t rawTemp;
  uint32_t realTemperature;

  uint32_t rawPressure;
  int64_t realPressure;

  int32_t BaroAlt;
	int32_t EstAlt;

} ms5611_t;
