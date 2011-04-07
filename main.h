#ifndef _MAIN_H
#define _MAIN_H

//-----------------------
//#define DEBUG 0
//-----------------------

#define VERSION_MAJOR	0
#define VERSION_MINOR	24
#define VERSION_PATCH	1
// 0 = A
// 1 = B
// 2 = C
// 3 = D
// 4 = E
// 5 = F
// 6 = G
// 7 = H
// 8 = I
// 9 = J
// 10 = k
// 11 = L
// 12 = M
// 13 = N
// 14 = o

#define VERSION_SERIAL_MAJOR	11
#define VERSION_SERIAL_MINOR	0

#define FC_SPI_COMPATIBLE		19
#define MK3MAG_I2C_COMPATIBLE	3

// FC STATUS FLAGS 
#define FC_STATUS_MOTOR_RUN  				0x01
#define FC_STATUS_FLY        				0x02
#define FC_STATUS_CALIBRATE  				0x04
#define FC_STATUS_START      				0x08
#define FC_STATUS_EMERGENCY_LANDING      	0x10
#define FC_STATUS_LOWBAT		      		0x20
#define FC_STATUS_VARIO_TRIM_UP	      		0x40
#define FC_STATUS_VARIO_TRIM_DOWN      		0x80

// FC STATUS FLAGS2
#define FC_STATUS2_CAREFREE                 0x01
#define FC_STATUS2_ALTITUDE_CONTROL         0x02


// FC ERRORS FLAGS
#define FC_ERROR0_GYRO_NICK 	0x01
#define FC_ERROR0_GYRO_ROLL 	0x02
#define FC_ERROR0_GYRO_YAW 		0x04
#define FC_ERROR0_ACC_NICK 		0x08
#define FC_ERROR0_ACC_ROLL 		0x10
#define FC_ERROR0_ACC_TOP  		0x20
#define FC_ERROR0_PRESSURE		0x40
#define FC_ERROR0_CAREFREE		0x80

#define FC_ERROR1_I2C   	 	0x01
#define FC_ERROR1_BL_MISSING 	0x02
#define FC_ERROR1_SPI_RX	 	0x04
#define FC_ERROR1_PPM	 		0x08
#define FC_ERROR1_MIXER			0x10
#define FC_ERROR1_RES1			0x20
#define FC_ERROR1_RES2			0x40
#define FC_ERROR1_RES3			0x80

// NC Errors
#define NCERR_FLAG_FC_COMPATIBLE			0x00000001
#define NCERR_FLAG_MK3MAG_COMPATIBLE		0x00000002
#define NCERR_FLAG_FC_COMMUNICATION			0x00000004
#define NCERR_FLAG_MK3MAG_COMMUNICATION		0x00000008
#define NCERR_FLAG_MKGPS_COMMUNICATION		0x00000010
#define NCERR_FLAG_BAD_COMPASS_HEADING		0x00000020
#define NCERR_FLAG_RC_SIGNAL_LOST			0x00000040
#define NCERR_FLAG_EEPROM_NOT_FOUND			0x00000080


#define LIMIT_MIN(value, min) {if(value <= min) value = min;}
#define LIMIT_MAX(value, max) {if(value >= max) value = max;}
#define LIMIT_MIN_MAX(value, min, max) {if(value <= min) value = min; else if(value >= max) value = max;}

extern u16 BeepTime;
extern u8  NCFlags;
extern u8 ClearFCStatusFlags;
void Interrupt_Init(void);
extern s16 GeoMagDec;


typedef struct
{
	u8 ActiveSetting;
	u8 User1;
	u8 User2;
	u8 User3;
	u8 User4;
	u8 User5;
	u8 User6;
	u8 User7;
	u8 User8;
	u8 NaviGpsModeControl;
	u8 NaviGpsGain;
	u8 NaviGpsP;
	u8 NaviGpsI;
	u8 NaviGpsD;
	u8 NaviGpsPLimit;
	u8 NaviGpsILimit;
	u8 NaviGpsDLimit;
	u8 NaviGpsACC;
	u8 NaviGpsMinSat;
	u8 NaviStickThreshold;
	u8 NaviOperatingRadius;
	u8 NaviWindCorrection;
	u8 NaviSpeedCompensation;
	u8 LowVoltageWarning;
	u8 NaviAngleLimitation;
	u8 NaviPH_LoginTime;
} __attribute__((packed)) Param_t;

typedef struct
{
	s8 StickNick;
	s8 StickRoll;
	s8 StickYaw;
	s8 StickGas;
	u8 Poti[8];
	u8 Poti5;
	u8 Poti6;
	u8 Poti7;
	u8 Poti8;
	u8 RC_Quality;
	u8 RC_RSSI;
	u8 BAT_Voltage;
	u16 BAT_Current;
	u16 BAT_UsedCapacity;
	u8 StatusFlags;
	u8 Error[5];
	u8 StatusFlags2;
} __attribute__((packed)) FC_t;


extern Param_t Parameter;
extern volatile FC_t FC;
extern s8 ErrorMSG[25];
extern u8 ErrorCode;
extern u8 StopNavigation;
#endif // _MAIN_H
