#ifndef _UART1_H
#define _UART1_H

#define UART_FLIGHTCTRL 0
#define UART_MK3MAG 	1
#define UART_MKGPS  	2

#include "ubx.h"

#define NC_ERROR0_SPI_RX  				0x01
#define NC_ERROR0_COMPASS_RX  			0x02
#define NC_ERROR0_FC_INCOMPATIBLE		0x04
#define NC_ERROR0_COMPASS_INCOMPATIBLE	0x08
#define NC_ERROR0_GPS_RX 				0x10
#define NC_ERROR0_COMPASS_VALUE			0x20

typedef struct
{
	u8 SWMajor;
	u8 SWMinor;
	u8 ProtoMajor;
	u8 ProtoMinor;
	u8 SWPatch;
	u8 HardwareError[5];
} __attribute__((packed)) UART_VersionInfo_t;

extern UART_VersionInfo_t UART_VersionInfo;

typedef struct
{
	s16 AngleNick;	// in 0.1 deg
	s16 AngleRoll;   // in 0.1 deg
	s16 Heading; 	// in 0.1 deg
	u8 reserve[8];
} __attribute__((packed)) Data3D_t;

extern Data3D_t Data3D;


extern const u8 ANALOG_LABEL[32][16];

typedef struct
{
	u8 Status[2];
	u16 Analog[32];    // Debugwerte
} __attribute__((packed)) DebugOut_t;

extern DebugOut_t DebugOut;

typedef struct
{
	u8	Digital[2];
	u8	RemoteButtons;
	s8	Nick;
	s8	Roll;
	s8	Yaw;
	u8	Gas;
	s8	Height;
	u8	free;
	u8	Frame;
	u8	Config;
} __attribute__((packed)) ExternControl_t;

extern ExternControl_t ExternControl;

typedef struct
{
 	s16 Nick;
	s16 Roll;
	s16 Compass;					// angle between north and head of the MK
} __attribute__((packed)) Attitude_t;

typedef struct
{
	u16 Distance;					// distance to target in dm
	s16 Bearing;					// course to target in deg
} __attribute__((packed)) GPS_PosDev_t;

#define NAVIDATA_VERSION 5

typedef struct
{
	u8 Version;						// version of the data structure
	GPS_Pos_t CurrentPosition;		// see ubx.h for details
	GPS_Pos_t TargetPosition;
	GPS_PosDev_t TargetPositionDeviation;
	GPS_Pos_t HomePosition;
	GPS_PosDev_t HomePositionDeviation;
	u8  WaypointIndex;				// index of current waypoints running from 0 to WaypointNumber-1
	u8  WaypointNumber;				// number of stored waypoints
	u8  SatsInUse;					// number of satellites used for position solution
	s16 Altimeter; 					// hight according to air pressure
	s16 Variometer;					// climb(+) and sink(-) rate
	u16 FlyingTime;					// in seconds
	u8  UBat;						// Battery Voltage in 0.1 Volts
	u16 GroundSpeed;				// speed over ground in cm/s (2D)
	s16 Heading;					// current flight direction in ° as angle to north
	s16	CompassHeading;				// current compass value in °
	s8  AngleNick;					// current Nick angle in 1°
	s8  AngleRoll;					// current Rick angle in 1°
	u8  RC_Quality;					// RC_Quality
	u8  FCStatusFlags;				// Flags from FC
	u8  NCFlags;					// Flags from NC
	u8  Errorcode;					// 0 --> okay
	u8  OperatingRadius;			// current operation radius around the Home Position in m
	s16 TopSpeed;					// velocity in vertical direction in cm/s
	u8  TargetHoldTime;				// time in s to stay at the given target, counts down to 0 if target has been reached
	u8  FCStatusFlags2;				// StatusFlags2 (since version 5 added)
	s16 SetpointAltitude;			// setpoint for altitude
	u8  Gas;						// for future use
	u16 Current;					// actual current in 0.1A steps
	u16 UsedCapacity;				// used capacity in mAh
} __attribute__((packed)) NaviData_t;

extern NaviData_t NaviData;

#define NC_FLAG_FREE			0x01
#define NC_FLAG_PH				0x02
#define NC_FLAG_CH				0x04
#define NC_FLAG_RANGE_LIMIT		0x08
#define NC_FLAG_NOSERIALLINK	0x10
#define NC_FLAG_TARGET_REACHED	0x20
#define NC_FLAG_MANUAL_CONTROL	0x40
#define NC_FLAG_GPS_OK			0x80

extern UART_TypeDef *DebugUART;
extern volatile u8 SerialLinkOkay;


void UART1_Init(void);
void UART1_Transmit(void);
void UART1_TransmitTxData(void);
void UART1_ProcessRxData(void);

s16  UART1_Putchar(char c);
void UART1_PutString(u8 *s);
extern u8 text[]; // globally used text buffer
extern u8 UART1_Request_SendFollowMe;
#endif //_UART1_H
