#ifndef _SPI_SLAVE_H
#define _SPI_SLAVE_H

#include "fifo.h"
#include "gps.h"


#define SS_PIN GPIO_ReadBit(GPIO2, GPIO_Pin_7)

#define SPI_FCCMD_USER			10
#define SPI_FCCMD_STICK    	 	11
#define SPI_FCCMD_MISC			12
#define SPI_FCCMD_PARAMETER1	13
#define SPI_FCCMD_VERSION		14
#define SPI_FCCMD_SERVOS		15
#define SPI_FCCMD_ACCU			16

extern s32 Kalman_K;
extern s32 Kalman_MaxDrift;
extern s32 Kalman_MaxFusion;
extern s32 ToFcGpsZ;
extern s32 ToFC_Rotate_C, ToFC_Rotate_S;
extern s32 HeadFreeStartAngle;
extern s16 FC_WP_EventChannel;
extern u32 ToFC_AltitudeRate;
extern s32 ToFC_AltitudeSetpoint;

typedef struct
{
	u8   Command;
	s16  AngleNick;	 // NickAngle in 0.1 deg
	s16  AngleRoll;	 // RollAngle in 0.1 deg
	s16  AccNick;
	s16  AccRoll;
	s16  GyroHeading; // Heading in 0.1 deg
	s16  GyroNick;
	s16  GyroRoll;
	s16  GyroYaw;
	u16  FCStatus;
	union
	{
		u8    Byte[12];
		s8	  sByte[12];
		u16   Int[6];
		s16   sInt[6];
		u32   Long[3];
		s32   sLong[3];
		float Float[3];
	} Param;
	u8 Chksum;
 } __attribute__((packed)) FromFlightCtrl_t;

#define SPI_NCCMD_OSD_DATA		100
#define SPI_NCCMD_GPS_POS		101
#define SPI_NCCMD_GPS_TARGET	102
#define SPI_NCCMD_KALMAN		103
#define SPI_NCCMD_VERSION		104
#define SPI_NCCMD_GPSINFO		105

typedef struct
{
	u8 Command;
	GPS_Stick_t GPSStick;
	s16 CompassHeading;
	s16 MagVecX;
	s16 MagVecY;
	s16 MagVecZ;
	s16 NCStatus;
	u16 BeepTime;
	union
	{
		u8    Byte[12];
		s8    sByte[12];
		u16	  Int[6];
		s16   sInt[6];
		u32   Long[3];
		s32   sLong[3];
		float Float[3];
	}Param;
  	u8 Chksum;
 } __attribute__((packed)) ToFlightCtrl_t;


typedef struct
{
   u8 Major;
   u8 Minor;
   u8 Patch;
   u8 Compatible;
   u8 Hardware;
} __attribute__((packed)) SPI_Version_t;

extern FromFlightCtrl_t   FromFlightCtrl;
extern ToFlightCtrl_t     ToFlightCtrl;
extern volatile u32 SPI0_Timeout;
extern SPI_Version_t FC_Version;

void SPI0_Init(void);
void SPI0_GetFlightCtrlVersion(void);
void SPI0_UpdateBuffer(void);



#endif //_SPI_SLAVE_H
