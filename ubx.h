#ifndef __UBX_H
#define __UBX_H
#include "buffer.h"

// Satfix types for GPSData.SatFix
#define SATFIX_NONE				0x00
#define SATFIX_DEADRECKOING		0x01
#define SATFIX_2D				0x02
#define SATFIX_3D				0x03
#define SATFIX_GPS_DEADRECKOING	0x04
#define SATFIX_TIMEONLY			0x05
// Flags for interpretation of the GPSData.Flags
#define FLAG_GPSFIXOK			0x01 // (i.e. within DOP & ACC Masks)
#define FLAG_DIFFSOLN			0x02 // (is DGPS used)
#define FLAG_WKNSET				0x04 // (is Week Number valid)
#define FLAG_TOWSET				0x08 //	(is Time of Week valid)

#define	INVALID		0x00
#define NEWDATA		0x01
#define PROCESSED	0x02

typedef struct
{
	s32 Longitude;  // in 1E-7 deg
	s32 Latitude;	// in 1E-7 deg
	s32 Altitude;	// in mm
	u8 Status;// validity of data
} __attribute__((packed)) GPS_Pos_t;


typedef struct
{
	u16			MsgCycleTime;	// time in ms since last gps data
	GPS_Pos_t	Position;       // Lat/Lon/Alt
	u8			Flags;			// Status Flags
	u8			NumOfSats;		// number of satelites
	u8 			SatFix;			// type of satfix
	u32    		Position_Accuracy;	// in cm 3d position accuracy
	s32			Speed_North;	// in cm/s
	s32			Speed_East;		// in cm/s
	s32			Speed_Top;		// in cm/s
	u32			Speed_Ground;	// 2D ground speed in cm/s
	s32			Heading;		// 1e-05 deg  Heading 2-D (current flight direction)
	u32			Speed_Accuracy;	// in cm/s 3d velocity accuracy
	u8			Status;			// status of data
} __attribute__((packed)) gps_data_t;

// The data are valid if the GPSData.Status is NEWDATA or PROCESSED.
// To achieve new data after reading the GPSData.Status should be set to PROCESSED.
extern gps_data_t  GPSData;


#define UBX_CLASS_CFG	0x06
#define UBX_CLASS_ACK	0x05

typedef struct
{
 	u8 Class;
	u8 Id;
	u16 Length;
}  __attribute__((packed)) ubxmsghdr_t;

#define UBX_MSG_DATA_SIZE 200
typedef struct
{
	u8 ClassMask;
	u8 IdMask;
	ubxmsghdr_t Hdr;
	u8 Data[UBX_MSG_DATA_SIZE];
	u8 Status;
} __attribute__((packed)) ubxmsg_t;
// msg obj to reveive
// set Class and Id and correspoinding masks of a message that should be received
extern ubxmsg_t UbxMsg;


extern u32 UBX_Timeout;

void UBX_Init(void);
void UBX_RxParser(u8 c);
u8 UBX_CreateMsg(Buffer_t* pBuff, u8* pData, u16 Len);


#endif // __UBX_H
