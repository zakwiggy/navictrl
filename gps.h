#ifndef __GPS_H
#define __GPS_H

#include "ubx.h"
#include "waypoints.h"

typedef struct
{
	s16 Nick;
	s16 Roll;
	s16 Yaw;
}  __attribute__((packed)) GPS_Stick_t;

#define CAM_UPDATE_AZIMUTH		0x01
#define CAM_UPDATE_ELEVATION	0x02
typedef struct
{
	s16 Azimuth;		// angle measured clockwise from north
	s16 Elevation; 		// angle measured upwards from horizont
	u8 UpdateMask;
} __attribute__((packed)) CAM_Orientation_t; 

extern CAM_Orientation_t CAM_Orientation;
extern Point_t* GPS_pWaypoint;
u8 GPS_CopyPosition(GPS_Pos_t * pGPSPosSrc, GPS_Pos_t* pGPSPosTgt);
void GPS_Init(void);
void GPS_Navigation(gps_data_t *pGPS_Data, GPS_Stick_t* pGPS_Stick);
void CalcHeadFree(void);

#define EVENTFLAG_1_WP_CHANNEL    1
#define EVENTFLAG_2_WP_CHANNEL    2

#endif //__GPS_H

