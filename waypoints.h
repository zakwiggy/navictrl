#ifndef _WAYPOINTS_H
#define _WAYPOINTS_H

#include "ubx.h"

#define POINT_TYPE_INVALID 255
#define POINT_TYPE_WP	0
#define POINT_TYPE_POI	1

typedef struct
{
 	GPS_Pos_t Position;		// the gps position of the waypoint, see ubx.h for details
	s16 Heading;			// orientation, 0 no action, 1...360 fix heading, neg. = Index to POI in WP List
	u8  ToleranceRadius;	// in meters, if the MK is within that range around the target, then the next target is triggered
	u8  HoldTime;			// in seconds, if the was once in the tolerance area around a WP, this time defines the delay before the next WP is triggered
	u8  Event_Flag;			// future implementation
	u8  Index;              // to indentify different waypoints, workaround for bad communications PC <-> NC
	u8  Type;				// typeof Waypoint
	u8  WP_EventChannelValue;  //
	u8 	AltitudeRate;		// rate to change the setpoint
	u8  reserve[8];		    // reserve
} __attribute__((packed)) Point_t;

// Init List, return TRUE on success
u8 PointList_Init(void);
// Clear List, return TRUE on success
u8 PointList_Clear(void);
// Returns number of points in the list
u8 PointList_GetCount(void);
// return pointer to point at position
Point_t* PointList_GetAt(u8 index);
// set a point in the list at index, returns its index on success, else 0
u8 PointList_SetAt(Point_t* pPoint);
// goto the first WP in the list and return pointer to it
Point_t* PointList_WPBegin(void);
// goto the last WP in the list and return pointer to it
Point_t* PointList_WPEnd(void);
// goto next WP in the list and return pointer to it
Point_t* PointList_WPNext(void);
// enables/disables waypoint function
void PointList_WPActive(u8 set);
// returns pointer to actual POI
Point_t* PointList_GetPOI(void);


#endif // _WAYPOINTS_H
