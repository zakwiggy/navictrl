#include "91x_lib.h"
#include "params.h"
#include "spi_slave.h"
#include "waypoints.h"

s16 NCParams[255];
u8 NCParamState[255];

void NCParams_Init(void)
{
	u16 i;
	for(i=0; i<256; i++)
	{
		NCParamState[i] = NCPARAM_STATE_UNDEFINED;
		NCParams[i] = 0;
	}
 	NCParams[NCPARAMS_GPS_TARGETSPEED] = 50; // 5.0 m/s
 	NCParams[NCPARAMS_NEW_COMPASS_DIRECTION_SETPOINT] = -1;
 	NCParams[NCPARAMS_ALTITUDE_RATE] = 0;
}

u8 NCParams_SetValue(u8 id, s16 *pvalue)
{
	NCParams[id] = *pvalue;
	NCParamState[id] = 	NCRARAM_STATE_VALID;

	switch(id)
	{
		case NCPARAMS_NEW_COMPASS_DIRECTION_SETPOINT:
		    if(NULL == PointList_GetPOI())
			{/*
				CAM_Orientation.Azimuth = *pvalue;
				CAM_Orientation.Elevation = 0;
				CAM_Orientation.UpdateMask = CAM_UPDATE_AZIMUTH;*/
			}
			break;

		default:
			break;
	}
	return(NCParamState[id]);
}

void NCParams_ClearValue(u8 id)
{
	NCParamState[id] = 	NCPARAM_STATE_UNDEFINED;
}


u8 NCParams_GetValue(u8 id, s16 *pvalue)
{
	if(pvalue == 0) return 0;
	if(NCParamState[id] == NCRARAM_STATE_VALID)
	{
		*pvalue = NCParams[id];
	}
	return(NCParamState[id]);
}

