#ifndef _PARAMS_H
#define _PARAMS_H

// ids 0...255
#define NCPARAMS_GPS_TARGETSPEED 				0
#define NCPARAMS_NEW_COMPASS_DIRECTION_SETPOINT 1
#define NCPARAMS_NEW_CAMERA_ELEVATION           2
#define NCPARAMS_ALTITUDE_RATE  				3
#define NCPARAMS_ALTITUDE_SETPOINT              4

#define NCPARAM_STATE_UNDEFINED 	0
#define NCRARAM_STATE_VALID			1

extern void NCParams_Init();
extern u8 NCParams_SetValue(u8 id, s16 *pvalue);
extern u8 NCParams_GetValue(u8 id, s16 *pvalue);
extern void NCParams_ClearValue(u8 id);

#endif // _PARAMS_H
