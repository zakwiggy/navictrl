#ifndef _SETTINGS_H
#define _SETTINGS_H

#include "91x_lib.h"

typedef enum
{
	PID_KML_LOGGING,
	PID_GPX_LOGGING,
	PID_GPS_AUTOCONFIG
} ParamId_t;

void Settings_Init(void);
void Settings_SetDefaultValues(void);
u8 Settings_GetParamValue(ParamId_t Pid, u16* pValue);

#endif // _SETTINGS_H


