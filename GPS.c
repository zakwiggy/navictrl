/*#######################################################################################*/
/*#######################################################################################*/

// IMPORTANT NOTE:

// This is only a dummy implementation for errorfree compiling of the NaviCtrl sources.

// The GPS navigation routines are NOT included !

/*#######################################################################################*/
/*#######################################################################################*/
/* !!! THIS IS NOT FREE SOFTWARE !!!  	                                                 */
/*#######################################################################################*/
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) 2008 Ingo Busker, Holger Buss
// + Nur für den privaten Gebrauch / NON-COMMERCIAL USE ONLY
// + FOR NON COMMERCIAL USE ONLY
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Es gilt für das gesamte Projekt (Hardware, Software, Binärfiles, Sourcecode und Dokumentation),
// + dass eine Nutzung (auch auszugsweise) nur für den privaten (nicht-kommerziellen) Gebrauch zulässig ist.
// + Sollten direkte oder indirekte kommerzielle Absichten verfolgt werden, ist mit uns (info@mikrokopter.de) Kontakt
// + bzgl. der Nutzungsbedingungen aufzunehmen.
// + Eine kommerzielle Nutzung ist z.B.Verkauf von MikroKoptern, Bestückung und Verkauf von Platinen oder Bausätzen,
// + Verkauf von Luftbildaufnahmen, usw.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Werden Teile des Quellcodes (mit oder ohne Modifikation) weiterverwendet oder veröffentlicht,
// + unterliegen sie auch diesen Nutzungsbedingungen und diese Nutzungsbedingungen incl. Copyright müssen dann beiliegen
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Sollte die Software (auch auszugesweise) oder sonstige Informationen des MikroKopter-Projekts
// + auf anderen Webseiten oder sonstigen Medien veröffentlicht werden, muss unsere Webseite "http://www.mikrokopter.de"
// + eindeutig als Ursprung verlinkt werden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Keine Gewähr auf Fehlerfreiheit, Vollständigkeit oder Funktion
// + Benutzung auf eigene Gefahr
// + Wir übernehmen keinerlei Haftung für direkte oder indirekte Personen- oder Sachschäden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die Portierung oder Nutzung der Software (oder Teile davon) auf andere Systeme (ausser der Hardware von www.mikrokopter.de) ist nur
// + mit unserer Zustimmung zulässig
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die Funktion printf_P() unterliegt ihrer eigenen Lizenz und ist hiervon nicht betroffen
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Redistributions of source code (with or without modifications) must retain the above copyright notice,
// + this list of conditions and the following disclaimer.
// +   * Neither the name of the copyright holders nor the names of contributors may be used to endorse or promote products derived
// +     from this software without specific prior written permission.
// +   * The use of this project (hardware, software, binary files, sources and documentation) is only permitted
// +     for non-commercial use (directly or indirectly)
// +     Commercial use (for excample: selling of MikroKopters, selling of PCBs, assembly, ...) is only permitted
// +     with our written permission
// +   * If sources or documentations are redistributet on other webpages, out webpage (http://www.MikroKopter.de) must be
// +     clearly linked as origin
// +   * porting the sources to other systems or using the software on other systems (except hardware from www.mikrokopter.de) is not allowed
//
// +  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// +  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// +  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// +  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// +  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// +  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// +  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// +  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// +  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// +  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// +  POSSIBILITY OF SUCH DAMAGE.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "91x_lib.h"
#include "main.h"
#include "uart1.h"
#include "GPS.h"
#include "timer1.h"
#include "spi_slave.h"
#include "waypoints.h"
#include "i2c.h"


#define M_PI_180 	(M_PI / 180.0f)
#define GPS_UPDATETIME_MS 200 		// 200ms is 5 Hz
typedef enum
{
	GPS_FLIGHT_MODE_UNDEF,
	GPS_FLIGHT_MODE_FREE,
	GPS_FLIGHT_MODE_AID,
	GPS_FLIGHT_MODE_WAYPOINT
} GPS_FlightMode_t;

typedef struct
{
	float Gain;
	float P;
	float I;
	float D;
	float A;
	float ACC;
	s32 P_Limit;
	s32 I_Limit;
	s32 D_Limit;
	s32 PID_Limit;
	u32 BrakingDuration;
	u8 MinSat;
	s8 StickThreshold;
	float WindCorrection;
	float SpeedCompensation;
	s32 OperatingRadius;
	GPS_FlightMode_t  FlightMode;
} __attribute__((packed)) GPS_Parameter_t;

typedef struct
{
	u8  Status;		// invalid, newdata, processed
	s32 North;		// in cm
	s32 East;		// in cm
	s32 Bearing;	// in deg
	s32 Distance;	// in cm
} __attribute__((packed)) GPS_Deviation_t;
GPS_Deviation_t CurrentTargetDeviation;		// Deviation from Target
GPS_Deviation_t CurrentHomeDeviation;		// Deviation from Home
GPS_Deviation_t TargetHomeDeviation;		// Deviation from Target to Home

GPS_Stick_t		GPS_Stick;
GPS_Parameter_t	GPS_Parameter;

// the gps reference positions
GPS_Pos_t GPS_HoldPosition 	= {0,0,0, INVALID};			// the hold position
GPS_Pos_t GPS_HomePosition 	= {0,0,0, INVALID};			// the home position
GPS_Pos_t * GPS_pTargetPosition = NULL;				    // pointer to the actual target position
u32 GPS_TargetRadius = 0;								// catch radius for target area
Waypoint_t* GPS_pWaypoint = NULL;						// pointer to the actual waypoint

//-------------------------------------------------------------
// Update GPSParamter
void GPS_UpdateParameter(void)
{
}

//-------------------------------------------------------------
// This function defines a good GPS signal condition
u8 GPS_IsSignalOK(void)
{
	return 0;
}

//------------------------------------------------------------
// Checks for manual control action
u8 GPS_IsManuallyControlled(void)
{
	return 0;
}

//------------------------------------------------------------
// copy GPS position from source position to target position
u8 GPS_CopyPosition(GPS_Pos_t * pGPSPosSrc, GPS_Pos_t* pGPSPosTgt)
{
	return 0;
}

//------------------------------------------------------------
// clear position data
u8 GPS_ClearPosition(GPS_Pos_t * pGPSPos)
{
	return 0;
}


//------------------------------------------------------------
void GPS_Neutral(void)
{
}

//------------------------------------------------------------
void GPS_Init(void)
{
}

//------------------------------------------------------------
// calculate the bearing to target position from its deviation
s32 DirectionToTarget_N_E(float northdev, float eastdev)
{
	return 0;
}


//------------------------------------------------------------
// Rescale xy-vector length if length limit is violated
// returns vector len after scaling
s32 GPS_LimitXY(s32 *x, s32 *y, s32 limit)
{
	return 0;
}

//------------------------------------------------------------
// transform the integer deg into float radians
inline double RadiansFromGPS(s32 deg)
{
	return 0.0;
}

//------------------------------------------------------------
// transform the integer deg into float deg
inline double DegFromGPS(s32 deg)
{
	return 0.0;
}

//------------------------------------------------------------
// calculate the deviation from the current position to the target position
u8 GPS_CalculateDeviation(GPS_Pos_t * pCurrentPos, GPS_Pos_t * pTargetPos, GPS_Deviation_t* pDeviationFromTarget)
{
	return 0;
}

void GPS_Navigation(gps_data_t *pGPS_Data, GPS_Stick_t* pGPS_Stick)
{
 	return;
}

void CalcHeadFree(void)
{
  return;
}
