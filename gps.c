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
#include "gps.h"
#include "timer1.h"
#include "spi_slave.h"
#include "waypoints.h"
#include "i2c.h"
#include "mymath.h"
#include "params.h"
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
	s32 Bearing;		// in deg
	s32 Distance;		// in cm
} __attribute__((packed)) GPS_Deviation_t;
GPS_Deviation_t CurrentTargetDeviation;		// Deviation from Target
GPS_Deviation_t CurrentHomeDeviation;		// Deviation from Home
GPS_Deviation_t TargetHomeDeviation;		// Deviation from Target to Home
GPS_Deviation_t POIDeviation;
GPS_Stick_t		GPS_Stick;
GPS_Parameter_t	GPS_Parameter;
CAM_Orientation_t CAM_Orientation;
// the gps reference positions
GPS_Pos_t GPS_POIPosition	= {0,0,0, INVALID};
GPS_Pos_t GPS_HoldPosition 	= {0,0,0, INVALID};			// the hold position
GPS_Pos_t GPS_HomePosition 	= {0,0,0, INVALID};			// the home position
GPS_Pos_t * GPS_pTargetPosition = NULL;				    // pointer to the actual target position
u32 GPS_TargetRadius = 0;								// catch radius for target area
Point_t* GPS_pWaypoint = NULL;						// pointer to the actual waypoint

//-------------------------------------------------------------
// Update GPSParamter
void GPS_UpdateParameter(void)
{
 	static GPS_FlightMode_t FlightMode_Old = GPS_FLIGHT_MODE_UNDEF;
	// in case of bad receiving conditions
	if(FC.RC_Quality < 100)
	{	// set fixed parameter
		GPS_Parameter.FlightMode = GPS_FLIGHT_MODE_WAYPOINT;
		GPS_Parameter.Gain 	= (float) 100;
		GPS_Parameter.P		= (float) 90;
		GPS_Parameter.I		= (float) 90;
		GPS_Parameter.D		= (float) 90;
		GPS_Parameter.A		= (float) 90;
		GPS_Parameter.ACC 	= (float) 0;
		GPS_Parameter.P_Limit = 90;
		GPS_Parameter.I_Limit = 90;
		GPS_Parameter.D_Limit = 90;
		GPS_Parameter.PID_Limit = 200;
		GPS_Parameter.BrakingDuration = 0;
		GPS_Parameter.SpeedCompensation = (float) 30;
		GPS_Parameter.MinSat = 6;
		GPS_Parameter.StickThreshold = 8;
		GPS_Parameter.WindCorrection = 0.0;
		GPS_Parameter.OperatingRadius = 0; // forces the aircraft to fly to home positon

	}
	else
	{
		// update parameter from FC
		if(StopNavigation) GPS_Parameter.FlightMode = GPS_FLIGHT_MODE_FREE;
		else
		{
			if(Parameter.NaviGpsModeControl <  50)
			{
				GPS_Parameter.FlightMode = GPS_FLIGHT_MODE_FREE;
				NCFlags &= ~(NC_FLAG_PH | NC_FLAG_CH);
				NCFlags |= NC_FLAG_FREE;
			}
			else if(Parameter.NaviGpsModeControl < 180)
			{
				GPS_Parameter.FlightMode = GPS_FLIGHT_MODE_AID;
				NCFlags &= ~(NC_FLAG_FREE | NC_FLAG_CH);
				NCFlags |= NC_FLAG_PH;
			}
			else
			{
				GPS_Parameter.FlightMode = GPS_FLIGHT_MODE_WAYPOINT;
				NCFlags &= ~(NC_FLAG_FREE | NC_FLAG_PH);
				NCFlags |= NC_FLAG_CH;
			}
		}
		GPS_Parameter.Gain 	= (float)Parameter.NaviGpsGain;
		GPS_Parameter.P		= (float)Parameter.NaviGpsP;
		GPS_Parameter.I		= (float)Parameter.NaviGpsI;
		GPS_Parameter.D		= (float)Parameter.NaviGpsD;
		GPS_Parameter.A		= (float)Parameter.NaviGpsD;
		GPS_Parameter.ACC 	= (float)Parameter.NaviGpsACC;
		GPS_Parameter.P_Limit = (s32)Parameter.NaviGpsPLimit;
		GPS_Parameter.I_Limit = (s32)Parameter.NaviGpsILimit;
		GPS_Parameter.D_Limit = (s32)Parameter.NaviGpsDLimit;
		GPS_Parameter.PID_Limit = 2* (u32)Parameter.NaviAngleLimitation;
		GPS_Parameter.BrakingDuration = (u32)Parameter.NaviPH_LoginTime;
		GPS_Parameter.SpeedCompensation = (float)Parameter.NaviSpeedCompensation;
		GPS_Parameter.MinSat = (u8)Parameter.NaviGpsMinSat;
		GPS_Parameter.StickThreshold = (s8)Parameter.NaviStickThreshold;
		GPS_Parameter.WindCorrection = (float)Parameter.NaviWindCorrection;
		GPS_Parameter.OperatingRadius = (s32)Parameter.NaviOperatingRadius * 100; // conversion of m to cm
	}
	// FlightMode changed?
	if(GPS_Parameter.FlightMode != FlightMode_Old)
	{
		BeepTime = 100; // beep to indicate that mode has been switched
		NCFlags &= ~NC_FLAG_TARGET_REACHED;

		if(GPS_Parameter.FlightMode == GPS_FLIGHT_MODE_WAYPOINT)
		{

			GPS_pWaypoint = PointList_WPBegin(); // reset WPList to begin
			
		}	
	}
	FlightMode_Old = GPS_Parameter.FlightMode;
}

//-------------------------------------------------------------
// This function defines a good GPS signal condition
u8 GPS_IsSignalOK(void)
{
	if( (GPSData.Status != INVALID) && (GPSData.SatFix == SATFIX_3D) &&  (GPSData.NumOfSats >= GPS_Parameter.MinSat)) return(1);
	else return(0);
}

//------------------------------------------------------------
// Checks for manual control action
u8 GPS_IsManuallyControlled(void)
{
	if( ( (abs(FC.StickNick) > GPS_Parameter.StickThreshold) || (abs(FC.StickRoll) > GPS_Parameter.StickThreshold)) && (GPS_Parameter.StickThreshold > 0) && (FC.RC_Quality > 150) )
	{
		NCFlags |= NC_FLAG_MANUAL_CONTROL;
		return(1);
	}
	else
	{
		NCFlags &= ~NC_FLAG_MANUAL_CONTROL;
		return(0);
	}
}

//------------------------------------------------------------
// copy GPS position from source position to target position
u8 GPS_CopyPosition(GPS_Pos_t * pGPSPosSrc, GPS_Pos_t* pGPSPosTgt)
{
	u8 retval = 0;
	if((pGPSPosSrc == NULL) || (pGPSPosTgt == NULL)) return(retval);	// bad pointer
	// copy only valid positions
	if(pGPSPosSrc->Status != INVALID)
	{
		// if the source GPS position is not invalid
		pGPSPosTgt->Longitude	= pGPSPosSrc->Longitude;
		pGPSPosTgt->Latitude	= pGPSPosSrc->Latitude;
		pGPSPosTgt->Altitude	= pGPSPosSrc->Altitude;
		pGPSPosTgt->Status 		= NEWDATA; // mark data in target position as new
		retval = 1;
	}
	return(retval);
}

//------------------------------------------------------------
// clear position data
u8 GPS_ClearPosition(GPS_Pos_t * pGPSPos)
{
 	u8 retval = FALSE;
	if(pGPSPos == NULL) return(retval);	// bad pointer
	else
	{
		pGPSPos->Longitude	= 0;
		pGPSPos->Latitude	= 0;
		pGPSPos->Altitude	= 0;
		pGPSPos->Status 	= INVALID;
		retval = TRUE;
	}
	return (retval);
}


//------------------------------------------------------------
void GPS_Neutral(void)
{
	GPS_Stick.Nick 	= 0;
	GPS_Stick.Roll 	= 0;
	GPS_Stick.Yaw 	= 0;
}

//------------------------------------------------------------
void GPS_Init(void)
{
	UART1_PutString("\r\n GPS init...");
	UBX_Init();
	GPS_Neutral();
	GPS_ClearPosition(&GPS_HoldPosition);
	GPS_ClearPosition(&GPS_HomePosition);
	GPS_pTargetPosition = NULL;
	PointList_Init();
	GPS_pWaypoint = PointList_WPBegin();
	
	GPS_UpdateParameter();
	UART1_PutString("ok");
	CAM_Orientation.Azimuth=-1;		// angle measured clockwise from north
	s16 Elevation=0; 		// angle measured upwards from horizon
	u8 UpdateMask=0;
}

//------------------------------------------------------------
// calculate the bearing to target position from its deviation
s32 DirectionToTarget_N_E(float northdev, float eastdev)
{
	s32 bearing;
	bearing = (s32)(atan2(northdev, eastdev) / M_PI_180);
	bearing = (270L - bearing)%360L;
	return(bearing);
}


//------------------------------------------------------------
// Rescale xy-vector length if length limit is violated
// returns vector len after scaling
s32 GPS_LimitXY(s32 *x, s32 *y, s32 limit)
{
	s32 dist;

	dist = (s32)hypot(*x,*y);	// the length of the vector
	if (dist > limit)
	// if vector length is larger than the given limit
	{	// scale vector compontents so that the length is cut off to limit
		*x = (s32)(( (double)(*x) * (double)limit ) / (double)dist);
		*y = (s32)(( (double)(*y) * (double)limit ) / (double)dist);
		dist = limit;
	}
	return(dist);
}

//------------------------------------------------------------
// transform the integer deg into float radians
inline double RadiansFromGPS(s32 deg)
{
  return ((double)deg * 1e-7f * M_PI_180); // 1E-7 because deg is the value in ° * 1E7
}

//------------------------------------------------------------
// transform the integer deg into float deg
inline double DegFromGPS(s32 deg)
{
  return ((double)deg  * 1e-7f); // 1E-7 because deg is the value in ° * 1E7
}

//------------------------------------------------------------
// calculate the deviation from the current position to the target position
u8 GPS_CalculateDeviation(GPS_Pos_t * pCurrentPos, GPS_Pos_t * pTargetPos, GPS_Deviation_t* pDeviationFromTarget)
{

	double temp1, temp2;
	// if given pointer is NULL
	if((pCurrentPos == NULL) || (pTargetPos == NULL)) goto baddata;
	// if positions	are invalid
	if((pCurrentPos->Status == INVALID) || (pTargetPos->Status == INVALID)) goto baddata;

	// The deviation from the current to the target position along north and east direction is
	// simple the lat/lon difference. To convert that angular deviation into an
	// arc length the spherical projection has to be considered.
	// The mean earth radius is 6371km. Therfore the arc length per latitude degree
	// is always 6371km * 2 * Pi / 360deg =  111.2 km/deg.
	// The arc length per longitude degree depends on the correspondig latitude and
	// is 111.2km * cos(latitude).

	// calculate the shortest longitude deviation from target
	temp1 = DegFromGPS(pCurrentPos->Longitude) - DegFromGPS(pTargetPos->Longitude);
	// outside an angular difference of -180 deg ... +180 deg its shorter to go the other way around
	// In our application we wont fly more than 20.000 km but along the date line this is important.
	if(temp1 > 180.0f) temp1 -= 360.0f;
	else if (temp1 < -180.0f) temp1 += 360.0f;
	temp1 *= cos((RadiansFromGPS(pTargetPos->Latitude) + RadiansFromGPS(pCurrentPos->Latitude))/2);
	// calculate latitude deviation from target
	// this is allways within -180 deg ... 180 deg
	temp2 = DegFromGPS(pCurrentPos->Latitude) - DegFromGPS(pTargetPos->Latitude);
	// deviation from target position in cm
	// i.e. the distance to walk from the target in northern and eastern direction to reach the current position

	pDeviationFromTarget->Status = INVALID;
	pDeviationFromTarget->North = (s32)(11119492.7f * temp2);
	pDeviationFromTarget->East  = (s32)(11119492.7f * temp1);
	// If the position deviation is small enough to neglect the earth curvature
	// (this is for our application always fulfilled) the distance to target
	// can be calculated by the pythagoras of north and east deviation.
	pDeviationFromTarget->Distance = (s32)(11119492.7f * hypot(temp1, temp2));
	if (pDeviationFromTarget->Distance == 0L) pDeviationFromTarget->Bearing = 0L;
	else pDeviationFromTarget->Bearing = DirectionToTarget_N_E(temp2, temp1);
	pDeviationFromTarget->Status = NEWDATA;
	return TRUE;

	baddata:
	pDeviationFromTarget->North 	= 0L;
	pDeviationFromTarget->East 		= 0L;
	pDeviationFromTarget->Distance 	= 0L;
	pDeviationFromTarget->Bearing 	= 0L;
	pDeviationFromTarget->Status = INVALID;
	return FALSE;
}
s32	GPSPosDevIntegral_North,GPSPosDevIntegral_East;
void GPS_Navigation(gps_data_t *pGPS_Data, GPS_Stick_t* pGPS_Stick)
{
	s32 D_North,D_East,P_North,P_East,I_North,I_East,PID_North,PID_East;
	s32	coscompass,PID_Nick,PID_Roll,sincompass;
	static u32 beep_rythm;
	static u32 GPSDataTimeout = 0;

	// pointer to current target position
	static GPS_Pos_t * pTargetPositionOld = NULL;
	static Point_t* GPS_pWaypointOld = NULL;

	static GPS_Pos_t RangedTargetPosition = {0,0,0, INVALID};		// the limited target position, this is derived from the target position with repect to the operating radius
	static s32 OperatingRadiusOld = -1;
	static u32 WPTime = 0;
		DebugOut.Analog[23] = GPSPosDevIntegral_North;
		DebugOut.Analog[24] = GPSPosDevIntegral_East;
			if(PointList_GetPOI()!=NULL)
				{
					DebugOut.Analog[25] = PointList_GetPOI()->Position.Longitude/10000;
					DebugOut.Analog[26] = PointList_GetPOI()->Position.Latitude/10000;				
				//GPS_CalculateDeviation(&(GPSData.Position), &(PointList_GetPOI()->Position), &POIDeviation);
				}
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+ Check for new data from GPS-receiver
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	switch(GPSData.Status)
	{
	 	case INVALID: // no gps data available
			// do nothing
			
			GPS_Parameter.PID_Limit = 0; // disables PID output
			break;

		case PROCESSED: // the current data have been allready processed
			// if no new data are available within the timeout switch to invalid state.
			if(CheckDelay(GPSDataTimeout)) GPSData.Status = INVALID;
			// wait for new gps data
			break;

		case NEWDATA: // handle new gps data
		
	        // update GPS Parameter from FC-Data via SPI interface
					
         	GPS_UpdateParameter();

			// wait maximum of 3 times the normal data update time before data timemout
			GPSDataTimeout = SetDelay(3 * GPS_UPDATETIME_MS);
			beep_rythm++;

			// debug
			DebugOut.Analog[21] = (s16)GPSData.Speed_North;
			DebugOut.Analog[22] = (s16)GPSData.Speed_East;
			DebugOut.Analog[31] = (s16)GPSData.NumOfSats;

			// If GPS signal condition is sufficient for a reliable position measurement
			if(GPS_IsSignalOK())
	  		{
				// update home deviation info
				GPS_CalculateDeviation(&(GPSData.Position), &GPS_HomePosition, &CurrentHomeDeviation);

				// if the MK is starting or the home position is invalid then store the home position
				if((FC.StatusFlags & FC_STATUS_START) || (GPS_HomePosition.Status == INVALID))
				{	// try to update the home position from the current position
					if(GPS_CopyPosition(&(GPSData.Position), &GPS_HomePosition))
					{
						BeepTime = 700; // beep on success
						GPS_CopyPosition(&GPS_HomePosition, &(NaviData.HomePosition));
					}
					GPS_pWaypoint = PointList_WPBegin(); // go to start of waypoint list, return NULL of the list is empty
				
				}

				/* The selected flight mode influences the target position pointer and therefore the behavior */

				// check for current flight mode and set the target pointer GPS_pTargetPosition	respectively
				switch(GPS_Parameter.FlightMode)
				{
					// the GPS control is deactived
					case GPS_FLIGHT_MODE_FREE:
					
						GPS_Parameter.PID_Limit = 0; // disables PID output
						// update hold position
						GPS_CopyPosition(&(GPSData.Position), &GPS_HoldPosition);
						// no target position
						GPS_pTargetPosition = NULL;
						GPS_TargetRadius = 0;
						break;

					// the GPS supports the position hold, if the pilot takes no action
					case GPS_FLIGHT_MODE_AID:

						if(GPS_IsManuallyControlled())
						{
							
							GPS_Parameter.PID_Limit = 0; // disables PID output, as long as the manual conrol is active
						    	GPS_CopyPosition(&(GPSData.Position), &GPS_HoldPosition);
							GPS_pTargetPosition = NULL;
							GPS_TargetRadius = 0;
						}
						else
						{
							GPS_pTargetPosition = &GPS_HoldPosition;
							GPS_TargetRadius = 100; // 1 meter
						}
						break;

					// the GPS control is directed to a target position
					// given by a waypoint or by the home position
					case GPS_FLIGHT_MODE_WAYPOINT:

						if(GPS_IsManuallyControlled()) // the human pilot takes the action
						{
							
							GPS_Parameter.PID_Limit = 0; // disables PID output, as long as the manual conrol is active
						    	GPS_CopyPosition(&(GPSData.Position), &GPS_HoldPosition);  // update hold position
							GPS_pTargetPosition = NULL;	// set target position invalid
							GPS_TargetRadius = 0;
						}
						else // no manual control  -> gps position hold active
						{
							// waypoint trigger logic
							if(GPS_pWaypoint != NULL) // waypoint exist
							{
								
								if(GPS_pWaypoint->Position.Status == INVALID) // should never happen
								{
									GPS_pWaypoint = PointList_WPNext(); // goto to next WP
									BeepTime = 255;
								}
								else // waypoint position is valid
								{
									// check if the pointer to the waypoint has been changed or the data have been updated
									if((GPS_pWaypoint != GPS_pWaypointOld) || (GPS_pWaypoint->Position.Status == NEWDATA))
									{
										
										GPS_pWaypointOld = GPS_pWaypoint;
									}
									// if WP has been reached once, wait hold time before trigger to next one
									if(NCFlags & NC_FLAG_TARGET_REACHED)
									{
										/* ToDo: Adjust GPS_pWaypoint->Heading, GPS_pWaypoint->Event handling */
										if(CheckDelay(WPTime))
										{
											
								 			GPS_pWaypoint = PointList_WPNext(); // goto to next waypoint, return NULL if end of list has been reached
							}
									} // EOF if(WPArrived)
									else
									{
									 	WPTime = SetDelay(GPS_pWaypoint->HoldTime * 1000); // set hold time stamp
									}
								}
							} // EOF waypoint trigger logic

							if(GPS_pWaypoint != NULL) // Waypoint exist
							{
								// update the hold position
								GPS_CopyPosition(&(GPSData.Position), &GPS_HoldPosition);
							 	GPS_pTargetPosition = &(GPS_pWaypoint->Position);
								GPS_TargetRadius = (s32)(GPS_pWaypoint->ToleranceRadius) * 100L;

							}
							else // no waypoint info available, i.e. the WPList is empty or the end of the list has been reached
							{
								// fly back to home postion
								if(GPS_HomePosition.Status == INVALID)
								{
									GPS_pTargetPosition = &GPS_HoldPosition; // fall back to hold mode if home position is not a
									GPS_TargetRadius = 100;
									BeepTime = 255; // beep to indicate missin home position
								}
								else // the home position is valid
								{
									// update the hold position
									GPS_CopyPosition(&(GPSData.Position), &GPS_HoldPosition);
									// set target to home position
									GPS_pTargetPosition = &GPS_HomePosition;
									GPS_TargetRadius = 100;
								}
							}
						} // EOF no manual control
						break;

					case GPS_FLIGHT_MODE_UNDEF:
					default:
						
						GPS_Parameter.PID_Limit = 0; // disables PID output
						// update hold position
						GPS_CopyPosition(&(GPSData.Position), &GPS_HoldPosition);
						// no target position
						GPS_pTargetPosition = NULL;
						GPS_TargetRadius = 0;
						break;

				}// EOF GPS Mode Handling


				/* Calculation of range target based on the real target */

				// if no target position exist clear the ranged target position
				if(GPS_pTargetPosition == NULL) GPS_ClearPosition(&RangedTargetPosition);
				else
				{	// if the target position has been changed or the value has been updated or the OperatingRadius has changed
					if((GPS_pTargetPosition != pTargetPositionOld)  || (GPS_pTargetPosition->Status == NEWDATA) || (GPS_Parameter.OperatingRadius != OperatingRadiusOld) )
					{
						GPSPosDevIntegral_North = 0;
						GPSPosDevIntegral_East = 0;
						BeepTime = 255; // beep to indicate setting of a new target position
						NCFlags &= ~NC_FLAG_TARGET_REACHED;	// clear target reached flag
						// calculate deviation of new target position from home position
						if(GPS_CalculateDeviation(GPS_pTargetPosition, &GPS_HomePosition, &TargetHomeDeviation))
						{
							// check distance from home position
							/*if(TargetHomeDeviation.Distance > GPS_Parameter.OperatingRadius)
							{
								//calculate ranged target position to be within the operation radius area
								NCFlags |= NC_FLAG_RANGE_LIMIT;

								TargetHomeDeviation.North = (s32)(((float)TargetHomeDeviation.North * (float)GPS_Parameter.OperatingRadius) / (float)TargetHomeDeviation.Distance);
								TargetHomeDeviation.East = (s32)(((float)TargetHomeDeviation.East * (float)GPS_Parameter.OperatingRadius) / (float)TargetHomeDeviation.Distance);

	



								RangedTargetPosition.Status = INVALID;
								RangedTargetPosition.Latitude = GPS_HomePosition.Latitude;
								RangedTargetPosition.Latitude += (s32)((float)TargetHomeDeviation.North / 1.11194927f);
								RangedTargetPosition.Longitude = GPS_HomePosition.Longitude;
								RangedTargetPosition.Longitude += (s32)((float)TargetHomeDeviation.East / (1.11194927f * cos(RadiansFromGPS(GPS_HomePosition.Latitude))) );
								RangedTargetPosition.Altitude = GPS_pTargetPosition->Altitude;
								RangedTargetPosition.Status = NEWDATA;
							 ToleranceRadius;	}
							else
							{	*/// the target is located within the operation radius area
								// simple copy the loaction to the ranged target position
							 	GPS_CopyPosition(GPS_pTargetPosition, &RangedTargetPosition);
								NCFlags &= ~NC_FLAG_RANGE_LIMIT;
							//}
						}
						else
						{	// deviation could not be determined
						 	GPS_ClearPosition(&RangedTargetPosition);
						}
						GPS_pTargetPosition->Status = PROCESSED;	// mark current target as processed!
					}
				}
				OperatingRadiusOld = GPS_Parameter.OperatingRadius;
				// remember last target position pointer
				pTargetPositionOld = GPS_pTargetPosition;

				/* Calculate position deviation from ranged target */
			
				// calculate deviation of current position to ranged target position in cm
				if(GPS_CalculateDeviation(&(GPSData.Position), &RangedTargetPosition, &CurrentTargetDeviation))
				{	// set target reached flag of we once reached the target point
					if(!(NCFlags & NC_FLAG_TARGET_REACHED) && (CurrentTargetDeviation.Distance < GPS_TargetRadius))
					{
						NCFlags |= NC_FLAG_TARGET_REACHED;	// set target reached flag
					}
					// implement your control code here based
					// in the info available in the CurrentTargetDeviation, GPSData and FromFlightCtrl.GyroHeading
//----------------------------------------------------------------------------------------------------------------------------------		
		NCParams_SetValue(NCPARAMS_NEW_COMPASS_DIRECTION_SETPOINT, &POIDeviation.Bearing);
		D_North = ((s32)GPS_Parameter.D * GPSData.Speed_North)/512;
		D_East =  ((s32)GPS_Parameter.D * GPSData.Speed_East)/512;

		// P-Part
		P_North = ((s32)GPS_Parameter.P * CurrentTargetDeviation.North)/2048;
		P_East =  ((s32)GPS_Parameter.P * CurrentTargetDeviation.East)/2048;

		// I-Part
		I_North = ((s32)GPS_Parameter.I * GPSPosDevIntegral_North)/8192;//add declaration
		I_East =  ((s32)GPS_Parameter.I * GPSPosDevIntegral_East)/8192;//add declaration

		// combine P & I
		PID_North = P_North + I_North;
		PID_East  = P_East + I_East;

		
		GPS_LimitXY(&P_North, &P_North, GPS_Parameter.P_Limit);
		
			GPSPosDevIntegral_North += CurrentTargetDeviation.North/16;
			GPSPosDevIntegral_East  += CurrentTargetDeviation.East/16;
			GPS_LimitXY(&GPSPosDevIntegral_North, &GPSPosDevIntegral_East, 320*GPS_Parameter.I_Limit);
		

		// combine PI- and D-Part

		GPS_LimitXY(&D_North, &D_East, GPS_Parameter.D_Limit);
		PID_North += D_North;
		PID_East  += D_East;

		// scale combination with gain.
		PID_North = (PID_North * (s32)GPS_Parameter.Gain) / 100;
		PID_East  = (PID_East  * (s32)GPS_Parameter.Gain) / 100;

		// GPS to nick and roll settings

		// A positive nick angle moves head downwards (flying forward).
		// A positive roll angle tilts left side downwards (flying left).
		// If compass heading is 0 the head of the copter is in north direction.
		// A positive nick angle will fly to north and a positive roll angle will fly to west.
		// In case of a positive north deviation/velocity the
		// copter should fly to south (negative nick).
		// In case of a positive east position deviation and a positive east velocity the
		// copter should fly to west (positive roll).
		// The influence of the GPSStickNick and GPSStickRoll variable is contrarily to the stick values
		// in the fc.c. Therefore a positive north deviation/velocity should result in a positive
		// GPSStickNick and a positive east deviation/velocity should result in a negative GPSStickRoll.

		coscompass = (s32)c_cos_8192(FromFlightCtrl.GyroHeading /10);//GYRO_DEG_FACTOR=ParamSet.GyroAccFactor*42
		sincompass = (s32)c_sin_8192(FromFlightCtrl.GyroHeading / 10);//GYRO_DEG_FACTOR
		PID_Nick =   (coscompass * PID_North + sincompass * PID_East) / 8192;
		PID_Roll  =  (sincompass * PID_North - coscompass * PID_East) / 8192;

	
		// limit resulting GPS control vector
		GPS_LimitXY(&PID_Nick, &PID_Roll, GPS_Parameter.PID_Limit);

		GPS_Stick.Nick = (s16)PID_Nick;
		GPS_Stick.Roll = (s16)PID_Roll;
	


//-----------------------------------------------------------------------------------------------------------------------------------------				
					//GPS_Stick.Nick = 0;
					//GPS_Stick.Roll = 0;
					//GPS_Stick.Yaw  = 0;
				}
				else // deviation could not be calculated
				{   // do nothing on gps sticks!
					GPS_Neutral();
					NCFlags &= ~NC_FLAG_TARGET_REACHED;	// clear target reached
				}

	 		}// eof if GPSSignal is OK
			else // GPSSignal not OK
			{
				GPS_Neutral();
				// beep if signal is not sufficient
				if(GPS_Parameter.FlightMode != GPS_FLIGHT_MODE_FREE)
				{
					if(!(GPSData.Flags & FLAG_GPSFIXOK) && !(beep_rythm % 5)) BeepTime = 100;
					else if (GPSData.NumOfSats < GPS_Parameter.MinSat && !(beep_rythm % 5)) BeepTime = 10;
				}
			}
			GPSData.Status = PROCESSED; // mark as processed
			break;
	}

	DebugOut.Analog[6] = NCFlags;

	DebugOut.Analog[27] = (s16)CurrentTargetDeviation.North;
	DebugOut.Analog[28] = (s16)CurrentTargetDeviation.East;
	DebugOut.Analog[29] = GPS_Stick.Nick;
	DebugOut.Analog[30] = GPS_Stick.Roll;

	// update navi data, send back to ground station
	GPS_CopyPosition(&(GPSData.Position),   &(NaviData.CurrentPosition));
	GPS_CopyPosition(&RangedTargetPosition, &(NaviData.TargetPosition));
	GPS_CopyPosition(&GPS_HomePosition,     &(NaviData.HomePosition));
	NaviData.SatsInUse = GPSData.NumOfSats;
	NaviData.TargetPositionDeviation.Distance = (u16)CurrentTargetDeviation.Distance/10; // dm
	NaviData.TargetPositionDeviation.Bearing  = (s16)CurrentTargetDeviation.Bearing;
	NaviData.HomePositionDeviation.Distance   = (u16)CurrentHomeDeviation.Distance/10; // dm
	NaviData.HomePositionDeviation.Bearing    = (s16)CurrentHomeDeviation.Bearing;
	NaviData.UBat = FC.BAT_UsedCapacity;
	NaviData.GroundSpeed = (u16)GPSData.Speed_Ground;
	NaviData.Heading = (s16)(GPSData.Heading/100000L);
	NaviData.CompassHeading = (s16)FromFlightCtrl.GyroHeading/10; // in deg
	NaviData.AngleNick = FromFlightCtrl.AngleNick / 10;	   // in deg
	NaviData.AngleRoll = FromFlightCtrl.AngleRoll / 10;	   // in deg
	NaviData.RC_Quality = (u8)FC.RC_Quality;
	NaviData.FCStatusFlags = (u8)FC.StatusFlags;
	NaviData.NCFlags = NCFlags;
	NaviData.OperatingRadius = Parameter.NaviOperatingRadius;
	NaviData.TopSpeed = (s16)GPSData.Speed_Top;  // in cm/s
	NaviData.TargetHoldTime = (u8)(GetDelay(WPTime)/1000); // in s
	//+++++++++++++++++++++++++++++++++++++++++++++++++++
	return;
}

void CalcHeadFree(void)
{
  return;
}
