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
	 
#include <string.h>
#include "91x_lib.h"
#include "waypoints.h"
#include "uart1.h"

// the waypoints list
#define MAX_LIST_LEN 31

Point_t PointList[MAX_LIST_LEN];
u8 WPIndex = 0;		// list index of GPS point representig the current WP, can be maximal WPCount
u8 POIIndex = 0;	// list index of GPS Point representing the current POI, can be maximal WPCount
u8 WPCount = 0;    	// number of waypoints
u8 PointCount = 0;		// number of wp in the list can be maximal equal to MAX_LIST_LEN
u8 POICount = 0;

u8 WPActive = FALSE;

u8 PointList_Init(void)
{
 	return PointList_Clear();
}

u8 PointList_Clear(void)
{
	u8 i;
	WPIndex = 0;	// real list position are 1 ,2, 3 ...
	POIIndex = 0;	// real list position are 1 ,2, 3 ...
	WPCount = 0;	// no waypoints
    POICount = 0;
	PointCount = 0;	// no contents
	WPActive = FALSE;
	NaviData.WaypointNumber = WPCount;
	NaviData.WaypointIndex = 0;

	for(i = 0; i < MAX_LIST_LEN; i++)
	{
		PointList[i].Position.Status = INVALID;
		PointList[i].Position.Latitude = 0;
		PointList[i].Position.Longitude = 0;
		PointList[i].Position.Altitude = 0;
		PointList[i].Heading = 361; 		// invalid value
		PointList[i].ToleranceRadius = 0;	// in meters, if the MK is within that range around the target, then the next target is triggered
		PointList[i].HoldTime = 0;			// in seconds, if the was once in the tolerance area around a WP, this time defines the delay before the next WP is triggered
		PointList[i].Type = POINT_TYPE_INVALID;
		PointList[i].Event_Flag = 0;		// future implementation
		PointList[i].AltitudeRate = 0;		// no change of setpoint
	}
	return TRUE;		
}

u8 PointList_GetCount(void)
{
 	return PointCount; // number of points in the list
}

Point_t* PointList_GetAt(u8 index)
{
	if((index > 0) && (index <= PointCount)) return(&(PointList[index-1]));	// return pointer to this waypoint
	else return(NULL);
}

u8 PointList_SetAt(Point_t* pPoint)
{
	// if index is in range
 	if((pPoint->Index > 0) && (pPoint->Index <= MAX_LIST_LEN))
	{
		// check list entry before update
		switch(PointList[pPoint->Index-1].Type)
		{
			case POINT_TYPE_INVALID: // was invalid
				switch(pPoint->Type)
				{
					default:
				 	case POINT_TYPE_INVALID:
						// nothing to do
						break;

					case POINT_TYPE_WP:
						WPCount++;
						PointCount++;
						break;
					
					case POINT_TYPE_POI:
						POICount++;
						PointCount++;
						break;
				}
				break; 
				
			case POINT_TYPE_WP: // was  a waypoint
				switch(pPoint->Type)
				{
				 	case POINT_TYPE_INVALID:
						WPCount--;
						PointCount--;
						break;

				   	default:
					case POINT_TYPE_WP:
						//nothing to do
						break;
					
					case POINT_TYPE_POI:
						POICount++;
						WPCount--;
						break;
				}
				break;
				
			case POINT_TYPE_POI: // was a poi
				switch(pPoint->Type)
				{
				 	case POINT_TYPE_INVALID:
						POICount--;
						PointCount--;
						break;

					case POINT_TYPE_WP:
						WPCount++;
						POICount--;
						break;
					
					case POINT_TYPE_POI:
					default:
						// nothing to do
						break;
				}
				break;		
		}
		memcpy(&PointList[pPoint->Index-1], pPoint, sizeof(Point_t)); // copy data to list entry										
		NaviData.WaypointNumber = WPCount;
		return pPoint->Index;
	}
	else return(0);
}

// returns the pointer to the first waypoint within the list
Point_t* PointList_WPBegin(void)
{
	u8 i;
	WPIndex = 0; // set list position invalid

	if(WPActive == FALSE) return(NULL);

	POIIndex = 0; // set invalid POI
	if(PointCount > 0) 
	{
		// search for first wp in list
		for(i = 0; i <MAX_LIST_LEN; i++)
		{
			if((PointList[i].Type == POINT_TYPE_WP) && (PointList[i].Position.Status != INVALID))
			{
				WPIndex = i + 1;
				break;
			}
		}
		if(WPIndex) // found a WP in the list
		{
			NaviData.WaypointIndex = 1;
			// update index to POI
			if(PointList[WPIndex-1].Heading < 0) POIIndex = (u8)(-PointList[WPIndex-1].Heading);
			else POIIndex = 0;			
		}
		else // some points in the list but no WP found
		{
			NaviData.WaypointIndex = 0;
			//Check for an existing POI
			for(i = 0; i < MAX_LIST_LEN; i++)
			{
				if((PointList[i].Type == POINT_TYPE_POI) && (PointList[i].Position.Status != INVALID))
				{
					POIIndex = i + 1;
					break;
				}
			}
		}
	}
	else // no point in the list
	{
   		POIIndex = 0;
		NaviData.WaypointIndex = 0;	
	}

	if(WPIndex) return(&(PointList[WPIndex-1]));
	else return(NULL);
}

// returns the last waypoint
Point_t* PointList_WPEnd(void)
{
	
	u8 i;
	WPIndex = 0; // set list position invalid
	POIIndex = 0; // set invalid

	if(WPActive == FALSE) return(NULL);

	if(PointCount > 0)
	{
		// search backward!
		for(i = 1; i <= MAX_LIST_LEN; i++)
		{
			if((PointList[MAX_LIST_LEN - i].Type == POINT_TYPE_WP) && (PointList[MAX_LIST_LEN - i].Position.Status != INVALID))
			{	
				WPIndex = MAX_LIST_LEN - i + 1;
				break;
			}
		}
		if(WPIndex) // found a WP within the list
		{
			NaviData.WaypointIndex = WPCount;
			if(PointList[WPIndex-1].Heading < 0) POIIndex = (u8)(-PointList[WPIndex-1].Heading);
			else POIIndex = 0;	
		}
		else // list contains some points but no WP in the list
		{
			// search backward for a POI!
			for(i = 1; i <= MAX_LIST_LEN; i++)
			{
				if((PointList[MAX_LIST_LEN - i].Type == POINT_TYPE_POI) && (PointList[MAX_LIST_LEN - i].Position.Status != INVALID))
				{	
					POIIndex = MAX_LIST_LEN - i + 1;
					break;
				}
			}
			NaviData.WaypointIndex = 0;	
		}
	}
	else // no point in the list
	{
		POIIndex = 0;
		NaviData.WaypointIndex = 0;
	}
	if(WPIndex) return(&(PointList[WPIndex-1]));
	else return(NULL);
}

// returns a pointer to the next waypoint or NULL if the end of the list has been reached
Point_t* PointList_WPNext(void)
{
	u8 wp_found = 0;
	if(WPActive == FALSE) return(NULL);
		
	if(WPIndex < MAX_LIST_LEN) // if there is a next entry in the list
	{
		u8 i;
		for(i = WPIndex; i < MAX_LIST_LEN; i++)	// start search for next at next list entry
		{
			if((PointList[i].Type == POINT_TYPE_WP) && (PointList[i].Position.Status != INVALID)) // jump over POIs
			{
			 	wp_found = i+1;
				break;
			}
		}
	}
	if(wp_found)
	{
		WPIndex = wp_found; // update list position
		NaviData.WaypointIndex++;
		if(PointList[WPIndex-1].Heading < 0) POIIndex = (u8)(-PointList[WPIndex-1].Heading);
		else POIIndex = 0;
		return(&(PointList[WPIndex-1]));	// return pointer to this waypoint
	}
	else 
	{  // no next wp found
		NaviData.WaypointIndex = 0;	
		POIIndex = 0;
		return(NULL);
	}
}	

void PointList_WPActive(u8 set)
{
	if(set)
	{	
		WPActive = TRUE;
		PointList_WPBegin(); // uopdates POI index
	}
	else
	{
		WPActive = FALSE;
		POIIndex = 0;  // disable POI also
	}
}
 
Point_t* PointList_GetPOI(void)
{
	return PointList_GetAt(POIIndex);	
}

