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
#include <stdlib.h>
#include <stdio.h>
#include "91x_lib.h"
#include "gpx.h"
#include "gpx_header.h"
#include "timer1.h"
#include "spi_slave.h"
#include "main.h"
#include "uart1.h"

//________________________________________________________________________________________________________________________________________
// Function: 	GPX_DocumentInit(GPX_Document_t *)
//
// Description:	This function initializes the gpx-document for further use.
//
//
// Returnvalue: '1' if document was initialized
//________________________________________________________________________________________________________________________________________

u8 GPX_DocumentInit(GPX_Document_t *doc)
{
	if(doc->state != GPX_DOC_CLOSED) GPX_DocumentClose(doc);						// close file if it was opened before
	doc->state	 = GPX_DOC_CLOSED;													// init state of the gpx-document
	doc->file	 = NULL;
	return(1);
}

//________________________________________________________________________________________________________________________________________
// Function: 	GPX_Document_Open(s8 *name, GPX_Document_t *doc);
//
// Description:	This function opens a new gpx-document with the specified name and creates the document header within the file.
//
//
// Returnvalue: '1' if the gpx-file could be created.
//________________________________________________________________________________________________________________________________________

u8 GPX_DocumentOpen(s8 *name, GPX_Document_t *doc)
{

	u8 retvalue = 0;

	if(doc == NULL) return(0);
	GPX_DocumentInit(doc);														// intialize the document with resetvalues
	doc->file = fopen_(name,'a');												// open a new file with the specified filename on the memorycard.

	if(doc->file != NULL)														// could the file be opened?
	{
		retvalue = 1;															// the document could be created on the drive.
		doc->state = GPX_DOC_OPENED;												// change document state to opened. At next a placemark has to be opened.
		fwrite_((void*)GPX_DOCUMENT_HEADER, sizeof(GPX_DOCUMENT_HEADER)-1,1,doc->file);// write the gpx-header to the document.
	}

	return(retvalue);
}

//________________________________________________________________________________________________________________________________________
// Function: 	DocumentClose(GPX_Document_t *doc);
//
// Description:	This function closes the document specified by doc.
//
//
// Returnvalue: '1' if the gpx-file could be closed.
//________________________________________________________________________________________________________________________________________

u8 GPX_DocumentClose(GPX_Document_t *doc)
{

	u8 retvalue = 1;

	if(doc == NULL) return(0);

	while(doc->state != GPX_DOC_CLOSED)								// close linestring, placemark and document before closing the file on the memorycard
	{
		switch(doc->state)
		{
			case GPX_DOC_TRACKSEGMENT_OPENED:
				GPX_TrackSegmentEnd(doc);						// write terminating tag to end tracksegment.
				break;

			case GPX_DOC_TRACK_OPENED:								// write terminating tag to close track.
				GPX_TrackEnd(doc);
				break;

			case GPX_DOC_OPENED:									// close the file on the memorycard
				if(doc->file != NULL)
				{
					fwrite_((void*)GPX_DOCUMENT_FOOTER, sizeof(GPX_DOCUMENT_FOOTER)-1,1,doc->file);	// write the gpx-footer to the document.
					fclose_(doc->file);
					retvalue = 1;
				}
				doc->state = GPX_DOC_CLOSED;
				break;

			default:
				doc->state = GPX_DOC_CLOSED;
				break;
		}
	}
	return(retvalue);
}

//________________________________________________________________________________________________________________________________________
// Function: 	u8 GPX_TrackBegin(GPX_Document_t);
//
// Description:	This function adds a track to the document.
//
//
// Returnvalue: '1' if the track could be opened
//________________________________________________________________________________________________________________________________________

u8 GPX_TrackBegin(GPX_Document_t *doc)
{

	u8 retvalue = 0;
	if(doc->state == GPX_DOC_OPENED)
	{
		if(doc->file != NULL)
		{
			doc->state = GPX_DOC_TRACK_OPENED;
			retvalue = 1;
			fwrite_((void*)GPX_TRACK_HEADER, sizeof(GPX_TRACK_HEADER)-1,1,doc->file);
		}
	}
	return(retvalue);
}

//________________________________________________________________________________________________________________________________________
// Function: 	u8 GPX_TrackEnd(KML_Document_t *doc)
//
// Description:	This function ends the track opened before.
//
//
// Returnvalue: 1' if the track could be closed
//________________________________________________________________________________________________________________________________________

u8 GPX_TrackEnd(GPX_Document_t *doc)
{

	u8 retvalue = 0;

	if(doc->state == GPX_DOC_TRACK_OPENED)
	{
		if(doc->file != NULL)
		{
			doc->state = GPX_DOC_OPENED;
			fwrite_((void*)GPX_TRACK_FOOTER, sizeof(GPX_TRACK_FOOTER)-1,1,doc->file);
			retvalue = 1;
		}
	}

	return(retvalue);
}

//________________________________________________________________________________________________________________________________________
// Function: 	u8 GPX_TrackSegmentBegin(GPX_Document_t *doc);
//
// Description:	This function starts a track segment.
//
//
// Returnvalue: '1' if the track segement could be started
//________________________________________________________________________________________________________________________________________

u8 GPX_TrackSegmentBegin(GPX_Document_t *doc)
{

	u8 retvalue = 0;

	if(doc->state == GPX_DOC_TRACK_OPENED)
	{
		if(doc->file != NULL)
		{
			doc->state = GPX_DOC_TRACKSEGMENT_OPENED;
			fwrite_((void*)GPX_TRACKSEGMENT_HEADER, sizeof(GPX_TRACKSEGMENT_HEADER)-1,1,doc->file);
			retvalue = 1;
		}
	}
	return(retvalue);
}

//________________________________________________________________________________________________________________________________________
// Function: 	u8 GPX_TrackSegmentEnd(GPX_Document_t *doc);
//
// Description:	This function ends the tracksegment opened before.
//
//
// Returnvalue: '1' if the track segment could be terminated
//________________________________________________________________________________________________________________________________________

u8 GPX_TrackSegmentEnd(GPX_Document_t *doc)
{

	u8 retvalue = 0;
	if(doc->state == GPX_DOC_TRACKSEGMENT_OPENED)
	{
		if(doc->file != NULL)
		{
			doc->state = GPX_DOC_TRACK_OPENED;
			fwrite_((void*)GPX_TRACKSEGMENT_FOOTER, sizeof(GPX_TRACKSEGMENT_FOOTER)-1,1,doc->file);
			retvalue = 1;
		}
	}
	return(retvalue);
}

//________________________________________________________________________________________________________________________________________
// Function: 	u8 GPX_TrackSegementAddPoint(GPS_Pos_t * pGPS_Position ,GPX_Document_t *doc);
//
// Description:	This function adds a pointof a track segement to the specified document.
//
//
// Returnvalue: '1' if a point was added
//________________________________________________________________________________________________________________________________________

u8 GPX_TrackSegementAddPoint(GPX_Document_t *doc)
{

	u8 retvalue = 0;
	s8 string[50];

	if(doc == NULL) return(0);

	if((GPSData.Position.Status != INVALID) && (NaviData.HomePosition.Status != INVALID))
	{
		if(doc->state == GPX_DOC_TRACKSEGMENT_OPENED)
		{
			if(doc->file != NULL)
			{
				s32 i32_1, i32_2;
				s16 i16_1;
				u8 u8_1, u8_2;
				// write <trkpt> tag
				if(GPSData.Position.Latitude < 0) u8_1 = '-';
				else u8_1 = '+';
				i32_1 = abs(GPSData.Position.Latitude)/10000000L;
				i32_2 = abs(GPSData.Position.Latitude)%10000000L;
				sprintf(string, "<trkpt lat=\"%c%ld.%07ld\" ",u8_1, i32_1, i32_2);
				fputs_(string, doc->file);
				if(GPSData.Position.Longitude < 0) u8_1 = '-';
				else u8_1 = '+';
				i32_1 = abs(GPSData.Position.Longitude)/10000000L;
				i32_2 = abs(GPSData.Position.Longitude)%10000000L;
				sprintf(string, "lon=\"%c%ld.%07ld\">\r\n",u8_1, i32_1, i32_2);
				fputs_(string, doc->file);
				// write <ele> taga
				i32_2 = GPSData.Position.Altitude - NaviData.HomePosition.Altitude;
				if(i32_2 < 0) i32_2 = 0; // avoid negative altitudes in log
				i32_1 = i32_2/1000L;
				i32_2 = i32_2%1000L;
				sprintf(string,"<ele>%ld.%03ld</ele>\r\n",i32_1, i32_2);
				fputs_(string, doc->file);
				// write <time> tag	only at a resolution of one second
				sprintf(string, "<time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\r\n",SystemTime.Year, SystemTime.Month, SystemTime.Day, SystemTime.Hour, SystemTime.Min, SystemTime.Sec);
				fputs_(string, doc->file);
				// write <sat> tag
				sprintf(string, "<sat>%d</sat>\r\n", NaviData.SatsInUse);
				fputs_(string, doc->file);
				// todo: add  <extensions> tag with additional data to be logged
				sprintf(string, "<extensions>\r\n");
				fputs_(string, doc->file);
				// Altimeter according to air pressure
				sprintf(string, "<Altimeter>%d</Altimeter>\r\n", NaviData.Altimeter);
				fputs_(string, doc->file);
				// Variometer according to air pressure
				sprintf(string, "<Variometer>%d</Variometer>\r\n", NaviData.Variometer);
				fputs_(string, doc->file);
				// Course in deg
				i16_1 = GPSData.Heading/100000L;
				sprintf(string, "<Course>%03d</Course>\r\n", i16_1);
				fputs_(string, doc->file);
				// Ground Speed in cm/s
				sprintf(string, "<GroundSpeed>%d</GroundSpeed>\r\n", NaviData.GroundSpeed);
				fputs_(string, doc->file);
				// Vertical Speed in cm/s
				sprintf(string, "<VerticalSpeed>%d</VerticalSpeed>\r\n", NaviData.TopSpeed);
				fputs_(string, doc->file);
				// Flight duration
				sprintf(string, "<FlightTime>%d</FlightTime>\r\n", NaviData.FlyingTime);
				fputs_(string, doc->file);
				// Ubat
				u8_1 = NaviData.UBat / 10;
				u8_2 = NaviData.UBat % 10;
 				sprintf(string, "<Voltage>%d.%01d</Voltage>\r\n", u8_1, u8_2);
				fputs_(string, doc->file);
				// Current
				u8_1 = NaviData.Current / 10;
				u8_2 = NaviData.Current % 10;
 				sprintf(string, "<Current>%d.%01d</Current>\r\n", u8_1, u8_2);
				fputs_(string, doc->file);
				// Capacity
 				sprintf(string, "<Capacity>%d</Capacity>\r\n", NaviData.UsedCapacity);
				fputs_(string, doc->file);
				// RC Quality
				sprintf(string, "<RCQuality>%d</RCQuality>\r\n", FC.RC_Quality);
				fputs_(string, doc->file);
				// RC Received Signal Strength Indication
				sprintf(string, "<RCRSSI>%d</RCRSSI>\r\n", FC.RC_RSSI);
				fputs_(string, doc->file);
				// Compassind deg
				i16_1 = FromFlightCtrl.GyroHeading / 10;
				sprintf(string, "<Compass>%03d</Compass>\r\n", i16_1);
				fputs_(string, doc->file);
				// Nick Angle ind deg
				sprintf(string, "<NickAngle>%03d</NickAngle>\r\n", NaviData.AngleNick);
				fputs_(string, doc->file);
				// Roll Angle in deg
				sprintf(string, "<RollAngle>%03d</RollAngle>\r\n", NaviData.AngleRoll);
				fputs_(string, doc->file);
				// NC Mode (contains the status)
				sprintf(string, "<NCFlag>%02X</NCFlag>\r\n", NCFlags);
				fputs_(string, doc->file);
				// Status of the complete MikroKopter
				sprintf(string, "<ErrorCode>%03d</ErrorCode>\r\n",ErrorCode);
				fputs_(string, doc->file);
				// Target Bearing in deg
				sprintf(string, "<TargetBearing>%03d</TargetBearing>\r\n", NaviData.TargetPositionDeviation.Bearing);
				fputs_(string, doc->file);
				// Target Distance in dm
				sprintf(string, "<TargetDistance>%d</TargetDistance>\r\n", NaviData.TargetPositionDeviation.Distance);
				fputs_(string, doc->file);
				// RC Sticks as Nick/Roll/Yaw
				sprintf(string, "<RCSticks>%d, %d, %d</RCSticks>\r\n", FC.StickNick,FC.StickRoll, FC.StickYaw);
				fputs_(string, doc->file);
				// GPS Sticks as Nick/Roll/Yaw
				sprintf(string, "<GPSSticks>%d, %d, %d</GPSSticks>\r\n", ToFlightCtrl.GPSStick.Nick, ToFlightCtrl.GPSStick.Roll, ToFlightCtrl.GPSStick.Yaw);
				fputs_(string, doc->file);

				// eof extensions
				sprintf(string, "</extensions>\r\n");
				fputs_(string, doc->file);
				sprintf(string, "</trkpt>\r\n");
				fputs_(string, doc->file);
				retvalue = 1;
			}
		}
	}
	return(retvalue);
}

//________________________________________________________________________________________________________________________________________
// Function: 	u8 KML_LoggGPSCoordinates(gps_data_t *, KML_Document_t *)
//
// Description:	This function opens and adds gpscoordinates to an GPX-Document. The document will be opened, if not already done
//
//
// Returnvalue: '1' if an gps coordinate was logged
//________________________________________________________________________________________________________________________________________

u8 GPX_LoggGPSCoordinates(GPX_Document_t *doc)
{
	u8 retval = 0;
	while(doc->state != GPX_DOC_TRACKSEGMENT_OPENED)				// automatic create document with default filename on the card.
	{
		switch(doc->state)
		{
	 		case GPX_DOC_CLOSED:									// document hasn't been opened yet therefore it will be initialized automatically
				retval = GPX_DocumentOpen("default.gpx",doc);	// open the gpx-document with a standardname.
			break;

			case GPX_DOC_OPENED:									// if a document has been opened before but no track exists:
		   		retval = GPX_TrackBegin(doc);
			break;

			case GPX_DOC_TRACK_OPENED:								// add tracksegement to the track
				retval = GPX_TrackSegmentBegin(doc);
			break;

			default:
				retval = 0;
			break;

		}
		if(retval != 1) return(retval); // stop on error
	}

	if(doc->state == GPX_DOC_TRACKSEGMENT_OPENED)						// if the document was opened add coordinates to the document.
	{
		retval = GPX_TrackSegementAddPoint(doc);						// add a track segment point
	}
	return(retval);
}

