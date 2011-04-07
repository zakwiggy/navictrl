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
#include "91x_lib.h"
#include "main.h"
#include "timer1.h"
#include "uart1.h"
#include "kml.h"
#include "gpx.h"
#include "ssc.h"
#include "settings.h"


#define LOG_FLUSH_INTERVAL 20000 // 20s

typedef enum
{
	LOGFILE_IDLE,
	LOGFILE_START,
	LOGFILE_CLOSED,
	LOGFILE_OPENED,
	LOGFILE_ERROR
} logfilestate_t;

// logger handler prototypes
logfilestate_t Logging_KML(u32 LogDelay);
logfilestate_t Logging_GPX(u32 LogDelay);

typedef struct
{
 	u32 KML_Interval;  // the kml-log interval (0 = off)
	u32 GPX_Interval;  // the gpx-log interval (0 = off)
} LogCfg_t;

LogCfg_t LogCfg = {500 , 1000};


//----------------------------------------------------------------------------------------------------
s8* GenerateKMLLogFileName(void)
{
	static u16 filenum = 0;	// file name counter
	static s8 filename[35];
	static DateTime_t LastTime = {0,0,0,0,0,0,0,0};

	if(SystemTime.Valid)
	{
		// if the day has been changed
		if((LastTime.Year != SystemTime.Year) || (LastTime.Month != SystemTime.Month) || (LastTime.Day != SystemTime.Day))
		{
			LastTime.Year = SystemTime.Year;
			LastTime.Month = SystemTime.Month;
			LastTime.Day = SystemTime.Day;
			LastTime.Valid = 1;
			filenum = 0; // reset file counter
		}
		sprintf(filename, "LOG/%04i%02i%02i/KML/GPS%05i.KML", SystemTime.Year, SystemTime.Month, SystemTime.Day, filenum);
		filenum++;
		return filename;
	}
	else return NULL;
}

//----------------------------------------------------------------------------------------------------
s8* GenerateGPXLogFileName(void)
{
	static u16 filenum = 0;	// file name counter
	static s8 filename[35];
	static DateTime_t LastTime = {0,0,0,0,0,0,0,0};

	if(SystemTime.Valid)
	{
		// if the day has been changed
		if((LastTime.Year != SystemTime.Year) || (LastTime.Month != SystemTime.Month) || (LastTime.Day != SystemTime.Day))
		{
			LastTime.Year = SystemTime.Year;
			LastTime.Month = SystemTime.Month;
			LastTime.Day = SystemTime.Day;
			LastTime.Valid = 1;
			filenum = 0; // reset file counter
		}
		sprintf(filename, "LOG/%04i%02i%02i/GPX/GPS%05i.GPX", SystemTime.Year, SystemTime.Month, SystemTime.Day, filenum);
		filenum++;
		return filename;
	}
	else return NULL;
}



//----------------------------------------------------------------------------------------------------
// logs the current gps position to a kml file
logfilestate_t Logging_KML(u32 LogDelay)
{
	static 	logfilestate_t logfilestate = LOGFILE_IDLE; // the current logfilestate
	static	s8* logfilename = NULL;						// the pointer to the logfilename
	static  u32 logtimer = 0, flushtimer = 0;       	// the log update timer
	static	KML_Document_t logfile; 					// the logfilehandle

	// initialize if LogDelay is zero
	if(!LogDelay)
	{
	 	switch(logfilestate)
		{
			case LOGFILE_OPENED:
				KML_DocumentClose(&logfile); // try to close it
				break;
		 	default:
				break;
		}
		logfilestate = LOGFILE_IDLE;
		logfilename = NULL;
		KML_DocumentInit(&logfile);
		logtimer = SetDelay(0); // set logtimer to now
		return logfilestate;
	}
	// no init
	if(CheckDelay(logtimer))
	{
		logtimer = SetDelay(LogDelay);	// standard interval

		if(FC.StatusFlags & FC_STATUS_MOTOR_RUN)
		{
			switch(logfilestate)
			{
				case LOGFILE_IDLE:
				case LOGFILE_CLOSED:
					if((GPSData.Status != INVALID) && (GPSData.Flags & FLAG_GPSFIXOK) && (GPSData.SatFix == SATFIX_3D) && (FC.StatusFlags & FC_STATUS_FLY))
					{
						logfilestate = LOGFILE_START;
					}
					break;
				case LOGFILE_START:
					// find unused logfile name
					do
					{	 // try to generate a new logfile name
					 	 logfilename = GenerateKMLLogFileName();
					}while((logfilename != NULL) && fexist_(logfilename));
					// if logfilename exist
					if(logfilename != NULL)
					{
						// try to create the log file
						if(KML_DocumentOpen(logfilename, &logfile))
						{
							flushtimer = SetDelay(LOG_FLUSH_INTERVAL);
							logfilestate = LOGFILE_OPENED; // goto next step
							UART1_PutString("\r\nOpening kml-file:");
							UART1_PutString(logfilename);
							UART1_PutString("\r\n");
						}
						else // could not be openend
						{
							logfilestate = LOGFILE_ERROR;
							UART1_PutString("\r\nError opening kml-file: ");
							UART1_PutString(logfilename);
							UART1_PutString("\r\n");
							logtimer = SetDelay(10);  // try again in open logfile in 10 mili sec
						}
					}
					else
					{
						logfilestate = LOGFILE_ERROR;
					 	UART1_PutString("\r\nError getting free kml-file name\r\n");
					}
					// else retry in next loop
					break;
				case LOGFILE_OPENED:
					// append new gps log data
					if((GPSData.Status != INVALID) && (GPSData.Flags & FLAG_GPSFIXOK) && (GPSData.SatFix == SATFIX_3D))
					{
						if(!KML_LoggGPSCoordinates(&logfile))
						{	// error logging data
							UART1_PutString("\r\nError logging to kml-file\r\n");
						 	KML_DocumentClose(&logfile);
							logfilestate = LOGFILE_ERROR;
						}
						else // sucessfully logged
						{
							if(CheckDelay(flushtimer))
							{
								flushtimer = SetDelay(LOG_FLUSH_INTERVAL);
								fflush_(logfile.file);
							}
						}

					}
					break;

				case LOGFILE_ERROR:
					break;

				default:
					logfilestate = LOGFILE_IDLE;
					break;
			}
		} // EOF motors are not running
		else // model is not flying
		{   // close log file if opened
			if(logfilestate == LOGFILE_OPENED)
			{
				if(KML_DocumentClose(&logfile))
				{
					UART1_PutString("\r\nClosing kml-file\r\n");
					logfilestate = LOGFILE_CLOSED;
				}
				else  // could not be closed
				{
					UART1_PutString("\r\nError closing kml-file\r\n");
				 	logfilestate =  LOGFILE_ERROR;
				}
			}
		} //EOF motors are not running
	} // EOF Check LogTimer

	return logfilestate;
}

//----------------------------------------------------------------------------------------------------
// logs gps and state info to a gpx file
logfilestate_t Logging_GPX(u32 LogDelay)
{
	static 	logfilestate_t logfilestate = LOGFILE_IDLE; // the current logfilestate
	static	s8* logfilename = NULL;						// the pointer to the logfilename
	static  u32 logtimer = 0, flushtimer = 0;       	// the log update timer
	static	GPX_Document_t logfile; 					// the logfilehandle

	// initialize if LogDelay is zero
	if(!LogDelay)
	{
	 	switch(logfilestate)
		{
			case LOGFILE_OPENED:
				GPX_DocumentClose(&logfile); // try to close it
				break;
		 	default:
				break;
		}
		logfilestate = LOGFILE_IDLE;
		logfilename = NULL;
		GPX_DocumentInit(&logfile);
		logtimer = SetDelay(0); // set logtimer to now
		return logfilestate;
	}
	// no init
	if(CheckDelay(logtimer))
	{
		logtimer = SetDelay(LogDelay);	// standard interval

		if(FC.StatusFlags & FC_STATUS_MOTOR_RUN)
		{
			switch(logfilestate)
			{
				case LOGFILE_IDLE:
				case LOGFILE_CLOSED:
					if((GPSData.Status != INVALID) && (GPSData.Flags & FLAG_GPSFIXOK) && (GPSData.SatFix == SATFIX_3D) && (FC.StatusFlags & FC_STATUS_FLY))
					{
						logfilestate = LOGFILE_START;
					}
					break;
				case LOGFILE_START:
					// find unused logfile name
					do
					{	 // try to generate a new logfile name
					 	 logfilename = GenerateGPXLogFileName();
					}while((logfilename != NULL) && fexist_(logfilename));
					// if logfilename exist
					if(logfilename != NULL)
					{
						// try to create the log file
						if(GPX_DocumentOpen(logfilename, &logfile))
						{
							flushtimer = SetDelay(LOG_FLUSH_INTERVAL);
							logfilestate = LOGFILE_OPENED; // goto next step
							UART1_PutString("\r\nOpening gpx-file:");
							UART1_PutString(logfilename);
							UART1_PutString("\r\n");
						}
						else // could not be openend
						{
							logfilestate = LOGFILE_ERROR;
							UART1_PutString("\r\nError opening gpx-file: ");
							UART1_PutString(logfilename);
							UART1_PutString("\r\n");
							logtimer = SetDelay(10);  // try again in open logfile in 10 mili sec
						}
					}
					else
					{
						logfilestate = LOGFILE_ERROR;
					 	UART1_PutString("\r\nError getting free gpx-file name\r\n");
					}
					// else retry in next loop
					break;
				case LOGFILE_OPENED:
					// append new gps log data
					if((GPSData.Status != INVALID) && (GPSData.Flags & FLAG_GPSFIXOK) && (GPSData.SatFix == SATFIX_3D))
					{
						if(!GPX_LoggGPSCoordinates(&logfile))
						{	// error logging data
							UART1_PutString("\r\nError logging to gpx-file\r\n");
						 	GPX_DocumentClose(&logfile);
							logfilestate = LOGFILE_ERROR;
						}
						else // successful log
						{
							if(CheckDelay(flushtimer))
							{
								flushtimer = SetDelay(LOG_FLUSH_INTERVAL);
								fflush_(logfile.file);
							}
						}
					}
					break;

				case LOGFILE_ERROR:
					break;

				default:
					logfilestate = LOGFILE_IDLE;
					break;
			}
		} // EOF model is flying
		else // model is not flying
		{   // close log file if opened
			if(logfilestate == LOGFILE_OPENED)
			{
				if(GPX_DocumentClose(&logfile))
				{
					UART1_PutString("\r\nClosing gpx-file\r\n");
					logfilestate = LOGFILE_CLOSED;
				}
				else  // could not be closed
				{
					UART1_PutString("\r\nError closing gpx-file\r\n");
				 	logfilestate =  LOGFILE_ERROR;
				}
			}
		} //EOF model is not flying
	} // EOF Check LogTimer

	return logfilestate;
}

//----------------------------------------------------------------------------------------------------
// initialize logging
void Logging_Init(void)
{
	LogCfg.KML_Interval = 500; //default
	Settings_GetParamValue(PID_KML_LOGGING, (u16*)&LogCfg.KML_Interval); // overwrite by settings value
	Logging_KML(0); // initialize
	LogCfg.GPX_Interval = 0; //default
	Settings_GetParamValue(PID_GPX_LOGGING, (u16*)&LogCfg.GPX_Interval); // overwrite by settings value
 	Logging_GPX(0);	// initialize
}

//----------------------------------------------------------------------------------------------------
// gobal logging handler
void Logging_Update(void)
{
	static u8 logmodule = 0;
	static u32 logtimer = 0;
	static logfilestate_t logstate = LOGFILE_IDLE;

	if(SD_SWITCH) // a card is in slot
	{
		if(CheckDelay(logtimer))
		{
			logtimer = SetDelay(10);  // faster makes no sense
			// call the logger handlers if no error has occured
			if(logstate != LOGFILE_ERROR)
			{  // run only one logging handler per loop
				switch(logmodule++)
				{
					case 0:
						logstate = Logging_KML(LogCfg.KML_Interval);
						break;
					case 1:
						logstate = Logging_GPX(LogCfg.GPX_Interval);
						logmodule = 0; // resart with first log module
						break;
					default:
						logmodule = 0;
						break;
				}
			}
			else // a logging error has occured
			{	
				/*
				// try to reinitialize sd-card when motors are not running
				if(!(FC.Flags & FCFLAG_MOTOR_RUN))
				{
					if(Fat16_IsValid()) // wait for reinizialization of fat16 from outside
					{
						Logging_Init(); // initialize the logs
						logstate = LOGFILE_IDLE;
						logtimer = SetDelay(10);	// try next log in 10 mili sec
					}
					else
					{   // retry in 5 seconds
						logtimer = SetDelay(5000);  // try again in 5 sec
					}
				} // EOF motors are not running
				*/
			} //EOF logfile error
		}  // EOF CheckDelay
	}// EOF Card in Slot
}
