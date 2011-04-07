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
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "91x_lib.h"
#include "fat16.h"
#include "settings.h"
#include "uart1.h"

typedef struct
{
	ParamId_t ParamId;
	s8  Name[17]; // 0 terminator is the last byte
	u8  Group;   // 0 = reserved  1,2,3,4
	u16 Value;
	u16 Default;
	u16 Min;
	u16 Max;
} __attribute__((packed)) Parameter_t;

Parameter_t CFG_Parameter[] =
{
  //{PID               , "1234567890123456" , Group, Value, Default,   Min, 	Max },
	{PID_KML_LOGGING   , "KMLLOGGING      " ,     1,   500,     500,    0,	60000}, // the log interval for KML logging, 0 = off
	{PID_GPX_LOGGING   , "GPXLOGGING      " ,     1,  1000,    1000,    0, 	60000},   // the log interval for GPX logging, 0 = off
	{PID_GPS_AUTOCONFIG, "GPSAUTOCONFIG   " ,     1,     1,       1,    0,      1}
};


//----------------------------------------------------------------------------------
// initialize all parameters by its default value
void Settings_SetDefaultValues(void)
{
	u8 i;
	// run thrue all parameters and set value to default
	for (i = 0; i < sizeof(CFG_Parameter) / sizeof(Parameter_t); i++)
	{
		CFG_Parameter[i].Value  = CFG_Parameter[i].Default;
	}
}

//----------------------------------------------------------------------------------
// set parameter from string based name and value
u8 Settings_SetParameterFromString(s8 *name, s8 *value)
{
	u8 i, retval = 0;
	s8 string[] = "                \0"; // null terminated string of 16 characters

	if((name == NULL) || (value == NULL)) return(retval);

	i = strlen(name);					// lenght of the parameter name
	if (i > 16) i = 16;					// cut off at 16

	memcpy(string, name, i); 			// copy name to local buffer

    // search name in parameter list
	for (i = 0; i < sizeof(CFG_Parameter) / sizeof(Parameter_t); i++)
	{
		if(strncmp(string, CFG_Parameter[i].Name, 16) == 0) // names are matching
		{
			CFG_Parameter[i].Value = (u16)atoi(value); // convert string to number and set value
			// out of range ?
			if((CFG_Parameter[i].Value < CFG_Parameter[i].Min) || (CFG_Parameter[i].Value > CFG_Parameter[i].Max))
			{	// print a warning
				sprintf(text,"\r\n%s <-- %d is out of range [%d...%d]", string, CFG_Parameter[i].Value, CFG_Parameter[i].Min, CFG_Parameter[i].Max);
				UART1_PutString(text);
				CFG_Parameter[i].Value = CFG_Parameter[i].Default;  // fallback to default
			}
			retval = 1; // value in range
			break; // end loop
		}
	}
	if(!retval)
	{
		sprintf(text,"\r\n%s <-- unknown parameter\r\n", string);
		UART1_PutString(text);
	}
	return(retval);
}

//----------------------------------------------------------------------------------
// read settings from file on sd-card
void Settings_Init(void)
{
	#define LINE_MAX 32
	File_t *fp;
	s8 settingsline[LINE_MAX];
	s8 *name, *value;
	u8 i;
	s8 filename[] = "settings.ini";

	UART1_PutString("\n\r Settings init...");
	Settings_SetDefaultValues();

	if(Fat16_IsValid())
	{	// check if settings file is existing
		if(fexist_(filename))
		{
			fp = fopen_(filename, 'r'); 		// try to open the file
			if (fp == NULL) 						// could not open the file
			{
				UART1_PutString("ERROR: Opening settings file!");
				return;
			}
			// read all lines from file
			while(fgets_(settingsline, LINE_MAX, fp) != 0)
			{
				if ( // ignorelines starting with \r,\n,' ',';','#'
					(settingsline[0] != '\n') &&
					(settingsline[0] != '\r') &&
					(settingsline[0] != ' ' ) &&
					(settingsline[0] != ';' ) &&
					(settingsline[0] != '#' )
					)
				{
					name  = strtok(settingsline, "="); // get name
					value = strtok(NULL, "="); // get value
					if ((name != NULL) && (value != NULL))
					{
						u8 i;
						for(i=0; name[i]; i++) name[i] = toupper(name[i]);
						Settings_SetParameterFromString(name, value);
					}
				}
			}
			fclose_(fp);
			UART1_PutString("ok");
			return;
		}
		else // settings file does not exist
		{	// create default settings file
			fp = fopen_("settings.ini", 'w'); 		// try to open the file
			if(fp == NULL)
			{
			 	UART1_PutString("ERROR: Creating default settings file!");
				return;
			}
			// run thrue all parameters and set value to default
			for (i = 0; i < sizeof(CFG_Parameter) / sizeof(Parameter_t); i++)
			{
				sprintf(settingsline, "%s = %d\r\n", CFG_Parameter[i].Name, CFG_Parameter[i].Default);
				fputs_(settingsline, fp); // write to file
			}
			fputs_("\r\n", fp); // newline at the end of file
			fclose_(fp);
			UART1_PutString("Default settings file created!");
			return;
		}
	}
	else // no acces to fat 16 filesystem
	{
		UART1_PutString("Using default values!");
		return;
	}
}


//----------------------------------------------------------------------------------
// read value of a paramter identified by its id
// returns 1 on success and 0 on error
u8 Settings_GetParamValue(ParamId_t Pid, u16 * pValue)
{
	u8 i, retval = 0;

	for (i = 0; i < sizeof(CFG_Parameter) / sizeof(Parameter_t); i++)
	{
		if(CFG_Parameter[i].ParamId == Pid)
		{
			*pValue = CFG_Parameter[i].Value;
			retval = 1;
			break;
		}
	}
	return retval;
}
