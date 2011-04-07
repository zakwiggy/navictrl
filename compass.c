/*#######################################################################################*/
/* !!! THIS IS NOT FREE SOFTWARE !!!  	                                                 */
/*#######################################################################################*/
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) 2010 Ingo Busker, Holger Buss
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
#include "91x_lib.h"
#include "compass.h"
#include "mk3mag.h"
#include "ncmag.h"
#include "spi_slave.h"
#include "mymath.h"
#include "uart1.h"
#include "fifo.h"
#include "led.h"
#include "main.h"


u8 CompassCalStateQueue[10];
fifo_t CompassCalcStateFiFo;

volatile s16vec_t MagVector;   // is written by mk3mag or ncmag implementation
volatile s16 Compass_Heading;   // is written by mk3mag or ncmag implementation
volatile u8  Compass_CalState; // is written by mk3mag or ncmag implementation

#define COMPASS_NONE	0
#define COMPASS_MK3MAG	1
#define COMPASS_NCMAG	2
u8 Compass_Device = COMPASS_NONE;

void Compass_Init(void)
{
	switch(Compass_Device)
	{
	 	case COMPASS_NONE:
			UART1_PutString("\r\n Looking for compass");
			if(Version_HW > 11)
			{
				if(      NCMAG_Init() ) Compass_Device = COMPASS_NCMAG;
				else if( MK3MAG_Init())	Compass_Device = COMPASS_MK3MAG;
			}
			else
			{
				if(     MK3MAG_Init() ) Compass_Device = COMPASS_MK3MAG;
				else if( NCMAG_Init() )	Compass_Device = COMPASS_NCMAG;
			}
			break;
	
		case COMPASS_NCMAG:
		   	if( NCMAG_Init() )	Compass_Device = COMPASS_NCMAG;
			else				Compass_Device = COMPASS_NONE;                
			break;

		case COMPASS_MK3MAG:
		default:
			// nothing to do
			break;

	}
	fifo_init(&CompassCalcStateFiFo, CompassCalStateQueue, sizeof(CompassCalStateQueue), NO_ITLine, NO_ITLine);
}


void Compass_CalcHeading(void)
{
	if((UART_VersionInfo.HardwareError[0] & NC_ERROR0_SPI_RX) || Compass_CalState)
	{
		Compass_Heading = -1;
	}
	else // fc attitude is avialable and no compass calibration active
	{
		// calculate attitude correction
		// a float implementation takes too long
		s16 tmp, Hx, Hy;
		s32 sinnick, cosnick, sinroll, cosroll;
		
		tmp = FromFlightCtrl.AngleNick/10; // in deg 
		sinnick = (s32)c_sin_8192(tmp);
		cosnick = (s32)c_cos_8192(tmp);
		tmp = FromFlightCtrl.AngleRoll/10; // in deg 
		sinroll = (s32)c_sin_8192(tmp);
		cosroll = (s32)c_cos_8192(tmp);
		// tbd. compensation signs and oriantation has to be fixed 
		Hx = (s16)((MagVector.Y * cosnick + MagVector.Z * sinnick)/8192L);
		Hy = (s16)((MagVector.X * cosroll - MagVector.Z * sinroll)/8192L);		
		// calculate heading
		tmp = (s16)(c_atan2_546(Hy, Hx)/546L);
		if (tmp > 0) tmp = 360 - tmp;
		else tmp = -tmp;
		Compass_Heading = tmp;
	}
}

void Compass_Update(void)
{
	// check for new cal state
	Compass_UpdateCalState();
	// initiate new compass communication
	switch(Compass_Device)
	{
	 	case COMPASS_MK3MAG:
	   		MK3MAG_Update();
			break;
		case COMPASS_NCMAG:
			NCMAG_Update(); 
		default:
			break; 
	}
}

// put cal state into fifo
void Compass_SetCalState(u8 CalState)
{
	fifo_put(&CompassCalcStateFiFo, CalState);	
}

// get cal state from fifo
void Compass_UpdateCalState()
{
	fifo_get(&CompassCalcStateFiFo, (u8*)&Compass_CalState);	
}

