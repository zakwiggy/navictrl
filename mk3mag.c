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
#include <string.h>
#include "91x_lib.h"
#include "mk3mag.h"
#include "i2c.h"
#include "timer1.h"
#include "led.h"
#include "main.h"
#include "uart1.h"
#include "compass.h"

#define MK3MAG_SLAVE_ADDRESS 		0x50  // i2C slave address

u8 MK3MAG_Present = 0;

typedef struct
{
  s16 Nick;
  s16 Roll;
} __attribute__((packed)) MK3MAG_WriteAttitude_t;

typedef struct
{
  u8 Major;
  u8 Minor;
  u8 Patch;
  u8 Compatible;
} __attribute__((packed)) MK3MAG_Version_t;

typedef struct
{
  u8 CalByte;
  u8 Dummy1;
  u8 Dummy2;
} __attribute__((packed)) MK3MAG_Cal_t;

// Transfer buffers
volatile MK3MAG_WriteAttitude_t		MK3MAG_WriteAttitude;
volatile MK3MAG_Version_t			MK3MAG_Version;
volatile MK3MAG_Cal_t				MK3MAG_WriteCal;
volatile MK3MAG_Cal_t				MK3MAG_ReadCal;

// -------------------------------------------------------------------------------
// the calculation of the MK3MAG packet checksum

// calculate the crc of  bytecount bytes in buffer
u8 MK3MAG_CalcCRC(u8* pBuffer, u8 bytecount)
{
	u8 i, crc = 0;
	for(i=0; i < bytecount; i++)
	{
	 	crc += pBuffer[i];
	}
	crc = ~crc;
	return crc;
}

// assuming the last byte in buffer is the crc byte
u8 MK3MAG_CheckCRC(u8* pBuffer, u8 BuffLen)
{
 	u8 crc = 0, retval = 0;
	if(BuffLen == 0) return(retval);
	crc = MK3MAG_CalcCRC(pBuffer, BuffLen-1);
	if(crc == pBuffer[BuffLen-1])
	{
		retval = 1;
		DebugOut.Analog[15]++; // count mk3mag ok
	}
	else
	{
		retval = 0;
		DebugOut.Analog[14]++; // count mk3mag crc error
	}
	return(retval);
}

// -------------------------------------------------------------------------------
// the I2C rx data handler functions

// rx data handler for version info request
void MK3MAG_UpdateVersion(u8* pRxBuffer, u8 RxBufferSize)
{	// if crc is ok and number of byte are matching
	if(MK3MAG_CheckCRC(pRxBuffer, RxBufferSize) && (RxBufferSize == (sizeof(MK3MAG_Version)+1)) )
	{
		memcpy((u8 *)&MK3MAG_Version, pRxBuffer, sizeof(MK3MAG_Version));
	}		  
}

// rx data handler for calibration request
void MK3MAG_UpdateCalibration(u8* pRxBuffer, u8 RxBufferSize)
{	// if crc is ok and number of byte are matching
	if(MK3MAG_CheckCRC(pRxBuffer, RxBufferSize) && (RxBufferSize == (sizeof(MK3MAG_ReadCal)+1)) )
	{
		memcpy((u8 *)&MK3MAG_ReadCal, pRxBuffer, sizeof(MK3MAG_ReadCal));
	}			  
}

// rx data handler for magnetic vector request
void MK3MAG_UpdateMagVector(u8* pRxBuffer, u8 RxBufferSize)
{	// if crc is ok and number of byte are matching
	if(MK3MAG_CheckCRC(pRxBuffer, RxBufferSize) && (RxBufferSize == (sizeof(MagVector)+1)) )
	{
		s16vec_t *pMagVector = (s16vec_t*)pRxBuffer;
		MagVector.X = pMagVector->Y;
		MagVector.Y = pMagVector->X;
		MagVector.Z = -pMagVector->Z;	 
		Compass_CalcHeading();
	}			  
}

//----------------------------------------------------------------
#define MK3MAG_CMD_VERSION   		0x01
#define MK3MAG_CMD_READ_MAGVECT    	0x02
#define MK3MAG_CMD_WRITE_CAL     	0x04

void MK3MAG_SendCommand(u8 command)
{
	// try to catch the I2C buffer
	if(I2C_LockBuffer(0))
	{
		u16 TxBytes = 0;
		u16 RxBytes = 0;
		I2C_pRxHandler_t pRxHandlerFunc = NULL;
		
		// update current command id
	  	I2C_Buffer[TxBytes++] = command;

		// set pointers to data area with respect to the command id
		switch (command)
		{
			case MK3MAG_CMD_VERSION:
				RxBytes = sizeof(MK3MAG_Version)+1;
				pRxHandlerFunc = &MK3MAG_UpdateVersion;
				break;
			case MK3MAG_CMD_WRITE_CAL:
				RxBytes = sizeof(MK3MAG_ReadCal)+1;
				pRxHandlerFunc = &MK3MAG_UpdateCalibration;
				memcpy((u8*)I2C_Buffer+1, (u8*)&MK3MAG_WriteCal, sizeof(MK3MAG_WriteCal));
				TxBytes += sizeof(MK3MAG_WriteCal);
				break;
			case MK3MAG_CMD_READ_MAGVECT:
				RxBytes = sizeof(MagVector)+1;
				pRxHandlerFunc = &MK3MAG_UpdateMagVector;
				break;
			default: // unknown command id
				RxBytes = 0;
				pRxHandlerFunc = NULL;
				break;
		}
		// update packet checksum
		I2C_Buffer[TxBytes] = MK3MAG_CalcCRC((u8*)I2C_Buffer, TxBytes);
		TxBytes++;
		// initiate I2C transmission
		I2C_Transmission(MK3MAG_SLAVE_ADDRESS, TxBytes, pRxHandlerFunc, RxBytes);
	} // EOF I2C_State == I2C_IDLE	
}


//----------------------------------------------------------------
u8 MK3MAG_Init(void)
{
	u8 msg[64];
	u8 repeat;
	u32 timeout;
	
	MK3MAG_Present = 0;

	MK3MAG_Version.Major = 0xFF;
	MK3MAG_Version.Minor = 0xFF;
	MK3MAG_Version.Patch = 0xFF;
	MK3MAG_Version.Compatible = 0xFF;

	Compass_Heading = -1;

	// polling of version info
	repeat = 0;
	do
	{
		MK3MAG_SendCommand(MK3MAG_CMD_VERSION);
		timeout = SetDelay(250); 
	   	do
		{
			if (MK3MAG_Version.Major != 0xFF) break; // break loop on success
		}while (!CheckDelay(timeout));
		UART1_PutString(".");
		repeat++;
	}while ((MK3MAG_Version.Major == 0xFF) && (repeat < 12)); // 12*250ms=3s
	// if we got it
	if (MK3MAG_Version.Major != 0xFF)
	{
		sprintf(msg, " MK3MAG V%d.%d%c", MK3MAG_Version.Major, MK3MAG_Version.Minor, 'a' + MK3MAG_Version.Patch);
		UART1_PutString(msg);
		if(MK3MAG_Version.Compatible != MK3MAG_I2C_COMPATIBLE)
		{
			UART1_PutString("\n\r MK3MAG not compatible!");
			UART_VersionInfo.HardwareError[0] |= NC_ERROR0_COMPASS_INCOMPATIBLE;
			LED_RED_ON;
		}
		else
		{	// version ok
			MK3MAG_Present = 1;
		}
	}
	return(MK3MAG_Present);
}

//----------------------------------------------------------------
void MK3MAG_Update(void)
{
	static u32 TimerUpdate = 0;
	static s16 x_max,y_max,z_max,x_min,y_min,z_min;

	if( (I2C_State == I2C_STATE_OFF) || !MK3MAG_Present ) return;
	
	if(CheckDelay(TimerUpdate))
	{
		// check for incomming compass calibration request
		Compass_UpdateCalState();
		MK3MAG_WriteCal.CalByte = Compass_CalState;
	
		if(MK3MAG_ReadCal.CalByte != MK3MAG_WriteCal.CalByte)
		{	// send new calibration state
			MK3MAG_SendCommand(MK3MAG_CMD_WRITE_CAL);
		}
		else
		{	// calibration state matches
			MK3MAG_SendCommand(MK3MAG_CMD_READ_MAGVECT); // initiate magvector transfer
		
			switch(Compass_CalState)
			{
				case 1:
					x_max = -30000; y_max = -30000; z_max = -30000;
					x_min = 30000; y_min = 30000; z_min = 30000;
					break;
				
				case 2:
						 if(MagVector.X > x_max) { x_max = MagVector.X; BeepTime = 60; }
					else if(MagVector.X < x_min) { x_min = MagVector.X; BeepTime = 20; }
						 if(MagVector.Y > y_max) { y_max = MagVector.Y; BeepTime = 60; }
					else if(MagVector.Y < y_min) { y_min = MagVector.Y; BeepTime = 20; }
					break;
				
				case 4:
						 if(MagVector.Z > z_max) { z_max = MagVector.Z; BeepTime = 80; }
					else if(MagVector.Z < z_min) { z_min = MagVector.Z; BeepTime = 80; }
					break;
					
				default:
					break;
			}	
		}
		TimerUpdate = SetDelay(20);    // every 20 ms are 50 Hz
	}
}
