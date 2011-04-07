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
#include <stdio.h>
#include "91x_lib.h"
#include "i2c.h"
#include "uart1.h"
#include "timer1.h"
#include "eeprom.h"
#include "led.h"


// The EEPROM M24C64 (64k) ist connected via I2C1 interface to the controller.
// The E0, E2, E3 pins are set to low. Therefore the slave adressbits b3, b2, b1 are 0.

#define EEPROM_I2C_ADDRESS 	0xA0
#define EEPROM_SIZE			65536L	// 64k bytes
#define EEPROM_WRITE		1
#define EEPROM_READ			0

// timing
#define I2C_IDLE_TIMEOUT 		150 // 150 ms
#define I2C_TRANSFER_TIMEOUT 	150 // 150 ms
#define I2C_ACCESS_RETRYS		20	// retry 20 times

// globals for rx handler
volatile u8	*EEPROM_pData = 0;
volatile u16 EEPROM_DataLen = 0;

// rx data handler for eeprom data read access
void EEPROM_RxDataHandler(u8* pRxBuffer, u8 RxBufferSize)
{	// if number of byte are matching
	if((RxBufferSize == EEPROM_DataLen))
	{	//copy data from primary buffer to target buffer 
		if(EEPROM_pData) memcpy((u8*)EEPROM_pData, pRxBuffer, EEPROM_DataLen);
	}	  
}

// ----------------------------------------------------------------------------------------
EEPROM_Result_t EEPROM_Transfer(u8 Direction, u16 Address, u8 *pData, u16 DataLen)
{
	u16 i, TxBytes = 0;
	u8 retry = 0;
	EEPROM_Result_t retval = EEPROM_ERROR_UNKNOWN;

	if(((u32)Address + (u32)DataLen) >= (EEPROM_SIZE - 1))
	{
		retval = EEPROM_ERROR_OUT_OF_ADDRESS_RANGE; 
		return(retval);
	}
	if((DataLen+2) > I2C_BUFFER_LEN)
	{
		retval = EEPROM_I2C_BUFFER_OVERRUN;
		return(retval);
	}
	do
	{
		if(!I2C_LockBuffer(I2C_IDLE_TIMEOUT)) return EEPROM_ERROR_I2C_IDLE_TIMEOUT;
		// buffer is now locked
		TxBytes = 0;
		// transmitt address
		I2C_Buffer[TxBytes++] = (u8)(0x00FF & (Address>>8));
		I2C_Buffer[TxBytes++] = (u8)(0x00FF & Address);
		if(Direction == EEPROM_WRITE)
		{	// copy data to i2c transfer buffer
			for(i = 0; i<DataLen;i++)
			{
				I2C_Buffer[TxBytes++] = pData[i];
			}
			// prepare pointer to rx data
			EEPROM_pData = 0;
			EEPROM_DataLen = 0;
			// start transmission
			if(!I2C_Transmission(EEPROM_I2C_ADDRESS, TxBytes, 0, 0))
			{
				return(retval);
			}
		}
		else // Direction == EEPROM_READ
		{
			// prepare pointer to rx data
			EEPROM_pData = pData;
			EEPROM_DataLen = DataLen;
			// start transmission
		 	if(!I2C_Transmission(EEPROM_I2C_ADDRESS, TxBytes, &EEPROM_RxDataHandler, DataLen))
			{
				return(retval);
			}
		}
		//wait for end of this transfer
		if(I2C_WaitForEndOfTransmission(I2C_TRANSFER_TIMEOUT)) 
		{
			if(I2C_Error == I2C_ERROR_NONE) return(EEPROM_SUCCESS);
			else retval = EEPROM_DATA_TRANSFER_INCOMPLETE;
		}
		else// i2c transfer timed out
		{
			return(EEPROM_ERROR_I2C_TRANSFER_TIMEOUT);	
		}
		// or retry		
	} while(++retry < I2C_ACCESS_RETRYS);
	return(retval);
}

EEPROM_Result_t EEPROM_WriteBlock(u16 Address, u8 *pData, u16 DataLen)
{
	u16 len;
	EEPROM_Result_t retval;

	// there can be 32 bytes written in a single cycle if the adresses are locates in the same row
	// i.e. the upper 11 bits of the addres are constant
	#define EEPROM_WRITE_BLOCK_SIZE 32

	len = EEPROM_WRITE_BLOCK_SIZE - Address%EEPROM_WRITE_BLOCK_SIZE; // number of bytes to match the next row
	if(len >= DataLen) len = DataLen; // do not write more than neccesarry

	do
	{
		retval = EEPROM_Transfer(EEPROM_WRITE, Address, pData, len);
		Address += len; // Address of the next row
		pData += len;	// pointer to start of next data block
		DataLen -= len; // reduce the number of to be transmitted
		if(DataLen > EEPROM_WRITE_BLOCK_SIZE) len = EEPROM_WRITE_BLOCK_SIZE; // limit next block size to 32
		else len = DataLen;
	}
	while(DataLen && (retval == EEPROM_SUCCESS)); // repeat until all data have been sent and no error has occured

	return(retval);
}

// a data block can be subsequently read after setting the start address once
EEPROM_Result_t EEPROM_ReadBlock(u16 Address, u8 *pData, u16 DataLen)
{
	EEPROM_Result_t retval = EEPROM_ERROR_UNKNOWN;
 	u16 AdrOffset = 0;
	u16 RxLen;

	while(DataLen > 0)
	{
		if(DataLen > I2C_BUFFER_LEN) RxLen = I2C_BUFFER_LEN;
		else RxLen = DataLen;
	   
		retval = EEPROM_Transfer(EEPROM_READ, Address+AdrOffset, &(pData[AdrOffset]), RxLen);
		if(retval != EEPROM_SUCCESS) break;
		AdrOffset += RxLen;
		DataLen -= RxLen;
	}
	return(retval);
}

u8 EEPROM_Init(void)
{
	u8 data[20];
	u8 i, retval = 0;
	UART1_PutString("\r\n EEprom init..");
	// check if data can be read from eeprom
	for(i=0;i<10;i++)
	{
		UART1_Putchar('.');
		if(EEPROM_SUCCESS == EEPROM_ReadBlock(0, data, 20)) retval = 1;
		if(retval) break;		
	}
	if(retval)  UART1_PutString("ok");
	else		UART1_PutString("failed");
	return(retval);
}
