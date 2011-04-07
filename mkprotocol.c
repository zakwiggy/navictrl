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
#include <stdarg.h>
#include "91x_lib.h"
#include "mkprotocol.h" 
#include "ramfunc.h"
#include "usb.h"

/**************************************************************/
/* Create serial output frame                                 */
/**************************************************************/
u8 MKProtocol_CreateSerialFrame(Buffer_t* pTxBuff, u8 CmdID, u8 Address, u8 numofbuffers , ...) //u8 *data, u8 len, ....
{
	va_list ap;

	u8 a,b,c;
	u8 ptr = 0;
	u16 tmpCRC = 0, i;

	u8* pdata = NULL;
	int len = 0;
	
	if(pTxBuff->Locked == TRUE) return(0);

	// tx-buffer is not in use
	// lock the buffer
	pTxBuff->Locked = TRUE; 
	pTxBuff->Position = 0;
	pTxBuff->pData[pTxBuff->Position++] = '#';			    // Start character
	pTxBuff->pData[pTxBuff->Position++] = 'a' + Address;	// Address (a=0; b=1,...)
	pTxBuff->pData[pTxBuff->Position++] = CmdID;			// Command

	va_start(ap, numofbuffers);
	if(numofbuffers)
	{
		pdata = va_arg(ap, u8*);
		len = va_arg(ap, int);
		ptr = 0;
		numofbuffers--;
	}
	while(len)
	{
		if(len)
		{
			a = pdata[ptr++];
			len--;
			if((!len) && numofbuffers) // try to jump to next buffer
			{
				pdata = va_arg(ap, u8*);
				len = va_arg(ap, int);
				ptr = 0;
				numofbuffers--;
			}
		}
		else a = 0;
		if(len)
		{
			b = pdata[ptr++];
			len--;
			if((!len) && numofbuffers) // try to jump to next buffer
			{
				pdata = va_arg(ap, u8*);
				len = va_arg(ap, int);
				ptr = 0;
				numofbuffers--;
			}
		}
		else b = 0;
		if(len)
		{
			c = pdata[ptr++];
			len--;
			if((!len) && numofbuffers) // try to jump to next buffer
			{
				pdata = va_arg(ap, u8*);
				len = va_arg(ap, int);
				ptr = 0;
				numofbuffers--;
			}
		}
		else c = 0;
		pTxBuff->pData[pTxBuff->Position++] = '=' + (a >> 2);
		pTxBuff->pData[pTxBuff->Position++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
		pTxBuff->pData[pTxBuff->Position++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
		pTxBuff->pData[pTxBuff->Position++] = '=' + ( c & 0x3f);
	}
	va_end(ap);
	// add crc
	for(i = 0; i < pTxBuff->Position; i++)
	{
		tmpCRC += pTxBuff->pData[i];
	}
	tmpCRC %= 4096;
	pTxBuff->pData[pTxBuff->Position++] = '=' + tmpCRC / 64;
	pTxBuff->pData[pTxBuff->Position++] = '=' + tmpCRC % 64;
	pTxBuff->pData[pTxBuff->Position++] = '\r';
	pTxBuff->DataBytes = pTxBuff->Position;
	pTxBuff->Position = 0;  // reset buffer position for transmision
	return(pTxBuff->Locked);
}

// typical called in an UART Rx ISR
/**************************************************************/
/* Collect serial frame                                       */
/**************************************************************/
u8 MKProtocol_CollectSerialFrame(Buffer_t* pRxBuff, u8 c)
{
	if(pRxBuff->Locked == FALSE)
	{ // rx buffer not locked
		if(c == '#') // if syncronisation character is received
		{
			pRxBuff->Position = 0;						// reset buffer
			pRxBuff->pData[pRxBuff->Position++] = c;	// copy 1st byte to buffer
			pRxBuff->DataBytes = 1;
		}
		else if (pRxBuff->Position < pRxBuff->Size) // rx buffer not full
		{
			pRxBuff->pData[pRxBuff->Position++] = c; // copy byte to rxd buffer
			pRxBuff->DataBytes++;
			// termination character received and sync has been established
			if ((c == '\r') && (pRxBuff->pData[0]== '#'))
			{
				// calculate checksum from transmitted data
				u16 crc = 0, i;
				u8 crc1, crc2;
				for(i = 0; i < (pRxBuff->Position-3); i++)
				{
					crc +=  pRxBuff->pData[i];  
				}		
				crc %= 4096;
				crc1 = '=' + crc / 64;
				crc2 = '=' + crc % 64;
				// compare checksum to transmitted checksum bytes
				if((crc1 == pRxBuff->pData[pRxBuff->Position-3]) && (crc2 == pRxBuff->pData[pRxBuff->Position-2]))
				{
				    // checksum is valid
					pRxBuff->Position = 0;					
					pRxBuff->Locked = TRUE;          			    // lock the rxd buffer 
					// if 2nd byte is an 'R' start bootloader
					if(pRxBuff->pData[2] == 'R')
 					{
						PowerOff();
						VIC_DeInit();
						Execute_Bootloader(); // Reset-Commando - Bootloader starten
 					}
				} // eof checksum valid
				else
				{	// checksum is invalid
					Buffer_Clear(pRxBuff);
				}  // eof checksum invalid
			} // eof termination character received
		} // rxd buffer not full
		else // rxd buffer overrun
		{
			Buffer_Clear(pRxBuff);
		} // eof rxd buffer overrun
	}
	return(pRxBuff->Locked);
}

/**************************************************************/
/* Decode detination address                                  */
/**************************************************************/
void MKProtocol_DecodeSerialFrameHeader(Buffer_t* pRxBuff, SerialMsg_t* pSerialMsg)
{
	if(pRxBuff->Locked)
	{
		pSerialMsg->Address = pRxBuff->pData[1] - 'a';
		pSerialMsg->CmdID = pRxBuff->pData[2];
	}
	else
	{
		pSerialMsg->Address = 0;
		pSerialMsg->CmdID = ' ';
	}	
}

/**************************************************************/
/* Decode data                                                */
/**************************************************************/
void MKProtocol_DecodeSerialFrameData(Buffer_t* pRxBuff, SerialMsg_t* pSerialMsg)
{
	u8 a,b,c,d;
	u8 x,y,z;
	u8 ptrIn = 3; // start with first data byte in rx buffer
	u8 ptrOut = 3;
	u8 len = pRxBuff->DataBytes - 6;	 // must be a multiple of 4 (3 bytes at begin and 3 bytes at end are no payload )
	while(len)
	{
		a = pRxBuff->pData[ptrIn++] - '=';
		b = pRxBuff->pData[ptrIn++] - '=';
		c = pRxBuff->pData[ptrIn++] - '=';
		d = pRxBuff->pData[ptrIn++] - '=';
		//if(ptrIn > ReceivedBytes - 3) break;

		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;

		if(len--) pRxBuff->pData[ptrOut++] = x; else break;
		if(len--) pRxBuff->pData[ptrOut++] = y; else break;
		if(len--) pRxBuff->pData[ptrOut++] = z; else break;
	}
	pSerialMsg->pData = &(pRxBuff->pData[3]);
	pSerialMsg->DataLen = ptrOut - 3;	// return number of data in bytes
	pRxBuff->Position = 0;
	pRxBuff->DataBytes = ptrOut;
}
