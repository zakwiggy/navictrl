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
#include <stdarg.h>
#include <string.h>
#include "91x_lib.h"
#include "config.h"
#include "main.h"
#include "uart0.h"
#include "uart1.h"
#include "timer1.h"
#include "ubx.h"
#include "mkprotocol.h"


//------------------------------------------------------------------------------------
// global variables

MKOSD_VersionInfo_t MKOSD_VersionInfo;

// UART0 MUXER
UART0_MuxerState_t UART0_Muxer = UART0_UNDEF;
u16 Uart0Baudrate = UART0_BAUD_RATE;
u16 Uart0MK3MagBaudrate = UART0_BAUD_RATE;

// the tx buffer
#define UART0_TX_BUFFER_LEN  150
u8 UART0_tbuffer[UART0_TX_BUFFER_LEN];
Buffer_t UART0_tx_buffer;

// the rx buffer
#define UART0_RX_BUFFER_LEN  150
u8 UART0_rbuffer[UART0_RX_BUFFER_LEN];
Buffer_t UART0_rx_buffer;

u8 UART0_Request_VersionInfo 	= FALSE;
u8 UART0_Request_NaviData		= FALSE;
u8 UART0_Request_ErrorMessage	= FALSE;
u32 UART0_NaviData_Timer;
u32 UART0_NaviData_Interval = 0;	// in ms

//------------------------------------------------------------------------------------
// functions

/********************************************************/
/*              Configure uart 0                        */
/********************************************************/
void UART0_Configure(u16 Baudrate)
{
	UART_InitTypeDef UART_InitStructure;

	SCU_APBPeriphClockConfig(__UART0, ENABLE);  // Enable the UART0 Clock

  	/* UART0 configured as follow:
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - BaudRate taken from function argument
          - Hardware flow control Disabled
          - Receive and transmit enabled
          - Receive and transmit FIFOs are Disabled
    */
	UART_StructInit(&UART_InitStructure);
    UART_InitStructure.UART_WordLength = 			UART_WordLength_8D;
    UART_InitStructure.UART_StopBits = 				UART_StopBits_1;
    UART_InitStructure.UART_Parity = 				UART_Parity_No ;
    UART_InitStructure.UART_BaudRate = 				Baudrate;
    UART_InitStructure.UART_HardwareFlowControl = 	UART_HardwareFlowControl_None;
    UART_InitStructure.UART_Mode = 					UART_Mode_Tx_Rx;
    UART_InitStructure.UART_FIFO = 					UART_FIFO_Enable;
    UART_InitStructure.UART_TxFIFOLevel = 			UART_FIFOLevel_1_2;
    UART_InitStructure.UART_RxFIFOLevel = 			UART_FIFOLevel_1_2;

	UART_DeInit(UART0);	// reset uart 0	to default
    UART_Init(UART0, &UART_InitStructure);  // initialize uart 0

	// enable uart 0 interrupts selective
    UART_ITConfig(UART0, UART_IT_Receive | UART_IT_ReceiveTimeOut  /*| UART_IT_FrameError*/, ENABLE);
	UART_Cmd(UART0, ENABLE); // enable uart 0
	// configure the uart 0 interupt line
 	VIC_Config(UART0_ITLine, VIC_IRQ, PRIORITY_UART0);
	// enable the uart 0 IRQ
	VIC_ITCmd(UART0_ITLine, ENABLE);
}

/********************************************************/
/*              Connect RXD & TXD to GPS                */
/********************************************************/
void UART0_Connect_to_MKGPS(u16 Baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	UART0_Muxer = UART0_UNDEF;

  	SCU_APBPeriphClockConfig(__GPIO6, ENABLE); // Enable the GPIO6 Clock
	// unmap UART0 from Compass
	// set port pin 5.1 (serial data from compass) to input and disconnect from IP
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull ;
    GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO5, &GPIO_InitStructure);
	// set port pin 5.0 (serial data to compass) to input
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO5, &GPIO_InitStructure);
	// map UART0 to GPS
	// set port pin 6.6 (serial data from gps) to input and connect to IP
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull ;
    GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Enable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1; //UART0_RxD
    GPIO_Init(GPIO6, &GPIO_InitStructure);
	// set port pin 6.7 (serial data to gps) to output
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinOutput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Enable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_OutputAlt3; //UART0_TX
    GPIO_Init(GPIO6, &GPIO_InitStructure);
	// configure the UART0
	UART0_Configure(Baudrate);
	
	UART0_Muxer = UART0_MKGPS;
}

/********************************************************/
/*              Connect RXD & TXD to MK3MAG             */
/********************************************************/
void UART0_Connect_to_MK3MAG(void)
{
	u16 Baudrate;

	GPIO_InitTypeDef GPIO_InitStructure;

	UART0_Muxer = UART0_UNDEF;

	SCU_APBPeriphClockConfig(__GPIO5, ENABLE);
   	// unmap UART0 from GPS
	// set port pin 6.6 (serial data from gps) to input and disconnect from IP
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
    GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO6, &GPIO_InitStructure);
	// set port pin 6.7 (serial data to gps) to input
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO6, &GPIO_InitStructure);

	// map UART0 to Compass
	// set port pin 5.1 (serial data from compass) to input and connect to IP
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull ;
    GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Enable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1; //UART0_RxD
    GPIO_Init(GPIO5, &GPIO_InitStructure);
	// set port pin 5.0 (serial data to compass) to output
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinOutput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Enable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_OutputAlt3; //UART0_TX
    GPIO_Init(GPIO5, &GPIO_InitStructure);
   	Baudrate = UART0_BAUD_RATE + ((UART0_BAUD_RATE * 2)/100);   // MK3Mag baudrate is a little bit higher...
	UART0_Configure(Baudrate);
	UART0_Muxer = UART0_MK3MAG;
}

/********************************************************/
/*                  Initialize UART0                    */
/********************************************************/
void UART0_Init(void)
{
	UART1_PutString("\r\n UART0 init...");

	UART0_Connect_to_MKGPS(UART0_BAUD_RATE);

	// initialize txd buffer
	Buffer_Init(&UART0_tx_buffer, UART0_tbuffer, UART0_TX_BUFFER_LEN);
	
	// initialize rxd buffer
	Buffer_Init(&UART0_rx_buffer, UART0_rbuffer, UART0_RX_BUFFER_LEN);

	UART1_PutString("ok");
}

/********************************************************/
/*            UART0 Interrupt Handler                   */
/********************************************************/
void UART0_IRQHandler(void)
{
	u8 c;
	// if receive irq (FIFO is over trigger level) or receive timeout irq (FIFO is not empty for longer times) has occured
 	if((UART_GetITStatus(UART0, UART_IT_Receive) != RESET) || (UART_GetITStatus(UART0, UART_IT_ReceiveTimeOut) != RESET) )
 	{
   		UART_ClearITPendingBit(UART0, UART_IT_Receive);			// clear receive interrupt flag
   		UART_ClearITPendingBit(UART0, UART_IT_ReceiveTimeOut);	// clear receive timeout interrupt flag

		// if debug UART is UART0
		if (DebugUART == UART0)
		{	// forward received data to the UART1 tx buffer
		 	while(UART_GetFlagStatus(UART0, UART_FLAG_RxFIFOEmpty) != SET)
			{
				// wait for space in the tx buffer of the UART1
				while(UART_GetFlagStatus(UART1, UART_FLAG_TxFIFOFull) == SET) {};
				// move the byte from the rx buffer of UART0 to the tx buffer of UART1
				UART_SendData(UART1, UART_ReceiveData(UART0));
			}
		}
		else // UART0 is not the DebugUART (normal operation)
		{
			// repeat until no byte is in the RxFIFO
	  		while (UART_GetFlagStatus(UART0, UART_FLAG_RxFIFOEmpty) != SET)
	  		{  
				c = UART_ReceiveData(UART0); // get byte from rx fifo
				switch(UART0_Muxer)
				{
					case UART0_MKGPS:
						UBX_RxParser(c); // if connected to GPS forward byte to ubx parser
						MKProtocol_CollectSerialFrame(&UART0_rx_buffer, c);	// ckeck for MK-Frames also
						break;
					case UART0_MK3MAG:
						// ignore any byte send from MK3MAG
						break;
					case UART0_UNDEF:
					default:
						// ignore the byte from unknown source
						break;
				} // eof switch(UART0_Muxer)
			} // eof while
		}  // eof UART0 is not the DebugUART
	} // eof receive irq or receive timeout irq
}

/**************************************************************/
/* Process incomming data from debug uart                     */
/**************************************************************/
void UART0_ProcessRxData(void)
{
	SerialMsg_t SerialMsg;
	// if data in the rxd buffer are not locked immediately return
	if((UART0_rx_buffer.Locked == FALSE) || (DebugUART == UART0) ) return;

	MKProtocol_DecodeSerialFrameHeader(&UART0_rx_buffer, &SerialMsg); // decode serial frame in rxd buffer	
	MKProtocol_DecodeSerialFrameData(&UART0_rx_buffer, &SerialMsg); // decode serial frame in rxd buffer

	switch(SerialMsg.Address) // check for Slave Address
	{
		case MKOSD_ADDRESS: // answers from the MKOSD
			switch(SerialMsg.CmdID)
			{
				case 'V':
					memcpy(&MKOSD_VersionInfo, SerialMsg.pData, sizeof(MKOSD_VersionInfo)); // copy echo pattern
					break;
				default:
					break;
			} // case MKOSD_ADDRESS	
			break;

		case NC_ADDRESS:  // own Slave Address
			switch(SerialMsg.CmdID)
			{
				case 'e': // request for the text of the error status
					UART0_Request_ErrorMessage = TRUE;
					break;
				case 'o': // request for navigation information
					UART0_NaviData_Interval = (u32) SerialMsg.pData[0] * 10;
					if(UART0_NaviData_Interval > 0) UART0_Request_NaviData = TRUE;
					break;
				default:
					break;
			} // case NC_ADDRESS
			// "break;" is missing here to fall thru to the common commands

		default:  // and any other Slave Address
			switch(SerialMsg.CmdID) // check CmdID
	  		{
				case 'v': // request for version info
					UART0_Request_VersionInfo = TRUE;
					break;
				default:
					// unsupported command recieved
					break;
			}
			break; // default:
	}
	Buffer_Clear(&UART0_rx_buffer);
}

/**************************************************************/
/*         Transmit tx buffer via uart0                       */
/**************************************************************/
void UART0_Transmit(void)
{
	u8 tmp_tx;

	IENABLE;

	if(DebugUART == UART0) return; // no data output if debug uart is rederected to UART0
	// if something has to be send and the txd fifo is not full
	if((UART0_tx_buffer.Locked == TRUE) && (UART_GetFlagStatus(UART0, UART_FLAG_TxFIFOFull) == RESET))
	{
		tmp_tx = UART0_tx_buffer.pData[UART0_tx_buffer.Position++]; // read next byte from txd buffer
		UART_SendData(UART0, tmp_tx); // put character to txd fifo
		// if terminating character or end of txd buffer reached
		if((tmp_tx == '\r') || (UART0_tx_buffer.Position == UART0_tx_buffer.Size))
		{
			Buffer_Clear(&UART0_tx_buffer);
		}
	}

	IDISABLE;
}


/**************************************************************/
/* Send the answers to incomming commands at the uart0        */
/**************************************************************/
void UART0_TransmitTxData(void)
{
	if(DebugUART == UART0) return;
	UART0_Transmit(); // output pending bytes in tx buffer
	if(UART0_tx_buffer.Locked == TRUE) return;

	else if(UART0_Request_ErrorMessage && (UART0_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART0_tx_buffer, 'E', NC_ADDRESS, 1, (u8 *)&ErrorMSG, sizeof(ErrorMSG));
		UART0_Request_ErrorMessage = FALSE;
	}
	else if(UART0_Request_VersionInfo && (UART0_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART0_tx_buffer, 'V', NC_ADDRESS,1, (u8 *)&UART_VersionInfo, sizeof(UART_VersionInfo));
		UART0_Request_VersionInfo = FALSE;
	}
	else if(( ((UART0_NaviData_Interval >0) && CheckDelay(UART0_NaviData_Timer) ) || UART0_Request_NaviData) && (UART0_tx_buffer.Locked == FALSE))
	{
		NaviData.Errorcode = ErrorCode;
		MKProtocol_CreateSerialFrame(&UART0_tx_buffer, 'O', NC_ADDRESS,1, (u8 *)&NaviData, sizeof(NaviData));
		UART0_NaviData_Timer = SetDelay(UART0_NaviData_Interval);
		UART0_Request_NaviData = FALSE;
	}	
	UART0_Transmit(); // output pending bytes in tx buffer
}


/**************************************************************/
/* Get the version of the MKOSD                               */
/**************************************************************/
u8 UART0_GetMKOSDVersion(void)
{
	u32 timeout;
	u8 msg[64];
	u8 retval = 0;

	MKOSD_VersionInfo.SWMajor = 0xFF;
	MKOSD_VersionInfo.SWMinor = 0xFF;
	MKOSD_VersionInfo.SWPatch = 0xFF;

	if(UART0_Muxer != UART0_MKGPS) UART0_Connect_to_MKGPS(UART0_BAUD_RATE);
	while(UART0_tx_buffer.Locked == TRUE) UART0_Transmit(); // output pending bytes in tx buffer;

	MKProtocol_CreateSerialFrame(&UART0_tx_buffer, 'v', MKOSD_ADDRESS, 0); // request for version info
	while(UART0_tx_buffer.Locked == TRUE) UART0_Transmit(); // output pending bytes in tx buffer;
	
	timeout = SetDelay(500);
	do
	{
		UART0_ProcessRxData();
		if(MKOSD_VersionInfo.SWMajor != 0xFF) break;
	}while(!CheckDelay(timeout));
	
	if(MKOSD_VersionInfo.SWMajor != 0xFF)
	{
		sprintf(msg, "\n\r MK-OSD V%d.%d%c", MKOSD_VersionInfo.SWMajor, MKOSD_VersionInfo.SWMinor, 'a'+MKOSD_VersionInfo.SWPatch);
		UART1_PutString(msg);
		retval = 1;
	}
	//else UART1_PutString("\n\r No version information from MK-OSD."); 
	return(retval);	
}

/**************************************************************/
/* Send a  message to the UBLOX device                        */
/**************************************************************/
u8 UART0_UBXSendMsg(u8* pData, u16 Len)
{
	u8 retval = 0;
	// check for connection to GPS
	if(UART0_Muxer != UART0_MKGPS) return(retval);
	while(UART0_tx_buffer.Locked == TRUE) UART0_Transmit(); // output pending bytes in tx buffer;
	UBX_CreateMsg(&UART0_tx_buffer, pData, Len);  // build ubx message frame
	while(UART0_tx_buffer.Locked == TRUE) UART0_Transmit(); // output pending bytes in tx buffer;
	return(1);
}

/**************************************************************/
/* Send a configuration message to the UBLOX device           */
/**************************************************************/
u8 UART0_UBXSendCFGMsg(u8* pData, u16 Len)
{
	u32 timeout;
	u8 retval = 0;
	// if data are not a CFG MSG
	if(pData[0]!= UBX_CLASS_CFG) return(retval);
	// prepare rx msg filter
	UbxMsg.Hdr.Class = UBX_CLASS_ACK;
	UbxMsg.Hdr.Id = 0xFF;
	UbxMsg.Hdr.Length = 0;
	UbxMsg.ClassMask = 0xFF;
	UbxMsg.IdMask = 0x00;
	UbxMsg.Status = INVALID;
	UART0_UBXSendMsg(pData, Len);
	// check for acknowledge msg
	timeout = SetDelay(100);
	do
	{
		if(UbxMsg.Status == NEWDATA) break;
	}while(!CheckDelay(timeout));
	if(UbxMsg.Status == NEWDATA)
	{	// 2 bytes payload
		if((UbxMsg.Data[0] == pData[0]) && (UbxMsg.Data[1] == pData[1]) && (UbxMsg.Hdr.Length == 2)) retval = UbxMsg.Hdr.Id;
	}
	UbxMsg.Status = INVALID;
	return(retval);
}

/**************************************************************/
/* Get Version Info from UBX Module                           */
/**************************************************************/
u8 UART0_GetUBXVersion(void)
{
	u32 timeout;
	u8 retval = 0xFF;
	u8 ubxmsg[]={0x0A, 0x04, 0x00, 0x00}; //MON-VER
	// prepare rx msg filter
	UbxMsg.Hdr.Class = 0x0A;
	UbxMsg.Hdr.Id = 0x04;
	UbxMsg.Hdr.Length = 0;
	UbxMsg.ClassMask = 0xFF;
	UbxMsg.IdMask = 0xFF;
	UbxMsg.Status = INVALID;
	UART0_UBXSendMsg(ubxmsg, sizeof(ubxmsg));
	// check for answer
	timeout = SetDelay(100);
	do
	{
		if(UbxMsg.Status == NEWDATA) break;
	}while(!CheckDelay(timeout));
	if((UbxMsg.Hdr.Length >= 40) && (UbxMsg.Status == NEWDATA))
	{
		UbxMsg.Data[4] = 0; //Only the fisrt 4 chsracters 
		UbxMsg.Data[39] = 0;
		UART1_PutString(" V");
		UART1_PutString((u8*)&UbxMsg.Data);
		UART1_PutString(" HW:");
		UART1_PutString((u8*)&UbxMsg.Data[30]);
		retval = UbxMsg.Data[0]-'0'; // major sw version	
	}
	UbxMsg.Status = INVALID;
	return(retval);
}
