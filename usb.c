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
#include "config.h"
#include "main.h"
#include "uart1.h"
#include "mkprotocol.h"
#include "waypoints.h"
#include "gps.h"
#include "timer1.h"
#include "uart0.h"
#include "uart1.h"
#include "uart2.h"
#include "menu.h"
#include "usb.h"

#define ABO_TIMEOUT 4000 // disable abo after 4 seconds
u32 USB_AboTimeOut = 0;

u16 Echo; // 2 bytes recieved will be sent back as echo

// the primary rx fifo
#define USB_RX_FIFO_LEN 512
u8 USB_rxfifobuffer[USB_RX_FIFO_LEN];
fifo_t USB_rx_fifo;

// the tx buffer
#define USB_TX_BUFFER_LEN  150
u8 USB_tbuffer[USB_TX_BUFFER_LEN];
Buffer_t USB_tx_buffer;

// the rx buffer
#define USB_RX_BUFFER_LEN  150
u8 USB_rbuffer[USB_RX_BUFFER_LEN];
Buffer_t USB_rx_buffer;

u8 USB_Request_VersionInfo 	= FALSE;
u8 USB_Request_SendFollowMe	= FALSE;
u8 USB_Request_ExternalControl= FALSE;
u8 USB_Request_Display		= FALSE;
u8 USB_Request_Display1 	= FALSE;
u8 USB_Request_DebugData 	= FALSE;
u8 USB_Request_DebugLabel	= 255;
u8 USB_Request_NaviData		= FALSE;
u8 USB_Request_ErrorMessage	= FALSE;
u8 USB_Request_Data3D		= FALSE;
u8 USB_Request_Echo		    = FALSE;
u8 USB_DisplayKeys = 0;
u8 USB_DisplayLine 			= 0;
u8 USB_ConfirmFrame = 0;

u32 USB_DebugData_Timer = 0;
u32 USB_DebugData_Interval = 0;	// in ms
u32 USB_NaviData_Timer = 0;
u32 USB_NaviData_Interval = 0;	// in ms
u32 USB_Data3D_Timer = 0;
u32 USB_Data3D_Interval = 0;	// in ms
u32 USB_Display_Timer = 0;
u32 USB_Display_Interval = 0;	// in ms

//-----------------------------------------------------------------
void USB_ConfigInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	UART1_PutString("\r\n USB init...");
  	#ifdef MCLK96MHZ
	//USB clock = MCLK/2 = 48MHz
	SCU_USBCLKConfig(SCU_USBCLK_MCLK2);
	#else
	//USB clock = MCLK = 48MHz
	SCU_USBCLKConfig(SCU_USBCLK_MCLK);
	#endif
	//Enable USB clock
	SCU_AHBPeriphClockConfig(__USB,ENABLE);
	SCU_AHBPeriphReset(__USB,DISABLE);
	SCU_AHBPeriphClockConfig(__USB48M,ENABLE);

	//Configure GPIO0 (D+ Pull-Up on P0.1)
	SCU_APBPeriphClockConfig(__GPIO0 ,ENABLE);
	SCU_APBPeriphReset(__GPIO0,DISABLE);

	// GPIO_DeInit(P0.1);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
	GPIO_InitStructure.GPIO_Alternate=GPIO_OutputAlt1;
	GPIO_Init (GPIO0, &GPIO_InitStructure);

	// initialize the rx fifo, block UART IRQ geting a byte from fifo
	fifo_init(&USB_rx_fifo, USB_rxfifobuffer, USB_RX_FIFO_LEN, NO_ITLine, USBLP_ITLine);

	// initialize txd buffer
	Buffer_Init(&USB_tx_buffer, USB_tbuffer, USB_TX_BUFFER_LEN);

	// initialize rxd buffer
	Buffer_Init(&USB_rx_buffer, USB_rbuffer, USB_RX_BUFFER_LEN);

	VIC_Config(USBLP_ITLine, VIC_IRQ, PRIORITY_USB);
	VIC_ITCmd(USBLP_ITLine, ENABLE);

	USB_Init();

	UART1_PutString("ok");
}


/**************************************************************/
/* Process incomming data from debug uart                     */
/**************************************************************/
void USB_ProcessRxData(void)
{
	u8 c;
	SerialMsg_t SerialMsg;

	// if data in the rxd buffer are not locked immediately return
	// if rx buffer is not locked
	if(USB_rx_buffer.Locked == FALSE)
	{
		//collect data from primary rx fifo
		while(fifo_get(&USB_rx_fifo, &c))
		{
			// break if complete frame has been collected
			if(MKProtocol_CollectSerialFrame(&USB_rx_buffer, c)) break;
		}
	}
   	if(USB_rx_buffer.Locked == FALSE) return;

	MKProtocol_DecodeSerialFrameHeader(&USB_rx_buffer, &SerialMsg); // decode serial frame in rxd buffer
	MKProtocol_DecodeSerialFrameData(&USB_rx_buffer, &SerialMsg); // decode serial frame in rxd buffer

    if(SerialMsg.CmdID != 'z') SerialLinkOkay = 250;	  // reset SerialTimeout, but not in case of the "ping"
	switch(SerialMsg.Address) // check for Slave Address
	{
		case NC_ADDRESS:  // own Slave Address
		switch(SerialMsg.CmdID)
		{
			case 'z': // connection checker
				memcpy(&Echo, SerialMsg.pData, sizeof(Echo)); // copy echo pattern
				USB_Request_Echo = TRUE;
				break;

			case 'e': // request for the text of the error status
				USB_Request_ErrorMessage = TRUE;
				break;

			default:
				// unsupported command recieved
				break;
		} // case NC_ADDRESS
		// "break;" is missing here to fall thru to the common commands

		default:  // and any other Slave Address

		switch(SerialMsg.CmdID) // check CmdID
  		{
			case 'a':// request for the labels of the analog debug outputs
				USB_Request_DebugLabel = SerialMsg.pData[0];
				if(USB_Request_DebugLabel > 31) USB_Request_DebugLabel = 31;
				break;
			/*
			case 'b': // submit extern control
				memcpy(&ExternControl, SerialMsg.pData, sizeof(ExternControl));
				USB_ConfirmFrame = ExternControl.Frame;
				break;
			*/
			case 'd': // request for debug data;
				USB_DebugData_Interval = (u32) SerialMsg.pData[0] * 10;
				if(USB_DebugData_Interval > 0) USB_Request_DebugData = TRUE;
				USB_AboTimeOut = SetDelay(ABO_TIMEOUT);
				break;

			case 'c': // request for 3D data;
				USB_Data3D_Interval = (u32) SerialMsg.pData[0] * 10;
				if(USB_Data3D_Interval > 0) USB_Request_Data3D = TRUE;
				USB_AboTimeOut = SetDelay(ABO_TIMEOUT);
				break;

			case 'h':// reqest for display line
				if((SerialMsg.pData[0]& 0x80) == 0x00)// old format
				{
				 	USB_DisplayLine = 2;
					USB_Display_Interval = 0;
				}
				else
				{
					USB_DisplayKeys |= ~SerialMsg.pData[0];
					USB_Display_Interval = (u32) SerialMsg.pData[1] * 10;
					USB_DisplayLine = 4;
					USB_AboTimeOut = SetDelay(ABO_TIMEOUT);
				}
				USB_Request_Display = TRUE;
				break;

			case 'l':// reqest for display columns
				MenuItem = SerialMsg.pData[0];
				USB_Request_Display1 = TRUE;
				break;

			case 'o': // request for navigation information
				USB_NaviData_Interval = (u32) SerialMsg.pData[0] * 10;
				if(USB_NaviData_Interval > 0) USB_Request_NaviData = TRUE;
				USB_AboTimeOut = SetDelay(ABO_TIMEOUT);
				break;

			case 'v': // request for version info
				USB_Request_VersionInfo = TRUE;
				break;
			default:
				// unsupported command recieved
				break;
		}
		break; // default:
	}
	Buffer_Clear(&USB_rx_buffer);
}


//-----------------------------------------------------------------
void USB_CableConfig(FunctionalState NewState)
{
	if (NewState == ENABLE)
	GPIO_WriteBit(GPIO0, GPIO_Pin_1, Bit_RESET);
	else
	GPIO_WriteBit(GPIO0, GPIO_Pin_1, Bit_SET);
}
//-----------------------------------------------------------------
void USB_EnterLowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}
//-----------------------------------------------------------------
void USB_LeaveLowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}


//-----------------------------------------------------------------
void USB_PutString(u8 *string)
{
	u8 i = 0;
	u16 timeout = 0;

	while (string[i++] != 0){} // get string len
	while (_GetEPTxStatus(ENDP1) != EP_TX_NAK){ if (timeout++ > 60000) return;}
	UserToPMABufferCopy(string, ENDP1_TXADDR, ++i);	// copy string to usb buffer
	SetEPTxCount(ENDP1,i);
	SetEPTxValid(ENDP1);
}

//-----------------------------------------------------------------
void USB_PutChar(u8 c)
{
	u16 timeout = 0;
	while (_GetEPTxStatus(ENDP1) != EP_TX_NAK){ if (timeout++ > 60000) return;}
	UserToPMABufferCopy(&c, ENDP1_TXADDR, 2);
	SetEPTxCount(ENDP1,2);
	SetEPTxValid(ENDP1);
}

//-----------------------------------------------------------------
void USB_SendData(u8 *pdata, u16 count)
{
	u8 i;
	count++;
	u16 timeout = 0;

	for (i=0;i< (count/64)+1;i++)
	{
		while (_GetEPTxStatus(ENDP1) != EP_TX_NAK){if (timeout++ > 60000) return;}
		if (i < (count/64))
		{
			UserToPMABufferCopy(&pdata[i*64], ENDP1_TXADDR, 64);
			SetEPTxCount(ENDP1,64);
		}
		else
		{
			UserToPMABufferCopy(&pdata[i*64], ENDP1_TXADDR, count % 64);
			SetEPTxCount(ENDP1, count % 64);
		}
		SetEPTxValid(ENDP1);
	}
}

/**************************************************************/
/*         Transmit tx buffer via usb                         */
/**************************************************************/
void USB_Transmit(void)
{   // nur blockweises kopieren des sendebuffers, nicht alles mit einem mal
	// if something has to be send and the txd fifo is not full

	if(USB_tx_buffer.Locked == TRUE)
	{
		if(_GetEPTxStatus(ENDP1) == EP_TX_NAK)
		{
			u16 i;
			if(USB_tx_buffer.Position < USB_tx_buffer.DataBytes)
			{
				i = USB_tx_buffer.DataBytes - USB_tx_buffer.Position; // bytes to send
				if(i > 64) i = 64; // limit packet size to 64 bytes
				UserToPMABufferCopy(&(USB_tx_buffer.pData[USB_tx_buffer.Position]), ENDP1_TXADDR, i);
				SetEPTxCount(ENDP1,i);
				SetEPTxValid(ENDP1);
	    		USB_tx_buffer.Position += i;
			}
		}
		if(USB_tx_buffer.Position >= USB_tx_buffer.DataBytes) // all bytes transfered
		{
			Buffer_Clear(&USB_tx_buffer); // clear buffer
		}
	}
}

/**************************************************************/
/* Send the answers to incomming commands at the debug uart   */
/**************************************************************/
void USB_TransmitTxData(void)
{
	USB_Transmit(); // output pending bytes in tx buffer
	if((USB_tx_buffer.Locked == TRUE)) return;

	if(CheckDelay(USB_AboTimeOut))
	{
		USB_DebugData_Interval = 0;
		USB_NaviData_Interval = 0;
		USB_Data3D_Interval = 0;
		USB_Display_Interval = 0;
	}

	if((USB_Request_DebugLabel != 0xFF) && (USB_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'A', NC_ADDRESS, 2, &USB_Request_DebugLabel, sizeof(USB_Request_DebugLabel), (u8 *) ANALOG_LABEL[USB_Request_DebugLabel], 16);
		USB_Request_DebugLabel = 0xFF;
	}
	else if(USB_ConfirmFrame && (USB_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'B', NC_ADDRESS, 1, &USB_ConfirmFrame, sizeof(USB_ConfirmFrame));
		USB_ConfirmFrame = 0;
	}
	else if( (( (USB_DebugData_Interval > 0) && CheckDelay(USB_DebugData_Timer)) || USB_Request_DebugData) && (USB_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'D', NC_ADDRESS, 1,(u8 *)&DebugOut, sizeof(DebugOut));
		USB_DebugData_Timer = SetDelay(USB_DebugData_Interval);
		USB_Request_DebugData = FALSE;
	}
	else if((( (USB_Data3D_Interval > 0) && CheckDelay(USB_Data3D_Timer) ) || USB_Request_Data3D) && (USB_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'C', NC_ADDRESS, 1,(u8 *)&Data3D, sizeof(Data3D));
		USB_Data3D_Timer = SetDelay(USB_Data3D_Interval);
		USB_Request_Data3D = FALSE;
	}
	else if(USB_Request_ExternalControl && (USB_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'G', NC_ADDRESS, 1, (u8 *)&ExternControl, sizeof(ExternControl));
		USB_Request_ExternalControl = FALSE;
	}
	else if( (( (USB_Display_Interval > 0) && CheckDelay(USB_Display_Timer)) || USB_Request_Display) && (USB_tx_buffer.Locked == FALSE))
	{
		Menu_Update(USB_DisplayKeys);
		USB_DisplayKeys = 0;
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'H', NC_ADDRESS, 1, (u8*)DisplayBuff, sizeof(DisplayBuff));
		USB_Request_Display = FALSE;
	}
	else if(USB_Request_Display1 && (USB_tx_buffer.Locked == FALSE))
	{
		Menu_Update(0);
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'L', NC_ADDRESS, 3, (u8*)&MenuItem, sizeof(MenuItem), (u8*)&MaxMenuItem, sizeof(MaxMenuItem),(u8*)DisplayBuff, sizeof(DisplayBuff));
		USB_Request_Display1 = FALSE;
	}
	else if(USB_Request_VersionInfo && (USB_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'V', NC_ADDRESS,1, (u8 *)&UART_VersionInfo, sizeof(UART_VersionInfo));
		USB_Request_VersionInfo = FALSE;
	}
	else if(( (USB_NaviData_Interval && CheckDelay(USB_NaviData_Timer) ) || USB_Request_NaviData) && (USB_tx_buffer.Locked == FALSE))
	{
		NaviData.Errorcode = ErrorCode;
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'O', NC_ADDRESS,1, (u8 *)&NaviData, sizeof(NaviData));
		USB_NaviData_Timer = SetDelay(USB_NaviData_Interval);
		USB_Request_NaviData = FALSE;
	}
	else if(USB_Request_ErrorMessage && (USB_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&USB_tx_buffer, 'E', NC_ADDRESS, 1, (u8 *)&ErrorMSG, sizeof(ErrorMSG));
		USB_Request_ErrorMessage = FALSE;
	}
	USB_Transmit(); // output pending bytes in tx buffer
}
