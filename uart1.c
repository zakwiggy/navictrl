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
#include "main.h"
#include "config.h"
#include "menu.h"
#include "gps.h"
#include "i2c.h"
#include "uart0.h"
#include "uart1.h"
#include "uart2.h"
#include "timer1.h"
#include "timer2.h"
#include "analog.h"
#include "compass.h"
#include "waypoints.h"
#include "mkprotocol.h"
#include "params.h"
#include "fifo.h"
#include "debug.h" 

#define FALSE	0
#define TRUE	1

#define ABO_TIMEOUT 4000 // disable abo after 4 seconds
u32 UART1_AboTimeOut = 0;

u8 UART1_Request_VersionInfo 	= FALSE;
u8 UART1_Request_ExternalControl= FALSE;
u8 UART1_Request_Display		= FALSE;
u8 UART1_Request_Display1 		= FALSE;
u8 UART1_Request_DebugData 		= FALSE;
u8 UART1_Request_DebugLabel		= 255;
u8 UART1_Request_NaviData		= FALSE;
u8 UART1_Request_ErrorMessage	= FALSE;
u8 UART1_Request_WritePoint		= 0xFF;
u8 UART1_Request_ReadPoint		= 0;
u8 UART1_Request_Data3D		    = FALSE;
u8 UART1_Request_Echo		    = FALSE;
u8 UART1_Request_ParameterId	= 0;
u8 UART1_Request_Parameter 		= FALSE;
u8 UART1_DisplayKeys			= 0;
u8 UART1_DisplayLine 			= 0;
u8 UART1_ConfirmFrame 			= 0;

UART_TypeDef *DebugUART = UART1;

// the primary rx fifo
#define UART1_RX_FIFO_LEN 512
u8 UART1_rxfifobuffer[UART1_RX_FIFO_LEN];
fifo_t UART1_rx_fifo;

// the rx buffer
#define UART1_RX_BUFFER_LEN  150
u8 UART1_rbuffer[UART1_RX_BUFFER_LEN];
Buffer_t UART1_rx_buffer;

// the tx buffer
#define UART1_TX_BUFFER_LEN  150
u8 UART1_tbuffer[UART1_TX_BUFFER_LEN];
Buffer_t UART1_tx_buffer;



volatile u8 SerialLinkOkay = 0;

u8 text[200];

const u8 ANALOG_LABEL[32][16] =
{
   //1234567890123456
	"AngleNick       ", //0
	"AngleRoll       ",
	"AccNick         ",
	"AccRoll         ",
	"OperatingRadius ",
	"FC-Flags        ", //5
	"NC-Flags        ",
	"NickServo       ",
	"RollServo       ",
	"GPS Data        ",
	"CompassHeading  ", //10
	"GyroHeading     ",
	"SPI Error       ",
	"SPI Okay        ",
	"I2C Error       ",
	"I2C Okay        ", //15
	"*POI_INDEX      ",//    "Kalman_K        ",
	"ACC_Speed_N     ",
	"ACC_Speed_E     ",
	"Speed_z         ",//    "GPS ACC         ",
	"20              ",//20
	"N_Speed         ",
	"E_Speed      	 ",
	"23              ",
	"24              ",
	"25              ",//25
	"26              ",
	"Distance N      ",
	"Distance E      ",
	"GPS_Nick        ",
	"GPS_Roll        ", //30
	"Used_Sats       "
};

DebugOut_t DebugOut;
ExternControl_t	ExternControl;
UART_VersionInfo_t UART_VersionInfo;
NaviData_t NaviData;
Data3D_t Data3D;
u16 Echo; // 2 bytes recieved will be sent back as echo

u32 UART1_DebugData_Timer = 0;
u32 UART1_DebugData_Interval = 0;	// in ms
u32 UART1_NaviData_Timer = 0;
u32 UART1_NaviData_Interval = 0;	// in ms
u32 UART1_Data3D_Timer = 0;
u32 UART1_Data3D_Interval = 0;		// in ms
u32 UART1_Display_Timer = 0;
u32 UART1_Display_Interval = 0;		// in ms

/********************************************************/
/*            Initialization the UART1                  */
/********************************************************/
void UART1_Init (void)


{
	GPIO_InitTypeDef GPIO_InitStructure;
	UART_InitTypeDef UART_InitStructure;

	// initialize txd buffer
	Buffer_Init(&UART1_tx_buffer, UART1_tbuffer, UART1_TX_BUFFER_LEN);

	// initialize rxd buffer
	Buffer_Init(&UART1_rx_buffer, UART1_rbuffer, UART1_RX_BUFFER_LEN);

	// initialize the rx fifo, block UART IRQ geting a byte from fifo
	fifo_init(&UART1_rx_fifo, UART1_rxfifobuffer, UART1_RX_FIFO_LEN, NO_ITLine, UART1_ITLine);

	SCU_APBPeriphClockConfig(__UART1, ENABLE);  // Enable the UART1 Clock
	SCU_APBPeriphClockConfig(__GPIO3, ENABLE);  // Enable the GPIO3 Clock

	/*Configure UART1_Rx pin GPIO3.2*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
	GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Enable;
	GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1; // UART1_RxD
	GPIO_Init(GPIO3, &GPIO_InitStructure);

	/*Configure UART1_Tx pin GPIO3.3*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinOutput;
	GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_Alternate = 	GPIO_OutputAlt2; // UART1_TX
	GPIO_Init(GPIO3, &GPIO_InitStructure);

	/* UART1 configured as follow:
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- BaudRate = 57600 baud
	- Hardware flow control Disabled
	- Receive and transmit enabled
	- Receive and transmit FIFOs are Disabled
	*/
	UART_StructInit(&UART_InitStructure);
	UART_InitStructure.UART_WordLength = 			UART_WordLength_8D;
	UART_InitStructure.UART_StopBits = 				UART_StopBits_1;
	UART_InitStructure.UART_Parity = 				UART_Parity_No ;
	UART_InitStructure.UART_BaudRate = 				UART1_BAUD_RATE;
	UART_InitStructure. UART_HardwareFlowControl = 	UART_HardwareFlowControl_None;
	UART_InitStructure.UART_Mode = 					UART_Mode_Tx_Rx;
	UART_InitStructure.UART_FIFO = 					UART_FIFO_Enable;
	UART_InitStructure.UART_TxFIFOLevel = 			UART_FIFOLevel_1_2;
	UART_InitStructure.UART_RxFIFOLevel = 			UART_FIFOLevel_1_2;

	UART_DeInit(UART1); // reset uart 1	to default
	UART_Init(UART1, &UART_InitStructure); // initialize uart 1
	// enable uart 1 interrupts selective
	UART_ITConfig(UART1, UART_IT_Receive | UART_IT_ReceiveTimeOut, ENABLE);
	UART_Cmd(UART1, ENABLE); // enable uart 1
	// configure the uart 1 interupt line
	VIC_Config(UART1_ITLine, VIC_IRQ, PRIORITY_UART1);
	// enable the uart 1 IRQ
	VIC_ITCmd(UART1_ITLine, ENABLE);

	// initialize the debug timer
	UART1_DebugData_Timer = SetDelay(UART1_DebugData_Interval);
	UART1_NaviData_Timer = SetDelay(UART1_NaviData_Interval)+500;

	// Fill Version Info Structure
	UART_VersionInfo.SWMajor = VERSION_MAJOR;
	UART_VersionInfo.SWMinor = VERSION_MINOR;
	UART_VersionInfo.SWPatch = VERSION_PATCH;
	UART_VersionInfo.ProtoMajor = VERSION_SERIAL_MAJOR;
	UART_VersionInfo.ProtoMinor = VERSION_SERIAL_MINOR;

	NaviData.Version = NAVIDATA_VERSION;

	UART1_PutString("\r\n UART1 init...ok");
}


/****************************************************************/
/*               USART1 receiver ISR                            */
/****************************************************************/
void UART1_IRQHandler(void)
{
	static u8 abortState = 0;
	u8 c;

	IENABLE;

	if((UART_GetITStatus(UART1, UART_IT_Receive) != RESET) || (UART_GetITStatus(UART1, UART_IT_ReceiveTimeOut) != RESET) )
	{
		// clear the pending bits!
		UART_ClearITPendingBit(UART1, UART_IT_Receive);
		UART_ClearITPendingBit(UART1, UART_IT_ReceiveTimeOut);
		// if debug UART is not UART1
		if (DebugUART != UART1)
		{	// forward received data to the debug UART tx buffer
			while(UART_GetFlagStatus(UART1, UART_FLAG_RxFIFOEmpty) != SET)
			{
				// move the byte from the rx buffer of UART1 to the tx buffer of DebugUART
				c = UART_ReceiveData(UART1);

				// check for abort condition (ESC ESC 0x55 0xAA 0x00)
				switch (abortState)
				{
					case 0:
				  		if (c == 27) abortState++;
				  		break;
					case 1:
				  		if (c == 27) abortState++;
						else abortState = 0;
				      	break;
					case 2:
				  		if (c == 0x55) abortState++;
						else abortState = 0;
				  		break;
					case 3:
				  		if (c == 0xAA) abortState++;
						else abortState = 0;
						break;
					case 4:
						if (c == 0x00)
						{
					   		if(DebugUART == UART0)
							{
								UART0_Connect_to_MKGPS(UART0_BAUD_RATE);
								TIMER2_Init(); // enbable servo outputs
								fifo_purge(&UART1_rx_fifo); // flush the whole fifo init buffer
							}
							DebugUART = UART1;
						}
						abortState = 0;
						break;
				} // end switch abort state
				// if the Debug uart is not UART1, redirect input to the Debug UART
				if (DebugUART != UART1)
				{
					// wait for space in the tx buffer of the DebugUART
					while(UART_GetFlagStatus(DebugUART, UART_FLAG_TxFIFOFull) == SET) {};
					// move byte to the tx fifo of the debug uart
					UART_SendData(DebugUART, c);
				}
			}
		}
		else  // DebugUART == UART1 (normal operation)
		{
			while(UART_GetFlagStatus(UART1, UART_FLAG_RxFIFOEmpty) != SET)
	 		{ // some byes in the hardware fifo
	  		    // get byte from hardware fifo
	     		c = UART_ReceiveData(UART1);
				// put into the software fifo
				if(!fifo_put(&UART1_rx_fifo, c))
				{	// fifo overflow
				 	//fifo_purge(&UART1_rx_fifo); // flush the whole buffer
				}
			} // EOF while some byes in the hardware fifo
		} // eof DebugUart = UART1
	}

	IDISABLE;
}

void UART1_ProcessMkProtocol(void)
{
	if(UART1_rx_buffer.Locked == FALSE) return;

	Point_t * pPoint = NULL;
	SerialMsg_t SerialMsg;

	// analyze header first
	MKProtocol_DecodeSerialFrameHeader(&UART1_rx_buffer, &SerialMsg);
	if( SerialMsg.Address == FC_ADDRESS )
	{
		switch(SerialMsg.CmdID)
		{
//			case 'v': // version
			case 'y': // serial poti values
			case 'b': // extern control
				Buffer_Copy(&UART1_rx_buffer, &UART2_tx_buffer); //forward to FC
	 			Buffer_Clear(&UART1_rx_buffer); // free rc buffer for next frame
				return;	//end process rx data
			break;
		}
	}

	MKProtocol_DecodeSerialFrameData(&UART1_rx_buffer, &SerialMsg); // decode serial frame in rxd buffer
    if(SerialMsg.CmdID != 'z') SerialLinkOkay = 250;	  // reset SerialTimeout, but not in case of the "ping"
	switch(SerialMsg.Address) // check for Slave Address
	{
		case NC_ADDRESS:  // own Slave Address
		switch(SerialMsg.CmdID)
		{
			case 'z': // connection checker
				memcpy(&Echo, SerialMsg.pData, sizeof(Echo)); // copy echo pattern
				UART1_Request_Echo = TRUE;
				break;

			case 'e': // request for the text of the error status
				UART1_Request_ErrorMessage = TRUE;
				break;

			case 's'://  new target position
				pPoint = (Point_t*)SerialMsg.pData;
				if(pPoint->Position.Status == NEWDATA)
				{
					//if(!(FC.StatusFlags & FC_STATUS_FLY)) PointList_Clear(); // flush the list	
					//pPoint->Index = 1; // must be one after empty list
					PointList_SetAt(pPoint);
					if(FC.StatusFlags & FC_STATUS_FLY) PointList_WPActive(TRUE); 
					GPS_pWaypoint = PointList_WPBegin(); // updates POI index
					if(GPS_pWaypoint != NULL) // if new WP exist

					{   // update WP hold time stamp immediately!
/*						if(GPS_pWaypoint->Heading > 0 && GPS_pWaypoint->Heading <= 360)

						{

						 CAM_Orientation.Azimuth = GPS_pWaypoint->Heading;
						 CAM_Orientation.UpdateMask |= CAM_UPDATE_AZIMUTH;

						}
*/
					}
					BeepTime = 50;
				}
				break;

			case 'u': // redirect debug uart
				switch(SerialMsg.pData[0])
				{
					case UART_FLIGHTCTRL:
						UART2_Init();				// initialize UART2 to FC pins
						fifo_purge(&UART1_rx_fifo);
						TIMER2_Deinit(); 			// reduce irq load
						DebugUART = UART2;
						break;
					case UART_MK3MAG:
						if(FC.StatusFlags & FC_STATUS_MOTOR_RUN) break; // not if the motors are running
						UART0_Connect_to_MK3MAG(); 	// mux UART0 to MK3MAG pins
						GPSData.Status = INVALID;
						fifo_purge(&UART1_rx_fifo);
						DebugUART = UART0;
						break;
					case UART_MKGPS:
						if(FC.StatusFlags & FC_STATUS_MOTOR_RUN) break; // not if the motors are running
						TIMER2_Deinit();			// disable servo outputs to reduce irq load
						UART0_Connect_to_MKGPS(UART0_BAUD_RATE); 	// connect UART0 to MKGPS pins
						GPSData.Status = INVALID;
						fifo_purge(&UART1_rx_fifo);
						DebugUART = UART0;
						break;
					default:
						break;
				}
				break;

			case 'w'://  Set point in list at index
				{
					pPoint = (Point_t*)SerialMsg.pData;

					if((pPoint->Position.Status == INVALID) && (pPoint->Index == 0))
					{
						PointList_Clear();
						GPS_pWaypoint = PointList_WPBegin();
						UART1_Request_WritePoint = 0; // return new point count	
					}
					else
					{  // update WP in list at index
						UART1_Request_WritePoint = PointList_SetAt(pPoint);
						if(FC.StatusFlags & FC_STATUS_FLY) PointList_WPActive(TRUE); 
						if(UART1_Request_WritePoint == pPoint->Index)
						{
							BeepTime = 500;
						}
					}
				}
				break;

			case 'x'://  Read Waypoint from List
				UART1_Request_ReadPoint = SerialMsg.pData[0];
				break;

			case 'j':// Set/Get NC-Parameter
				switch(SerialMsg.pData[0])
				{
					case 0: // get
					break;

					case 1: // set
					{
						s16 value;
						value = SerialMsg.pData[2] + (s16)SerialMsg.pData[3] * 0x0100;
						NCParams_SetValue(SerialMsg.pData[1], &value);
					}
					break;

					default:
					break;
				}
				UART1_Request_ParameterId = SerialMsg.pData[1];
				UART1_Request_Parameter = TRUE;
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
				UART1_Request_DebugLabel = SerialMsg.pData[0];
				if(UART1_Request_DebugLabel > 31) UART1_Request_DebugLabel = 31;
				break;
			/*
			case 'b': // submit extern control
				memcpy(&ExternControl, SerialMsg.pData, sizeof(ExternControl));

				UART1_ConfirmFrame = ExternControl.Frame;
				break;
			*/
			case 'd': // request for debug data;
				UART1_DebugData_Interval = (u32) SerialMsg.pData[0] * 10;
				if(UART1_DebugData_Interval > 0) UART1_Request_DebugData = TRUE;
				UART1_AboTimeOut = SetDelay(ABO_TIMEOUT);
				break;

			case 'c': // request for 3D data;
				UART1_Data3D_Interval = (u32) SerialMsg.pData[0] * 10;
				if(UART1_Data3D_Interval > 0) UART1_Request_Data3D = TRUE;
				UART1_AboTimeOut = SetDelay(ABO_TIMEOUT);
				break;

			case 'h':// reqest for display line
				if((SerialMsg.pData[0]& 0x80) == 0x00)// old format
				{
				 	UART1_DisplayLine = 2;
					UART1_Display_Interval = 0;
				}
				else
				{
					UART1_DisplayKeys |= ~SerialMsg.pData[0];
					UART1_Display_Interval = (u32) SerialMsg.pData[1] * 10;
					UART1_DisplayLine = 4;
					UART1_AboTimeOut = SetDelay(ABO_TIMEOUT);
				}
				UART1_Request_Display = TRUE;
				break;

			case 'l':// reqest for display columns
				MenuItem = SerialMsg.pData[0];
				UART1_Request_Display1 = TRUE;
				break;

			case 'o': // request for navigation information
				UART1_NaviData_Interval = (u32) SerialMsg.pData[0] * 10;
				if(UART1_NaviData_Interval > 0) UART1_Request_NaviData = TRUE;
				UART1_AboTimeOut = SetDelay(ABO_TIMEOUT);
				break;

			case 'v': // request for version info
				UART1_Request_VersionInfo = TRUE;
				break;
			default:
				// unsupported command recieved
				break;
		}
		break; // default:
	}
	Buffer_Clear(&UART1_rx_buffer); // free rc buffer for next frame
}
mavlink_message_t msg_tx; 
mavlink_status_t status; 
void UART1_ProcessMavlink(void)
{Point_t point;
	mavlink_waypoint_t wp;
	DebugOut.Analog[23] = 2;
    switch(msg_tx.msgid) 
    { 
	
        case MAVLINK_MSG_ID_WAYPOINT:
            mavlink_msg_waypoint_decode(&msg_tx, &wp);
            point.Position.Status = NEWDATA;
	    point.Index=wp.x;
		if(point.Position.Status == NEWDATA)
				{
					//if(!(FC.StatusFlags & FC_STATUS_FLY)) PointList_Clear(); // flush the list	
					//pPoint->Index = 1; // must be one after empty list
					PointList_SetAt(&point);
					if(FC.StatusFlags & FC_STATUS_FLY) PointList_WPActive(TRUE); 
					GPS_pWaypoint = PointList_WPBegin(); // updates POI index
					if(GPS_pWaypoint != NULL) // if new WP exist

					{   // update WP hold time stamp immediately!
/*						if(GPS_pWaypoint->Heading > 0 && GPS_pWaypoint->Heading <= 360)


						{

						 CAM_Orientation.Azimuth = GPS_pWaypoint->Heading;
						 CAM_Orientation.UpdateMask |= CAM_UPDATE_AZIMUTH;


						}
*/
					}
					BeepTime = 50;
				}
	DebugOut.Analog[24] = (int)wp.x; 
            break;
    }
}
/**************************************************************/
/* Process incomming data from debug uart                     */
/**************************************************************/
void UART1_ProcessRxData(void)
{
	// return on forwarding uart  or unlocked rx buffer
	if(DebugUART != UART1) return;

	u8 c;
	// if rx buffer is not locked
	if(UART1_rx_buffer.Locked == FALSE)
	{ //DebugOut.Analog[24]++;
		//collect data from primary rx fifo
		while(fifo_get(&UART1_rx_fifo, &c))
		{	if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg_tx, &status))UART1_ProcessMavlink();
			// break if complete frame is collected
			if(MKProtocol_CollectSerialFrame(&UART1_rx_buffer, c)) break;
		}
	}
   	UART1_ProcessMkProtocol();
}


/*****************************************************/
/*                   Send a character                */
/*****************************************************/
s16 UART1_Putchar(char c)
{
	if (c == '\n') UART1_Putchar('\r');
	// wait until txd fifo is not full
	while (UART_GetFlagStatus(UART1, UART_FLAG_TxFIFOFull) != RESET);
	// transmit byte
	UART_SendData(UART1, c);
	return (0);
}

/*****************************************************/
/*       Send a string to the debug uart              */
/*****************************************************/
void UART1_PutString(u8 *s)
{
	if(s == NULL) return;
	while (*s != '\0' && DebugUART == UART1)
	{
		UART1_Putchar(*s);
		s ++;
	}
}


/**************************************************************/
/*         Transmit tx buffer via debug uart                  */
/**************************************************************/
void UART1_Transmit(void)
{
	u8 tmp_tx;
	if(DebugUART != UART1) return;
	// if something has to be send and the txd fifo is not full
	if(UART1_tx_buffer.Locked == TRUE)
	{
		// while there is some space in the tx fifo
		while(UART_GetFlagStatus(UART1, UART_FLAG_TxFIFOFull) != SET)
		{
			tmp_tx = UART1_tx_buffer.pData[UART1_tx_buffer.Position++]; // read next byte from txd buffer
			UART_SendData(UART1, tmp_tx); // put character to txd fifo
			// if terminating character or end of txd buffer reached
			if((tmp_tx == '\r') || (UART1_tx_buffer.Position == UART1_tx_buffer.DataBytes))
			{
				Buffer_Clear(&UART1_tx_buffer); // clear txd buffer
				break; // end while loop
			}
		}
	}
}

/**************************************************************/
/* Send the answers to incomming commands at the debug uart   */
/**************************************************************/
void UART1_TransmitTxData(void)
{
	if(DebugUART != UART1) return;

	if(CheckDelay(UART1_AboTimeOut))
	{
		UART1_DebugData_Interval = 0;
		UART1_NaviData_Interval = 0;
		UART1_Data3D_Interval = 0;
		UART1_Display_Interval = 0;
	}

	UART1_Transmit(); // output pending bytes in tx buffer
	if((UART1_tx_buffer.Locked == TRUE)) return;

	if(UART1_Request_Parameter && (UART1_tx_buffer.Locked == FALSE))
	{
		s16 ParamValue;	 
		NCParams_GetValue(UART1_Request_ParameterId, &ParamValue);
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'J', NC_ADDRESS, 2, &UART1_Request_ParameterId, sizeof(UART1_Request_ParameterId), &ParamValue, sizeof(ParamValue)); // answer the param request
		UART1_Request_Parameter = FALSE;
	}
	else if(UART1_Request_Echo && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'Z', NC_ADDRESS, 1, &Echo, sizeof(Echo)); // answer the echo request
		Echo = 0; // reset echo value
		UART1_Request_Echo = FALSE;
	}
	else if((UART1_Request_WritePoint!= 0xFF) && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'W', NC_ADDRESS, 1, &UART1_Request_WritePoint, sizeof(UART1_Request_WritePoint));
		UART1_Request_WritePoint = 0xFF;
	}
	else if((UART1_Request_ReadPoint) && (UART1_tx_buffer.Locked == FALSE))
	{
		u8 PointCount = PointList_GetCount();
		if (UART1_Request_ReadPoint <= PointCount)
		{
			MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'X', NC_ADDRESS, 3, &PointCount, 1, &UART1_Request_ReadPoint, 1, PointList_GetAt(UART1_Request_ReadPoint), sizeof(Point_t));
		}
		else
		{
			MKProtocol_CreateSerialFrame(&UART1_tx_buffer,'X', NC_ADDRESS, 1, &PointCount, sizeof(PointCount));
		}
	 	UART1_Request_ReadPoint = 0;
	}
	else if((UART1_Request_DebugLabel != 0xFF) && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'A', NC_ADDRESS, 2, &UART1_Request_DebugLabel, sizeof(UART1_Request_DebugLabel), (u8 *) ANALOG_LABEL[UART1_Request_DebugLabel], 16);
		UART1_Request_DebugLabel = 0xFF;
	}
	else if(( ((UART1_NaviData_Interval > 0) && CheckDelay(UART1_NaviData_Timer) ) || UART1_Request_NaviData) && (UART1_tx_buffer.Locked == FALSE))
	{
		NaviData.Errorcode = ErrorCode;
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'O', NC_ADDRESS,1, (u8 *)&NaviData, sizeof(NaviData));
		UART1_NaviData_Timer = SetDelay(UART1_NaviData_Interval);
		UART1_Request_NaviData = FALSE;
	}
	else if( (( (UART1_DebugData_Interval > 0) && CheckDelay(UART1_DebugData_Timer)) || UART1_Request_DebugData) && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'D', NC_ADDRESS, 1,(u8 *)&DebugOut, sizeof(DebugOut));
		UART1_DebugData_Timer = SetDelay(UART1_DebugData_Interval);
		UART1_Request_DebugData = FALSE;
	}
	else if((( (UART1_Data3D_Interval > 0) && CheckDelay(UART1_Data3D_Timer) ) || UART1_Request_Data3D) && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'C', NC_ADDRESS, 1,(u8 *)&Data3D, sizeof(Data3D));
		UART1_Data3D_Timer = SetDelay(UART1_Data3D_Interval);
		UART1_Request_Data3D = FALSE;
	}
	/*
	else if(UART1_ConfirmFrame && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'B', NC_ADDRESS, 1, &UART1_ConfirmFrame, sizeof(UART1_ConfirmFrame));
		UART1_ConfirmFrame = 0;
	}
	*/
	/*
	else if(UART1_Request_ExternalControl && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'G', NC_ADDRESS, 1, (u8 *)&ExternControl, sizeof(ExternControl));
		UART1_Request_ExternalControl = FALSE;
	}
	*/
	else if( (( (UART1_Display_Interval > 0) && CheckDelay(UART1_Display_Timer)) || UART1_Request_Display) && (UART1_tx_buffer.Locked == FALSE))
	{
		if(UART1_DisplayLine > 3)
		{
			Menu_Update(UART1_DisplayKeys);
			UART1_DisplayKeys = 0;
			MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'H', NC_ADDRESS, 1, (u8*)DisplayBuff, sizeof(DisplayBuff));
		}
		else
		{
			UART1_DisplayLine = 2;
			sprintf(text,"!!! incompatible !!!");
			MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'H', NC_ADDRESS, 2, &UART1_DisplayLine, sizeof(UART1_DisplayLine), (u8*)&text, 20);
			if(UART1_DisplayLine++ > 3) UART1_DisplayLine = 0;
		}
		UART1_Display_Timer = SetDelay(UART1_Display_Interval);
		UART1_Request_Display = FALSE;
	}
	else if(UART1_Request_Display1 && (UART1_tx_buffer.Locked == FALSE))
	{
		Menu_Update(0);
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'L', NC_ADDRESS, 3, (u8*)&MenuItem, sizeof(MenuItem), (u8*)&MaxMenuItem, sizeof(MaxMenuItem),(u8*)DisplayBuff, sizeof(DisplayBuff));
		UART1_Request_Display1 = FALSE;
	}
	else if(UART1_Request_VersionInfo && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'V', NC_ADDRESS,1, (u8 *)&UART_VersionInfo, sizeof(UART_VersionInfo));
		UART1_Request_VersionInfo = FALSE;
	}
	else if(UART1_Request_ErrorMessage && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer, 'E', NC_ADDRESS, 1, (u8 *)&ErrorMSG, sizeof(ErrorMSG));
		UART1_Request_ErrorMessage = FALSE;
	}
#ifdef DEBUG															// only include functions if DEBUG is defined
	if(SendDebugOutput && (UART1_tx_buffer.Locked == FALSE))
	{
		MKProtocol_CreateSerialFrame(&UART1_tx_buffer,'0', NC_ADDRESS, 1, (u8 *) &tDebug, sizeof(tDebug));
		SendDebugOutput = 0;
	}
#endif	
	UART1_Transmit(); // output pending bytes in tx buffer
}

