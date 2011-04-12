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
#include "91x_lib.h"
#include "led.h"
#include "gps.h"
#include "uart1.h"
#include "spi_slave.h"
#include "compass.h"
#include "timer1.h"
#include "timer2.h"
#include "config.h"
#include "main.h"
#include "compass.h"
#include "params.h"

#define SPI_RXSYNCBYTE1 0xAA
#define SPI_RXSYNCBYTE2 0x83
#define SPI_TXSYNCBYTE1 0x81
#define SPI_TXSYNCBYTE2 0x55

//communication packets
FromFlightCtrl_t   FromFlightCtrl;
ToFlightCtrl_t     ToFlightCtrl;
#define SPI0_TIMEOUT 500 // 500ms
volatile u32 SPI0_Timeout = 0;

// tx packet buffer
#define SPI_TXBUFFER_LEN (2 + sizeof(ToFlightCtrl)) // 2 bytes at start are for synchronization
volatile u8 SPI_TxBuffer[SPI_TXBUFFER_LEN];
volatile u8 SPI_TxBufferIndex = 0;
u8 *Ptr_TxChksum = NULL ;  // pointer to checksum in TxBuffer

// rx packet buffer
#define SPI_RXBUFFER_LEN sizeof(FromFlightCtrl)
volatile u8 SPI_RxBuffer[SPI_RXBUFFER_LEN];
volatile u8 SPI_RxBufferIndex = 0;
volatile u8 SPI_RxBuffer_Request = 0;
#define SPI_COMMAND_INDEX 0

s32 Kalman_K = 32;
s32 Kalman_MaxDrift = 5 * 16;
s32 Kalman_MaxFusion = 64;
s32 ToFcGpsZ = 0;

u8 SPI_CommandSequence[] = { SPI_NCCMD_VERSION, SPI_NCCMD_KALMAN, SPI_NCCMD_GPSINFO ,SPI_NCCMD_KALMAN, SPI_NCCMD_KALMAN};
u8 SPI_CommandCounter = 0;
s32 ToFC_Rotate_C = 64, ToFC_Rotate_S = 0;
s32 HeadFreeStartAngle = 0;
s16 FC_WP_EventChannel = 0; // gibt einen Schaltkanal an die FC weiter, wenn der Wegpunkt erreicht wurde
u32 ToFC_AltitudeRate = 0;
s32 ToFC_AltitudeSetpoint = 0;
u8  FromFC_VarioCharacter = ' ';

SPI_Version_t FC_Version;

//--------------------------------------------------------------
void SSP0_IRQHandler(void)
{
	static u8 rxchksum = 0;
	u8 rxdata;

	#define SPI_SYNC1	0
	#define SPI_SYNC2	1
	#define SPI_DATA	2
	static u8 SPI_State = SPI_SYNC1;

	IENABLE;

	// clear pending bits
	SSP_ClearITPendingBit(SSP0, SSP_IT_RxTimeOut);
	SSP_ClearITPendingBit(SSP0, SSP_IT_RxFifo);

	// while RxFIFO not empty
	while (SSP_GetFlagStatus(SSP0, SSP_FLAG_RxFifoNotEmpty) == SET)
	{
		rxdata =  SSP0->DR; // catch the received byte
		// Fill TxFIFO while its not full or end of packet is reached
		while (SSP_GetFlagStatus(SSP0, SSP_FLAG_TxFifoNotFull) == SET)
		{
			if (SPI_TxBufferIndex  < SPI_TXBUFFER_LEN)   // still data to send ?
			{
				SSP0->DR = SPI_TxBuffer[SPI_TxBufferIndex];	  // send a byte
				*Ptr_TxChksum += SPI_TxBuffer[SPI_TxBufferIndex]; // update checksum
				SPI_TxBufferIndex++; // pointer to next byte
			}
			else // end of packet is reached reset and copy data to tx buffer
			{
				SPI_TxBufferIndex = 0; 	// reset buffer index
				ToFlightCtrl.Chksum = 0;  // initialize checksum
				ToFlightCtrl.BeepTime = BeepTime;  // set beeptime
				BeepTime = 0; // reset local beeptime
				// copy contents of ToFlightCtrl->SPI_TxBuffer
				memcpy((u8 *) &(SPI_TxBuffer[2]), (u8 *) &ToFlightCtrl, sizeof(ToFlightCtrl));
			}
		}
		switch (SPI_State)
		{
			case SPI_SYNC1:
				SPI_RxBufferIndex = 0; // reset buffer index
				rxchksum = rxdata;     // init checksum
				if (rxdata == SPI_RXSYNCBYTE1)
				{   // 1st syncbyte ok
					SPI_State = SPI_SYNC2;  // step to sync2
				}
				break;
			case SPI_SYNC2:
				if (rxdata == SPI_RXSYNCBYTE2)
				{  // 2nd Syncbyte ok
					rxchksum += rxdata;
					SPI_State = SPI_DATA;
				}  // 2nd Syncbyte does not match
				else
				{
					SPI_State  = SPI_SYNC1; //jump back to sync1
				}
				break;
			case SPI_DATA:
				SPI_RxBuffer[SPI_RxBufferIndex++]= rxdata; // copy databyte to rx buffer
				if (SPI_RxBufferIndex >= SPI_RXBUFFER_LEN) // end of packet is reached
				{
					if (rxdata == rxchksum) // verify checksum byte
					{
						// copy SPI_RxBuffer -> FromFlightCtrl
						if(!SPI_RxBuffer_Request) // block writing to FromFlightCtrl on reading access
						{
							memcpy((u8 *) &FromFlightCtrl, (u8 *) SPI_RxBuffer, sizeof(FromFlightCtrl));
							SPI_RxBuffer_Request = 1;
						}
						// reset timeout counter on good packet
						SPI0_Timeout = SetDelay(SPI0_TIMEOUT);
						DebugOut.Analog[13]++;
					}
					else // bad checksum byte
					{
						DebugOut.Analog[12]++; // increase SPI chksum error counter
					}
					SPI_State  = SPI_SYNC1; // reset state
				}
				else // end of packet not reached
				{
					rxchksum += rxdata;	 // update checksum
				}
				break;
			default:
				SPI_State  = SPI_SYNC1;
				break;
		}
	}

	IDISABLE;
}

//--------------------------------------------------------------
void SPI0_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SSP_InitTypeDef   SSP_InitStructure;

	UART1_PutString("\r\n SPI init...");

	SCU_APBPeriphClockConfig(__GPIO2 ,ENABLE);
	SCU_APBPeriphClockConfig(__SSP0 ,ENABLE);

	GPIO_DeInit(GPIO2);
	//SSP0_CLK, SSP0_MOSI, SSP0_NSS pins
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinInput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_InputAlt1; //SSP0_SCLK, SSP0_MOSI, SSP0_NSS
	GPIO_Init (GPIO2, &GPIO_InitStructure);

	// SSP0_MISO pin GPIO2.6
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt2; //SSP0_MISO
	GPIO_Init (GPIO2, &GPIO_InitStructure);

	SSP_DeInit(SSP0);
	SSP_StructInit(&SSP_InitStructure);
	SSP_InitStructure.SSP_FrameFormat = SSP_FrameFormat_Motorola;
	SSP_InitStructure.SSP_Mode = SSP_Mode_Slave;
	SSP_InitStructure.SSP_SlaveOutput = SSP_SlaveOutput_Enable;
	SSP_InitStructure.SSP_CPHA = SSP_CPHA_1Edge;
	SSP_InitStructure.SSP_CPOL = SSP_CPOL_Low;
	SSP_InitStructure.SSP_ClockRate = 0;

	SSP_Init(SSP0, &SSP_InitStructure);
	SSP_ITConfig(SSP0, SSP_IT_RxFifo | SSP_IT_RxTimeOut, ENABLE);

	SSP_Cmd(SSP0, ENABLE);
	// initialize the syncbytes in the tx buffer
	SPI_TxBuffer[0] = SPI_TXSYNCBYTE1;
	SPI_TxBuffer[1] = SPI_TXSYNCBYTE2;
	// set the pointer to the checksum byte in the tx buffer
	Ptr_TxChksum = (u8 *) &(((ToFlightCtrl_t *) &(SPI_TxBuffer[2]))->Chksum);

	ToFlightCtrl.GPSStick.Nick = 0;
	ToFlightCtrl.GPSStick.Roll = 0;
	ToFlightCtrl.GPSStick.Yaw = 0;

	VIC_Config(SSP0_ITLine, VIC_IRQ, PRIORITY_SPI0);
	VIC_ITCmd(SSP0_ITLine, ENABLE);

	SPI0_Timeout = SetDelay(4*SPI0_TIMEOUT);

	UART1_PutString("ok");
}


//------------------------------------------------------
void SPI0_UpdateBuffer(void)
{
	static u32 timeout = 0;
	static u8 counter = 50;
	static u8 CompassCalState = 0;
	static u8 FCCalibActive = 0;
	s16 tmp;

	if (SPI_RxBuffer_Request)
	{
		// avoid sending data via SPI during the update of the  ToFlightCtrl structure
		VIC_ITCmd(SSP0_ITLine, DISABLE); // disable SPI interrupt
		ToFlightCtrl.CompassHeading = Compass_Heading;
		DebugOut.Analog[10] = ToFlightCtrl.CompassHeading;
		if(ToFlightCtrl.CompassHeading >= 0) ToFlightCtrl.CompassHeading = (360 + ToFlightCtrl.CompassHeading + FromFlightCtrl.GyroYaw / 12) % 360;
		ToFlightCtrl.MagVecX = MagVector.X;
		ToFlightCtrl.MagVecY = MagVector.Y;
		ToFlightCtrl.MagVecZ = MagVector.Z;
		ToFlightCtrl.NCStatus = 0;
		// cycle spi commands
		ToFlightCtrl.Command = SPI_CommandSequence[SPI_CommandCounter++];
		// restart command cycle at the end
		if (SPI_CommandCounter >= sizeof(SPI_CommandSequence)) SPI_CommandCounter = 0;

		switch (ToFlightCtrl.Command)
		{
			case  SPI_NCCMD_KALMAN:
				CalcHeadFree();
				ToFlightCtrl.Param.sByte[0] = (s8) Kalman_K;
				ToFlightCtrl.Param.sByte[1] = (s8) Kalman_MaxFusion;
				ToFlightCtrl.Param.sByte[2] = (s8) Kalman_MaxDrift;
				ToFlightCtrl.Param.Byte[3]	= (u8) SerialLinkOkay;
				ToFlightCtrl.Param.sByte[4] = (s8) ToFcGpsZ;
				ToFlightCtrl.Param.Byte[5] = (s8) ToFC_Rotate_C;
				ToFlightCtrl.Param.Byte[6] = (s8) ToFC_Rotate_S;
                		
				//ToFlightCtrl.Param.Byte[7] = 
				if(CAM_Orientation.UpdateMask & CAM_UPDATE_AZIMUTH)
				{
					ToFlightCtrl.Param.sInt[4] = CAM_Orientation.Azimuth;
					CAM_Orientation.UpdateMask &= ~CAM_UPDATE_AZIMUTH;
				}
				else
				{
					ToFlightCtrl.Param.sInt[4] = -1;
				}

				if(NCRARAM_STATE_VALID == NCParams_GetValue(NCPARAMS_NEW_CAMERA_ELEVATION, &tmp))  // Elevation set via 'j' command
				{
					ToFlightCtrl.Param.sInt[5] = tmp;
				}
				else
				{
					if(FC.StatusFlags2 & FC_STATUS2_CAREFREE) ToFlightCtrl.Param.sInt[5] = CAM_Orientation.Elevation; // only, if carefree is active
					else ToFlightCtrl.Param.sInt[5] = 0;
				}
				break;

			case SPI_NCCMD_VERSION:
				ToFlightCtrl.Param.Byte[0] = VERSION_MAJOR;
				ToFlightCtrl.Param.Byte[1] = VERSION_MINOR;
				ToFlightCtrl.Param.Byte[2] = VERSION_PATCH;
				ToFlightCtrl.Param.Byte[3] = FC_SPI_COMPATIBLE;
				ToFlightCtrl.Param.Byte[4] = Version_HW;
				ToFlightCtrl.Param.Byte[5] = DebugOut.Status[0];
				ToFlightCtrl.Param.Byte[6] = DebugOut.Status[1];
				ToFlightCtrl.Param.Byte[7] = ErrorCode;
				break;

			case SPI_NCCMD_GPSINFO:
				ToFlightCtrl.Param.Byte[0] = GPSData.Flags;
				ToFlightCtrl.Param.Byte[1] = GPSData.NumOfSats;
				ToFlightCtrl.Param.Byte[2] = GPSData.SatFix;
				ToFlightCtrl.Param.Byte[3] = GPSData.Speed_Ground / 100; // m/s
				ToFlightCtrl.Param.Int[2]  = NaviData.HomePositionDeviation.Distance; // dm   //4&5
				ToFlightCtrl.Param.sInt[3] = NaviData.HomePositionDeviation.Bearing;  // deg  //6&7
				ToFlightCtrl.Param.Byte[8] = (s8)(FC_WP_EventChannel - 110);

				if(NCRARAM_STATE_VALID == NCParams_GetValue(NCPARAMS_ALTITUDE_RATE, &tmp))
				{
					ToFlightCtrl.Param.Byte[9] = (u8)tmp;
				}
				else
				{
					ToFlightCtrl.Param.Byte[9] = (u8)ToFC_AltitudeRate;
				}
				if(NCRARAM_STATE_VALID == NCParams_GetValue(NCPARAMS_ALTITUDE_SETPOINT, &tmp))
				{
					ToFlightCtrl.Param.sInt[5] = tmp;
				}
				else
				{
					ToFlightCtrl.Param.sInt[5] = (s16)ToFC_AltitudeSetpoint;
				}
//				DebugOut.Analog[25] = (s16)ToFlightCtrl.Param.Byte[9];
//				DebugOut.Analog[20] = ToFlightCtrl.Param.sInt[5];
				break;

			default:
				break;
// 0 = 0,1
// 1 = 2,3
// 2 = 4,5
// 3 = 6,7
// 4 = 8,9
// 5 = 10,11
		}
		VIC_ITCmd(SSP0_ITLine, ENABLE);  	// enable SPI interrupt


		switch(FromFlightCtrl.Command)
		{
			case SPI_FCCMD_USER:
				Parameter.User1 = FromFlightCtrl.Param.Byte[0];
				Parameter.User2 = FromFlightCtrl.Param.Byte[1];
				Parameter.User3 = FromFlightCtrl.Param.Byte[2];
				Parameter.User4 = FromFlightCtrl.Param.Byte[3];
				Parameter.User5 = FromFlightCtrl.Param.Byte[4];
				Parameter.User6 = FromFlightCtrl.Param.Byte[5];
				Parameter.User7 = FromFlightCtrl.Param.Byte[6];
				Parameter.User8 = FromFlightCtrl.Param.Byte[7];
				if(ClearFCStatusFlags)
				{
					FC.StatusFlags = 0;
					ClearFCStatusFlags = 0;
				}
				FC.StatusFlags |= FromFlightCtrl.Param.Byte[8];
				if(FC.StatusFlags&FC_STATUS_CALIBRATE && !FCCalibActive)
				{
				 	Compass_Init();
					FCCalibActive = 1;
				}
				else
				{
				 	FCCalibActive = 0;
				}
				Parameter.ActiveSetting = FromFlightCtrl.Param.Byte[9];
				DebugOut.Analog[5] = FC.StatusFlags;
				FC.StatusFlags2 = FromFlightCtrl.Param.Byte[11];
				NaviData.FCStatusFlags = FC.StatusFlags;
				NaviData.FCStatusFlags2 = FC.StatusFlags2;
				HeadFreeStartAngle = (s32) FromFlightCtrl.Param.Byte[10] * 20; // convert to 0.1°

				break;

		  	case SPI_FCCMD_ACCU:
				FC.BAT_Current = FromFlightCtrl.Param.Int[0];
				FC.BAT_UsedCapacity = FromFlightCtrl.Param.Int[1];
				FC.BAT_Voltage = FromFlightCtrl.Param.Byte[4];
				Parameter.LowVoltageWarning = FromFlightCtrl.Param.Byte[5];
				FromFC_VarioCharacter = FromFlightCtrl.Param.Byte[6];
				if(FromFC_VarioCharacter == '+' || FromFC_VarioCharacter == '-') // manual setpoint clears the NC-Parameter command
				 {
				  NCParams_ClearValue(NCPARAMS_ALTITUDE_RATE);
				 }

				NaviData.UBat = FC.BAT_Voltage;
				NaviData.Current = FC.BAT_Current;
				NaviData.UsedCapacity = FC.BAT_UsedCapacity;
				break;

#define CHK_POTI(b,a) { if(a < 248) b = a; else b = FC.Poti[255 - a]; }
#define CHK_POTI_MM(b,a,min,max) {CHK_POTI(b,a); LIMIT_MIN_MAX(b, min, max); }

			case SPI_FCCMD_PARAMETER1:
				CHK_POTI_MM(Parameter.NaviGpsModeControl,FromFlightCtrl.Param.Byte[0],0,255);
				CHK_POTI_MM(Parameter.NaviGpsGain,FromFlightCtrl.Param.Byte[1],0,255);
				CHK_POTI_MM(Parameter.NaviGpsP,FromFlightCtrl.Param.Byte[2],0,255);
				CHK_POTI_MM(Parameter.NaviGpsI,FromFlightCtrl.Param.Byte[3],0,255);
				CHK_POTI_MM(Parameter.NaviGpsD,FromFlightCtrl.Param.Byte[4],0,255);
				CHK_POTI_MM(Parameter.NaviGpsACC,FromFlightCtrl.Param.Byte[5],0,255);
				Parameter.NaviGpsMinSat = FromFlightCtrl.Param.Byte[6];
				Parameter.NaviStickThreshold = FromFlightCtrl.Param.Byte[7];
				CHK_POTI_MM(Parameter.NaviOperatingRadius,FromFlightCtrl.Param.Byte[8],0,255);
				CHK_POTI_MM(Parameter.NaviWindCorrection,FromFlightCtrl.Param.Byte[9],0,255);
				CHK_POTI_MM(Parameter.NaviSpeedCompensation,FromFlightCtrl.Param.Byte[10],0,255);
				CHK_POTI_MM(Parameter.NaviAngleLimitation,FromFlightCtrl.Param.Byte[11],0,255);
				break;

			case SPI_FCCMD_STICK:
				FC.StickGas  	= FromFlightCtrl.Param.sByte[0];
				FC.StickYaw 	= FromFlightCtrl.Param.sByte[1];
				FC.StickRoll	= FromFlightCtrl.Param.sByte[2];
				FC.StickNick	= FromFlightCtrl.Param.sByte[3];
				FC.Poti[0]		= FromFlightCtrl.Param.Byte[4];
				FC.Poti[1] 	 	= FromFlightCtrl.Param.Byte[5];
				FC.Poti[2]	 	= FromFlightCtrl.Param.Byte[6];
				FC.Poti[3]	 	= FromFlightCtrl.Param.Byte[7];
				FC.Poti[4]		= FromFlightCtrl.Param.Byte[8];
				FC.Poti[5]	 	= FromFlightCtrl.Param.Byte[9];
				FC.Poti[6]	 	= FromFlightCtrl.Param.Byte[10];
				FC.Poti[7]	 	= FromFlightCtrl.Param.Byte[11];
				break;

			case SPI_FCCMD_MISC:
				if(CompassCalState != FromFlightCtrl.Param.Byte[0])
				{	// put only new CompassCalState into queue to send via I2C
					CompassCalState = FromFlightCtrl.Param.Byte[0];
					Compass_SetCalState(CompassCalState);
				}
				Parameter.NaviPH_LoginTime = FromFlightCtrl.Param.Byte[1];
				NaviData.Variometer = (NaviData.Variometer + 2 * (FromFlightCtrl.Param.sInt[1] - NaviData.Altimeter)) / 2; // provisorisch
				NaviData.Altimeter = FromFlightCtrl.Param.sInt[1]; // in 5cm
				NaviData.SetpointAltitude = FromFlightCtrl.Param.sInt[2]; // in 5cm
				CHK_POTI_MM(Parameter.NaviGpsPLimit,FromFlightCtrl.Param.Byte[6],0,255);
				CHK_POTI_MM(Parameter.NaviGpsILimit,FromFlightCtrl.Param.Byte[7],0,255);
				CHK_POTI_MM(Parameter.NaviGpsDLimit,FromFlightCtrl.Param.Byte[8],0,255);
				FC.RC_Quality	= FromFlightCtrl.Param.Byte[9];
				FC.RC_RSSI		= FromFlightCtrl.Param.Byte[10];
				if(!FC.RC_RSSI) NaviData.RC_Quality = FC.RC_Quality; else NaviData.RC_Quality = FC.RC_RSSI;
//				NaviData.RC_RSSI = FC.RC_RSSI;
				NaviData.Gas	= (FC.BAT_Voltage * (u32) FromFlightCtrl.Param.Byte[11]) / (u32) Parameter.LowVoltageWarning;
				break;

			case SPI_FCCMD_SERVOS:
				ServoParams.Refresh		= FromFlightCtrl.Param.Byte[0];
				ServoParams.CompInvert  = FromFlightCtrl.Param.Byte[1];
				ServoParams.NickControl	= FromFlightCtrl.Param.Byte[2];
				ServoParams.NickComp	= FromFlightCtrl.Param.Byte[3];
				ServoParams.NickMin		= FromFlightCtrl.Param.Byte[4];
				ServoParams.NickMax		= FromFlightCtrl.Param.Byte[5];
				ServoParams.RollControl	= FromFlightCtrl.Param.Byte[6];
				ServoParams.RollComp	= FromFlightCtrl.Param.Byte[7];
				ServoParams.RollMin		= FromFlightCtrl.Param.Byte[8];
				ServoParams.RollMax		= FromFlightCtrl.Param.Byte[9];
				break;

			case SPI_FCCMD_VERSION:
				FC_Version.Major		= FromFlightCtrl.Param.Byte[0];
				FC_Version.Minor		= FromFlightCtrl.Param.Byte[1];
				FC_Version.Patch		= FromFlightCtrl.Param.Byte[2];
				FC_Version.Compatible	= FromFlightCtrl.Param.Byte[3];
				FC_Version.Hardware		= FromFlightCtrl.Param.Byte[4];
				FC.Error[0] 			= FromFlightCtrl.Param.Byte[5];
				FC.Error[1] 			= FromFlightCtrl.Param.Byte[6];
				FC.Error[2] 			= FromFlightCtrl.Param.Byte[7];
				FC.Error[3] 			= FromFlightCtrl.Param.Byte[8];
				FC.Error[4] 			= FromFlightCtrl.Param.Byte[9];
				DebugOut.Status[0] |= 0x01; // status of FC Present
				DebugOut.Status[0] |= 0x02; // status of BL Present
				if(FC.Error[0] || FC.Error[1] || FC.Error[2] || FC.Error[3] || FC.Error[4]) DebugOut.Status[1] |= 0x01;
				else DebugOut.Status[1] &= ~0x01;
				break;
			default:
				break;
		}

		// every time we got new data from the FC via SPI call the navigation routine
		// and update GPSStick that are returned to FC
		GPS_Navigation(&GPSData, &(ToFlightCtrl.GPSStick));
		ClearFCStatusFlags = 1;

		if(counter)
		{
			counter--;					 // count down to enable servo
			if(!counter) TIMER2_Init();  // enable Servo Output
		}

		SPI_RxBuffer_Request = 0;
		timeout = SetDelay(80); // 80 ms, new data are send every 20 ms

		DebugOut.Analog[0] = FromFlightCtrl.AngleNick;
		DebugOut.Analog[1] = FromFlightCtrl.AngleRoll;
		DebugOut.Analog[2] = FromFlightCtrl.AccNick;
		DebugOut.Analog[3] = FromFlightCtrl.AccRoll;
		DebugOut.Analog[11] = FromFlightCtrl.GyroHeading/10;// in deg
		Data3D.AngleNick = FromFlightCtrl.AngleNick;		// in 0.1 deg
		Data3D.AngleRoll = FromFlightCtrl.AngleRoll;		// in 0.1 deg
		Data3D.Heading	 = FromFlightCtrl.GyroHeading;		// in 0.1 deg
	}	// EOF if(SPI_RxBuffer_Request)
	else // no new SPI data
	{
		if(CheckDelay(timeout) && (counter == 0))
		{
			TIMER2_Deinit();  // disable Servo Output
			counter = 50;     // reset counter for enabling Servo Output
		}
	}
}

//------------------------------------------------------
void SPI0_GetFlightCtrlVersion(void)
{
	u32 timeout;
	u8 repeat;
	u8 msg[64];

	UART1_PutString("\r\n Looking for FlightControl");
	FC_Version.Major = 0xFF;
	FC_Version.Minor = 0xFF;
	FC_Version.Patch = 0xFF;
	FC_Version.Compatible = 0xFF;

	// polling FC version info
	repeat = 0;
	do
	{
		timeout = SetDelay(250);
		do
		{
			SPI0_UpdateBuffer();
			if (FC_Version.Major != 0xFF)  break;
		}while (!CheckDelay(timeout));
		UART1_PutString(".");
		repeat++;
	}while((FC_Version.Major == 0xFF) && (repeat < 40)); // 40*250ms = 10s
	// if we got it
	if (FC_Version.Major != 0xFF)
	{
		sprintf(msg, " FC V%d.%d%c HW:%d.%d", FC_Version.Major, FC_Version.Minor, 'a'+FC_Version.Patch, FC_Version.Hardware/10,FC_Version.Hardware%10);
		UART1_PutString(msg);
	}
	else UART1_PutString("\n\r not found!");
}


