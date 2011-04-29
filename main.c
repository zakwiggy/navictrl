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
// + Teile, die nicht eindeutig mit unserer Lizenzvereinbarung gekennzeichnet sind, unterliegen u.U. eigenen Vereinbarungen:
// + z.B. Die Funktion printf_P() unterliegt ihrer eigenen Lizenz
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
//#define MCLK96MHZ
const unsigned long _Main_Crystal = 25000;
#include <stdio.h>
#include "91x_lib.h"
#include "led.h"
#include "uart0.h"
#include "uart1.h"
#include "uart2.h"
#include "gps.h"
#include "i2c.h"
#include "compass.h"
#include "timer1.h"
#include "timer2.h"
#include "analog.h"
#include "spi_slave.h"
#include "fat16.h"
#include "usb.h"
#include "sdc.h"
#include "logging.h"
#include "params.h"
#include "settings.h"
#include "config.h"
#include "main.h"
#include "debug.h"

#include "eeprom.h"

u32 TimerCheckError;
u8 ErrorCode = 0;
u16 BeepTime;
u8  NCFlags = 0;
s16 GeoMagDec = 0; // local magnetic declination in 0.1 deg

u8 ClearFCStatusFlags = 0;
u8 StopNavigation = 0;
Param_t Parameter;
volatile FC_t FC;

s8 ErrorMSG[25];

//----------------------------------------------------------------------------------------------------
void SCU_Config(void)
{
	/* configure PLL and set it as master clock source */
	SCU_MCLKSourceConfig(SCU_MCLK_OSC);		// set master clock source to external oscillator clock (25MHz) before diabling the PLL
	SCU_PLLCmd(DISABLE);					// now disable the PLL
	#ifdef MCLK96MHZ
	SCU_BRCLKDivisorConfig(SCU_BRCLK_Div2); // set BRCLK to MCLK/2 = 48MHz
	SCU_PCLKDivisorConfig(SCU_PCLK_Div4); 	// set PCLK	(APB bus clock) divisor to 4 (half Reference Clock)
	SCU_RCLKDivisorConfig(SCU_RCLK_Div2);	// set RCLK	(Reference Clock) divisor to 1 (full PPL clock)
	SCU_HCLKDivisorConfig(SCU_HCLK_Div2);	// set HCLK	(AHB bus clock) divisor to 1 (full Reference Clock)
	SCU_PLLFactorsConfig(192,25,2); 		// PLL = 96 MHz, Feedback Divider N=192, Pre-Divider M=25, Post-Divider P=2
	#else
	SCU_BRCLKDivisorConfig(SCU_BRCLK_Div1); // set BRCLK to MCLK = 48MHz
	SCU_PCLKDivisorConfig(SCU_PCLK_Div2); 	// set PCLK	(APB bus clock) divisor to 2 (half Reference Clock)
	SCU_RCLKDivisorConfig(SCU_RCLK_Div1);	// set RCLK	(Reference Clock) divisor to 1 (full PPL clock)
	SCU_HCLKDivisorConfig(SCU_HCLK_Div1);	// set HCLK	(AHB bus clock) divisor to 1 (full Reference Clock)
	SCU_PLLFactorsConfig(192,25,3); 		// PLL = 48 MHz, Feedback Divider N=192, Pre-Divider M=25, Post-Divider P=3
	#endif
	SCU_PLLCmd(ENABLE); 					// Enable PLL (is disabled by SCU_PLLFactorsConfig)
	SCU_MCLKSourceConfig(SCU_MCLK_PLL); 	// set master clock source to PLL
}

//----------------------------------------------------------------------------------------------------
void GetNaviCtrlVersion(void)
{
	u8 msg[25];

	sprintf(msg,"\n\r NaviCtrl HW: V%d.%d SW: V%d.%d%c",  Version_HW/10, Version_HW%10, VERSION_MAJOR,  VERSION_MINOR, 'a'+ VERSION_PATCH);
	UART1_PutString(msg);
}

//----------------------------------------------------------------------------------------------------

void CheckErrors(void)
{
	UART_VersionInfo.HardwareError[0] = 0;

	if(CheckDelay(I2C1_Timeout) || (Compass_Heading < 0)) DebugOut.Status[1] |= 0x08;
	else DebugOut.Status[1] &= ~0x08; // MK3Mag green status

	if((FC.Error[1] & FC_ERROR1_I2C) || (FC.Error[1] & FC_ERROR1_BL_MISSING)) DebugOut.Status[1] |= 0x02;
	else DebugOut.Status[1] &= ~0x02; // BL-Ctrl green status

	if(CheckDelay(SPI0_Timeout))
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"no FC communication ");
		ErrorCode = 3;
		StopNavigation = 1;
		DebugOut.Status[0] &= ~0x01; // status of FC Present
		DebugOut.Status[0] &= ~0x02; // status of BL Present
		UART_VersionInfo.HardwareError[0] |= NC_ERROR0_SPI_RX;
	}
	else if(CheckDelay(I2C1_Timeout))
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"no compass communication ");
		//Reset I2CBus
		I2C1_Deinit();
		I2C1_Init();
		ErrorCode = 4;
		StopNavigation = 1;
		UART_VersionInfo.HardwareError[0] |= NC_ERROR0_COMPASS_RX;
		DebugOut.Status[1] |= 0x08;
	}
	else if(FC_Version.Compatible != FC_SPI_COMPATIBLE)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"FC not compatible ");
		ErrorCode = 1;
		StopNavigation = 1;
		UART_VersionInfo.HardwareError[0] |= NC_ERROR0_FC_INCOMPATIBLE;
	}

	else if(FC.Error[0] & FC_ERROR0_GYRO_NICK)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"ERR: FC Nick Gyro");
		ErrorCode = 10;
	}
	else if(FC.Error[0] & FC_ERROR0_GYRO_ROLL)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"ERR: FC Roll Gyro");
		ErrorCode = 11;
	}
	else if(FC.Error[0] & FC_ERROR0_GYRO_YAW)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"ERR: FC Yaw Gyro");
		ErrorCode = 12;
	}
	else if(FC.Error[0] & FC_ERROR0_ACC_NICK)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"ERR: FC Nick ACC");
		ErrorCode = 13;
	}
	else if(FC.Error[0] & FC_ERROR0_ACC_ROLL)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"ERR: FC Roll ACC");
		ErrorCode = 14;
	}
	else if(FC.Error[0] & FC_ERROR0_ACC_TOP)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"ERR: FC Z-ACC");
		ErrorCode = 15;
	}
	else if(FC.Error[0] & FC_ERROR0_PRESSURE)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"ERR: Pressure sensor");
		ErrorCode = 16;
	}
	else if(FC.Error[1] &  FC_ERROR1_I2C)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"ERR: FC I2C");
		ErrorCode = 17;
	}
	else if(FC.Error[1] &  FC_ERROR1_BL_MISSING)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"ERR: Bl Missing");
		ErrorCode = 18;
	}
	else if(FC.Error[1] &  FC_ERROR1_MIXER)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"Mixer Error");
		ErrorCode = 19;
	}
	else if(CheckDelay(UBX_Timeout))
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"no GPS communication ");
		ErrorCode = 5;
		StopNavigation = 1;
		UART_VersionInfo.HardwareError[0] |= NC_ERROR0_GPS_RX;
	}
	else if(Compass_Heading < 0)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"bad compass value ");
		ErrorCode = 6;
		StopNavigation = 1;
		UART_VersionInfo.HardwareError[0] |= NC_ERROR0_COMPASS_VALUE;
	}
	else if((FC.Error[1] &  FC_ERROR1_SPI_RX))
	{
	 	LED_RED_ON;
		sprintf(ErrorMSG,"FC spi rx error ");
		ErrorCode = 8;
		StopNavigation = 1;
	}
	else if(FC.Error[0] &  FC_ERROR0_CAREFREE)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"FC: Carefree Error");
		ErrorCode = 20;
	}
	else if(FC.Error[1] &  FC_ERROR1_PPM)
	{
		LED_RED_ON;
		sprintf(ErrorMSG,"RC Signal lost ");
		ErrorCode = 7;
	}
	else // no error occured
	{
		sprintf(ErrorMSG,"No Error               ");
		ErrorCode = 0;
		StopNavigation = 0;
		LED_RED_OFF;
	}
	if(UART_VersionInfo.HardwareError[0] || UART_VersionInfo.HardwareError[1]) DebugOut.Status[1] |= 0x04;
	else DebugOut.Status[1] &= ~0x04;
}

// the handler will be cyclic called by the timer 1 ISR
// used is for critical timing parts that normaly would handled
// within the main loop that could block longer at logging activities
void EXTIT3_IRQHandler(void)
{
	IENABLE;

	VIC_ITCmd(EXTIT3_ITLine,DISABLE); // disable irq
	VIC_SWITCmd(EXTIT3_ITLine,DISABLE); // clear pending bit
	Compass_Update();		// update compass communication
	Analog_Update(); 		// get new ADC values
	VIC_ITCmd(EXTIT3_ITLine, ENABLE); // enable irq

	IDISABLE;
}

//----------------------------------------------------------------------------------------------------
int main(void)
{
	/* Configure the system clocks */
	SCU_Config();
	/* init VIC (Vectored Interrupt Controller)	*/
	SCU_AHBPeriphClockConfig(__VIC,ENABLE);	// enable AHB bus clock for VIC
	SCU_AHBPeriphReset(__VIC, DISABLE);		// disable reset state for VIC
	VIC_DeInit();							// deinitializes the VIC module registers to their default reset values.
	VIC_InitDefaultVectors();

	// initialize timer 1 for System Clock and delay rountines
	TIMER1_Init();
	// initialize the LEDs (needs Timer 1)
	Led_Init();
	// initialize the debug UART1
	UART1_Init();
	UART1_PutString("\r\n---------------------------------------------");
	// initialize usb
	USB_ConfigInit();
	// initialize timer 2 for servo outputs
	//TIMER2_Init();
	// initialize UART2 to FLIGHTCTRL
	UART2_Init();
	// initialize UART0 (to MKGPS or MK3MAG)
	UART0_Init();
	// initialize adc
	Analog_Init();
	// initialize SPI0 to FC
	SPI0_Init();
	// initialize i2c bus (needs Timer 1)
	I2C1_Init();
	// initialize fat16 partition on sd card (needs Timer 1)
	Fat16_Init();
	// initialize NC params
	NCParams_Init();
	// initialize the settings
	Settings_Init();
	// initialize logging (needs settings)
	Logging_Init();

	TimerCheckError = SetDelay(3000);
	UART1_PutString("\r\n++++++++++++++++++++++++++++++++++++++++++");
	UART1_PutString("\n\r Version information:");

	GetNaviCtrlVersion();
	DebugOut.Status[0] = 0x04 | 0x08; // NC and MK3Mag
	DebugOut.Status[1] = 0x00;

   	Compass_Init();

	SPI0_GetFlightCtrlVersion();
	if(FC_Version.Compatible != FC_SPI_COMPATIBLE)
	{
		UART1_PutString("\n\r Flight-Ctrl not compatible");
		LED_RED_ON;
	}

	GPS_Init(&(ToFlightCtrl.GPSStick));

	// ---------- Prepare the isr driven
	// set to absolute lowest priority
    VIC_Config(EXTIT3_ITLine, VIC_IRQ, PRIORITY_SW);
	// enable interrupts
    VIC_ITCmd(EXTIT3_ITLine, ENABLE);

	Debug_OK("START");

	for (;;) // the endless main loop
	{
		UART0_ProcessRxData(); 	// process request
		UART1_ProcessRxData(); 	// process request
		USB_ProcessRxData(); 	// process request
		UART0_TransmitTxData(); // send answer
		UART1_TransmitTxData(); // send answer
		UART2_TransmitTxData(); // send answer
		USB_TransmitTxData(); 	// send answer
		SPI0_UpdateBuffer();    // handle new SPI Data
		// ---------------- Error Check Timing ----------------------------
		if(CheckDelay(TimerCheckError))
		{
			TimerCheckError = SetDelay(1000);
			if(CheckDelay(SPI0_Timeout)) GPS_Navigation(&GPSData, &(ToFlightCtrl.GPSStick)); // process the GPS data even if the FC is not connected
			CheckErrors();
			if(FC.StatusFlags & FC_STATUS_FLY) NaviData.FlyingTime++; // we want to count the battery-time
//			else NaviData.FlyingTime = 0; // not the time per flight
			if(SerialLinkOkay) SerialLinkOkay--;
			if(SerialLinkOkay < 250 - 5) NCFlags |= NC_FLAG_NOSERIALLINK; // 5 seconds timeout for serial communication
			else NCFlags &= ~NC_FLAG_NOSERIALLINK;
			if(StopNavigation && (Parameter.NaviGpsModeControl >=  50)) BeepTime = 1000;
		}
 		// ---------------- Logging  ---------------------------------------
		Logging_Update();  // could be block some time for at max. 2 seconds, therefore move time critical part of the mainloop into the ISR of timer 1
	}
}


