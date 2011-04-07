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
#include "91x_lib.h"
#include "config.h"

//________________________________________________________________________________________________________________________________________
// Module name:			ssc.c 
// Compiler used:		avr-gcc 3.4.5
// Last Modifikation:	24.07.2007
// Version:				1.02
// Authors:				Stephan Busker 	
// Description:			Source files for access to the synchrnous serial channel.
//						Copyright (C) 2007 Stephan Busker
//........................................................................................................................................
// ext. Functions:		extern void 			SSC_Init(void);		
//						extern u8 				SSC_GetChar (void);
//						extern void 			SSC_PutChar (u8 Byte);
//						extern void 			SSC_Disable(void); 
//						extern void 			SSC_Enable(void); 
//						extern void				SSC_ClearRxFifo();
//........................................................................................................................................
// URL:					www.Mikro-Control.de
// mailto:				stephan.busker@mikro-control.de
//________________________________________________________________________________________________________________________________________



//________________________________________________________________________________________________________________________________________
// Function: 	SSC_Enable(void);
// 
// Description:	This function enables chipselect of the sdcard (active low) 
//				
//
// Returnvalue: none
//________________________________________________________________________________________________________________________________________

void SSC_Enable(void) 
{ 
	// enable chipselect of the sd-card (P5.4 -> SD-CS, active low). 
	GPIO_WriteBit(GPIO5, GPIO_Pin_4 , Bit_RESET);
}
 
//________________________________________________________________________________________________________________________________________
// Function: 	SSC_Disable(void);
// 
// Description:	This function disables chipselect of the sdcard (active low) 
//				
//
// Returnvalue: none
//________________________________________________________________________________________________________________________________________

void SSC_Disable(void) 
{ 
	// disable chipselect of the sd-card (P5.4 -> SD-CS, active low). 
	GPIO_WriteBit(GPIO5, GPIO_Pin_4 , Bit_SET);
}


//________________________________________________________________________________________________________________________________________
// Function: 	SSC_Init(void);
// 
// Description:	This function initialises the synchronus serial channel to the sdcard. 
//				
//
// Returnvalue: none
//________________________________________________________________________________________________________________________________________

void SSC_Init(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	SSP_InitTypeDef   	SSP_InitStructure;
	WIU_InitTypeDef 	WIU_InitStructure;
	// enable APB clock for SPI1
	SCU_APBPeriphClockConfig(__SSP1 ,ENABLE);
	// configure P5.4 -> SD-CS as an output pin
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt1;
	GPIO_Init (GPIO5, &GPIO_InitStructure);
   	// configure P3.4 -> SCK1 and P3.6 -> MOSI1 as an output pin
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt2;
	GPIO_Init (GPIO3, &GPIO_InitStructure);
	// configure P3.5 <- MISO1 as an input pin
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinInput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_InputAlt1;
	GPIO_Init (GPIO3, &GPIO_InitStructure);
	// configure SPI1
	SSP_DeInit(SSP1);
	SSP_StructInit(&SSP_InitStructure);
	SSP_InitStructure.SSP_FrameFormat = SSP_FrameFormat_Motorola;
	SSP_InitStructure.SSP_Mode = SSP_Mode_Master;
	SSP_InitStructure.SSP_CPHA = SSP_CPHA_1Edge;
	SSP_InitStructure.SSP_CPOL = SSP_CPOL_Low;
	// Set Baud Rate (Prescaler)
	// bit rate is BRCLK/SSP_ClockPrescaler/(1+SSP_ClockRate))
	// With MSCLK = 48MHz/2 = BRCLK we get for the SPICLK = 24Mhz / 8 / (1+5) = 500 kHz
	SSP_InitStructure.SSP_ClockRate = 5; //5
	SSP_InitStructure.SSP_ClockPrescaler = 8;
	SSP_Init(SSP1, &SSP_InitStructure);
	SSC_Disable();
	SSP_Cmd(SSP1, ENABLE);

	// Configure SD_SWITCH at pin GPIO5.3 as an external irq 11

	// configure the port
	SCU_APBPeriphClockConfig(__GPIO5, ENABLE); // Enable the GPIO5 Clock 	
    GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
	GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO5, &GPIO_InitStructure); 
	
	// configure the EXT11 interrupt line (P5.3) as an IRQ with the lowest priority
  	SCU_APBPeriphClockConfig(__WIU, ENABLE);
	WIU_Cmd(ENABLE);
	WIU_ClearITPendingBit(WIU_Line11);
	//WIU_DeInit();
	WIU_InitStructure.WIU_TriggerEdge = WIU_FallingEdge;
	WIU_InitStructure.WIU_Line = WIU_Line11;
	WIU_Init(&WIU_InitStructure);
	// The	EXTIT1_IRQHandler() is called every time the SD-Switch is activated (falling edge)
	// by inserting an sd-card
	SCU_WakeUpLineConfig(11);
    VIC_Config(EXTIT1_ITLine, VIC_IRQ, PRIORITY_SDSWITCH);
    VIC_ITCmd(EXTIT1_ITLine, ENABLE);
}

void SSC_Deinit(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;

	SSP_Cmd(SSP1, DISABLE);
	SSP_DeInit(SSP1);

	// configure P5.4 -> SD-CS as an input pin
	GPIO_InitStructure.GPIO_Direction = GPIO_PinInput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_InputAlt1;
	GPIO_Init (GPIO5, &GPIO_InitStructure);
   	// configure P3.4 -> SCK1 and P3.6 -> MOSI1 as an input pin
	GPIO_InitStructure.GPIO_Direction = GPIO_PinInput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_InputAlt1;
	GPIO_Init (GPIO3, &GPIO_InitStructure);
	// configure P3.5 <- MISO1 as an input pin
	GPIO_InitStructure.GPIO_Direction = GPIO_PinInput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_InputAlt1;
	GPIO_Init (GPIO3, &GPIO_InitStructure);

	// disable APB clock for SPI1
	SCU_APBPeriphClockConfig(__SSP1 ,DISABLE);

	// configure the port of the SDC-Switch
	SCU_APBPeriphClockConfig(__GPIO5, ENABLE); // Enable the GPIO5 Clock 	
    GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
	GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO5, &GPIO_InitStructure); 
}


//________________________________________________________________________________________________________________________________________
// Function: 	SSC_GetChar(void);
// 
// Description:	This function reads one byte from the SSC
//				
//
// Returnvalue: the byte received.
//________________________________________________________________________________________________________________________________________

u8 SSC_GetChar (void)
{
	u8 Byte = 0;
	while(SSP_GetFlagStatus(SSP1, SSP_FLAG_TxFifoNotFull) != SET);	// wait for space in the tx fifo
	SSP_SendData(SSP1, 0xFF);// send dymmy byte (0xFF) as master to receive a byte from the slave
	while(SSP_GetFlagStatus(SSP1, SSP_FLAG_TxFifoEmpty) != SET); // wait for the byte to be sent
	Byte = SSP_ReceiveData(SSP1); // read the byte transmitted from the slave
	return (Byte);
}

//________________________________________________________________________________________________________________________________________
void SSC_ClearRxFifo (void)
{
	// wait that the tx fifo is empty
	while(SSP_GetFlagStatus(SSP1, SSP_FLAG_TxFifoEmpty) != SET);
	// then empty the rx fifo by reading all the bytes that are available
	while(SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty) == SET) SSP_ReceiveData(SSP1);
}
//________________________________________________________________________________________________________________________________________
// Function: 	SSC_PutChar(u8 Byte);
// 
// Description:	This function writes one byte to the SSC 
//				
//
// Returnvalue: none
//________________________________________________________________________________________________________________________________________

void SSC_PutChar (u8 Byte)
{
	// wait for some space in the tx fifo
	while(SSP_GetFlagStatus(SSP1, SSP_FLAG_TxFifoNotFull) != SET);
	// put the byte to send in the tx fifo
	SSP_SendData(SSP1, Byte);
}








