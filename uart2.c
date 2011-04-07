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
//#include <stdio.h>
#include "91x_lib.h"
#include "config.h"
#include "uart1.h"
#include "mkprotocol.h"

// the tx buffer
#define UART2_TX_BUFFER_LEN  150
u8 UART2_tbuffer[UART2_TX_BUFFER_LEN];
Buffer_t UART2_tx_buffer;

/********************************************************/
/*                  Initialize UART2                    */
/********************************************************/
void UART2_Init(void)
{
    UART_InitTypeDef UART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	UART1_PutString("\r\n UART2 init...");

	// initialize txd buffer
	Buffer_Init(&UART2_tx_buffer, UART2_tbuffer, UART2_TX_BUFFER_LEN);

	SCU_APBPeriphClockConfig(__UART2, ENABLE);  // Enable the UART2 Clock

	SCU_APBPeriphClockConfig(__GPIO5, ENABLE);  // Enable the GPIO5 Clock
    /*Configure UART2_Rx pin GPIO5.2*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
    GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Enable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1; // UART2_RxD
    GPIO_Init(GPIO5, &GPIO_InitStructure);

	SCU_APBPeriphClockConfig(__GPIO3, ENABLE);  // Enable the GPIO3 Clock
    /*Configure UART2_Tx pin GPIO3.0*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinOutput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_OutputAlt2; // UART2_TX
    GPIO_Init(GPIO3, &GPIO_InitStructure);


	/* UART2 configured as follow:
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
    UART_InitStructure.UART_BaudRate = 				UART2_BAUD_RATE;
    UART_InitStructure.UART_HardwareFlowControl = 	UART_HardwareFlowControl_None;
    UART_InitStructure.UART_Mode = 					UART_Mode_Tx_Rx;
    UART_InitStructure.UART_FIFO = 					UART_FIFO_Enable;
    UART_InitStructure.UART_TxFIFOLevel = 			UART_FIFOLevel_1_2;
    UART_InitStructure.UART_RxFIFOLevel = 			UART_FIFOLevel_1_2;

	UART_DeInit(UART2);	// reset uart 2	to default
    UART_Init(UART2, &UART_InitStructure);  // initialize uart 2

	// enable uart 2 interrupts selective
    UART_ITConfig(UART2, UART_IT_Receive | UART_IT_ReceiveTimeOut, ENABLE);
	UART_Cmd(UART2, ENABLE); // enable uart 2
	// configure the uart 2 interupt line
 	VIC_Config(UART2_ITLine, VIC_IRQ, PRIORITY_UART2);
	// enable the uart 2 IRQ
	VIC_ITCmd(UART2_ITLine, ENABLE);

	UART1_PutString("ok");
}


void UART2_Deinit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	UART1_PutString("\r\n UART2 deinit...");
	VIC_ITCmd(UART2_ITLine, DISABLE); // disable the uart 2 IRQ
	UART_Cmd(UART2, DISABLE); // disable uart 2
	UART_DeInit(UART2);	// reset uart 0	to default

	SCU_APBPeriphClockConfig(__UART2, DISABLE);  // disable the UART2 Clock

	SCU_APBPeriphClockConfig(__GPIO5, ENABLE);
   	// unmap UART2 from FC
	// set port pin 5.2 (serial data from FC) to input and disconnect from IP
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
    GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO5, &GPIO_InitStructure);

	SCU_APBPeriphClockConfig(__GPIO3, ENABLE);
	// set port pin 3.0 (serial data to FC) to input and disconnect from IP
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO3, &GPIO_InitStructure);

	UART1_PutString("ok");
}

/********************************************************/
/*            UART2 Interrupt Handler                   */
/********************************************************/
void UART2_IRQHandler(void)
{
	IENABLE;	

	// if receive irq or receive timeout irq has occured
 	if((UART_GetITStatus(UART2, UART_IT_Receive) != RESET) || (UART_GetITStatus(UART2, UART_IT_ReceiveTimeOut) != RESET) )
 	{
   		UART_ClearITPendingBit(UART2, UART_IT_Receive);			// clear receive interrupt flag
   		UART_ClearITPendingBit(UART2, UART_IT_ReceiveTimeOut);	// clear receive timeout interrupt flag

		// if debug UART is UART2
		if (DebugUART == UART2)
		{	// forward received data to the UART1 tx buffer
		 	while(UART_GetFlagStatus(UART2, UART_FLAG_RxFIFOEmpty) != SET)
			{
				// wait for space in the tx buffer of the UART1
				while(UART_GetFlagStatus(UART1, UART_FLAG_TxFIFOFull) == SET) {};
				// move the byte from the rx buffer of UART2 to the tx buffer of UART1
				UART_SendData(UART1, UART_ReceiveData(UART2));
			}
		}
		else
		{
		 	// ignore serial data from the FC
			while(UART_GetFlagStatus(UART2, UART_FLAG_RxFIFOEmpty) != SET)
			{
				UART_ReceiveData(UART2);
			}
		}
	} // eof receive irq or receive timeout irq
	
	IDISABLE;
}

/**************************************************************/
/*         Transmit tx buffer via uart2                       */
/**************************************************************/
void UART2_Transmit(void)
{
	u8 tmp_tx;
	if(DebugUART == UART2) return; // no data output if debug uart is rederected to UART2
	// if something has to be send and the txd fifo is not full
	if((UART2_tx_buffer.Locked == TRUE) && (UART_GetFlagStatus(UART2, UART_FLAG_TxFIFOFull) == RESET))
	{
		tmp_tx = UART2_tx_buffer.pData[UART2_tx_buffer.Position++]; // read next byte from txd buffer
		UART_SendData(UART2, tmp_tx); // put character to txd fifo
		// if terminating character or end of txd buffer reached
		if((tmp_tx == '\r') || (UART2_tx_buffer.Position == UART2_tx_buffer.Size))
		{
			Buffer_Clear(&UART2_tx_buffer);
		}
	}
}

/**************************************************************/
/* Send the answers to incomming commands at the uart2        */
/**************************************************************/
void UART2_TransmitTxData(void)
{
	if(DebugUART == UART2) return;
	UART2_Transmit(); // output pending bytes in tx buffer
}


void UART2_ProcessRxData(void)
{

}
