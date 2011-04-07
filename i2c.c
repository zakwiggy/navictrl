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
#include "i2c.h"
#include "uart1.h"
#include "timer1.h"
#include "config.h"
#include "led.h"


volatile u8 I2C_State = I2C_STATE_OFF; 		// only one byte, because of sync by nesting irqs
volatile u8 I2C_Error = I2C_ERROR_NOACK;    // only one byte!

// number of bytes to send
volatile u8 I2C_TxBufferSize;
// number of bytes to receive
volatile u8 I2C_RxBufferSize;
// the transfer buffer
volatile u8 I2C_Buffer[I2C_BUFFER_LEN];
// the transfer direction 
volatile u8 I2C_Direction;
// the slave address
volatile u8 I2C_SlaveAddress = 0x00;
// function pointer to process the received bytes
I2C_pRxHandler_t I2C_pRxHandler = NULL;
// goblal timeout 
volatile u32 I2C1_Timeout = 0;

//--------------------------------------------------------------
void I2C1_Init(void)
{
	I2C_InitTypeDef   I2C_Struct;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	I2C_State = I2C_STATE_OFF;

	UART1_PutString("\r\n I2C init...");
	// enable Port 2 peripherie
	SCU_APBPeriphClockConfig(__GPIO2, ENABLE);
	// disable a reset state
	SCU_APBPeriphReset(__GPIO2, DISABLE);

	// free a busy bus

	// At switch on I2C devices can get in a state where they
	// are still waiting for a command due to all the bus lines bouncing
	// around at startup have started clocking data into the device(s).
	// Enable the ports as open collector port outputs
	// and clock out at least 9 SCL pulses, then generate a stop
	// condition and then leave the clock line high.

	// configure P2.2->I2C1_CLKOUT and P2.3->I2C1_DOUT to normal port operation
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_OpenCollector;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt1;
	GPIO_Init(GPIO2, &GPIO_InitStructure);

	u8 i;
	u32 delay;
	// set SCL high and then SDA to low (start condition)
	GPIO_WriteBit(GPIO2, GPIO_Pin_2, Bit_SET);
	delay = SetDelay(1);
	while (!CheckDelay(delay));
	GPIO_WriteBit(GPIO2, GPIO_Pin_3, Bit_RESET);
	// toggle SCL at least 10 times from high to low to high
	for(i = 0; i < 10; i++)
	{
		delay = SetDelay(1);
		while (!CheckDelay(delay));

		GPIO_WriteBit(GPIO2, GPIO_Pin_2, Bit_RESET);
		delay = SetDelay(1);
		while (!CheckDelay(delay));
		GPIO_WriteBit(GPIO2, GPIO_Pin_2, Bit_SET);
	}
	delay = SetDelay(1);
	while (!CheckDelay(delay));
	// create stop condition setting SDA HIGH when SCL is HIGH
	GPIO_WriteBit(GPIO2, GPIO_Pin_3, Bit_SET);


	// reconfigure P2.2->I2C1_CLKOUT and P2.3->I2C1_DOUT
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Type = GPIO_Type_OpenCollector;
	GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
	GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt2; //I2C1_CLKOUT, I2C1_DOUT
	GPIO_Init(GPIO2, &GPIO_InitStructure);

	// enable I2C peripherie
	SCU_APBPeriphClockConfig(__I2C1,ENABLE);
	// reset I2C peripherie
	SCU_APBPeriphReset(__I2C1,ENABLE);
	SCU_APBPeriphReset(__I2C1,DISABLE);

	I2C_DeInit(I2C1);
	I2C_StructInit(&I2C_Struct);
	I2C_Struct.I2C_GeneralCall = I2C_GeneralCall_Disable;
	I2C_Struct.I2C_Ack = I2C_Ack_Enable;
	I2C_Struct.I2C_CLKSpeed = I2C1_CLOCK;
	I2C_Struct.I2C_OwnAddress = 0x00;
	I2C_Init(I2C1, &I2C_Struct);

	// empty rx and tx buffer counters
	I2C_TxBufferSize = 0;
	I2C_RxBufferSize = 0;

	I2C_Cmd(I2C1, ENABLE);
	I2C_ITConfig(I2C1, ENABLE);

	VIC_Config(I2C1_ITLine, VIC_IRQ , PRIORITY_I2C1);

	I2C1_Timeout = SetDelay(10*I2C1_TIMEOUT);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_State = I2C_STATE_IDLE;

	// start some dummy transmissions cycles
	// to get the irq routine to work
	for(i=0;i<10;i++)
	{
		I2C_State = I2C_STATE_BUFFBUSY;
		I2C_Transmission(0,1,0,1);
		if(I2C_WaitForEndOfTransmission(10)) break;
		UART1_Putchar('.');
	}
	UART1_PutString("ok");
}


//--------------------------------------------------------------
void I2C1_Deinit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	UART1_PutString("\r\n I2C deinit...");
	I2C_GenerateStart(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	VIC_ITCmd(I2C1_ITLine, DISABLE);
	I2C_State = I2C_STATE_OFF;
	I2C_ITConfig(I2C1, DISABLE);
	I2C_Cmd(I2C1, DISABLE);
	I2C_DeInit(I2C1);
	SCU_APBPeriphClockConfig(__I2C1, DISABLE);

	// set ports to input
	SCU_APBPeriphClockConfig(__GPIO2, ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
	GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
	GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
	GPIO_Init(GPIO2, &GPIO_InitStructure);

	// empty rx and tx buffer
	I2C_TxBufferSize = 0;
	I2C_RxBufferSize = 0;

	I2C1_Timeout = SetDelay(10*I2C1_TIMEOUT);

	UART1_PutString("ok");
}

//--------------------------------------------------------------
void I2C1_IRQHandler(void)
{
	static u8 Rx_Idx = 0, Tx_Idx = 0;
	u16 status;

	//IENABLE;  // do not enable IRQ nesting for I2C!!!!
	// detemine I2C State
	status = I2C_GetLastEvent(I2C1);

	if(status & (I2C_FLAG_AF|I2C_FLAG_BERR))  // if an acknowledge failure or bus error occured
	{	// Set and subsequently clear the STOP bit while BTF is set.
		while(I2C_GetFlagStatus (I2C1, I2C_FLAG_BTF) != RESET)
		{
			I2C_GenerateSTOP (I2C1, ENABLE);  // free the bus
			I2C_GenerateSTOP (I2C1, DISABLE); // free the bus
		}
		I2C_State = I2C_STATE_IDLE;
		I2C_Error = I2C_ERROR_NOACK;
		VIC_ITCmd(I2C1_ITLine, DISABLE);
		LED_GRN_OFF;
		return;
	}
	else
	{	// depending on current i2c state
		switch(status)
		{
			// the start condition was initiated on the bus
			case I2C_EVENT_MASTER_MODE_SELECT:
				LED_GRN_ON;
				// update current bus state variable
				// jump to rx state if there is nothing to send
				switch(I2C_Direction)
				{
					case I2C_MODE_TRANSMITTER:
						I2C_State = I2C_STATE_TX_PROGRESS;
						break;

					case I2C_MODE_RECEIVER:
						if (I2C_RxBufferSize == 0) // nothing to send?
						{
							I2C_GenerateSTOP (I2C1, ENABLE);
							VIC_ITCmd(I2C1_ITLine, DISABLE);
							LED_GRN_OFF;
							I2C_State = I2C_STATE_IDLE;
							I2C_Error = I2C_ERROR_NONE;
							return;
						}
						else
						{
							I2C_State = I2C_STATE_RX_PROGRESS;
						}
						break;

					default: // invalid direction
						I2C_GenerateSTOP (I2C1, ENABLE);
						VIC_ITCmd(I2C1_ITLine, DISABLE);
						LED_GRN_OFF;
						I2C_State = I2C_STATE_IDLE;
						I2C_Error = I2C_ERROR_UNKNOWN;
						return;
				}
				// enable acknowledge
				I2C_AcknowledgeConfig (I2C1, ENABLE);
				// send address/direction byte on the bus
				I2C_Send7bitAddress(I2C1, I2C_SlaveAddress, I2C_Direction);
				break;

			// the address byte was send
			case I2C_EVENT_MASTER_MODE_SELECTED:
				// Clear EV6 by set again the PE bit
				I2C_Cmd(I2C1, ENABLE);
				switch(I2C_State)
				{
					case I2C_STATE_TX_PROGRESS:
					// send 1st data byte
					Tx_Idx = 0;
					I2C_SendData(I2C1, I2C_Buffer[Tx_Idx]);
					Tx_Idx++;
					// reset timeout
					I2C1_Timeout = SetDelay(I2C1_TIMEOUT); // after inactivity the I2C1 bus will be reset
					break;

					case I2C_STATE_RX_PROGRESS:
					Rx_Idx = 0;
					// disable acknoledge if only one byte has to be read
					if(I2C_RxBufferSize == 1) I2C_AcknowledgeConfig (I2C1, DISABLE);		
					break;

					default: // unknown I2C state
					// should never happen
					I2C_GenerateSTOP (I2C1, ENABLE);
					LED_GRN_OFF;
					VIC_ITCmd(I2C1_ITLine, DISABLE);
					I2C_State = I2C_STATE_IDLE;
					I2C_Error = I2C_ERROR_UNKNOWN;
					return;
					break;
				}
				break;

			// the master has transmitted a byte and slave has been acknowledged
			case I2C_EVENT_MASTER_BYTE_TRANSMITTED:

				// some bytes have to be transmitted
				if(Tx_Idx < I2C_TxBufferSize)
				{
					I2C_SendData(I2C1, I2C_Buffer[Tx_Idx]);
					Tx_Idx++;
				}
				else // last byte was send
				{
					// generate stop or repeated start condition
					if (I2C_RxBufferSize > 0) // is any answer byte expected?
					{
						I2C_Direction = I2C_MODE_RECEIVER; // switch to master receiver after repeated start condition
						I2C_GenerateStart(I2C1, ENABLE);   // initiate repeated start condition on the bus
					}
					else
					{   // stop communication
						I2C_GenerateSTOP(I2C1, ENABLE);	// generate stop condition to free the bus
						VIC_ITCmd(I2C1_ITLine, DISABLE);
						LED_GRN_OFF;
						I2C_State = I2C_STATE_IDLE;			// ready for new actions
						I2C_Error = I2C_ERROR_NONE;	
					}
				}
				break;

			// the master has received a byte from the slave
			case I2C_EVENT_MASTER_BYTE_RECEIVED:
				// some bytes have to be received
				if ( Rx_Idx+1 < I2C_RxBufferSize)
				{ 	// copy received byte  from the data register to the rx-buffer
					I2C_Buffer[Rx_Idx] = I2C_ReceiveData(I2C1);
				}
				else // if the last byte was received
				{
					// generate a STOP condition on the bus before reading data register
					I2C_GenerateSTOP(I2C1, ENABLE);
					I2C_Buffer[Rx_Idx] = I2C_ReceiveData(I2C1);
					// call the rx handler function to process recieved data
					if(I2C_pRxHandler != NULL) (*I2C_pRxHandler)((u8*)I2C_Buffer, I2C_RxBufferSize);
					I2C1_Timeout = SetDelay(I2C1_TIMEOUT);
					DebugOut.Analog[15]++;
					VIC_ITCmd(I2C1_ITLine, DISABLE);
					LED_GRN_OFF;
					I2C_State = I2C_STATE_IDLE;
					I2C_Error = I2C_ERROR_NONE;
					return;
				}
				Rx_Idx++;
				// if the 2nd last byte was received disable acknowledge for the last one
				if ( (Rx_Idx + 1) == I2C_RxBufferSize )
				{
					I2C_AcknowledgeConfig(I2C1, DISABLE);
				}
				break;

			default:// unknown event
				// should never happen
				I2C_GenerateSTOP (I2C1, ENABLE);
				LED_GRN_OFF;
				VIC_ITCmd(I2C1_ITLine, DISABLE);
				I2C_State = I2C_STATE_IDLE;
				I2C_Error = I2C_ERROR_UNKNOWN;
				break;
		}
	}
	//IDISABLE;	 // do not enable IRQ nesting for I2C!!!!
}

// ----------------------------------------------------------------------------------------
// wait for end of transmission
u8 I2C_WaitForEndOfTransmission(u32 timeout)
{
	u32 time = SetDelay(timeout);
	while(I2C_State != I2C_STATE_IDLE)
	{
		if(CheckDelay(time)) return(0);
	}
	return(1);
}

// ----------------------------------------------------------------------------------------
// try to get access to the transfer buffer	 within a timeout limit
// returs 1 on success and 0 on error/timeout
u8 I2C_LockBuffer(u32 timeout)
{	
	if(I2C_WaitForEndOfTransmission(timeout))
	{
		I2C_State = I2C_STATE_BUFFBUSY;
		I2C_Error = I2C_ERROR_UNKNOWN;
		return(1);
	}
	else return(0);
}
// ----------------------------------------------------------------------------------------
// initate an i2c transmission
u8 I2C_Transmission(u8 SlaveAddr, u8 TxBytes, I2C_pRxHandler_t pRxHandler, u8 RxBytes)
{
	u8 retval = 0;
	if(I2C_State == I2C_STATE_BUFFBUSY)
	{
		if((RxBytes > I2C_BUFFER_LEN) || (TxBytes > I2C_BUFFER_LEN))
		{
		  	I2C_State = I2C_STATE_IDLE;
			return(retval);
		}	
		I2C_RxBufferSize = RxBytes;
		I2C_TxBufferSize = TxBytes;
		// set direction to master transmitter
		if( (I2C_TxBufferSize > 0) && (I2C_TxBufferSize < I2C_BUFFER_LEN) ) I2C_Direction = I2C_MODE_TRANSMITTER;
		else if (( I2C_RxBufferSize > 0 ) && (I2C_RxBufferSize < I2C_BUFFER_LEN) ) I2C_Direction = I2C_MODE_RECEIVER;
		else // nothing to send or receive
		{
			I2C_State = I2C_STATE_IDLE;
			I2C_Error = I2C_ERROR_NONE;
			I2C_TxBufferSize = 0;
			I2C_RxBufferSize = 0;
			return(retval);
		}
		// update slave address and rx data handler	funbction pointer
		I2C_SlaveAddress = SlaveAddr;
		I2C_pRxHandler = pRxHandler;
		// test on busy flag and clear it
		I2C_CheckEvent( I2C1, I2C_FLAG_BUSY );
		// enable I2C IRQ
		VIC_ITCmd(I2C1_ITLine, ENABLE);
		// initiate start condition on the bus
		I2C_GenerateStart(I2C1, ENABLE);
		retval = 1;
	 }
	 return(retval);
}
