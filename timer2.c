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
#include "timer2.h"
#include "uart1.h"
#include "spi_slave.h"
#include "config.h"

#define CR1_OLVL1_MASK    0x0100
#define CR1_OLVL2_MASK    0x0200

volatile ServoParams_t ServoParams;

#define TIM2_FREQ 625000 // 625kHz,  the same clock like at the FC
// frame len 22.5 ms = 14063 * 1.6 us
// stop pulse: 0.3 ms = 188 * 1.6 us
// min servo pulse: 0.6 ms =  375 * 1.6 us
// max servo pulse: 2.4 ms = 1500 * 1.6 us
// resolution: 1500 - 375 = 1125 steps
#define PPM_STOPPULSE 188
//#define PPM_FRAMELEN 14063
#define PPM_FRAMELEN (1757 * ServoParams.Refresh) // 22.5 ms / 8 Channels = 2.8125ms per Servo Channel
#define MINSERVOPULSE 375
#define MAXSERVOPULSE 1500
#define SERVORANGE (MAXSERVOPULSE - MINSERVOPULSE)

//----------------------------------------------------------------------------------------------------
void TIM2_IRQHandler(void)
{
	#define MULTIPLYER 4
	static s16 ServoNickOffset = (255 / 2) * MULTIPLYER; // initial value near center position
	static s16 ServoRollOffset = (255 / 2) * MULTIPLYER; // initial value near center position
	
	static u16 LowPulseTime1 = 14063;
	static u16 LowPulseTime2 = 14063;

	s16 ServoNickValue = 0;
	s16 ServoRollValue = 0;

	u16 pulselen;

	IENABLE;

	if(TIM_GetFlagStatus(TIM2, TIM_FLAG_OC1) == SET)
	{
		TIM_ClearFlag(TIM2, TIM_FLAG_OC1); // clear irq pending bit
		if (TIM2->CR1 & CR1_OLVL1_MASK) // start of high pulse
		{
			pulselen = MINSERVOPULSE + SERVORANGE/2;
			ServoNickOffset = (ServoNickOffset * 3 + (s16)ServoParams.NickControl * MULTIPLYER) / 4; // lowpass offset
			ServoNickValue = ServoNickOffset; // offset (Range from 0 to 255 * 3 = 765)
			if(ServoParams.CompInvert & 0x01)
			{	// inverting movement of servo FromFlightCtrl.AngleNick
				ServoNickValue += (s16)( ( (s32)ServoParams.NickComp * MULTIPLYER * (FromFlightCtrl.AngleNick) ) / (256L) );
			}
			else
			{	// non inverting movement of servo FromFlightCtrl.AngleNick
				ServoNickValue -= (s16)( ( (s32)ServoParams.NickComp * MULTIPLYER * (FromFlightCtrl.AngleNick) ) / (256L) );
			}
			// limit servo value to its parameter range definition
			if(ServoNickValue < ((s16)ServoParams.NickMin * MULTIPLYER) )
			{
				ServoNickValue = (s16)ServoParams.NickMin * MULTIPLYER;
			}
			else
			if(ServoNickValue > ((s16)ServoParams.NickMax * MULTIPLYER) )
			{
				ServoNickValue = (s16)ServoParams.NickMax * MULTIPLYER;
			}

			pulselen += ServoNickValue - (256 / 2) * MULTIPLYER; // shift ServoNickValue to center position
			DebugOut.Analog[7] = ServoNickValue / MULTIPLYER;
			LowPulseTime1 = PPM_FRAMELEN - pulselen;
			TIM2->CR1 &= ~CR1_OLVL1_MASK; // make next a low pulse
		}
		else // start of low pulse
		{
			pulselen = LowPulseTime1;			
			TIM2->CR1 |= CR1_OLVL1_MASK;  // make next a high pulse
		}
		TIM2->OC1R += pulselen;
	}

	if(TIM_GetFlagStatus(TIM2, TIM_FLAG_OC2) == SET)
	{
		TIM_ClearFlag(TIM2, TIM_FLAG_OC2); // clear irq pending bit
		if (TIM2->CR1 & CR1_OLVL2_MASK) // was high pulse
		{
			pulselen = MINSERVOPULSE + SERVORANGE/2;
			ServoRollOffset = (ServoRollOffset * 3 + (s16)ServoParams.RollControl * MULTIPLYER) / 4; // lowpass offset
			ServoRollValue = ServoRollOffset; // offset (Range from 0 to 255 * 3 = 765)
			if(ServoParams.CompInvert & 0x02)
			{	// inverting movement of servo FromFlightCtrl.AngleRoll
				ServoRollValue += (s16)( ( (s32)ServoParams.RollComp * MULTIPLYER * (FromFlightCtrl.AngleRoll) ) / (256L) );
			}
			else
			{	// non inverting movement of servo FromFlightCtrl.AngleRoll
				ServoRollValue -= (s16)( ( (s32)ServoParams.RollComp * MULTIPLYER * (FromFlightCtrl.AngleRoll) ) / (256L) );
			}
			// limit servo value to its parameter range definition
			if(ServoRollValue < ((s16)ServoParams.RollMin * MULTIPLYER) )
			{
				ServoRollValue = (s16)ServoParams.RollMin * MULTIPLYER;
			}
			else
			if(ServoRollValue > ((s16)ServoParams.RollMax * MULTIPLYER) )
			{
				ServoRollValue = (s16)ServoParams.RollMax * MULTIPLYER;
			}

			pulselen += ServoRollValue - (256 / 2) * MULTIPLYER; // shift ServoNickValue to center position
			DebugOut.Analog[8] = ServoRollValue / MULTIPLYER;
			LowPulseTime2 = PPM_FRAMELEN - pulselen;
			TIM2->CR1 &= ~CR1_OLVL2_MASK; // make next a low pulse
		}
		else
		{
			pulselen = LowPulseTime2;
			TIM2->CR1 |= CR1_OLVL2_MASK; // make next a high pulse
		}
		TIM2->OC2R += pulselen;
	}

	IDISABLE;
}

//----------------------------------------------------------------------------------------------------
// Servo Timer
//----------------------------------------------------------------------------------------------------
void TIMER2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_InitTypeDef   TIM_InitStructure;

	UART1_PutString("\r\n Timer2 init...");

	SCU_APBPeriphClockConfig(__GPIO6, ENABLE); // Enable the GPIO6 Clock

	// configure the servo pins
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinOutput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull ;
    GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Enable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_OutputAlt2; //TIM2_OCMP1
    GPIO_Init(GPIO6, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinOutput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Enable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_OutputAlt2; //TIM2_OCMP2
    GPIO_Init(GPIO6, &GPIO_InitStructure);

	SCU_APBPeriphClockConfig(__TIM23, ENABLE);

   	TIM_DeInit(TIM2); 
	TIM_StructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Mode = TIM_OCM_CHANNEL_12; // Output Compare Channels 1 & 2 Mode
	TIM_InitStructure.TIM_OC1_Modes = TIM_WAVE;	// OCMP1 pin is dedicated to the OC1 capability of the TIM
	TIM_InitStructure.TIM_OC2_Modes = TIM_WAVE;	// OCMP2 pin is dedicated to the OC2 capability of the TIM
	TIM_InitStructure.TIM_Clock_Source = TIM_CLK_APB; // APB clock source is used
	TIM_InitStructure.TIM_Pulse_Level_1 = TIM_LOW; // output low at OCMP1 pin on compare match
	TIM_InitStructure.TIM_Pulse_Level_2 = TIM_LOW; // output low at OCMP2 pin on compare match
	TIM_InitStructure.TIM_Prescaler = (SCU_GetPCLKFreqValue() * 1000) / TIM2_FREQ;

	TIM_Init(TIM2, &TIM_InitStructure);

	TIM_ITConfig(TIM2, TIM_IT_OC1|TIM_IT_OC2, ENABLE);  // enable interrupts for the OC 1 & 2

	VIC_Config(TIM2_ITLine, VIC_IRQ, PRIORITY_TIMER2);
	VIC_ITCmd(TIM2_ITLine, ENABLE);

	TIM2->OC1R = 10;
	TIM2->OC2R = 20;

	// set servo params to defaults
	ServoParams.Refresh = 5;
	ServoParams.CompInvert = 0; 
	ServoParams.NickControl = 127;
	ServoParams.NickComp = 40;
	ServoParams.NickMin = 50; 
	ServoParams.NickMax = 205;
	ServoParams.RollControl = 127;
	ServoParams.RollComp = 40;
	ServoParams.RollMin = 50; 
	ServoParams.RollMax = 205; 
	
	TIM_CounterCmd(TIM2, TIM_CLEAR); // reset timer
	TIM_CounterCmd(TIM2, TIM_START); // start the timer
     	
	UART1_PutString("ok");
}

void TIMER2_Deinit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	

	UART1_PutString("\r\n Timer2 deinit...");

	VIC_ITCmd(TIM2_ITLine, DISABLE);
	TIM_CounterCmd(TIM2, TIM_STOP); // stop the timer
	TIM_CounterCmd(TIM2, TIM_CLEAR); // stop the timer
	TIM_ITConfig(TIM2, TIM_IT_OC1|TIM_IT_OC2, DISABLE);  // disable interrupts for the OC 1 & 2
	TIM_DeInit(TIM2);
	SCU_APBPeriphClockConfig(__TIM23, DISABLE);

	// configure the servo pins as input
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_OpenCollector;
    GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO6, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = 	GPIO_PinInput;
    GPIO_InitStructure.GPIO_Pin = 			GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Type = 			GPIO_Type_OpenCollector;
	GPIO_InitStructure.GPIO_IPInputConnected = 	GPIO_IPInputConnected_Disable;
    GPIO_InitStructure.GPIO_Alternate = 	GPIO_InputAlt1;
    GPIO_Init(GPIO6, &GPIO_InitStructure);
}
