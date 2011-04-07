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
#include "timer1.h"
#include "uart1.h"
#include "config.h"

u32 CountMilliseconds;
DateTime_t SystemTime;

//----------------------------------------------------------------------------------------------------
void TIM1_IRQHandler(void)
{
	IENABLE;

	if(TIM_GetFlagStatus(TIM1, TIM_FLAG_OC1) == SET)
	{
		TIM_ClearFlag(TIM1, TIM_FLAG_OC1); // clear irq pending bit
		TIM1->OC1R += 200;    // Timerfreq is 200kHz, generate an interrupt every 1ms
		CountMilliseconds++;
		
		// generate SW Interrupt to make a regular timing 
		// independent from the mainloop at the lowest IRQ priority
		VIC_SWITCmd(EXTIT3_ITLine, ENABLE);	
	}

	IDISABLE;
}

//----------------------------------------------------------------------------------------------------
// 1ms Timer
//----------------------------------------------------------------------------------------------------
void TIMER1_Init(void)
{
	TIM_InitTypeDef   TIM_InitStructure;

	UART1_PutString("\r\n Timer1 init...");

	#define TIM1_FREQ 200000 // 200kHz
	// TimerOCR set in IntHandler

	SCU_APBPeriphClockConfig(__TIM01, ENABLE);

	TIM_DeInit(TIM1); 
	TIM_StructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Mode = TIM_OCM_CHANNEL_1;
	TIM_InitStructure.TIM_OC1_Modes = TIM_TIMING;
	TIM_InitStructure.TIM_Clock_Source = TIM_CLK_APB;
	TIM_InitStructure.TIM_Prescaler = (SCU_GetPCLKFreqValue() * 1000) / TIM1_FREQ;	// is only valid up to 48 MHz !
	TIM_Init (TIM1, &TIM_InitStructure);

	TIM_ITConfig(TIM1, TIM_IT_OC1, ENABLE);
	TIM_CounterCmd(TIM1, TIM_START);

	VIC_Config(TIM1_ITLine, VIC_IRQ, PRIORITY_TIMER1);
	VIC_ITCmd(TIM1_ITLine, ENABLE);

	SystemTime.Year = 0;
	SystemTime.Month = 0;
	SystemTime.Day = 0;
	SystemTime.Hour = 0;
	SystemTime.Min = 0;
	SystemTime.Sec = 0;
	SystemTime.mSec = 0;
	SystemTime.Valid = 0;

	CountMilliseconds = 0;

	UART1_PutString("ok");

}

// -----------------------------------------------------------------------
u32 SetDelay (u32 t)
{
	return(CountMilliseconds + t -1);
}

// -----------------------------------------------------------------------
u8 CheckDelay(u32 t)
{
	return(((t - CountMilliseconds)& 0x80000000) >> 27);
}

// -----------------------------------------------------------------------
// this function calculates the time difference of t to the current time in ms
// if t < current time the delay is reportet as zero
u32 GetDelay(u32 t)
{
	u32 delay = 0;
	delay =  t - CountMilliseconds;
	if(delay & 0x80000000) delay = 0; // avoid negative delay values
	return(delay);
}

// -----------------------------------------------------------------------
void Delay_ms(u32 w)
{
	u32 akt;
	akt = CountMilliseconds + w;
	while(1)
	{
		if(akt<=CountMilliseconds) return;
	}
}
