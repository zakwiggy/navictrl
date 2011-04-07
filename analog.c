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
#include "uart1.h"
#include "analog.h"
#include "timer1.h"
#include "config.h"

AnalogData_t AnalogData;

void ADC_IRQHandler(void)
{
	IENABLE;

	if(ADC_GetFlagStatus(ADC_FLAG_ECV) == SET)
	{
		ADC_ClearFlag(ADC_FLAG_ECV);
		AnalogData.Ch1 = ADC_GetConversionValue(ADC_Channel_1);
		AnalogData.Ch2 = ADC_GetConversionValue(ADC_Channel_2);
		AnalogData.Ch3 = ADC_GetConversionValue(ADC_Channel_3);
		AnalogData.Ch4 = ADC_GetConversionValue(ADC_Channel_4);
		AnalogData.Ch5 = ADC_GetConversionValue(ADC_Channel_5);
		AnalogData.Ch6 = ADC_GetConversionValue(ADC_Channel_6);
		AnalogData.Ch7 = ADC_GetConversionValue(ADC_Channel_7);
	}
	IDISABLE;
}

void Analog_Init(void)
{
	ADC_InitTypeDef   ADC_InitStructure;

	UART1_PutString("\r\n ADC init...");

	SCU_APBPeriphClockConfig(__GPIO4, ENABLE); // Enable the GPIO4 Clock

	ADC_DeInit(); // reset register to default values

	// configure port 4 pins as analog inputs
	GPIO_ANAPinConfig(GPIO_ANAChannel1, ENABLE);
	GPIO_ANAPinConfig(GPIO_ANAChannel2, ENABLE);
	GPIO_ANAPinConfig(GPIO_ANAChannel3, ENABLE);
	GPIO_ANAPinConfig(GPIO_ANAChannel4, ENABLE);
	GPIO_ANAPinConfig(GPIO_ANAChannel5, ENABLE);
	GPIO_ANAPinConfig(GPIO_ANAChannel6, ENABLE);
	GPIO_ANAPinConfig(GPIO_ANAChannel7, ENABLE);

	ADC_Cmd(ENABLE); // power on the ADC
	ADC_StandbyModeCmd(DISABLE); // disable Standby Mode
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Channel_1_Mode = ADC_NoThreshold_Conversion;
	ADC_InitStructure.ADC_Channel_2_Mode = ADC_NoThreshold_Conversion;
	ADC_InitStructure.ADC_Channel_3_Mode = ADC_NoThreshold_Conversion;
	ADC_InitStructure.ADC_Channel_4_Mode = ADC_NoThreshold_Conversion;
	ADC_InitStructure.ADC_Channel_5_Mode = ADC_NoThreshold_Conversion;
	ADC_InitStructure.ADC_Channel_6_Mode = ADC_NoThreshold_Conversion;
	ADC_InitStructure.ADC_Channel_7_Mode = ADC_NoThreshold_Conversion;
	ADC_InitStructure.ADC_Select_Channel = ADC_Channel_1;
	ADC_InitStructure.ADC_Scan_Mode = ENABLE; // convert all selected channels
	ADC_InitStructure.ADC_Conversion_Mode = ADC_Single_Mode;
	ADC_Init(&ADC_InitStructure);

	ADC_PrescalerConfig(255);  // PCLK/255  (24MHz/255 = 94.1kHZ )
	ADC_ITConfig(ADC_IT_ECV, ENABLE); // enable end of conversion IRQ

	VIC_Config(ADC_ITLine, VIC_IRQ, PRIORITY_ADC);
	VIC_ITCmd(ADC_ITLine, ENABLE);

	UART1_PutString("ok");
}

void Analog_Deinit(void)
{
	VIC_ITCmd(ADC_ITLine, DISABLE);
	ADC_ConversionCmd(ADC_Conversion_Stop);
	ADC_ITConfig(ADC_IT_ECV, DISABLE); // disable end of conversion IRQ
	ADC_Cmd(DISABLE); // power down the ADC
	ADC_DeInit(); // reset register to default values
}

void Analog_Update(void)
{
	static u32 AnalogTimer = 0;

	if(CheckDelay(AnalogTimer))
	{
		AnalogTimer = SetDelay(25);
 		ADC_ConversionCmd(ADC_Conversion_Start);
	}
}
