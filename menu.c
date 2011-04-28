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
#include <stdlib.h>
#include "91x_lib.h"
#include "main.h"
#include "led.h"
#include "ubx.h"
#include "GPS.h"
#include "timer1.h"
#include "compass.h"
#include "spi_slave.h"
#include "menu.h"
#include "uart1.h"
#include "ncmag.h"

u8 DispPtr = 0;
s8 DisplayBuff[DISPLAYBUFFSIZE];


u8 MenuItem = 0;
u8 MaxMenuItem = 15;



void Menu_Putchar(char c)
{
   if(DispPtr < DISPLAYBUFFSIZE) DisplayBuff[DispPtr++] = c; ;
}

void Menu_Clear(void)
{
	u8 i;
	for( i = 0; i < DISPLAYBUFFSIZE; i++) DisplayBuff[i] = ' ';
}


// Display with 20 characters in 4 lines
void Menu_Update(u8 Keys)
{
 	s32 i1,i2;
	u8 sign;

	if(Keys & KEY1) { if(MenuItem > 0) MenuItem--; else MenuItem = MaxMenuItem;}
	if(Keys & KEY2) { if(MenuItem == MaxMenuItem) MenuItem = 0; else MenuItem++;}
	if((Keys & KEY1) && (Keys & KEY2)) MenuItem = 0;
	if(MenuItem > MaxMenuItem) MenuItem = MaxMenuItem;
	Menu_Clear();
	// print menu item number in the upper right corner
  	if(MenuItem < 10)
  	{
		LCD_printfxy(17,0,"[%i]",MenuItem);
	}
	else // Menuitem >= 10
	{
		LCD_printfxy(16,0,"[%i]",MenuItem);
	}

  	switch(MenuItem)
   	{
		// Version Info
		case 0:
			LCD_printfxy(0,0,"++  Navi-Ctrl  ++");
			LCD_printfxy(0,1,"HW V%d.%d SW V%d.%d%c", Version_HW/10, Version_HW%10, VERSION_MAJOR, VERSION_MINOR, 'a'+ VERSION_PATCH);
            if(ErrorCode)
			{
				LCD_printfxy(0,2,"Error: %d",ErrorCode);
            	LCD_printfxy(0,3,"%s",ErrorMSG);
			}
			else LCD_printfxy(0,3,"(c) Zak, test01.2");
			break;
		case 1:
			if (GPSData.Status == INVALID)
			{
				LCD_printfxy(0,0,"No GPS data");
				LCD_printfxy(0,1,"Lon:                ");
				LCD_printfxy(0,2,"Lat:                ");
				LCD_printfxy(0,3,"Alt:                ");
			}
			else // newdata or processed
			{
				LCD_printfxy(0,0,"Sat:%02d ", GPSData.NumOfSats);
				switch (GPSData.SatFix)
				{
				case SATFIX_NONE:
					LCD_printfxy(7,0,"NoFix");
					break;
				case SATFIX_2D:
					LCD_printfxy(7,0,"2DFix");
					break;
				case SATFIX_3D:
					LCD_printfxy(7,0,"3DFix");
					break;
				default:
					LCD_printfxy(7,0,"??Fix");
					break;
				}
				if(GPSData.Flags & FLAG_DIFFSOLN)
				{
					LCD_printfxy(12,0,"/DGPS");
				}
				else
				{
					LCD_printfxy(12,0,"     ");
				}

				if(GPSData.Position.Longitude < 0) sign = '-';
				else sign = '+';
				i1 = abs(GPSData.Position.Longitude)/10000000L;
				i2 = abs(GPSData.Position.Longitude)%10000000L;
				LCD_printfxy(0,1,"Lon:%c%03ld.%07ld deg",sign, i1, i2);
				if(GPSData.Position.Latitude < 0) sign = '-';
				else sign = '+';
				i1 = abs(GPSData.Position.Latitude)/10000000L;
				i2 = abs(GPSData.Position.Latitude)%10000000L;
				LCD_printfxy(0,2,"Lat:%c%03ld.%07ld deg",sign, i1, i2);
				if(GPSData.Position.Altitude < 0) sign = '-';
				else sign = '+';
				i1 = abs(GPSData.Position.Altitude)/1000L;
				i2 = abs(GPSData.Position.Altitude)%1000L;
				LCD_printfxy(0,3,"Alt:%c%04ld.%03ld m", sign, i1, i2);
			}
			break;
		case 2:
			if (GPSData.Status == INVALID)
			{
				LCD_printfxy(0,0,"No GPS data");
				LCD_printfxy(0,1,"Speed N:            ");
				LCD_printfxy(0,2,"Speed E:            ");
				LCD_printfxy(0,3,"Speed T:            ");
			}
			else // newdata or processed
			{
				LCD_printfxy(0,0,"Sat:%02d ", GPSData.NumOfSats);
				switch (GPSData.SatFix)
				{
				case SATFIX_NONE:
					LCD_printfxy(7,0,"NoFix");
					break;
				case SATFIX_2D:
					LCD_printfxy(7,0,"2DFix");
					break;
				case SATFIX_3D:
					LCD_printfxy(7,0,"3DFix");
					break;
				default:
					LCD_printfxy(7,0,"??Fix");
					break;
				}
				if(GPSData.Flags & FLAG_DIFFSOLN)
				{
					LCD_printfxy(12,0,"/DGPS");
				}
				else
				{
					LCD_printfxy(12,0,"     ");
				}

				LCD_printfxy(0,1,"Speed N: %+4ld cm/s",GPSData.Speed_North);
				LCD_printfxy(0,2,"Speed E: %+4ld cm/s",GPSData.Speed_East);
				LCD_printfxy(0,3,"Speed T: %+4ld cm/s",GPSData.Speed_Top);
			}
			break;
		case 3:
			LCD_printfxy(0,0,"GPS UTC Time");
			if (!SystemTime.Valid)
			{
				LCD_printfxy(0,1,"                    ");
				LCD_printfxy(0,2,"  No time data!     ");
				LCD_printfxy(0,3,"                    ");
			}
			else // newdata or processed
			{
				LCD_printfxy(0,1,"                    ");
				LCD_printfxy(0,2,"Date: %02i/%02i/%04i",SystemTime.Month, SystemTime.Day, SystemTime.Year);
				LCD_printfxy(0,3,"Time: %02i:%02i:%02i.%03i", SystemTime.Hour, SystemTime.Min, SystemTime.Sec, SystemTime.mSec);
			}
			break;
		case 4: // Navi Params 1 from FC
			LCD_printfxy(0,0,"NaviMode: %3i" ,  Parameter.NaviGpsModeControl);
            LCD_printfxy(0,1,"G  :%3i P  :%3i ",Parameter.NaviGpsGain, Parameter.NaviGpsP);
            LCD_printfxy(0,2,"I  :%3i D  :%3i ",Parameter.NaviGpsI, Parameter.NaviGpsD);
		    LCD_printfxy(0,3,"ACC:%3i SAT:%3i ",Parameter.NaviGpsACC, Parameter.NaviGpsMinSat);
			break;
	   	case 5: // Navi Params 2 from FC
			LCD_printfxy(0,0,"Stick TS:  %3i", Parameter.NaviStickThreshold);
            LCD_printfxy(0,1,"MaxRadius: %3i m", NaviData.OperatingRadius);
            LCD_printfxy(0,2,"WindCorr:  %3i", Parameter.NaviWindCorrection);
		    LCD_printfxy(0,3,"SpeedComp: %3i", Parameter.NaviSpeedCompensation);
			break;
		case 6: // Navi Params 3 from FC
			LCD_printfxy(0,0,"Angle-Limit: %3i", Parameter.NaviAngleLimitation);
            LCD_printfxy(0,1,"    P-Limit: %3i", Parameter.NaviGpsPLimit);
            LCD_printfxy(0,2,"    I-Limit: %3i", Parameter.NaviGpsILimit);
		    LCD_printfxy(0,3,"    D-Limit: %3i", Parameter.NaviGpsDLimit);
			break;
		case 7:
			LCD_printfxy(0,0,"Home Position");
			if(NaviData.HomePosition.Status == INVALID)
			{
				LCD_printfxy(0,1,"                     ");
				LCD_printfxy(0,2,"    Is not set.      ");
				LCD_printfxy(0,3,"                     ");
			}
			else
			{
				if(NaviData.HomePosition.Longitude < 0) sign = '-';
				else sign = '+';
				i1 = abs(NaviData.HomePosition.Longitude)/10000000L;
				i2 = abs(NaviData.HomePosition.Longitude)%10000000L;
				LCD_printfxy(0,1,"Lon:%c%03ld.%07ld deg",sign, i1, i2);
				if(NaviData.HomePosition.Latitude < 0) sign = '-';
				else sign = '+';
				i1 = abs(NaviData.HomePosition.Latitude)/10000000L;
				i2 = abs(NaviData.HomePosition.Latitude)%10000000L;
				LCD_printfxy(0,2,"Lat:%c%03ld.%07ld deg",sign, i1, i2);
				if(NaviData.HomePosition.Altitude < 0) sign = '-';
				else sign = '+';
				i1 = abs(NaviData.HomePosition.Altitude)/1000L;
				i2 = abs(NaviData.HomePosition.Altitude)%1000L;
				LCD_printfxy(0,3,"Alt:%c%04ld.%03ld m",sign, i1, i2);
			}
			break;
		case 8: // RC stick controls from FC
			LCD_printfxy(0,0,"RC-Sticks" );
            LCD_printfxy(0,1,"Ni:%4i  Ro:%4i ",FC.StickNick, FC.StickRoll);
            LCD_printfxy(0,2,"Gs:%4i  Ya:%4i ",FC.StickGas, FC.StickYaw);
			break;
		case 9: // RC poti controls from FC
			LCD_printfxy(0,0,"RC-Potis 1" );
            LCD_printfxy(0,1,"Po1:%3i  Po2:%3i ",FC.Poti[0], FC.Poti[1]);
            LCD_printfxy(0,2,"Po3:%3i  Po4:%3i ",FC.Poti[2], FC.Poti[3]);
			break;
		case 10: // RC poti controls from FC
			LCD_printfxy(0,0,"RC-Potis 2" );
            LCD_printfxy(0,1,"Po5:%3i  Po6:%3i ",FC.Poti[4], FC.Poti[5]);
            LCD_printfxy(0,2,"Po7:%3i  Po8:%3i ",FC.Poti[6], FC.Poti[7]);
			break;
		case 11: // attitude from FC
			if(FromFlightCtrl.AngleNick < 0) sign = '-';
			else sign = '+';
			i1 = abs(FromFlightCtrl.AngleNick)/10;
			i2 = abs(FromFlightCtrl.AngleNick)%10;
		 	LCD_printfxy(0,0,"GyroNick:%c%03ld.%01ld", sign, i1, i2);
			if(FromFlightCtrl.AngleRoll < 0) sign = '-';
			else sign = '+';
			i1 = abs(FromFlightCtrl.AngleRoll)/10;
			i2 = abs(FromFlightCtrl.AngleRoll)%10;
            LCD_printfxy(0,1,"GyroRoll:%c%03ld.%01ld", sign, i1, i2);
			if(FromFlightCtrl.AccNick < 0) sign = '-';
			else sign = '+';
			i1 = abs(FromFlightCtrl.AccNick)/10;
			i2 = abs(FromFlightCtrl.AccNick)%10;
			LCD_printfxy(0,2," AccNick:%c%03ld.%01ld", sign, i1, i2);
		   	if(FromFlightCtrl.AccRoll < 0) sign = '-';
			else sign = '+';
			i1 = abs(FromFlightCtrl.AccRoll)/10;
			i2 = abs(FromFlightCtrl.AccRoll)%10;
            LCD_printfxy(0,3," AccRoll:%c%03ld.%01ld", sign, i1, i2);
			break;
		case 12: // gyros from FC
		 	LCD_printfxy(0,0,"GyroNick:  %4i", FromFlightCtrl.GyroNick);
            LCD_printfxy(0,1,"GyroRoll:  %4i", FromFlightCtrl.GyroRoll);
			LCD_printfxy(0,2,"GyroYaw:   %4i", FromFlightCtrl.GyroYaw);
			break;
		case 13: // Remote Control Level from FC
            LCD_printfxy(0,0,"RC-Level:    %3i", FC.RC_Quality);
			LCD_printfxy(0,1,"Ubat:        %2i.%1i V", FC.BAT_Voltage/10, FC.BAT_Voltage%10);
			LCD_printfxy(0,2,"CompHeading: %3i", Compass_Heading);
			if(GeoMagDec < 0) sign = '-';
			else sign = '+';
			LCD_printfxy(0,3,"GeoMagDec:  %c%i.%1i", sign, abs(GeoMagDec)/10,abs(GeoMagDec)%10);
            break;
		case 14: // User Parameter
			LCD_printfxy(0,0,"UP1:%3i  UP2:%3i",Parameter.User1,Parameter.User2);
			LCD_printfxy(0,1,"UP3:%3i  UP4:%3i",Parameter.User3,Parameter.User4);
			LCD_printfxy(0,2,"UP5:%3i  UP6:%3i",Parameter.User5,Parameter.User6);
			LCD_printfxy(0,3,"UP7:%3i  UP8:%3i",Parameter.User7,Parameter.User8);
			break;
		case 15: // magnetic field
			if(Compass_CalState)
			{
				LCD_printfxy(0,0,"Calibration:");
				LCD_printfxy(0,1,"Step %d/", Compass_CalState);
			   	LCD_printfxy(0,2,"X %4i Y %4i Z %4i",MagVector.X,MagVector.Y,MagVector.Z);
				LCD_printfxy(9,3,"(ESC)(NEXT)");
				switch(Compass_CalState)
				{
					case 1:
					case 3:
						LCD_printfxy(7,1,"pause");
						break;

					case 2:
						LCD_printfxy(7,1,"horizontal");
						break;

					case 4:
						LCD_printfxy(7,1,"vertical");
						break;

					case 5:
						LCD_printfxy(7,1,"data saved");
						LCD_printfxy(8,3,"      (END) ");
						break;
					
					default:
						break;
				}
			}
			else
			{
				LCD_printfxy(0,0,"Magnetic Field");
			 	LCD_printfxy(0,1,"X:%5i",MagVector.X);
				LCD_printfxy(0,2,"Y:%5i",MagVector.Y);
				LCD_printfxy(0,3,"Z:%5i",MagVector.Z);
				LCD_printfxy(15,3,"(CAL)");
			}
			if(Keys & KEY4) //  next step
			{
				if(Compass_CalState <5) Compass_SetCalState(Compass_CalState+1);
				else Compass_SetCalState(0);
			}
			if(Keys & KEY3)Compass_SetCalState(0);  // cancel
			break;
		default:
			//MaxMenuItem = MenuItem - 1;
			MenuItem = 0;
			break;
    }
}
