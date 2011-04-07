/*#######################################################################################*/
/* !!! THIS IS NOT FREE SOFTWARE !!!  	                                                 */
/*#######################################################################################*/
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) 2010 Ingo Busker, Holger Buss
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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "91x_lib.h"
#include "ncmag.h"
#include "i2c.h"
#include "timer1.h"
#include "led.h"
#include "uart1.h"
#include "eeprom.h"
#include "mymath.h"
#include "main.h"

u8 NCMAG_Present = 0;
u8 NCMAG_IsCalibrated = 0;

#define MAG_TYPE_NONE		0
#define MAG_TYPE_HMC5843 	1
#define MAG_TYPE_LSM303DLH	2
u8 NCMAG_MagType = MAG_TYPE_NONE;

#define CALIBRATION_VERSION 1
#define EEPROM_ADR_MAG_CALIBRATION 50

#define NCMAG_MIN_RAWVALUE -2047
#define NCMAG_MAX_RAWVALUE  2047
#define NCMAG_INVALID_DATA -4096

typedef struct
{
	s16 Range;
	s16 Offset;
} __attribute__((packed)) Scaling_t;

typedef struct
{
	Scaling_t MagX;
	Scaling_t MagY;
	Scaling_t MagZ;
	u8 Version;
	u8 crc;
} __attribute__((packed)) Calibration_t;

Calibration_t Calibration;         	// calibration data in RAM 

// i2c MAG interface
#define MAG_SLAVE_ADDRESS 	0x3C	// i2C slave address mag. sensor registers

// register mapping
#define REG_MAG_CRA			0x00
#define REG_MAG_CRB			0x01
#define REG_MAG_MODE		0x02
#define REG_MAG_DATAX_MSB	0x03
#define REG_MAG_DATAX_LSB	0x04
#define REG_MAG_DATAY_MSB	0x05
#define REG_MAG_DATAY_LSB	0x06
#define REG_MAG_DATAZ_MSB	0x07
#define REG_MAG_DATAZ_LSB	0x08
#define REG_MAG_STATUS		0x09
#define REG_MAG_IDA			0x0A
#define REG_MAG_IDB			0x0B
#define REG_MAG_IDC			0x0C

// bit mask for configuration mode
#define CRA_MODE_MASK		0x03
#define CRA_MODE_NORMAL		0x00	//default
#define CRA_MODE_POSBIAS	0x01
#define CRA_MODE_NEGBIAS	0x02
#define CRA_MODE_SELFTEST	0x03

// bit mask for measurement mode
#define MODE_MASK			0xFF
#define MODE_CONTINUOUS		0x00
#define MODE_SINGLE			0x01	// default
#define MODE_IDLE			0x02
#define MODE_SLEEP			0x03

// bit mask for rate
#define CRA_RATE_MASK		0x1C

// bit mask for gain
#define CRB_GAIN_MASK		0xE0

// ids
#define MAG_IDA		0x48
#define MAG_IDB		0x34
#define MAG_IDC		0x33

// the special HMC5843 interface
// bit mask for rate
#define HMC5843_CRA_RATE_0_5HZ		0x00
#define HMC5843_CRA_RATE_1HZ		0x04
#define HMC5843_CRA_RATE_2HZ		0x08
#define HMC5843_CRA_RATE_5HZ		0x0C
#define HMC5843_CRA_RATE_10HZ		0x10 	//default
#define HMC5843_CRA_RATE_20HZ		0x14
#define HMC5843_CRA_RATE_50HZ		0x18
// bit mask for gain
#define HMC5843_CRB_GAIN_07GA		0x00
#define HMC5843_CRB_GAIN_10GA		0x20	//default
#define HMC5843_CRB_GAIN_15GA		0x40
#define HMC5843_CRB_GAIN_20GA		0x60
#define HMC5843_CRB_GAIN_32GA		0x80
#define HMC5843_CRB_GAIN_38GA		0xA0
#define HMC5843_CRB_GAIN_45GA		0xC0
#define HMC5843_CRB_GAIN_65GA		0xE0
// self test value
#define HMC5843_TEST_XSCALE		715
#define HMC5843_TEST_YSCALE		715
#define HMC5843_TEST_ZSCALE		715


// the special LSM302DLH interface
// bit mask for rate
#define LSM303DLH_CRA_RATE_0_75HZ	0x00
#define LSM303DLH_CRA_RATE_1_5HZ	0x04
#define LSM303DLH_CRA_RATE_3_0HZ	0x08
#define LSM303DLH_CRA_RATE_7_5HZ	0x0C
#define LSM303DLH_CRA_RATE_15HZ		0x10 	//default
#define LSM303DLH_CRA_RATE_30HZ		0x14
#define LSM303DLH_CRA_RATE_75HZ		0x18
// bit mask for gain
#define LSM303DLH_CRB_GAIN_XXGA		0x00
#define LSM303DLH_CRB_GAIN_13GA		0x20	//default
#define LSM303DLH_CRB_GAIN_19GA		0x40
#define LSM303DLH_CRB_GAIN_25GA		0x60
#define LSM303DLH_CRB_GAIN_40GA		0x80
#define LSM303DLH_CRB_GAIN_47GA		0xA0
#define LSM303DLH_CRB_GAIN_56GA		0xC0
#define LSM303DLH_CRB_GAIN_81GA		0xE0
// self test value
#define LSM303DLH_TEST_XSCALE	655
#define LSM303DLH_TEST_YSCALE	655
#define LSM303DLH_TEST_ZSCALE	630

// the i2c ACC interface
#define ACC_SLAVE_ADDRESS		0x30	// i2c slave for acc. sensor registers
// register mapping
#define REG_ACC_CTRL1 			0x20
#define REG_ACC_CTRL2 			0x21
#define REG_ACC_CTRL3 			0x22
#define REG_ACC_CTRL4 			0x23
#define REG_ACC_CTRL5 			0x24
#define REG_ACC_HP_FILTER_RESET 0x25
#define REG_ACC_REFERENCE		0x26
#define REG_ACC_STATUS			0x27
#define REG_ACC_X_LSB			0x28
#define REG_ACC_X_MSB			0x29
#define REG_ACC_Y_LSB			0x2A
#define REG_ACC_Y_MSB			0x2B
#define REG_ACC_Z_LSB			0x2C
#define REG_ACC_Z_MSB			0x2D



typedef struct
{
	u8 A;
	u8 B;
	u8 C;
} __attribute__((packed)) Identification_t;

volatile Identification_t NCMAG_Identification;

typedef struct
{
	u8 cra;
	u8 crb;
	u8 mode;
} __attribute__((packed)) MagConfig_t;

volatile MagConfig_t MagConfig;

typedef struct
{
	u8 ctrl_1;
	u8 ctrl_2;
	u8 ctrl_3;
	u8 ctrl_4;
	u8 ctrl_5;
} __attribute__((packed)) AccConfig_t;

volatile AccConfig_t AccConfig;

volatile s16vec_t AccRawVector;
volatile s16vec_t MagRawVector;


u8 NCMag_CalibrationWrite(void)
{
	u8 i, crc = 0xAA;
	EEPROM_Result_t eres;
	u8 *pBuff = (u8*)&Calibration;

	Calibration.Version = CALIBRATION_VERSION;
	for(i = 0; i<(sizeof(Calibration)-1); i++)
	{
		crc += pBuff[i]; 	
	}
	Calibration.crc = ~crc;
	eres = EEPROM_WriteBlock(EEPROM_ADR_MAG_CALIBRATION, pBuff, sizeof(Calibration));
	if(EEPROM_SUCCESS == eres) i = 1;
	else i = 0;
	return(i);	
}

u8 NCMag_CalibrationRead(void)
{
	u8 i, crc = 0xAA;
	u8 *pBuff = (u8*)&Calibration;

	if(EEPROM_SUCCESS == EEPROM_ReadBlock(EEPROM_ADR_MAG_CALIBRATION, pBuff, sizeof(Calibration)))
	{
		for(i = 0; i<(sizeof(Calibration)-1); i++)
		{
			crc += pBuff[i]; 	
		}
		crc = ~crc;
		if(Calibration.crc != crc) return(0); // crc mismatch
		if(Calibration.Version == CALIBRATION_VERSION) return(1);
	}
	return(0);
}


void NCMAG_Calibrate(void)
{
	static s16 Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0, Zmin = 0, Zmax = 0;
	static s16 X = 0, Y = 0, Z = 0;
	static u8 OldCalState = 0;	

	X = (4*X + MagRawVector.X + 3)/5;
	Y = (4*Y + MagRawVector.Y + 3)/5;
	Z = (4*Z + MagRawVector.Z + 3)/5;

	switch(Compass_CalState)
	{
		case 1:
			// 1st step of calibration
			// initialize ranges
			// used to change the orientation of the NC in the horizontal plane
			Xmin =  10000;
			Xmax = -10000;
			Ymin =  10000;
			Ymax = -10000;
			Zmin =  10000;
			Zmax = -10000;
			break;
		
		case 2: // 2nd step of calibration
			// find Min and Max of the X- and Y-Sensors during rotation in the horizontal plane
			if(X < Xmin) 		{ Xmin = X; BeepTime = 20;}
			else if(X > Xmax) 	{ Xmax = X; BeepTime = 20;}
			if(Y < Ymin) 		{ Ymin = Y; BeepTime = 60;}
			else if(Y > Ymax) 	{ Ymax = Y; BeepTime = 60;}
			break;

		case 3: // 3rd step of calibration
			// used to change the orientation of the MK3MAG vertical to the horizontal plane
			break;

		case 4:
			// find Min and Max of the Z-Sensor
		  	if(Z < Zmin) 	  { Zmin = Z; BeepTime = 80;}
			else if(Z > Zmax) { Zmax = Z; BeepTime = 80;}
			break;
		
		case 5:
			// Save values
			if(Compass_CalState != OldCalState) // avoid continously writing of eeprom!
			{
				#define MIN_CALIBRATION    256
				Calibration.MagX.Range = Xmax - Xmin;
				Calibration.MagX.Offset = (Xmin + Xmax) / 2;
				Calibration.MagY.Range = Ymax - Ymin;
				Calibration.MagY.Offset = (Ymin + Ymax) / 2;
				Calibration.MagZ.Range = Zmax - Zmin;
				Calibration.MagZ.Offset = (Zmin + Zmax) / 2;
				if((Calibration.MagX.Range > MIN_CALIBRATION) && (Calibration.MagY.Range > MIN_CALIBRATION) && (Calibration.MagZ.Range > MIN_CALIBRATION))
				{
					NCMAG_IsCalibrated = NCMag_CalibrationWrite();
					BeepTime = 2500;
					UART1_PutString("\r\n Calibration okay");
				}
				else
				{
					// restore old calibration data from eeprom
					NCMAG_IsCalibrated = NCMag_CalibrationRead();
					UART1_PutString("\r\n Calibration FAILED - Values too low: ");
					if(Calibration.MagX.Range < MIN_CALIBRATION) UART1_PutString("X! ");
					if(Calibration.MagY.Range < MIN_CALIBRATION) UART1_PutString("Y! ");
					if(Calibration.MagZ.Range < MIN_CALIBRATION) UART1_PutString("Z! ");
				}
			}
			break;
			
		default:
			break;	
	}
	OldCalState = Compass_CalState;
}

// ---------- call back handlers -----------------------------------------

// rx data handler for id info request
void NCMAG_UpdateIdentification(u8* pRxBuffer, u8 RxBufferSize)
{	// if number of bytes are matching
	if(RxBufferSize == sizeof(NCMAG_Identification) )
	{
		memcpy((u8 *)&NCMAG_Identification, pRxBuffer, sizeof(NCMAG_Identification));
	}
}
// rx data handler for magnetic sensor raw data
void NCMAG_UpdateMagVector(u8* pRxBuffer, u8 RxBufferSize)
{	// if number of bytes are matching
	if(RxBufferSize == sizeof(MagRawVector) )
	{	// byte order from big to little endian
		s16 raw;
		raw = pRxBuffer[0]<<8;
		raw+= pRxBuffer[1];
		if(raw >= NCMAG_MIN_RAWVALUE && raw <= NCMAG_MAX_RAWVALUE) MagRawVector.X = raw; 
		raw = pRxBuffer[2]<<8;
		raw+= pRxBuffer[3];
		if(raw >= NCMAG_MIN_RAWVALUE && raw <= NCMAG_MAX_RAWVALUE) MagRawVector.Y = raw;
		raw = pRxBuffer[4]<<8;
		raw+= pRxBuffer[5];
		if(raw >= NCMAG_MIN_RAWVALUE && raw <= NCMAG_MAX_RAWVALUE) MagRawVector.Z = raw;
	}
	if(Compass_CalState || !NCMAG_IsCalibrated)
	{	// mark out data invalid
		MagVector.X = MagRawVector.X;
		MagVector.Y = MagRawVector.Y;
		MagVector.Z = MagRawVector.Z;
		Compass_Heading = -1;
	}
	else
	{
		// update MagVector from MagRaw Vector by Scaling
		MagVector.X = (s16)((1024L*(s32)(MagRawVector.X - Calibration.MagX.Offset))/Calibration.MagX.Range);
		MagVector.Y = (s16)((1024L*(s32)(MagRawVector.Y - Calibration.MagY.Offset))/Calibration.MagY.Range);
		MagVector.Z = (s16)((1024L*(s32)(MagRawVector.Z - Calibration.MagZ.Offset))/Calibration.MagZ.Range);
		Compass_CalcHeading();
	}
}
// rx data handler  for acceleration raw data
void NCMAG_UpdateAccVector(u8* pRxBuffer, u8 RxBufferSize)
{	// if number of byte are matching
	if(RxBufferSize == sizeof(AccRawVector) )
	{ 
		memcpy((u8*)&AccRawVector, pRxBuffer,sizeof(AccRawVector));
	}
}
// rx data handler for reading magnetic sensor configuration
void NCMAG_UpdateMagConfig(u8* pRxBuffer, u8 RxBufferSize)
{	// if number of byte are matching
	if(RxBufferSize == sizeof(MagConfig) )
	{
		memcpy((u8*)(&MagConfig), pRxBuffer, sizeof(MagConfig));
	}
}
// rx data handler for reading acceleration sensor configuration
void NCMAG_UpdateAccConfig(u8* pRxBuffer, u8 RxBufferSize)
{	// if number of byte are matching
	if(RxBufferSize == sizeof(AccConfig) )
	{
		memcpy((u8*)&AccConfig, pRxBuffer, sizeof(AccConfig));
	}
}
//----------------------------------------------------------------------


// ---------------------------------------------------------------------
u8 NCMAG_SetMagConfig(void)
{
	u8 retval = 0;
	// try to catch the i2c buffer within 100 ms timeout
	if(I2C_LockBuffer(100))
	{
		u8 TxBytes = 0;
		I2C_Buffer[TxBytes++] = REG_MAG_CRA; 	
		memcpy((u8*)(&I2C_Buffer[TxBytes]), (u8*)&MagConfig, sizeof(MagConfig));
		TxBytes += sizeof(MagConfig);
		if(I2C_Transmission(MAG_SLAVE_ADDRESS, TxBytes, 0, 0))
		{
		 	if(I2C_WaitForEndOfTransmission(100))
			{
			 	if(I2C_Error == I2C_ERROR_NONE) retval = 1;
			}
		}
	}
	return(retval);		
}

// ----------------------------------------------------------------------------------------
u8 NCMAG_GetMagConfig(void)
{
	u8 retval = 0;
	// try to catch the i2c buffer within 100 ms timeout
	if(I2C_LockBuffer(100))
	{
		u8 TxBytes = 0;
		I2C_Buffer[TxBytes++] = REG_MAG_CRA; 
		if(I2C_Transmission(MAG_SLAVE_ADDRESS, TxBytes, &NCMAG_UpdateMagConfig, sizeof(MagConfig)))
		{
		 	if(I2C_WaitForEndOfTransmission(100))
			{
			 	if(I2C_Error == I2C_ERROR_NONE) retval = 1;
			}
		}
	}
	return(retval);		
}

// ----------------------------------------------------------------------------------------
u8 NCMAG_SetAccConfig(void)
{
	u8 retval = 0;
	// try to catch the i2c buffer within 100 ms timeout
	if(I2C_LockBuffer(100))
	{
		u8 TxBytes = 0;
		I2C_Buffer[TxBytes++] = REG_ACC_CTRL1; 	
		memcpy((u8*)(&I2C_Buffer[TxBytes]), (u8*)&AccConfig, sizeof(AccConfig));
		TxBytes += sizeof(AccConfig);
		if(I2C_Transmission(ACC_SLAVE_ADDRESS, TxBytes, 0, 0))
		{
		 	if(I2C_WaitForEndOfTransmission(100))
			{
			 	if(I2C_Error == I2C_ERROR_NONE) retval = 1;
			}
		}
	}
	return(retval);		
}

// ----------------------------------------------------------------------------------------
u8 NCMAG_GetAccConfig(void)
{
	u8 retval = 0;
	// try to catch the i2c buffer within 100 ms timeout
	if(I2C_LockBuffer(100))
	{
		u8 TxBytes = 0;
		I2C_Buffer[TxBytes++] = REG_ACC_CTRL1; 
		if(I2C_Transmission(ACC_SLAVE_ADDRESS, TxBytes, &NCMAG_UpdateAccConfig, sizeof(AccConfig)))
		{
		 	if(I2C_WaitForEndOfTransmission(100))
			{
			 	if(I2C_Error == I2C_ERROR_NONE) retval = 1;
			}
		}
	}
	return(retval);		
}

// ----------------------------------------------------------------------------------------
u8 NCMAG_GetIdentification(void)
{
	u8 retval = 0;
	// try to catch the i2c buffer within 100 ms timeout
	if(I2C_LockBuffer(100))
	{
		u16 TxBytes = 0;
		NCMAG_Identification.A = 0xFF;
		NCMAG_Identification.B = 0xFF;
		NCMAG_Identification.C = 0xFF;
	  	I2C_Buffer[TxBytes++] = REG_MAG_IDA;
		// initiate transmission
		if(I2C_Transmission(MAG_SLAVE_ADDRESS, TxBytes, &NCMAG_UpdateIdentification, sizeof(NCMAG_Identification)))
		{
		 	if(I2C_WaitForEndOfTransmission(100))
			{
			 	if(I2C_Error == I2C_ERROR_NONE) retval = 1;
			}
		}
	}
	return(retval);
}

// ----------------------------------------------------------------------------------------
void NCMAG_GetMagVector(void)
{
	// try to catch the I2C buffer within 0 ms
	if(I2C_LockBuffer(0))
	{
		u16 TxBytes = 0;
		// set register pointer
	  	I2C_Buffer[TxBytes++] = REG_MAG_DATAX_MSB;
		// initiate transmission
		I2C_Transmission(MAG_SLAVE_ADDRESS, TxBytes, &NCMAG_UpdateMagVector, sizeof(MagVector));
	}
}

//----------------------------------------------------------------
void NCMAG_GetAccVector(void)
{
	// try to catch the I2C buffer within 0 ms
	if(I2C_LockBuffer(0))
	{
		u16 TxBytes = 0;
		// set register pointer
	  	I2C_Buffer[TxBytes++] = REG_ACC_X_LSB;
		// initiate transmission
		I2C_Transmission(ACC_SLAVE_ADDRESS, TxBytes, &NCMAG_UpdateAccVector, sizeof(AccRawVector));
	}
}

// --------------------------------------------------------
void NCMAG_Update(void)
{
	static u32 TimerUpdate = 0;

	if( (I2C_State == I2C_STATE_OFF) || !NCMAG_Present )
	{
		Compass_Heading = -1;
		return;
	}

	if(CheckDelay(TimerUpdate))
	{
		// check for new calibration state
		Compass_UpdateCalState();
		if(Compass_CalState) NCMAG_Calibrate();
		NCMAG_GetMagVector(); //Get new data;
		TimerUpdate = SetDelay(20);    // every 20 ms are 50 Hz
	}
}

// --------------------------------------------------------
u8 NCMAG_SelfTest(void)
{
	u8 msg[64];
	static u8 done = 0;

	if(done) return(1);	   // just make it once
	
	#define LIMITS(value, min, max) {min = (80 * value)/100; max = (120 * value)/100;}
	u32 time;
	s32 XMin = 0, XMax = 0, YMin = 0, YMax = 0, ZMin = 0, ZMax = 0;
	s16 xscale, yscale, zscale, scale_min, scale_max;
	u8 crb_gain, cra_rate;
 	u8 i = 0, retval = 1;

	switch(NCMAG_MagType)
	{
		case MAG_TYPE_HMC5843:
			crb_gain = HMC5843_CRB_GAIN_10GA;
			cra_rate = HMC5843_CRA_RATE_50HZ;
			xscale = HMC5843_TEST_XSCALE;
			yscale = HMC5843_TEST_YSCALE;
			zscale = HMC5843_TEST_ZSCALE;
			break;

		case MAG_TYPE_LSM303DLH:
			crb_gain = LSM303DLH_CRB_GAIN_13GA;
			cra_rate = LSM303DLH_CRA_RATE_75HZ;
			xscale = LSM303DLH_TEST_XSCALE;
			yscale = LSM303DLH_TEST_YSCALE;
			zscale = LSM303DLH_TEST_ZSCALE;
			break;

		default:
		return(0);
	}

	MagConfig.cra = cra_rate|CRA_MODE_POSBIAS;
	MagConfig.crb = crb_gain;
	MagConfig.mode = MODE_CONTINUOUS;
	// activate positive bias field
	NCMAG_SetMagConfig();
	// wait for stable readings
	time = SetDelay(50);
	while(!CheckDelay(time));
	// averaging
	#define AVERAGE 20
	for(i = 0; i<AVERAGE; i++)
	{
		NCMAG_GetMagVector();
		time = SetDelay(20);
    	while(!CheckDelay(time));
		XMax += MagRawVector.X;
		YMax += MagRawVector.Y;
		ZMax += MagRawVector.Z;
	}
	MagConfig.cra = cra_rate|CRA_MODE_NEGBIAS;
	// activate positive bias field
	NCMAG_SetMagConfig();
    // wait for stable readings
	time = SetDelay(50);
	while(!CheckDelay(time));
	// averaging
	for(i = 0; i < AVERAGE; i++)
	{
		NCMAG_GetMagVector();
		time = SetDelay(20);
    	while(!CheckDelay(time));
		XMin += MagRawVector.X;
		YMin += MagRawVector.Y;
		ZMin += MagRawVector.Z;
	}
	// setup final configuration
	MagConfig.cra = cra_rate|CRA_MODE_NORMAL;
	// activate positive bias field
	NCMAG_SetMagConfig();
	// check scale for all axes
	// prepare scale limits
	LIMITS(xscale, scale_min, scale_max);
	xscale = (XMax - XMin)/(2*AVERAGE);
	if((xscale > scale_max) || (xscale < scale_min)) 
     {
	  retval = 0;
      sprintf(msg, "\r\n Value X: %d not %d-%d !", xscale, scale_min,scale_max);
	  UART1_PutString(msg);
     }
	LIMITS(yscale, scale_min, scale_max);
	yscale = (YMax - YMin)/(2*AVERAGE);
	if((yscale > scale_max) || (yscale < scale_min)) 
     {
	  retval = 0;
      sprintf(msg, "\r\n Value Y: %d not %d-%d !", yscale, scale_min,scale_max);
	  UART1_PutString(msg);
     }
	LIMITS(zscale, scale_min, scale_max);
	zscale = (ZMax - ZMin)/(2*AVERAGE);
	if((zscale > scale_max) || (zscale < scale_min))      
	 {
	  retval = 0;
      sprintf(msg, "\r\n Value Z: %d not %d-%d !", zscale, scale_min,scale_max);
	  UART1_PutString(msg);
     }
	done = retval;
	return(retval);
}


//----------------------------------------------------------------
u8 NCMAG_Init(void)
{
	u8 msg[64];
	u8 retval = 0;
	u8 repeat;

	NCMAG_Present = 0;
	NCMAG_MagType = MAG_TYPE_HMC5843;	// assuming having an HMC5843
	// polling for LSM302DLH option
	repeat = 0;
	do
	{
		retval = NCMAG_GetAccConfig();
		if(retval) break; // break loop on success
		UART1_PutString(".");
		repeat++;
	}while(repeat < 3);
	if(retval) NCMAG_MagType = MAG_TYPE_LSM303DLH; // must be a LSM303DLH
	// polling of identification
	repeat = 0;
	do
	{
		retval = NCMAG_GetIdentification();
		if(retval) break; // break loop on success
		UART1_PutString(".");
		repeat++;
	}while(repeat < 12);
	// if we got an answer to id request
   	if(retval)
	{
		u8 n1[] = "HMC5843";
		u8 n2[] = "LSM303DLH";
		u8* pn;
		if(NCMAG_MagType == MAG_TYPE_LSM303DLH) pn = n2;
		else pn = n1;
		sprintf(msg, " %s ID%d/%d/%d", pn, NCMAG_Identification.A, NCMAG_Identification.B, NCMAG_Identification.C);
		UART1_PutString(msg);
		if (    (NCMAG_Identification.A == MAG_IDA)
		     && (NCMAG_Identification.B == MAG_IDB)
			 && (NCMAG_Identification.C == MAG_IDC))
		{
			NCMAG_Present = 1;
			if(!NCMAG_SelfTest())
			{
				UART1_PutString(" Selftest failed!");
				LED_RED_ON;
				NCMAG_IsCalibrated = 0;
			}
			else
			{
				if(EEPROM_Init())
				{
					NCMAG_IsCalibrated = NCMag_CalibrationRead();
					if(!NCMAG_IsCalibrated) UART1_PutString("\r\n Not calibrated!");
				}
				else UART1_PutString("\r\n Calibration data not available!");
			}
		}
		else
		{
		 	UART1_PutString("\n\r Not compatible!");
			UART_VersionInfo.HardwareError[0] |= NC_ERROR0_COMPASS_INCOMPATIBLE;
			LED_RED_ON;
		}
	}
	else // nothing found
	{
		NCMAG_MagType = MAG_TYPE_NONE;
		UART1_PutString("not found!"); 	
	}
	return(NCMAG_Present);
}

