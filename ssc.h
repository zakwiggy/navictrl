#ifndef _SSC_H
#define _SSC_H


//________________________________________________________________________________________________________________________________________
// 
// Functions needed for accessing the sdcard low level via SPI.
//				
//________________________________________________________________________________________________________________________________________

#define SD_SWITCH !(GPIO_ReadBit(GPIO5, GPIO_Pin_3))

void SSC_Init(void);
void SSC_Deinit(void);
u8 	 SSC_GetChar (void);
void SSC_PutChar (u8);
void SSC_Enable(void);
void SSC_Disable(void);	 
void SSC_ClearRxFifo(void);


#endif //_SSC_H
