#ifndef _LED_H
#define _LED_H

extern u8 Led_Grn_Inv;
extern u8 Led_Red_Inv;

#define LED_GRN_ON	{if(Led_Grn_Inv) GPIO_WriteBit(GPIO5, GPIO_Pin_6, Bit_RESET); else GPIO_WriteBit(GPIO5, GPIO_Pin_6, Bit_SET);}
#define LED_GRN_OFF {if(Led_Grn_Inv) GPIO_WriteBit(GPIO5, GPIO_Pin_6, Bit_SET);   else GPIO_WriteBit(GPIO5, GPIO_Pin_6, Bit_RESET);}
#define LED_GRN_TOGGLE 	if (GPIO_ReadBit(GPIO5, GPIO_Pin_6)) GPIO_WriteBit(GPIO5, GPIO_Pin_6, Bit_RESET); else GPIO_WriteBit(GPIO5, GPIO_Pin_6, Bit_SET);

#define LED_RED_ON	{if(Led_Red_Inv) GPIO_WriteBit(GPIO5, GPIO_Pin_7, Bit_RESET); else GPIO_WriteBit(GPIO5, GPIO_Pin_7, Bit_SET);}
#define LED_RED_OFF {if(Led_Red_Inv) GPIO_WriteBit(GPIO5, GPIO_Pin_7, Bit_SET);   else GPIO_WriteBit(GPIO5, GPIO_Pin_7, Bit_RESET);}
#define LED_RED_TOGGLE 	if (GPIO_ReadBit(GPIO5, GPIO_Pin_7)) GPIO_WriteBit(GPIO5, GPIO_Pin_7, Bit_RESET); else GPIO_WriteBit(GPIO5, GPIO_Pin_7, Bit_SET);

extern u8 Version_HW;
void Led_Init(void);

#endif //_LED_H
