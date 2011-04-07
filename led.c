#include "91x_lib.h"
#include "led.h"

u8 Version_HW = 0;
u8 Led_Grn_Inv = 0;
u8 Led_Red_Inv = 0;

void Led_Init(void)
{
	u8 p5;
	u16 i;
	GPIO_InitTypeDef GPIO_InitStructure;

	SCU_APBPeriphClockConfig(__GPIO5, ENABLE); // Enable the GPIO5 Clock
	
	/*Configure LED_GRN at pin GPIO5.6 and LED_RED at pin GPIO5.7 as output*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
	GPIO_InitStructure.GPIO_Pin = 		GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Type = 		GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt1;
	GPIO_Init(GPIO5, &GPIO_InitStructure);
	// set both ports low
	GPIO_WriteBit(GPIO5, GPIO_Pin_6, Bit_RESET);
	GPIO_WriteBit(GPIO5, GPIO_Pin_7, Bit_RESET);

	// Check LED Polarity
	/*Configure LED_GRN at pin GPIO5.6 and LED_RED at pin GPIO5.7 as input*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinInput;
	GPIO_InitStructure.GPIO_Pin = 		GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Type = 		GPIO_Type_PushPull;
	GPIO_InitStructure.GPIO_Alternate = GPIO_InputAlt1;
	GPIO_Init(GPIO5, &GPIO_InitStructure);
    
	// get polarity of LED ports
	for(i=0;i<500;i++) p5 = GPIO_Read(GPIO5);
	Led_Grn_Inv = 0x01 & (p5>>6);
	Led_Red_Inv = 0x01 & (p5>>7);

	/*Configure LED_GRN at pin GPIO5.6 and LED_RED at pin GPIO5.7 as output*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
	GPIO_InitStructure.GPIO_Pin = 		GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Type = 		GPIO_Type_PushPull ;
	GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt1;
	GPIO_Init(GPIO5, &GPIO_InitStructure);

	LED_GRN_OFF;
	LED_RED_OFF;

	if(Led_Grn_Inv)
	{
		if(Led_Red_Inv) Version_HW = 30; //future use
		else			Version_HW = 20; // NC 2.0	
	}
	else
	{
	 	if(Led_Red_Inv)	Version_HW = 40; // future use
		else			Version_HW = 11; // NC 1.x
	}

}
