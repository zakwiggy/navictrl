#ifndef _TIMER2_H
#define _TIMER2_H

typedef struct
{
	u8 Refresh;
	u8 CompInvert; 
	u8 NickControl;
	u8 NickComp;
	u8 NickMin; 
	u8 NickMax;
	u8 RollControl;
	u8 RollComp;
	u8 RollMin; 
	u8 RollMax;      	
} __attribute__((packed)) ServoParams_t;


extern volatile ServoParams_t ServoParams;

void TIMER2_Init(void);
void TIMER2_Deinit(void);

#endif	// _TIMER2_H
