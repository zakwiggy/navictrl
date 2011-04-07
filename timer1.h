#ifndef _TIMER1_H
#define _TIMER1_H


typedef struct{
	u16	Year;
	u8	Month;
	u8	Day;
	u8	Hour;
	u8	Min;
	u8	Sec;
	u16 mSec;
	u8  Valid;
}  DateTime_t;

extern DateTime_t SystemTime;

extern u32 CountMilliseconds;

void TIMER1_Init(void);
u32 SetDelay(u32 t);
u32 GetDelay(u32 t);
u8 CheckDelay(u32 t);
void Delay_ms(u32 w);



#endif // _TIMER1_H
