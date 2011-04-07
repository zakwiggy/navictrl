#ifndef _ANALOG_H
#define _ANALOG_H

typedef struct
{
	u16 Ch1;
	u16 Ch2;
	u16 Ch3;
	u16 Ch4;
	u16 Ch5;
	u16 Ch6;
	u16 Ch7;
} __attribute__((packed)) AnalogData_t;

extern AnalogData_t AnalogData;

void Analog_Init(void);
void Analog_Deinit(void);
void Analog_Update(void);

#endif	// _ANALOG_H
