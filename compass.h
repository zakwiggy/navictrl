#ifndef _COMPASS_H
#define _COMPASS_H

typedef struct
{
  s16 X;
  s16 Y;
  s16 Z;
} __attribute__((packed)) s16vec_t;

extern volatile s16vec_t MagVector;	  // current magnetic field vector
extern volatile s16 Compass_Heading;  // current heading direction
extern volatile u8  Compass_CalState; // current calibration state

#define COMPASS_NONE	0
#define COMPASS_MK3MAG	1
#define COMPASS_NCMAG	2
extern u8 Compass_Device;

void Compass_Init(void);
void Compass_Update(void);
void Compass_CalcHeading(void);
void Compass_SetCalState(u8 CalState);
void Compass_UpdateCalState(void);

#endif	//  _COMPASS_H
