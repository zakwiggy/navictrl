#ifndef __NCMAG_H
#define __NCMAG_H

#include "compass.h" 

extern s16vec_t ACC_Vector;

u8 NCMAG_Init(void);
void NCMAG_Update(void);

#endif // __NCMAG_H

