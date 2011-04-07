#ifndef _MYMATH_H
#define _MYMATH_H

// trigonometry resolution of angle is 1 deg
s16 c_sin_8192(s16 angle);
s16 c_cos_8192(s16 angle);

// higher resolution trigonometry with angle in deg is arg/div
s16 c_sin_8192_res(s16 arg, s16 div);
s16 c_cos_8192_res(s16 arg, s16 div);

// fast arctan implementation
s32 c_atan2_546(s32 y, s32 x);

#endif // _MYMATH_H
