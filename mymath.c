#include "91x_lib.h"
#include "mymath.h"

// discrete mathematics

// sinus with argument in degree at an angular resolution of 1 degree and a discretisation of 13 bit.
const s16 sinlookup[91] = {0, 143, 286, 429, 571, 714, 856, 998, 1140, 1282, 1423, 1563, 1703, 1843, 1982, 2120, 2258, 2395, 2531, 2667, 2802, 2936, 3069, 3201, 3332, 3462, 3591, 3719, 3846, 3972, 4096, 4219, 4341, 4462, 4581, 4699, 4815, 4930, 5043, 5155, 5266, 5374, 5482, 5587, 5691, 5793, 5893, 5991, 6088, 6183, 6275, 6366, 6455, 6542, 6627, 6710, 6791, 6870, 6947, 7022, 7094, 7165, 7233, 7299, 7363, 7424, 7484, 7541, 7595, 7648, 7698, 7746, 7791, 7834, 7875, 7913, 7949, 7982, 8013, 8041, 8068, 8091, 8112, 8131, 8147, 8161, 8172, 8181, 8187, 8191, 8192};

s16 c_sin_8192(s16 angle)
{
	s8 m,n;
	s16 sinus;

	// avoid negative angles
	if (angle < 0)
	{
		m = -1;
		angle = -angle;
	}
	else m = +1;

	// fold angle to interval 0 to 359
	angle %= 360;

	// check quadrant
	if (angle <= 90) n = 1; // first quadrant
	else if ((angle > 90) && (angle <= 180)) {angle = 180 - angle; n = 1;} // second quadrant
	else if ((angle > 180) && (angle <= 270)) {angle = angle - 180; n = -1;} // third quadrant
	else {angle = 360 - angle; n = -1;}	//fourth quadrant
	// get lookup value
	sinus = sinlookup[angle];
	// calculate sinus value
	return (sinus * m * n);
}

// cosinus with argument in degree at an angular resolution of 1 degree and a discretisation of 13 bit.
s16 c_cos_8192(s16 angle)
{
	return (c_sin_8192(90 - angle));
}

// higher resolution angle in deg is arg/div
s16 c_sin_8192_res(s16 arg, s16 div)
{
	s16 angle, rest;
	s32 tmp;

	angle = arg/div;
	rest  = arg%div;

	if(rest>0)
	{
		tmp = (div-rest)*(s32)c_sin_8192(angle);
		tmp += rest * (s32)c_sin_8192(angle+1);
		tmp /= div;
		return(tmp);
	}
	else if(rest<0)
	{
		tmp = (div+rest)*(s32)c_sin_8192(angle);
		tmp -= rest * (s32)c_sin_8192(angle-1);
		tmp /= div;
		return(tmp);
	}
	else
	{
		return(c_sin_8192(angle));
	}

}

s16 c_cos_8192_res(s16 arg, s16 div)
{
	return(c_sin_8192_res(90*div - arg, div));
}


// integer based atan2 that returns angle in counts of 1/546.13°  
s32 c_atan2_546(s32 y, s32 x)
{
	s32 qx, qy, q;
	
	if( x < 0) qx = -x;
	else qx = x;
	if( y < 0) qy = -y;
	else qy = y;
	if(qy <= qx)
	{	// scale down to avoid overflow in quadratic interpolation
		while(qy > (1<<15)) 
		{
			qy/=2;
			qx/=2;
		}
		// calculate the quatratic interpolation
		q = (((qy<<13)/qx)*qy)/qx;
		q = (qy<<15)/qx-q;
	}
	else
	{   // scale down to avoid overflow in quadratic interpolation
		while(qx>(1<<15)) 
		{
			qy/=2;
			qx/=2;
		}
		// calculate the quatratic interpolation
		q = (((qx<<13)/qy)*qx)/qy;
		q = (qx<<15)/qy-q;
		q = 2*((1<<15)-(1<<13)) - q;
	}
	if(y < 0)
	{
		if(x < 0)  q = q - 4*((1<<15)-(1<<13));
		else q = -q;
	}
	else if( x < 0) q = 4*((1<<15)-(1<<13)) - q;
	return(q);
}
