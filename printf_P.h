#ifndef _PRINTF_P_H_
#define _PRINTF_P_H_

// function pointer to external callback put character function
typedef void (*pVoidFnctChar) (char );

void _printf_P (pVoidFnctChar pPutchar, char const *fmt0, ...);

#endif //_PRINTF_P_H_

