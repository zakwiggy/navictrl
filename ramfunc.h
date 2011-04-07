#ifndef _RAMFUNC_H
#define _RAMFUNC_H

typedef  void (*pFunction)(void);
#define __ramfunc __attribute__ ((long_call, section (".ramfunc")))

__ramfunc void Execute_Bootloader(void);
#endif
