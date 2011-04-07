#ifndef _MENU_H
#define _MENU_H

#include "printf_P.h"

#define DISPLAYBUFFSIZE 80

#define KEY1    0x01
#define KEY2    0x02
#define KEY3    0x04
#define KEY4    0x08



extern s8 DisplayBuff[DISPLAYBUFFSIZE];
extern u8 DispPtr;
extern u8 MenuItem;
extern u8 MaxMenuItem;

void Menu_Update(u8 Keys);
void Menu_Clear(void);
void Menu_Putchar(char c);

#define LCD_printfxy(x,y,format, args...)  { DispPtr = y * 20 + x; _printf_P(&Menu_Putchar, format , ## args);}
#define LCD_printf(format, args...)        {  _printf_P(&Menu_Putchar, format , ## args);}


#endif // _MENU_H
