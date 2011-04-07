#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "91x_lib.h"
#include "main.h"
#include "debug.h"

#ifdef DEBUG	// only include functions if DEBUG is defined in main.h

#warning : "### DEBUG-Funktion aktiv ###"



u8 Debug_BufPtr = 0;
Debug_t  tDebug;
u8 SendDebugOutput = 0;

// function called from _printf_P to output character
void Debug_Putchar(char c)
{
	if (!SendDebugOutput)
	{
		tDebug.Text[Debug_BufPtr++] = c;									// copy character to buffer
		if (Debug_BufPtr > 30) Debug_BufPtr = 30;							// avoid buffer overflow
	} 
}

void DebugSend(u8 cmd)
{
	if (!SendDebugOutput)
	{
		tDebug.Cmd = cmd;			
		tDebug.Text[Debug_BufPtr] = '\0';						  		// end of text marker
		Debug_BufPtr = 0;												// set bufferindex to 0
		SendDebugOutput = 1;											// set flag to trasmit data the next time in serial transmit function
	}	
}
#endif

