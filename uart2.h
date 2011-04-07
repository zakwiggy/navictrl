#ifndef __UART2_H
#define __UART2_H

#include "mkprotocol.h"

extern Buffer_t UART2_tx_buffer;

void UART2_Init (void);
void UART2_Deinit (void);
void UART2_TransmitTxData(void);
void UART2_ProcessRxData(void);


#endif //__UART2_H
