#ifndef _USB_H
#define _USB_H

#include "fifo.h"
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_pwr.h"
#include "usb_desc.h"

extern fifo_t USB_rx_fifo;

void USB_ConfigInit(void);
void USB_CableConfig(FunctionalState NewState);
void USB_EnterLowPowerMode(void);
void USB_LeaveLowPowerMode(void);
void USB_PutString(u8 *string);
void USB_PutChar(u8 c);
void USB_SendData(u8 *pdata, u16 count);
void USB_ProcessRxData(void);
void USB_TransmitTxData(void);

#endif
