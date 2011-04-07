/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Date First Issued  : 10/01/2008 : V1.0
* Description        : Endpoint routines
********************************************************************************
* History:
* 10/01/2008 : V1.0
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "usb.h"
#include "mkprotocol.h"
#include "91x_lib.h"
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
	u8 USB_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
	u32 i, USB_BufferCount;

	USB_BufferCount= GetEPRxCount(ENDP3);
	PMAToUserBufferCopy(USB_Buffer, ENDP3_RXADDR, USB_BufferCount);
	for(i=0; i<USB_BufferCount;i++)
	{
		// put into the software fifo
		if(!fifo_put(&USB_rx_fifo, USB_Buffer[i]))
		{	// fifo overflow
		 	//fifo_purge(&USB_rx_fifo); // flush the whole buffer
		}
	}
	SetEPRxValid(ENDP3);
}
/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{

}
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

