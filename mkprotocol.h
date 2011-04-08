#ifndef _MKPROTOCOL_H
#define _MKPROTOCOL_H
#include "buffer.h"
#include "mavlink/include/protocol.h"
#include "mavlink/include/common/mavlink_msg_waypoint.h"
// slave addresses
#define ANY_ADDRESS 0
#define FC_ADDRESS 1
#define NC_ADDRESS 2
#define MK3MAG_ADDRESS 3
#define MKOSD_ADDRESS 4
#define BL_ADDRESS 5

typedef struct
{
	u8 Address;
	u8 CmdID;
 	u8* pData;
	u16 DataLen;
} __attribute__((packed)) SerialMsg_t;

u8 MKProtocol_CollectSerialFrame(Buffer_t* pRxBuff, u8 c);
u8 MKProtocol_CreateSerialFrame(Buffer_t* pTxBuff, u8 CmdID, u8 Address, u8 numofbuffers , ...); //u8 *data, u8 len, ....;
void MKProtocol_DecodeSerialFrameHeader(Buffer_t* pRxBuff, SerialMsg_t* pSerialMsg);
void MKProtocol_DecodeSerialFrameData(Buffer_t* pRxBuff, SerialMsg_t* pSerialMsg);

#endif // _MKPROTOCOL_H
