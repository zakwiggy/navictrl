#ifndef __UART0_H
#define __UART0_H

typedef enum
{
	UART0_UNDEF,
	UART0_MK3MAG,
	UART0_MKGPS
} UART0_MuxerState_t;

extern UART0_MuxerState_t UART0_Muxer;

typedef struct
{
	u8 SWMajor;
	u8 SWMinor;
	u8 ProtoMajor;
	u8 ProtoMinor;
	u8 SWPatch;
	u8 Reserved[5];
}  __attribute__((packed)) MKOSD_VersionInfo_t;

extern MKOSD_VersionInfo_t MKOSD_VersionInfo;

void UART0_Init (void);
void UART0_Connect_to_MKGPS(u16 Baudrate);
void UART0_Connect_to_MK3MAG(void);
void UART0_TransmitTxData(void);
void UART0_ProcessRxData(void);
u8 UART0_GetMKOSDVersion(void);
u8 UART0_GetUBXVersion(void);
u8 UART0_UBXSendCFGMsg(u8* pData, u16 Len);

#endif //__UART0_H

