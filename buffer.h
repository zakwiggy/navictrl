#ifndef __BUFFER_H
#define __BUFFER_H

typedef struct
{
	u8* pData;
	u16 Size;
	u16 DataBytes;
	u16 Position;
	u8  Locked;
} __attribute__((packed)) Buffer_t;

void Buffer_Init(Buffer_t* pBuffer, u8* pDataBuffer, u16 DataBufferSize);
void Buffer_Clear(Buffer_t* pBuffer);
u8 Buffer_Copy(Buffer_t* pSrcBuffer, Buffer_t* pDstBuffer);
#endif // __BUFFER_H
