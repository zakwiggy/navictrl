#ifndef __I2C_H
#define __I2C_H

extern volatile u32 I2C1_Timeout;

// I2C states
#define	I2C_STATE_UNDEF			0
#define	I2C_STATE_IDLE			1
#define I2C_STATE_BUFFBUSY		2
#define I2C_STATE_TX_PENDING	3
#define I2C_STATE_TX_PROGRESS 	4
#define	I2C_STATE_RX_PENDING	5
#define I2C_STATE_RX_PROGRESS	6
#define I2C_STATE_OFF			7

#define I2C_ERROR_NONE			0
#define I2C_ERROR_UNKNOWN		1
#define I2C_ERROR_NOACK			2

// current I2C state
extern volatile u8 I2C_State;
// the last I2C error
extern volatile u8 I2C_Error;

#define I2C1_TIMEOUT 500 // 500 ms

// define the size of the rx/tx buffer
#define I2C_BUFFER_LEN 100
// transfer buffer should be filled after a successful
// I2C_LockBuffer(...) and before a start of transmission
// initiated by  I2C_Transmission(...).
extern volatile u8 I2C_Buffer[];

void I2C1_Init(void);

void I2C1_Deinit(void);

// the pointer to the rxbuffer handler function
// called by the IRQ routine after all bytes are recieved from slave
typedef void (*I2C_pRxHandler_t) (u8* pRxBuffer, u8 RxBufferSize);
// Initiate i2c transmission
// A transmission sends first TxBytes from I2C_Buffer to slave
// and then RxBytes are read from slave to I2C_Buffer
// replacing the byte that have been sent.
// Then the RxHandler function is called to handle the result.
// This function returns imediatly after a start condition.
// returns 1 if a transmission has been started, otherwise 0
u8 I2C_Transmission(u8 SlaveAddr, u8 TxBytes, I2C_pRxHandler_t pRxHandler, u8 RxBytes);
// try to allocate the I2C_Buffer within the timeout limit
// returns 1 on success
u8 I2C_LockBuffer(u32 timeout);
// wait until transmission progess is finished or timeout
// returns 1 if no timeout occurs
u8 I2C_WaitForEndOfTransmission(u32 timeout);

#endif // I2C_H

