#ifndef __EEPROM_H
#define __EEPROM_H

typedef enum
{
  EEPROM_SUCCESS = 0,
  EERPOM_ERROR_I2C_DEINIT,
  EEPROM_ERROR_I2C_IDLE_TIMEOUT,
  EEPROM_ERROR_I2C_TRANSFER_TIMEOUT,
  EEPROM_ERROR_OUT_OF_ADDRESS_RANGE,
  EEPROM_DATA_TRANSFER_INCOMPLETE,
  EEPROM_I2C_BUFFER_OVERRUN,
  EEPROM_NO_ACK,
  EEPROM_ERROR_UNKNOWN
} EEPROM_Result_t;

u8 EEPROM_Init(void);
EEPROM_Result_t EEPROM_WriteBlock(u16 Address, u8 *pData, u16 DataLen);
EEPROM_Result_t EEPROM_ReadBlock(u16 Address, u8 *pData, u16 DataLen);

#endif // EEPROM_H

