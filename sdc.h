#ifndef _SDC_H
#define _SDC_H


//________________________________________________________________________________________________________________________________________
// 
// Functions needed for accessing the sdcard.
//				
//________________________________________________________________________________________________________________________________________

typedef enum
{
  SD_SUCCESS = 0,
  SD_ERROR_NOCARD,
  SD_ERROR_RESET,
  SD_ERROR_INITIALIZE,
  SD_ERROR_BAD_RESPONSE,
  SD_ERROR_BAD_VOLTAGE_RANGE,
  SD_ERROR_NO_SDCARD,
  SD_ERROR_TIMEOUT,
  SD_ERROR_CRC_DATA,
  SD_ERROR_WRITE_DATA,
  SD_ERROR_READ_DATA,
  SD_ERROR_SET_BLOCKLEN,
  SD_ERROR_UNKNOWN
} SD_Result_t;

SD_Result_t	SDC_Init(void);
SD_Result_t SDC_GetSector (u32 ,u8 *);
SD_Result_t	SDC_PutSector (u32, const u8 *);
SD_Result_t	SDC_Deinit(void);

#endif // _SDC_H


