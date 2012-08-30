/*
 * onewire.h
 *
 *  Version 1.0.1
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_


#include <stdint.h>

// select USART for 1-wire usage
//#define OW_USART1
//#define OW_USART2
#define OW_USART3


// return status 
#define OW_NO_DEVICE	   0
#define OW_OK			   1

// ----------------------------------------------------------------------------

#define OW_CMD_SEARCH         0xF0
#define OW_CMD_MATCH_ROM      0x55
#define OW_CMD_SKIP_ROM       0xCC

// ----------------------------------------------------------------------------

#define OW_ROM_SIZE        8

// The struct is used to store the state of the search.
typedef struct
{
   uint8_t rom[OW_ROM_SIZE];       // The ROM of the last found device

   uint8_t lastDiscrepancy;      
   uint8_t crc;
   uint8_t lastDeviceFlag;
} OW_SearchState;

// ----------------------------------------------------------------------------

// Initializes the hardware
void OW_Init(void);

// Perform reset pulse and checks devices presence
// Returns OW_OK or OW_NO_DEVICE 
uint8_t OW_Reset(void);

// Writes one byte into the 1-wire bus
void OW_WriteByte(uint8_t dataByte);

// Reads one byte from the 1-wire bus
uint8_t OW_ReadByte(void);

// Writes the buffer of data into the 1-wire bus
void OW_WriteBuff(uint8_t *pData, uint8_t dataSize);

// Reads the data from the 1-wire bus and returns it in the pBuff
void OW_ReadBuff(uint8_t *pBuff, uint8_t buffSize);

// Initialize the search structure
void OW_SearchInit(OW_SearchState *pSearchState);

// Searches for the next device on the 1-wire bus
// returns 1 if a device was found otherwise - 0
uint8_t OW_SearchNext(OW_SearchState *pSearchState);

#endif /* ONEWIRE_H_ */
