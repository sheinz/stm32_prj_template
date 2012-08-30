#ifndef __CRC8_H__
#define __CRC8_H__

#include <stdint.h>

// Calculates crc using the stored value and a new value
uint8_t calc_crc(uint8_t storedValue, uint8_t value);


#endif //__CRC8_H__

