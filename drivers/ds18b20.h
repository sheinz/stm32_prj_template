#ifndef __DS18B20_H__
#define __DS18B20_H__

#include "onewire.h"

typedef uint8_t DS18B20_Sensor[OW_ROM_SIZE];


// The method searches for the DS18B20 sensors on the 1-wire bus and returns the 
// number of sensor found. Also it stores the ROMs of the found sensors in the pSensorArray
uint8_t DS18B20_SearchSensors(DS18B20_Sensor pSensorArray[], uint8_t arraySize);

// Start measurement for the specific sensor
void DS18B20_StartMeasurement(DS18B20_Sensor sensor);

// Start measurement for all sensors
void DS18B20_StartMeasurementAll(void);

// Returns the raw data from the sensor
uint16_t DS18B20_ReadRawTemperature(DS18B20_Sensor sensor);

// Returns the temperature in Celsius in pTemperature and in pFraction
void DS18B20_ReadTemperature(DS18B20_Sensor sensor, int16_t *pTemperature, uint16_t *pFraction);

// Returns the temperature in float value
float DS18B20_ReadFloatTemperature(DS18B20_Sensor sensor);


#endif //__DS18B20_H__

