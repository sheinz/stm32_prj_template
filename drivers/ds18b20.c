
#include "ds18b20.h"
#include "crc8.h"

#define DS18B20_DEVICE_ID        0x28  

#define DS18B20_SCRATCHPAD_SIZE  9


#define DS18B20_CMD_CONVERT_T    0x44
#define DS18B20_CMD_READ         0xBE

// ----------------------------------------------------------------------------

uint8_t DS18B20_SearchSensors(DS18B20_Sensor pSensorArray[], uint8_t arraySize)
{
   OW_SearchState searchState;
   uint8_t numberOfSensors = 0;
   uint8_t i;

   OW_SearchInit(&searchState);

   while (OW_SearchNext(&searchState))
   {
      if (searchState.rom[0] == DS18B20_DEVICE_ID)
      {
         for (i = 0; i < OW_ROM_SIZE; i++)
         {
            pSensorArray[numberOfSensors][i] = searchState.rom[i];   // copy rom
         }

         numberOfSensors++;
      }
   }
   
   return numberOfSensors;
}

// ----------------------------------------------------------------------------

void DS18B20_StartMeasurement(DS18B20_Sensor sensor)
{
   OW_Reset();

   OW_WriteByte(OW_CMD_MATCH_ROM);

   OW_WriteBuff(sensor, OW_ROM_SIZE);     // send ROM of the sensor

   OW_WriteByte(DS18B20_CMD_CONVERT_T);
}

// ----------------------------------------------------------------------------

void DS18B20_StartMeasurementAll(void)
{
   OW_Reset();

   OW_WriteByte(OW_CMD_SKIP_ROM);      // send to all sensors

   OW_WriteByte(DS18B20_CMD_CONVERT_T);
}

// ----------------------------------------------------------------------------

uint16_t DS18B20_ReadRawTemperature(DS18B20_Sensor sensor)
{
   uint8_t sp[DS18B20_SCRATCHPAD_SIZE];
   uint8_t i, crc = 0;
   uint16_t temp = 0;

   OW_Reset();

   OW_WriteByte(OW_CMD_MATCH_ROM);

   OW_WriteBuff(sensor, OW_ROM_SIZE);     // send ROM of the sensor

   OW_WriteByte(DS18B20_CMD_READ);

   OW_ReadBuff(sp, DS18B20_SCRATCHPAD_SIZE);

   for (i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
   {
      crc = calc_crc(crc, sp[i]);
   }

   if (crc == 0)
   {
      temp = sp[0];
      temp |= (uint16_t)(sp[1]) << 8;
   }

   return temp;
}

// ----------------------------------------------------------------------------

void DS18B20_ReadTemperature(DS18B20_Sensor sensor, int16_t *pTemperature, uint16_t *pFraction)
{
   uint16_t temp;

   temp = DS18B20_ReadRawTemperature(sensor);

   *pFraction = 625 * (temp & 0x000F) ;    // 625 is from the datasheet

   *pTemperature = temp >> 4;
}

// ----------------------------------------------------------------------------

float DS18B20_ReadFloatTemperature(DS18B20_Sensor sensor)
{
   float temp;
   uint16_t raw;

   raw = DS18B20_ReadRawTemperature(sensor);

   if (raw & (1<<15))      // is sign bit is set
   {
      raw = ~raw;    // invert bits
      temp = 0 - 625 * ((raw & 0x000F) + 1);
      temp = temp / 10000;
      temp -= raw >> 4;    // subtract as it is negative
   }
   else
   {
      temp = 625 * (raw & 0x000F);
      temp = temp / 10000;
      temp += raw >> 4;
   }

   return temp;
}

// ----------------------------------------------------------------------------

