/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */

#include "onewire.h"
#include "crc8.h"
#include "stm32f10x.h"



// ----------- Configuration --------------------------------------------------

// for FreeRTOS usage
#define OW_GIVE_TICK_RTOS

// select USART for 1-wire usage
//#define OW_USART1
//#define OW_USART2
#define OW_USART3

// ----------------------------------------------------------------------------


#ifdef OW_GIVE_TICK_RTOS
   #include "FreeRTOS.h"
   #include "task.h"
#endif

#ifdef OW_USART1
   #define OW_USART 		USART1
   #define OW_DMA_CH_RX 	DMA1_Channel5
   #define OW_DMA_CH_TX 	DMA1_Channel4
   #define OW_DMA_FLAG		DMA1_FLAG_TC5
   #define OW_USART_DR     (USART1->DR)
#endif

#ifdef OW_USART2
   #define OW_USART 		USART2
   #define OW_DMA_CH_RX 	DMA1_Channel6
   #define OW_DMA_CH_TX 	DMA1_Channel7
   #define OW_DMA_FLAG		DMA1_FLAG_TC6
   #define OW_USART_DR     (USART2->DR)
#endif


#ifdef OW_USART3
   #define OW_USART 		USART3
   #define OW_DMA_CH_RX 	DMA1_Channel3
   #define OW_DMA_CH_TX 	DMA1_Channel2
   #define OW_DMA_FLAG		DMA1_FLAG_TC3
   #define OW_USART_DR     (USART3->DR)
#endif


// ----------------------------------------------------------------------------

// Rx/Tx buffer
uint8_t ow_buf[8];

#define OW_0	   0x00
#define OW_1	   0xff
#define OW_READ	0xff


//-----------------------------------------------------------------------------
// Converts one byte into eight byte for transmission via USART
// ow_byte - source byte
// ow_bits - the result 8 bytes
//-----------------------------------------------------------------------------
void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) 
{
	uint8_t i;

	for (i = 0; i < 8; i++) 
   {
		if (ow_byte & 0x01) 
      {
			*ow_bits = OW_1;
		} 
      else 
      {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// Converts 8 bytes buffer into one byte
// ow_bits - pointer to the buffer 8 byte length
//-----------------------------------------------------------------------------
uint8_t OW_toByte(uint8_t *ow_bits) 
{
	uint8_t ow_byte, i;

	ow_byte = 0;
	for (i = 0; i < 8; i++) 
   {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_READ) 
      {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}

//-----------------------------------------------------------------------------
// USART & DMA init
//-----------------------------------------------------------------------------
void OW_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;

#ifdef OW_USART1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
				                 ENABLE);

		// USART TX
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		// USART RX
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif 
#ifdef OW_USART2
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
				                 ENABLE);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif
#ifdef OW_USART3
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,
         ENABLE);

      // USART TX
      GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
      GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;    // configure the TX as OpenDrain so the diode is not needed
      GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

      GPIO_Init(GPIOB, &GPIO_InitStruct);

      // USART RX
      GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
      GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
      GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

      GPIO_Init(GPIOB, &GPIO_InitStruct);

      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(OW_USART, &USART_InitStructure);
	USART_Cmd(OW_USART, ENABLE);
}

//-----------------------------------------------------------------------------
// Reset and check devices presence
//-----------------------------------------------------------------------------
uint8_t OW_Reset(void) 
{
	uint8_t ow_presence;
	USART_InitTypeDef USART_InitStructure;

   // set common fields
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

   // Set speed to 9600
	USART_InitStructure.USART_BaudRate = 9600;
	USART_Init(OW_USART, &USART_InitStructure);

	// send 0xf0 using speed 9600
	USART_ClearFlag(OW_USART, USART_FLAG_TC);
	USART_SendData(OW_USART, 0xf0);
	while (USART_GetFlagStatus(OW_USART, USART_FLAG_TC) == RESET) 
   {
#ifdef OW_GIVE_TICK_RTOS
      taskYIELD();
#endif
	}

	ow_presence = USART_ReceiveData(OW_USART);

   // Set speed to 115200
	USART_InitStructure.USART_BaudRate = 115200;
	USART_Init(OW_USART, &USART_InitStructure);

	if (ow_presence != 0xf0) 
   {
		return OW_OK;
	}

	return OW_NO_DEVICE;
}

// ----------------------------------------------------------------------------

void OW_WriteBit(uint8_t bit)
{
   USART_ClearFlag(OW_USART, USART_FLAG_TC);

   USART_SendData(OW_USART, (bit) ? OW_1 : OW_0);

   while (USART_GetFlagStatus(OW_USART, USART_FLAG_TC) == RESET)
   {
#ifdef OW_GIVE_TICK_RTOS
      taskYIELD();
#endif
   }
}

// ----------------------------------------------------------------------------

uint8_t OW_ReadBit(void)
{
   uint8_t bit = 0;

   USART_ClearFlag(OW_USART, USART_FLAG_RXNE);

   USART_SendData(OW_USART, OW_READ);

   while (USART_GetFlagStatus(OW_USART, USART_FLAG_RXNE) == RESET)
   {
#ifdef OW_GIVE_TICK_RTOS
      taskYIELD();
#endif
   }

   if (USART_ReceiveData(OW_USART) == 0xFF)
   {
      bit = 1;
   }

   return bit;
}

// ----------------------------------------------------------------------------

void OW_WriteByte(uint8_t dataByte)
{
   DMA_InitTypeDef DMA_InitStructure;

   OW_toBits(dataByte, ow_buf);

   // Init common fields
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART_DR);
   DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
   DMA_InitStructure.DMA_BufferSize = 8;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

   // DMA to read
   DMA_DeInit(OW_DMA_CH_RX);
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
   DMA_Init(OW_DMA_CH_RX, &DMA_InitStructure);

   // DMA to write
   DMA_DeInit(OW_DMA_CH_TX);
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
   DMA_Init(OW_DMA_CH_TX, &DMA_InitStructure);

   // start sending cycle
   USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
   USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
   DMA_Cmd(OW_DMA_CH_RX, ENABLE);
   DMA_Cmd(OW_DMA_CH_TX, ENABLE);

   // Wait until 8 bytes are received
   while (DMA_GetFlagStatus(OW_DMA_FLAG) == RESET)
   {
#ifdef OW_GIVE_TICK_RTOS
      taskYIELD();
#endif
   }

   // disable DMA
   DMA_Cmd(OW_DMA_CH_TX, DISABLE);
   DMA_Cmd(OW_DMA_CH_RX, DISABLE);
   USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);
}

// ----------------------------------------------------------------------------

void OW_WriteBuff(uint8_t *pData, uint8_t dataSize)
{
   while (dataSize--)
   {
      OW_WriteByte(*pData);
      pData++;
   }
}

// ----------------------------------------------------------------------------

uint8_t OW_ReadByte(void)
{
   // In order to read data from the BUS we need to write there 0xFF and the received data will be in the ow_buf
   OW_WriteByte(0xFF);

   return OW_toByte(ow_buf);
}

// ----------------------------------------------------------------------------

void OW_ReadBuff(uint8_t *pBuff, uint8_t buffSize)
{
   while (buffSize--)
   {
      *pBuff = OW_ReadByte();
      pBuff++;
   }
}

// ----------------------------------------------------------------------------

void OW_SearchInit(OW_SearchState *pSearchState)
{
   pSearchState->lastDiscrepancy = 0;
   pSearchState->lastDeviceFlag = 0;
   pSearchState->crc = 0;
}

// ----------------------------------------------------------------------------
   //--------------------------------------------------------------------------
   // Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
   // search state.
   // Return TRUE  : device found, ROM number in ROM_NO buffer
   //        FALSE : device not found, end of search
   //
uint8_t OW_SearchNext(OW_SearchState *pSearchState)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number, search_result;
   uint8_t id_bit, cmp_id_bit;
   uint8_t rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = 0;
   pSearchState->crc = 0;

   // if the last call was not the last one
   if (!pSearchState->lastDeviceFlag)
   {
      // 1-Wire reset
      if (OW_Reset() == OW_NO_DEVICE)
      {
         // reset the search
         pSearchState->lastDiscrepancy = 0;
         pSearchState->lastDeviceFlag = 0;
         return 0;
      }

      // issue the search command 
      OW_WriteByte(OW_CMD_SEARCH);

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = OW_ReadBit();
         cmp_id_bit = OW_ReadBit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1))
            break;
         else
         {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit)
               search_direction = id_bit;  // bit write value for search
            else
            {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < pSearchState->lastDiscrepancy)
                  search_direction = ((pSearchState->rom[rom_byte_number] & rom_byte_mask) > 0);
               else
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == pSearchState->lastDiscrepancy);

                  // if 0 was picked then record its position in LastZero
                  if (search_direction == 0)
                  {
                     last_zero = id_bit_number;
                  }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
               pSearchState->rom[rom_byte_number] |= rom_byte_mask;
            else
               pSearchState->rom[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            OW_WriteBit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0)
            {
               // accumulate the CRC
               pSearchState->crc = calc_crc(pSearchState->crc, 
                                            pSearchState->rom[rom_byte_number]); 
               rom_byte_number++;
               rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < OW_ROM_SIZE);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if ((id_bit_number >= 65) && (pSearchState->crc == 0))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         pSearchState->lastDiscrepancy = last_zero;

         // check for last device
         if (pSearchState->lastDiscrepancy == 0)
            pSearchState->lastDeviceFlag = 1;

         search_result = 1;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !pSearchState->rom[0])
   {
      pSearchState->lastDiscrepancy = 0;
      pSearchState->lastDeviceFlag = 0;
      search_result = 0;
   }

   return search_result;
}

