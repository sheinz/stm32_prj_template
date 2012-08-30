/**
  ******************************************************************************
  * @file    GPIO/IOToggle/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "gui.h"
#include "GRAPH.h"

#include "lcd_drv.h"
#include "ds18b20.h"
#include "rtc.h"

#define NUMBER_OF_SENSORS  2

static DS18B20_Sensor sensors[NUMBER_OF_SENSORS];

static xSemaphoreHandle keySemaphore;

// ----------------------------------------------------------------------------

void vTestTask( void * pvParameters )
{   
   uint8_t numberOfSensors;
   float fTemp1, fTemp2;
   GRAPH_Handle hGraph;
   GRAPH_DATA_Handle hData1, hData2;
   GRAPH_SCALE_Handle hScale;
   RTC_t dateTime;
   

   GUI_Init();
   
   OW_Init();

   rtc_init();
   
   dateTime.year = 2012;
   dateTime.month = 8;
   dateTime.mday = 29;
   dateTime.hour = 23;
   dateTime.min = 8;
   dateTime.sec = 0;
   dateTime.wday = 3;

   //rtc_settime(&dateTime);

   hGraph = GRAPH_CreateEx(0, 20, 320, 220, WM_HBKWIN, WM_CF_SHOW | WM_CF_CONST_OUTLINE | WM_CF_MEMDEV,
     GRAPH_CF_GRID_FIXED_X, 0);

   
   GRAPH_SetColor (hGraph, GUI_DARKGRAY,     GRAPH_CI_BK);
   GRAPH_SetColor (hGraph, GUI_DARKGRAY, GRAPH_CI_BORDER);
   GRAPH_SetColor (hGraph, GUI_BLACK,  GRAPH_CI_FRAME);
   GRAPH_SetColor (hGraph, GUI_GREEN,   GRAPH_CI_GRID);
   
   GRAPH_SetBorder(hGraph, 30, 0, 0, 0);
   GRAPH_SetGridVis  (hGraph, 1);
   GRAPH_SetGridDistX(hGraph, 0);
   GRAPH_SetGridDistY(hGraph, 20);
   GRAPH_SetGridOffY(hGraph, 10);
   GRAPH_SetLineStyleH(hGraph, GUI_LS_DOT);
   WM_BringToBottom  (hGraph);

   hData1 = GRAPH_DATA_YT_Create(GUI_BLUE, 285, 0, 0);
   GRAPH_DATA_YT_SetAlign(hData1, GRAPH_ALIGN_LEFT); 
   GRAPH_AttachData(hGraph, hData1);

   hData2 = GRAPH_DATA_YT_Create(GUI_RED, 285, 0, 0);
   GRAPH_DATA_YT_SetAlign(hData2, GRAPH_ALIGN_LEFT); 
   GRAPH_AttachData(hGraph, hData2);

   hScale = GRAPH_SCALE_Create(20, GUI_TA_RIGHT, GRAPH_SCALE_CF_VERTICAL, 20);
   GRAPH_SCALE_SetOff(hScale, 30);
   GRAPH_SCALE_SetFactor(hScale, 0.25);
   GRAPH_AttachScale(hGraph, hScale);

   while (1)
   {      
      numberOfSensors = DS18B20_SearchSensors(sensors, NUMBER_OF_SENSORS);

      GUI_DispStringAt("NumberOfSensors:", 0, 00);
      GUI_DispDecAt(numberOfSensors, 100, 0, 1);

      rtc_gettime(&dateTime);

      GUI_DispString("  Date: ");
      GUI_DispDec(dateTime.mday, 2);
      GUI_DispString("/");
      GUI_DispDec(dateTime.month, 2);
      GUI_DispString("/");
      GUI_DispDec(dateTime.year, 4);

      GUI_DispString(" Time: ");
      GUI_DispDec(dateTime.hour, 2);
      GUI_DispString(":");
      GUI_DispDec(dateTime.min, 2);
      GUI_DispString(":");
      GUI_DispDec(dateTime.sec, 2);
      
      vTaskDelay(1);

      DS18B20_StartMeasurementAll();
      
      vTaskDelay(750);

      //DS18B20_ReadTemperature(sensors[0], &temp1, &fract1);
      GUI_DispStringAt("Temperature: ", 0, 10);
      fTemp1 = DS18B20_ReadFloatTemperature(sensors[0]);
      fTemp2 = DS18B20_ReadFloatTemperature(sensors[1]);
      
      //GUI_DispDecAt(temp1, 0, 30, 3);
      //GUI_DispDecAt(fract1, 30, 30, 4);
      GUI_DispFloat(fTemp1, 6);
      GUI_DispString("   ");
      GUI_DispFloat(fTemp2, 6);


      GRAPH_DATA_YT_AddValue(hData1, 30 + (signed short)(fTemp1 * 4));
      GRAPH_DATA_YT_AddValue(hData2, 30 + (signed short)(fTemp2 * 4));

      GUI_Exec();           /* Do the background work ... Update windows etc.) */
      GUI_X_ExecIdle(); 
      //vTaskDelay(10 * 60 * 1000);   // 10 min
      xSemaphoreTake(keySemaphore, 10 * 60 * 1000);
      //xSemaphoreGive(keySemaphore);    // release semaphore even if there was a timeout and the semaphore wasn't taken
      //xSemaphoreTake(keySemaphore, 5000);
   }
}

void vKeyTask( void * pvParameters )
{
   GPIO_InitTypeDef GPIO_InitStructure;

   // Configure PB15 as input with pull up resistor
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOB, &GPIO_InitStructure);


   while (1)
   {

      if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
      {  // the key is pressed
         GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
         xSemaphoreGive(keySemaphore);       // release semaphore, so the temp can be taken
         taskYIELD();      // switch context
         xSemaphoreTake(keySemaphore, 0);   
      }
      else
      {
         GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
      }

      vTaskDelay(100);
   }

}

// ----------------------------------------------------------------------------

int main(void)
{        
   GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );   

   /* GPIOD Periph clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

   /* Configure PB5 in output pushpull mode */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   lcd_BusInit();

   vSemaphoreCreateBinary(keySemaphore);
   
   // Create test task
   xTaskCreate(vTestTask, "TestTask", 1024, NULL, 10, ( xTaskHandle * ) NULL);

   // Create key task
   xTaskCreate(vKeyTask, "KeyTask", 1024, NULL, 20, ( xTaskHandle * ) NULL);

   /* Start the scheduler. */
	vTaskStartScheduler();
	
	/* Will only get here if there was not enough heap space to create the
	idle task. */
	return 0;
}

// ----------------------------------------------------------------------------

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

