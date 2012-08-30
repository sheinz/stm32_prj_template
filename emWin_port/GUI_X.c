/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2012  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.16 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software has been licensed to  ARM LIMITED whose registered office
is situated at  110 Fulbourn Road,  Cambridge CB1 9NJ,  England solely
for  the  purposes  of  creating  libraries  for  ARM7, ARM9, Cortex-M
series,  and   Cortex-R4   processor-based  devices,  sublicensed  and
distributed as part of the  MDK-ARM  Professional  under the terms and
conditions  of  the   End  User  License  supplied  with  the  MDK-ARM
Professional. 
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : GUI_X.C
Purpose     : Config / System dependent externals for GUI
---------------------------END-OF-HEADER------------------------------
*/

#include "GUI.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*********************************************************************
*
*       Global data
*/

static xSemaphoreHandle xSemaphore;

/*********************************************************************
*
*      Timing:
*                 GUI_X_GetTime()
*                 GUI_X_Delay(int)

  Some timing dependent routines require a GetTime
  and delay function. Default time unit (tick), normally is
  1 ms.
*/

int GUI_X_GetTime(void) 
{ 
  return xTaskGetTickCount(); 
}

// ----------------------------------------------------------------------------

void GUI_X_Delay(int ms) 
{ 
  int tEnd = xTaskGetTickCount() + ms;
  while ((tEnd - xTaskGetTickCount()) > 0);
}

/*********************************************************************
*
*       GUI_X_Init()
*
* Note:
*     GUI_X_Init() is called from GUI_Init is a possibility to init
*     some hardware which needs to be up and running before the GUI.
*     If not required, leave this routine blank.
*/

void GUI_X_Init(void) 
{
}

// ----------------------------------------------------------------------------

void GUI_X_ExecIdle(void) 
{
   vTaskDelay(1);
}

// ----------------------------------------------------------------------------
   
void  GUI_X_InitOS (void)
{ 
   vSemaphoreCreateBinary( xSemaphore );
}

// ----------------------------------------------------------------------------

void  GUI_X_Lock (void)
{ 
   xSemaphoreTake(xSemaphore, (portTickType)10);   // trying to obtain the semaphore
}

// ----------------------------------------------------------------------------

void  GUI_X_Unlock (void)
{ 
   xSemaphoreGive(xSemaphore);
}

// ----------------------------------------------------------------------------

U32  GUI_X_GetTaskId (void) 
{ 
   return ((U32)(xTaskGetCurrentTaskHandle()));
}   
   
   
/*********************************************************************
*
*      Logging: OS dependent

Note:
  Logging is used in higher debug levels only. The typical target
  build does not use logging and does therefor not require any of
  the logging routines below. For a release build without logging
  the routines below may be eliminated to save some space.
  (If the linker is not function aware and eliminates unreferenced
  functions automatically)

*/

void GUI_X_Log     (const char *s) { GUI_USE_PARA(s); }
void GUI_X_Warn    (const char *s) { GUI_USE_PARA(s); }
void GUI_X_ErrorOut(const char *s) { GUI_USE_PARA(s); }

/*************************** End of file ****************************/
