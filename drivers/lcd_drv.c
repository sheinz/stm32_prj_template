/************************************************************************/
/*                                                                      */
/************************************************************************/


#include "lcd_drv.h"
#include "stm32f10x_fsmc.h"

// ----------------------------------------------------------------------------

#define Bank1_LCD_D    ((uint32_t)0x60020000)    // display Data ADDR
#define Bank1_LCD_C    ((uint32_t)0x60000000)	 // display Reg ADDR

// ----------------------------------------------------------------------------

void lcd_BusInit(void)
{
   FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
   FSMC_NORSRAMTimingInitTypeDef  p;
   GPIO_InitTypeDef  GPIO_InitStructure; 

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE); 
   
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);	   

   // Configure LCD back light
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;			      // PD13 is LCD back light       
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		   
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
   GPIO_Init(GPIOD, &GPIO_InitStructure);    
   GPIO_SetBits(GPIOD, GPIO_Pin_13);			               // Turn ON CLD back light


   // Configure LCD RST pin
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		   
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		   
   GPIO_Init(GPIOE, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0       // D2
      | GPIO_Pin_1       // D3
      | GPIO_Pin_4       // nOE
      | GPIO_Pin_5       // nWE
      | GPIO_Pin_8       // D13
      | GPIO_Pin_9       // D14
      | GPIO_Pin_10      // D15
      | GPIO_Pin_14      // D0
      | GPIO_Pin_15;     // D1
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	   // Alternative function Push-pull
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7       // D4
      | GPIO_Pin_8       // D5
      | GPIO_Pin_9       // D6
      | GPIO_Pin_10      // D7
      | GPIO_Pin_11      // D8
      | GPIO_Pin_12      // D9
      | GPIO_Pin_13      // D10
      | GPIO_Pin_14      // D11
      | GPIO_Pin_15;     // D12
   // Same speed and mode
   GPIO_Init(GPIOE, &GPIO_InitStructure); 

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;       // CS
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;     // RS
   GPIO_Init(GPIOD, &GPIO_InitStructure); 

   p.FSMC_AddressSetupTime = 0x02;
   p.FSMC_AddressHoldTime = 0x00;
   p.FSMC_DataSetupTime = 0x05;
   p.FSMC_BusTurnAroundDuration = 0x00;
   p.FSMC_CLKDivision = 0x00;
   p.FSMC_DataLatency = 0x00;
   p.FSMC_AccessMode = FSMC_AccessMode_B;

   FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
   FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
   FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
   FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
   FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
   FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
   FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
   FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
   FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
   FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
   FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
   FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
   FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
   FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

   FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

   FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}

// ----------------------------------------------------------------------------

void lcd_WriteReg(unsigned short index)
{
	*(__IO uint16_t *) (Bank1_LCD_C)= index;
}			 

// ----------------------------------------------------------------------------

void lcd_WriteCmd(unsigned short index,unsigned short val)
{	
	*(__IO uint16_t *) (Bank1_LCD_C)= index;	
	*(__IO uint16_t *) (Bank1_LCD_D)= val;
}

// ----------------------------------------------------------------------------

unsigned short lcd_ReadData(void)
{
	unsigned int a=0;
	a=*(__IO uint16_t *) (Bank1_LCD_D);    // WTF?   
	a=*(__IO uint16_t *) (Bank1_LCD_D);
	return(a);	
}

// ----------------------------------------------------------------------------

void lcd_WriteData(unsigned short val)
{   
	*(__IO uint16_t *) (Bank1_LCD_D)= val; 	
}

// ----------------------------------------------------------------------------

void lcd_WriteDataBuff(unsigned short * pData, int NumWords)
{
   for (; NumWords; NumWords--) 
   {
      *(__IO uint16_t *) (Bank1_LCD_D) = *pData;
      pData++;
   }
}

// ----------------------------------------------------------------------------

void lcd_ReadDataBuf(unsigned short * pData, int NumWords)
{
   for (; NumWords; NumWords--) 
   {
      *pData = lcd_ReadData();      // todo: eliminate excessive call
      pData++;
   }
}

// ----------------------------------------------------------------------------

void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

// ----------------------------------------------------------------------------

void lcd_Init(void)
{	
   unsigned int i;   
   
   // Reset LCD controller
   GPIO_ResetBits(GPIOE, GPIO_Pin_1);     // Pull RST down
   Delay(0xAFFf);					   
   GPIO_SetBits(GPIOE, GPIO_Pin_1 );		// Push RST up
	Delay(0xAFFf);	

	lcd_WriteCmd(0x00E3, 0x3008); // Set internal timing
	lcd_WriteCmd(0x00E7, 0x0012); // Set internal timing
	lcd_WriteCmd(0x00EF, 0x1231); // Set internal timing
	lcd_WriteCmd(0x0000, 0x0001); // Start Oscillation
	lcd_WriteCmd(0x0001, 0x0100); // set SS and SM bit
	lcd_WriteCmd(0x0002, 0x0700); // set 1 line inversion

	lcd_WriteCmd(0x0003, 0x1030); // set GRAM write direction and BGR=0,262K colors,1 transfers/pixel.
	lcd_WriteCmd(0x0004, 0x0000); // Resize register
	lcd_WriteCmd(0x0008, 0x0202); // set the back porch and front porch
	lcd_WriteCmd(0x0009, 0x0000); // set non-display area refresh cycle ISC[3:0]
	lcd_WriteCmd(0x000A, 0x0000); // FMARK function
	lcd_WriteCmd(0x000C, 0x0000); // RGB interface setting
	lcd_WriteCmd(0x000D, 0x0000); // Frame marker Position
	lcd_WriteCmd(0x000F, 0x0000); // RGB interface polarity
//Power On sequence 
	lcd_WriteCmd(0x0010, 0x0000); // SAP, BT[3:0], AP, DSTB, SLP, STB
	lcd_WriteCmd(0x0011, 0x0007); // DC1[2:0], DC0[2:0], VC[2:0]
	lcd_WriteCmd(0x0012, 0x0000); // VREG1OUT voltage
	lcd_WriteCmd(0x0013, 0x0000); // VDV[4:0] for VCOM amplitude
	Delay(200); // Dis-charge capacitor power voltage
	lcd_WriteCmd(0x0010, 0x1690); // SAP, BT[3:0], AP, DSTB, SLP, STB
	lcd_WriteCmd(0x0011, 0x0227); // R11h=0x0221 at VCI=3.3V, DC1[2:0], DC0[2:0], VC[2:0]
	Delay(50); // Delay 50ms
	lcd_WriteCmd(0x0012, 0x001C); // External reference voltage= Vci;
	Delay(50); // Delay 50ms
	lcd_WriteCmd(0x0013, 0x1800); // R13=1200 when R12=009D;VDV[4:0] for VCOM amplitude
	lcd_WriteCmd(0x0029, 0x001C); // R29=000C when R12=009D;VCM[5:0] for VCOMH
	lcd_WriteCmd(0x002B, 0x000D); // Frame Rate = 91Hz
	Delay(50); // Delay 50ms
	lcd_WriteCmd(0x0020, 0x0000); // GRAM horizontal Address
	lcd_WriteCmd(0x0021, 0x0000); // GRAM Vertical Address
// ----------- Adjust the Gamma Curve ----------//
	lcd_WriteCmd(0x0030, 0x0007);
	lcd_WriteCmd(0x0031, 0x0302);
	lcd_WriteCmd(0x0032, 0x0105);
	lcd_WriteCmd(0x0035, 0x0206);
	lcd_WriteCmd(0x0036, 0x0808);
	lcd_WriteCmd(0x0037, 0x0206);
	lcd_WriteCmd(0x0038, 0x0504);
	lcd_WriteCmd(0x0039, 0x0007);
	lcd_WriteCmd(0x003C, 0x0105);
	lcd_WriteCmd(0x003D, 0x0808);
//------------------ Set GRAM area ---------------//
	lcd_WriteCmd(0x0050, 0x0000); // Horizontal GRAM Start Address
	lcd_WriteCmd(0x0051, 0x00EF); // Horizontal GRAM End Address
	lcd_WriteCmd(0x0052, 0x0000); // Vertical GRAM Start Address
	lcd_WriteCmd(0x0053, 0x013F); // Vertical GRAM Start Address
	lcd_WriteCmd(0x0060, 0xA700); // Gate Scan Line
	lcd_WriteCmd(0x0061, 0x0001); // NDL,VLE, REV
	lcd_WriteCmd(0x006A, 0x0000); // set scrolling line
//-------------- Partial Display Control ---------//
	lcd_WriteCmd(0x0080, 0x0000);
	lcd_WriteCmd(0x0081, 0x0000);
	lcd_WriteCmd(0x0082, 0x0000);
	lcd_WriteCmd(0x0083, 0x0000);
	lcd_WriteCmd(0x0084, 0x0000);
	lcd_WriteCmd(0x0085, 0x0000);
//-------------- Panel Control -------------------//
	lcd_WriteCmd(0x0090, 0x0010);
	lcd_WriteCmd(0x0092, 0x0000);
	lcd_WriteCmd(0x0093, 0x0003);
	lcd_WriteCmd(0x0095, 0x0110);
	lcd_WriteCmd(0x0097, 0x0000);
	lcd_WriteCmd(0x0098, 0x0000);
   lcd_WriteCmd(0x0007, 0x0133); // 262K color and display ON
  	
   lcd_WriteCmd(32, 0);
   lcd_WriteCmd(33, 0x013F);
   lcd_WriteReg(34);
	for(i=0;i<76800;i++)
	{
      lcd_WriteData(0xFFFF);
	}   
}


