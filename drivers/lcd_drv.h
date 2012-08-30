#ifndef __LCD_DRV_H__
#define __LCD_DRV_H__
/************************************************************************/
/*                                                                      */
/************************************************************************/


// Initialize the Bus to communicate with LCD
void lcd_BusInit(void);

// Initialize LCD
void lcd_Init(void);

void lcd_WriteReg(unsigned short index);		 
void lcd_WriteCmd(unsigned short index,unsigned short val);
unsigned short lcd_ReadData(void);
void lcd_WriteData(unsigned short val);
void lcd_WriteDataBuff(unsigned short * pData, int NumWords);
void lcd_ReadDataBuf(unsigned short * pData, int NumWords); 

#endif // __LCD_H__
