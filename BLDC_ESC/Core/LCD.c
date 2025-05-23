/*
 * LCD.c
 *
 *  Created on: Feb 14, 2025
 *      Author: peterh17
 */


#define _LCD_C

#include "main.h"
#include "LCD.h"

// RS to PA8
// E to PA9
// D4 to PB3
// D5 to PB5
// D6 to PB4
// D7 to PB10

#define LCD_RS GPIO_PIN_11  	// port A
#define LCD_EN GPIO_PIN_12  	// port B
#define LCD_Interface_No 4  // 4-bit interface
//#define LCD_D4 GPIO_PIN_3
//#define LCD_D5 GPIO_PIN_5
//#define LCD_D6 GPIO_PIN_4
//#define LCD_D7 GPIO_PIN_10

const unsigned short LCDpinMapBoard[LCD_Interface_No] = {0x0080,0x0040,0x0020,0x0010}; // upper 4 nibbles
const unsigned short LCDpinMapLCM[LCD_Interface_No] = {0x0400,0x0010,0x0020,0x0008}; 	// PB10, PB4, PB5, PB3

void LcdStrobe()
{
	HAL_GPIO_WritePin(GPIOA, LCD_EN, GPIO_PIN_SET);
	HAL_Delay(0.1);
	HAL_GPIO_WritePin(GPIOA, LCD_EN, GPIO_PIN_RESET);
	HAL_Delay(0.1);
}

void LcdWriteCmd(unsigned short data)
{
	// Translate upper 4 nibbles to PB10, PB4, PB5 and PB3
	LcdPortPinConvert(data);
	HAL_GPIO_WritePin(GPIOA,LCD_RS,GPIO_PIN_RESET);		// clear RS, Low is for Cmd
	LcdStrobe();

	data = (data << 4) & 0x00F0;						// load lower nibbles
	// Translate upper 4 nibbles to PB10, PB4, PB5 and PB3
	LcdPortPinConvert(data);
	HAL_GPIO_WritePin(GPIOA,LCD_RS,GPIO_PIN_RESET);		// clear RS, Low is for Cmd
	LcdStrobe();
	HAL_Delay(0.1);
}

void LcdWriteData(unsigned short data)
{
	// Translate upper 4 nibbles to PB10, PB4, PB5 and PB3
	LcdPortPinConvert(data);
	HAL_GPIO_WritePin(GPIOA,LCD_RS,GPIO_PIN_SET);		// clear RS, high is for Data
	LcdStrobe();

	data = (data << 4) & 0x00F0;						// load lower nibbles
	// Translate upper 4 nibbles to PB10, PB4, PB5 and PB3
	LcdPortPinConvert(data);
	HAL_GPIO_WritePin(GPIOA,LCD_RS,GPIO_PIN_SET);		// clear RS, high is for Data
	LcdStrobe();
	HAL_Delay(0.1);
}

void LcdClear()
{
	LcdWriteCmd(0x01);
	HAL_Delay(5);
}

void LcdPutS(char *s)
{
	while (*s)
	  LcdWriteData(*s++);
}

void LcdPutCh (char data)
{
	unsigned short sTemp = (unsigned short)data;

	// Translate upper 4 nibbles to PB10, PB4, PB5 and PB3
	LcdPortPinConvert(sTemp);
	HAL_GPIO_WritePin(GPIOA,LCD_RS,GPIO_PIN_SET);		// clear RS, high is for Data
	LcdStrobe();

	sTemp = (sTemp << 4) & 0x00F0;						// load lower nibbles
	// Translate upper 4 nibbles to PB10, PB4, PB5 and PB3
	LcdPortPinConvert(sTemp);
	HAL_GPIO_WritePin(GPIOA,LCD_RS,GPIO_PIN_SET);		// clear RS, high is for Data
	LcdStrobe();
	HAL_Delay(0.1);
}

void LcdGoto(int row, int col)
{
	char addr;
	if (row == 0)
		addr = 0x00;
	if (row == 1)
		addr = 0x40;

	addr += col;
	LcdWriteCmd(0x80 | addr);
}

void LcdInit()
{
	unsigned short data;

	data = 0x0000;
	// Translate upper 4 nibbles to PB10, PB4, PB5 and PB3
	LcdPortPinConvert(data);
	HAL_Delay(50);

	data = 0x0030;
	// Translate upper 4 nibbles to PB10, PB4, PB5 and PB3
	LcdPortPinConvert(data);
	LcdStrobe();
	HAL_Delay(30);
	LcdStrobe();
	HAL_Delay(20);
	LcdStrobe();
	HAL_Delay(20);

	data = 0x0020;
	// Translate upper 4 nibbles to PB10, PB4, PB5 and PB3
	LcdPortPinConvert(data);
	LcdStrobe();
	HAL_Delay(5);
	LcdWriteCmd(0x28);
	HAL_Delay(5);
	LcdWriteCmd(0x0F);
	HAL_Delay(5);
	LcdWriteCmd(0x01);
	HAL_Delay(5);
	LcdWriteCmd(0x06);
	HAL_Delay(5);

}

void LcdPortPinConvert(unsigned short data)
{
	unsigned short sIndex;

	for (sIndex=0; sIndex<LCD_Interface_No; sIndex++)
	{
		if (data & LCDpinMapBoard[sIndex])
			GPIOB->ODR |= LCDpinMapLCM[sIndex];
		else
			GPIOB->ODR &= ~LCDpinMapLCM[sIndex];
	}
}



