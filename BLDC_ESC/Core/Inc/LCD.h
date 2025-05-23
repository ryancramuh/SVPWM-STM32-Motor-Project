/*
 * LCD.h
 *
 *  Created on: Feb 14, 2025
 *      Author: peterh17
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#ifdef _LCD_C
   #define SCOPE
#else
   #define SCOPE extern
#endif

SCOPE void LcdStrobe(void);
SCOPE void LcdWriteCmd(unsigned short);
SCOPE void LcdWriteData(unsigned short);
SCOPE void LcdClear(void);
SCOPE void LcdPutS(char * s);
SCOPE void LcdPutCh(char c);
SCOPE void LcdGoto(int, int);
SCOPE void LcdInit(void);
SCOPE void LcdPortPinConvert(unsigned short);

#undef SCOPE
#endif /* INC_LCD_H_ */

