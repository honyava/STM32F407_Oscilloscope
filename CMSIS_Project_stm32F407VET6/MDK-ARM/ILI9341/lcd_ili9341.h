#ifndef LCD_ILI9341_H
#define LCD_ILI9341_H
//#include "stm32f4xx_hal.h"
#include "main.h"

/***************************************************************************************/
// To define address where we will write data
// For writing data
#define LCD_DATA                                                 ((uint32_t)0x60020000)
// For writing commands
#define LCD_REG                                                  ((uint32_t)0x60000000)
/***************************************************************************************/
void writeLCDCommand(unsigned int reg,unsigned int value);
void writeLCDData(unsigned int data);
void initLCD(void);

#endif
