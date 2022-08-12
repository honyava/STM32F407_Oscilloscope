#include "lcd_ili9341.h"

void writeLCDCommand(unsigned int reg,unsigned int value)
{
    *(uint16_t *) (LCD_REG) = reg;
    *(uint16_t *) (LCD_DATA) = value;
}
void writeLCDData(unsigned int data)
{
    *(uint16_t *) (LCD_DATA)= data;
}

void initLCD()
{
    writeLCDCommand(0x0000,0x0001);
    Delay(10);
    writeLCDCommand(0x0015,0x0030);
    writeLCDCommand(0x0011,0x0040);
    writeLCDCommand(0x0010,0x1628);
    writeLCDCommand(0x0012,0x0000);
    writeLCDCommand(0x0013,0x104d);
    Delay(10);
    writeLCDCommand(0x0012,0x0010);
    Delay(10);
    writeLCDCommand(0x0010,0x2620);
    writeLCDCommand(0x0013,0x344d);
    Delay(10);
    writeLCDCommand(0x0001,0x0100);
    writeLCDCommand(0x0002,0x0300);
    writeLCDCommand(0x0003,0x1030);
    writeLCDCommand(0x0008,0x0604);
    writeLCDCommand(0x0009,0x0000);
    writeLCDCommand(0x000A,0x0008);
    writeLCDCommand(0x0041,0x0002);
    writeLCDCommand(0x0060,0x2700);
    writeLCDCommand(0x0061,0x0001);
    writeLCDCommand(0x0090,0x0182);
    writeLCDCommand(0x0093,0x0001);
    writeLCDCommand(0x00a3,0x0010);
    Delay(10);
    writeLCDCommand(0x30,0x0000);
    writeLCDCommand(0x31,0x0502);
    writeLCDCommand(0x32,0x0307);
    writeLCDCommand(0x33,0x0305);
    writeLCDCommand(0x34,0x0004);
    writeLCDCommand(0x35,0x0402);
    writeLCDCommand(0x36,0x0707);
    writeLCDCommand(0x37,0x0503);
    writeLCDCommand(0x38,0x1505);
    writeLCDCommand(0x39,0x1505);
    Delay(10);
    writeLCDCommand(0x0007,0x0001);
    Delay(10);
    writeLCDCommand(0x0007,0x0021);
    writeLCDCommand(0x0007,0x0023);
    Delay(10);
    writeLCDCommand(0x0007,0x0033);
    Delay(10);
    writeLCDCommand(0x0007,0x0133);
}
