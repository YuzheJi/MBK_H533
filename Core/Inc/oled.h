#ifndef __OLED_H
#define __OLED_H

#define OLED_ADDRESS  0x78

#include "main.h"
#include <stdint.h>

// two types of widths
#define OLED_8X16				8
#define OLED_6X8				6

void OLED_Init(void);
void OLED_Update(void);
void OLED_Clear(void);
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);
void OLED_Reverse(void);
void OLED_DrawPoint(int16_t X, int16_t Y, uint8_t Bit);
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize);
void OLED_ShowString(int16_t X, int16_t Y, const char *String, uint8_t FontSize);
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image);
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...);

#endif
