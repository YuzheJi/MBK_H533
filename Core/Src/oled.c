#include "oled.h"
#include "oled_font.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern I2C_HandleTypeDef hi2c1;

// buffer
uint8_t OLED_DisplayBuf[8][128];

// write cmd
void OLED_WriteCommand(uint8_t Command)
{
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, &Command, 1, 100);
}

// write data
void OLED_WriteData(uint8_t *Data, uint8_t Count)
{
    // 0x40 表示后续为数据
    // HAL_I2C_Mem_Write 内部会自动处理 Start, 地址, 控制位(0x40), 数据流和 Stop
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDRESS, 0x40, I2C_MEMADD_SIZE_8BIT, Data, Count, HAL_MAX_DELAY);
}

// set cursor
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                    /* Set Page Start Address [cite: 53] */
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));    /* Set Column Address High Nibble */
    OLED_WriteCommand(0x00 | (X & 0x0F));           /* Set Column Address Low Nibble */
}

// update data in buff into oled
void OLED_Update(void)
{
    for (uint8_t j = 0; j < 8; j++)
    {
        OLED_WriteCommand(0xB0 | j); 
        OLED_WriteCommand(0x10);    
        OLED_WriteCommand(0x00);    
        OLED_WriteData(OLED_DisplayBuf[j], 128);
    }
}

// clear the buff
void OLED_Clear(void)
{
    memset(OLED_DisplayBuf, 0x00, sizeof(OLED_DisplayBuf));
}

// clear the buff for an area
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    for (int16_t j = Y; j < Y + Height; j++)
    {
        for (int16_t i = X; i < X + Width; i++)
        {
            if (i >= 0 && i <= 127 && j >= 0 && j <= 63)
            {
                OLED_DisplayBuf[j / 8][i] &= ~(0x01 << (j % 8));
            }
        }
    }
}

// reverse color
void OLED_Reverse(void)
{
    for (uint8_t j = 0; j < 8; j++)
    {
        for (uint8_t i = 0; i < 128; i++)
        {
            OLED_DisplayBuf[j][i] ^= 0xFF;
        }
    }
}

// initialzation
void OLED_Init(void)
{
    HAL_Delay(100);   
    OLED_WriteCommand(0xAE);   
    OLED_WriteCommand(0xD5); 
    OLED_WriteCommand(0x80);   
    OLED_WriteCommand(0xA8); 
    OLED_WriteCommand(0x3F);   
    OLED_WriteCommand(0xD3); 
    OLED_WriteCommand(0x00);  
    OLED_WriteCommand(0x40);   
    OLED_WriteCommand(0xA1);   
    OLED_WriteCommand(0xC8); 
    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12); 
    OLED_WriteCommand(0x81); 
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); 
    OLED_WriteCommand(0xF1); 
    OLED_WriteCommand(0xDB); 
    OLED_WriteCommand(0x30); 
    OLED_WriteCommand(0xA4); 
    OLED_WriteCommand(0xA6); 
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14); 
    OLED_WriteCommand(0xAF); 
    
    OLED_Clear();
    OLED_Update();           
}

// draw point, used for other funcs
void OLED_DrawPoint(int16_t X, int16_t Y, uint8_t Bit)
{
    if (X >= 0 && X <= 127 && Y >= 0 && Y <= 63)
    {
        if (Bit) OLED_DisplayBuf[Y / 8][X] |= (0x01 << (Y % 8));
        else     OLED_DisplayBuf[Y / 8][X] &= ~(0x01 << (Y % 8));
    }
}

// draw one char
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
    if (FontSize == OLED_6X8) // 6x8 字体
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            uint8_t Data = OLED_F6x8[Char - ' '][i];
            for (uint8_t j = 0; j < 8; j++) 
                OLED_DrawPoint(X + i, Y + j, (Data >> j) & 0x01);
        }
    }
    else if(FontSize == OLED_8X16)// 8x16 字体
    {
        for (uint8_t j = 0; j < 2; j++)
        {
            for (uint8_t i = 0; i < 8; i++)
            {
                uint8_t Data = OLED_F8x16[Char - ' '][j * 8 + i];
                for (uint8_t bit = 0; bit < 8; bit++) 
                    OLED_DrawPoint(X + i, Y + j * 8 + bit, (Data >> bit) & 0x01);
            }
        }
    }
}

// draw string
void OLED_ShowString(int16_t X, int16_t Y, const char *String, uint8_t FontSize)
{
    for (uint16_t i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(X + i * (FontSize == 8 ? 8 : 6), Y, String[i], FontSize);
    }
}

// draw image
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
    int16_t Page, Shift;
    OLED_ClearArea(X, Y, Width, Height); // clear that area

    for (uint8_t j = 0; j < (Height - 1) / 8 + 1; j++)
    {
        for (uint8_t i = 0; i < Width; i++)
        {
            if (X + i >= 0 && X + i <= 127)
            {
                Page = Y / 8;
                Shift = Y % 8;
                if (Y < 0) { Page -= 1; Shift += 8; }

                if (Page + j >= 0 && Page + j <= 7)
                {
                    OLED_DisplayBuf[Page + j][X + i] |= Image[j * Width + i] << Shift;
                }
                if (Page + j + 1 >= 0 && Page + j + 1 <= 7)
                {
                    OLED_DisplayBuf[Page + j + 1][X + i] |= Image[j * Width + i] >> (8 - Shift);
                }
            }
        }
    }
}

// fomatted print
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)
{
    char String[64];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    OLED_ShowString(X, Y, String, FontSize);
}
