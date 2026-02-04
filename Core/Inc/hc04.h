#ifndef __HC04_H
#define __HC04_H

#include "main.h"
#include <stdint.h>

extern UART_HandleTypeDef huart1;

void BT_SendString_DMA(const char *str);
void BT_Printf_DMA(const char *format, ...);
void BT_SetBaudRate(UART_HandleTypeDef *huart);

#endif
