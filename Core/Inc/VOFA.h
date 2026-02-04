#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"

extern UART_HandleTypeDef huart1;

void VOFA_JustFloat_Send(UART_HandleTypeDef *huart, float d1, float d2, float d3, float d4, float d5, float d6);

#endif 
