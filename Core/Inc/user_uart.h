#ifndef __USER_USART_H
#define __USER_USART_H

#include "main.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif