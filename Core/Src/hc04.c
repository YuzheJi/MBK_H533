#include "hc04.h"
#include "main.h"
#include "elec391_533.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

void BT_SendString_DMA(const char *str) {
    
    while (huart1.gState != HAL_UART_STATE_READY); 
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)str, strlen(str));
}

void BT_Printf_DMA(const char *format, ...) {
    
    static char buf[128]; 
    va_list args;
    
    while (huart1.gState != HAL_UART_STATE_READY);

    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    if (len > 0) {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len);
    }
}

void BT_SetBaudRate(UART_HandleTypeDef *huart){

    uint8_t cmd[] = "AT+BAUD=921600"; 
    HAL_UART_Transmit(huart, cmd, sizeof(cmd) - 1, 100);
    HAL_Delay(500);
}
