#include "elec391_533.h"
#include "VOFA.h"

void VOFA_JustFloat_Send(UART_HandleTypeDef *huart, float d1, float d2, float d3, float d4, float d5, float d6) {
    vofa_buf[0] = d1;
    vofa_buf[1] = d2;
    vofa_buf[2] = d3;
    vofa_buf[3] = d4;
    vofa_buf[4] = d5;
    vofa_buf[5] = d6;
    
    // write the tail
    uint32_t *tail = (uint32_t *)&vofa_buf[6];
    *tail = 0x7F800000; 
    HAL_UART_Transmit_DMA(huart, (uint8_t *)vofa_buf, 28);
}
