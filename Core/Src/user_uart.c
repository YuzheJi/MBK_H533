#include "user_uart.h"
#include "main.h"
#include "hc04.h"
#include "elec391_533.h"
#include <stdint.h>

extern UART_HandleTypeDef huart1;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART1)
    {   
        if (Size < BT_RX_LEN) {
            BT_DMA_rx_buff[Size] = '\0';
        }

        if (BT_DMA_rx_buff[0] == ',') {
            BT_cmd_type = 1; 
        }
        else if (BT_DMA_rx_buff[0] == ';') {

            if(BT_DMA_rx_buff[1] == 'A') BT_cmd_type = 2; 
            else if(BT_DMA_rx_buff[1] == 'B') BT_cmd_type = 3;
            else if(BT_DMA_rx_buff[1] == 'C') BT_cmd_type = 4;
        }
        else {
            BT_cmd_type = 127; 
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, BT_DMA_rx_buff, BT_RX_LEN);
    }
}