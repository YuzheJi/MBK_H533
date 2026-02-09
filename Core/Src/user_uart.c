#include "user_uart.h"
#include "main.h"
#include "hc04.h"
#include "elec391_533.h"
#include <stdint.h>

extern UART_HandleTypeDef huart1;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART1)
    {   

        // banned states
        if(system_mode == 2 || (system_mode == 3 && mode3_state == 0)){
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, BT_DMA_rx_buff, BT_RX_LEN);
            return;
        }

        if (Size < BT_RX_LEN) {
            BT_DMA_rx_buff[Size] = '\0';
        }

        if (BT_DMA_rx_buff[0] == ';') {

            // Pid calibration cmd only valid if under pid cali mode
            // Recevive Kp
            if(system_mode == 1 && BT_DMA_rx_buff[1] == 'A') {
                sscanf((char*)BT_DMA_rx_buff, ";A%f", &Kp);
            }

            // Recevive Ki
            else if(system_mode == 1 && BT_DMA_rx_buff[1] == 'B') {
                sscanf((char*)BT_DMA_rx_buff, ";B%f", &Ki);
            }

            // Recevive Kd
            else if(system_mode == 1 && BT_DMA_rx_buff[1] == 'C') {
                sscanf((char*)BT_DMA_rx_buff, ";C%f", &Kd);
            }

            // Recevive Tar
            else if(system_mode == 1 && BT_DMA_rx_buff[1] == 'D') {
                sscanf((char*)BT_DMA_rx_buff, ";D%f", &target);
                err_acc = 0;
                err_prev = 0;
                settle_count = 0;
            }
            
            // Mode change: halt
            else if(BT_DMA_rx_buff[1] == 'E') {
                system_mode = 0;
            }

            // Mode change: pid calibration
            else if(BT_DMA_rx_buff[1] == 'F') {
                system_mode = 1;
            }

            // Mode change: homing
            else if(BT_DMA_rx_buff[1] == 'G') {
                system_mode = 2;
            }

            // Mode change: playing
            else if(BT_DMA_rx_buff[1] == 'H') {
                system_mode = 3;
                mode3_state = 0;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            }
        }

        else {
            system_mode = 0; 
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, BT_DMA_rx_buff, BT_RX_LEN);
    }
}