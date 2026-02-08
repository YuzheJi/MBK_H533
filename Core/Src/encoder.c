// notice: encoder is using timer3
// external variables: 
// uint16_t speed, CNT taken from timer3, every 40ms
// uint32_t loaction, sum of all speeds
// remember to enable encoder mode after sys initialization

#include "encoder.h"
#include "main.h"

int16_t get_encoder(){

    // get CNT and clear it
    uint16_t temp;
    
    temp = __HAL_TIM_GetCounter(&htim3);
    __HAL_TIM_SetCounter(&htim3, 0);
    return (int16_t) temp;
}

void clear_encoder(){
    __HAL_TIM_SET_COUNTER(&htim3, 0); 
    return;
}
