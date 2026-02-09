#include "hc04.h"
#include "oled.h"
#include "encoder.h"
#include "main.h"
#include "elec391_533.h"
#include <string.h>
#include "homing_control.h"

void homing_control(uint8_t dir){

    // start homing calibration
    homing_flag = 0;

    // dir = 1 pos rotation
    if(dir == 1) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 680);
        while(homing_flag == 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 420);
        HAL_Delay(200);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 560);
    }

    // dir = 0 neg rotation 
    else if (dir == 0){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 380);
    }

    // wait for 5 * 0.5 = 5ms
    while(homing_flag == 0);

    // reset homing logic 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
    HAL_Delay(500);

    // homing done clear distance 
    clear_encoder();
    target = 0;
    location = 0;
    speed = 0;
    counter = 0;
    counter_acc = 0;

    homing_flag = 0;
    return;
}    