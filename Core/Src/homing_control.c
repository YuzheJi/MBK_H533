#include "hc04.h"
#include "oled.h"
#include "encoder.h"
#include "main.h"
#include "elec391_533.h"
#include <string.h>
#include "homing_control.h"

void homing_control(uint8_t dir){

    // start homing calibration
    homing_count = 0;

    OLED_Printf(0, 0, OLED_8X16, "Homing...");
    OLED_Update();

    // dir = 1 pos rotation
    if(dir == 1) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 630);
    }

    // dir = 0 neg rotation 
    else if (dir == 0){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 380);
    }

    // wait for 5 * 0.5 = 5ms
    while(homing_count == 0);

    // reset homing logic 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
    HAL_Delay(200);

    // homing done clear distance 
    clear_encoder();
    location = 0;
    speed = 0;
    counter = 0;
    counter_acc = 0;

    homing_count = 0;
    system_mode = 0;
    return;
}    