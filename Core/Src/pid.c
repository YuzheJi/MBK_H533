#include "main.h"
#include "pid.h"
#include "elec391_533.h"
#include "VOFA.h"
#include <stdlib.h>

#define I_MAX  1.500f
#define I_MIN -1.500f

#define I_ON_ZONE 2.0f

#define PWM_DEAD_ZONE 0
// 150 160: fric
#define PWM_DEAD_ZONE_OFFSET_MINUS 70
#define PWM_DEAD_ZONE_OFFSET_PLUS  75

#define DEAD_BAND 0.7f

void pid_control(void){

    float pout, dout, iout;
    int32_t pid_pwm;
    int32_t pid_temp = 500;
    uint8_t i_flag;

    err = target - location;
    err_acc += err;

    if(err < DEAD_BAND && err > -DEAD_BAND){
        if(settle_count < SETTLE_TH) settle_count ++;
    }
    else {
        if(settle_count > 0) settle_count --;
    }

    if(settle_count == SETTLE_TH){
        BSP_LED_On(LED_GREEN);
    }
    else{
        BSP_LED_Off(LED_GREEN);
    }

    // Dead band
    if(err < DEAD_BAND && err > -DEAD_BAND){
        err_acc = 0;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
        VOFA_JustFloat_Send(&huart1, (float)location, (float)target, 500.0, 0, 0, 0);
        return;
    }

    // PD: for begining, PID for almost there
    if (err < I_ON_ZONE && err > -I_ON_ZONE) {
        err_acc += err;
        // limit for integrals
        if (err_acc > I_MAX) err_acc = I_MAX;
        if (err_acc < I_MIN) err_acc = I_MIN;
        i_flag = 1;
    } else {
        err_acc = 0;
        i_flag = 0;
    }

    pout = Kp * err;
    iout = i_flag * Ki * err_acc;
    dout = Kd * (err - err_prev);
    
    pid_temp = (int) (1000.0f * (pout + iout + dout) / 12.0f);
    pid_temp += 1000;
    pid_temp /= 2;

    // duty cycle offset
    if(pid_temp < 500 - PWM_DEAD_ZONE){
        pid_temp -= PWM_DEAD_ZONE_OFFSET_MINUS;
    }
    else if(pid_temp > 500 + PWM_DEAD_ZONE){
        pid_temp += PWM_DEAD_ZONE_OFFSET_PLUS;
    }
    else{
        pid_temp = 500;
    }

    // pwm limit
    if(pid_temp > 1000){
        pid_pwm = 1000;
    }
    else if (pid_temp < 0){
        pid_pwm = 0;
    }
    else {
        pid_pwm = pid_temp;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pid_pwm);
    err_prev = err;

    VOFA_JustFloat_Send(&huart1, (float)location, (float)target, (float)pid_pwm, (float)pout, (float)iout, (float)dout);
    return;
}
