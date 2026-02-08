#include "main.h"
#include "elec391_533.h"
#include "encoder.h"
#include "pid.h"
#include "VOFA.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  // timer 2 for general timer count
  if (htim->Instance == TIM2){

    // frame scaling
    msec_count_frame++;
    if(msec_count_frame >= FRAME_RATE){
        msec_count_frame = 0;
        main_update = 1;
    }
  }

  // timer 4 for measurement
  else if(htim->Instance == TIM4){

    // measure scaling 
    half_msec_count_measure++;

    // pid control
    if(system_mode == 0 && half_msec_count_measure >= MEASURE_RATE * 2){
      
      half_msec_count_measure = 0;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
      counter = get_encoder();
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      counter_acc += counter;

      speed = (float) counter * 60.0f * 100.0f / 44.0f / GEAR; 
      rad_s = (float) counter * 2 * PI * 100.0f / 44.0f;
      location = counter_acc * 360.0f / 44.0f / GEAR; 

      // VOFA_JustFloat_Send(&huart1, rad_s, 0, 0, 0, 0, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
    }

    else if(system_mode == 1 && half_msec_count_measure >= MEASURE_RATE * 2){
      
      half_msec_count_measure = 0;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
      counter = get_encoder();
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      counter_acc += counter;

      speed = (float) counter * 60.0f * 100.0f / 44.0f / GEAR; 
      rad_s = (float) counter * 2 * PI * 100.0f / 44.0f;
      location = counter_acc * 360.0f / 44.0f / GEAR; 

      // VOFA_JustFloat_Send(&huart1, rad_s, 0, 0, 0, 0, 0);
      pid_control();
    }

    // homing logic 
    else if(system_mode == 2){
      homing_count = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
      if(homing_count == 1){
        BSP_LED_On(LED_GREEN);
      }
    }

    BSP_LED_Off(LED_GREEN);
  }
}
