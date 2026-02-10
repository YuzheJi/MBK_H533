#include "main.h"
#include "elec391_533.h"
#include "encoder.h"
#include "pid.h"
#include "VOFA.h"
#include "hc04.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  // timer 2 for general timer count
  if (htim->Instance == TIM2){

    // frame scaling
    msec_count_frame++;
    if(msec_count_frame >= FRAME_RATE){
        msec_count_frame = 0;
        main_update = 1;
    }

    // event checking
    if(system_mode == 3 && mode3_state == 1){

      target = SONG[event_index].positions[0];
      if(settle_count >= SETTLE_TH){
        mode3_state = 2;
        press_count = 0;
      }
    }

    if(system_mode == 3 && mode3_state == 2){

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      if(press_count >= SONG[event_index].duration_ms){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        if(event_index == SONG_EVENT - 1){
          system_mode = 0;
        }
        else{
          mode3_state = 1;
          event_index ++;
          settle_count = 0;
        }  
      }
    }
  }

  // timer 4 for measurement
  else if(htim->Instance == TIM4){

    // measure scaling 
    half_msec_count_measure++;

    if(half_msec_count_measure >= MEASURE_RATE){
      
      half_msec_count_measure = 0;
      counter = get_encoder();
      counter_acc += counter;

      speed = (float) counter * 60.0f * 1000.0f / 44.0f / GEAR; 
      rad_s = (float) counter * 2 * PI * 1000.0f / 44.0f;
      location = counter_acc * 360.0f / 44.0f / GEAR; 

      // VOFA_JustFloat_Send(&huart1, (float)speed, (float)location, 0, 0, 0, 0);

      if(system_mode == 0){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
      }
      else if(system_mode == 1){
        pid_control();
      }

      if(system_mode == 3){
        if(mode3_state == 1 || mode3_state == 2) pid_control();
        else if(mode3_state != 0) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
      }
    }

    // homing logic 
    if(system_mode == 2 || (system_mode == 3 && mode3_state == 0)){
      homing_flag = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
      if(homing_flag == 1){
        BSP_LED_On(LED_GREEN);
      }
    }

    BSP_LED_Off(LED_GREEN);
  }

  // timer 5 for counting time
  else if(htim->Instance == TIM5){
    if(system_mode == 3 && mode3_state == 2){
      press_count++;
    }
  }
}
