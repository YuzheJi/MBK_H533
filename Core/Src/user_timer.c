#include "main.h"
#include "elec391_533.h"
#include "encoder.h"
#include "pid.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  // timer 2 for general timer count
  if (htim->Instance == TIM2){

    // frame scaling
    msec_count_frame++;
    if(msec_count_frame >= FRAME_RATE){
        msec_count_frame = 0;
        main_update = 1;
        BSP_LED_Toggle(LED_GREEN);
    }
  }

  // timer 4 for measurement
  else if(htim->Instance == TIM4){

    // measure scaling 
    half_msec_count_measure++;
    if(half_msec_count_measure >= MEASURE_RATE * 2){
      half_msec_count_measure = 0;

      counter = get_encoder();
      counter_acc += counter;

      speed = (float) counter * 60.0f * 100.0f / 44.0f / GEAR; 
      rad_s = (float) counter * 2 * PI / 44.0f / GEAR;
      location = counter_acc * 360.0f / 44.0f / GEAR; 

      pid_control();
    }
  }
}
