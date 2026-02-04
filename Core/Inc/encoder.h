// notice: encoder is using timer3
// external variables: 
// uint16_t speed, CNT taken from timer3, every 40ms
// uint32_t loaction, sum of all speeds
// remember to enable encoder mode after sys initialization

#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include <stdint.h>


extern TIM_HandleTypeDef htim3;

int16_t get_encoder(void);

#endif
