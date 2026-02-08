#ifndef __HOMING_CONTROL_H
#define __HOMING_CONTROL_H

#include "main.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim1;

void homing_control(uint8_t dir);

#endif