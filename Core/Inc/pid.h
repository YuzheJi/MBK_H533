#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

void pid_control(void);

#endif
