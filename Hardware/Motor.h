#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "pid.h"

void Motor_Init(void);
void Motor_SetPWM(uint32_t pwm);
void Motor_UpdateSpeed(void);

#endif
