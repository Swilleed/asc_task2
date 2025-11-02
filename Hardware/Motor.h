#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "pid.h"

typedef struct Motor {
    PID_TypeDef PID;

    int8_t TargetSpeed;
    int8_t CurrentSpeed;
    int32_t EncoderCount;
} Motor_TypeDef;

extern Motor_TypeDef Motor1;
extern Motor_TypeDef Motor2;

void Motor_Init(void);
void Motor_SetSpeed(Motor_TypeDef *motor, int16_t speed);
void Motor_Speed_Update(Motor_TypeDef *motor);
void Motor_SetDirection(Motor_TypeDef *motor, int16_t speed);

#endif
