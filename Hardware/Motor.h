#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
typedef struct Motor {
    PID_TypeDef PID;

    int8_t TargetSpeed;
    int8_t CurrentSpeed;
    uint32_t EncoderCount;
} Motor_TypeDef;

void Motor_Init(void);
void Motor_SetSpeed(Motor_TypeDef *motor, int8_t speed);
void Motor_SetSpeed_PID(Motor_TypeDef *motor, int8_t targetSpeed);
void Motor_SetSpeed(Motor_TypeDef *motor, int8_t speed);
void Motor_Speed_Update(Motor_TypeDef *motor);

#endif
