#include "stm32f10x.h" // Device header
#include "Motor.h"
#include "PWM.h"

extern int8_t CurrentSpeed1;
extern int8_t CurrentSpeed2;

#define MOTOR_PWM_MAX 99

Motor_TypeDef Motor1;
Motor_TypeDef Motor2;

static uint16_t Motor_ClampDuty(int16_t value)
{
    if (value < 0) {
        value = -value;
    }
    if (value > MOTOR_PWM_MAX) {
        value = MOTOR_PWM_MAX;
    }
    return (uint16_t)value;
}

static void Motor_WriteDuty(Motor_TypeDef *motor, uint16_t duty)
{
    if (motor == &Motor1) {
        PWM_SetCompare3(duty);
    }
    else if (motor == &Motor2) {
        PWM_SetCompare4(duty);
    }
}

static void Motor_SetDirectionPins(Motor_TypeDef *motor, int16_t command)
{
    if (motor == &Motor1) {
        if (command >= 0) {
            GPIO_SetBits(GPIOB, GPIO_Pin_12);
            GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        }
        else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
            GPIO_SetBits(GPIOB, GPIO_Pin_13);
        }
    }
    else if (motor == &Motor2) {
        if (command >= 0) {
            GPIO_SetBits(GPIOB, GPIO_Pin_14);
            GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        }
        else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
            GPIO_SetBits(GPIOB, GPIO_Pin_15);
        }
    }
}

void Motor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    PWM_Init();

    PID_Init(&Motor1.PID);
    PID_Init(&Motor2.PID);

    Motor1.TargetSpeed = 0;
    Motor1.CurrentSpeed = 0;
    Motor1.EncoderCount = 0;
    Motor2.TargetSpeed = 0;
    Motor2.CurrentSpeed = 0;
    Motor2.EncoderCount = 0;

    Motor_SetDirectionPins(&Motor1, 0);
    Motor_SetDirectionPins(&Motor2, 0);
    Motor_WriteDuty(&Motor1, 0);
    Motor_WriteDuty(&Motor2, 0);
}

void Motor_SetSpeed(Motor_TypeDef *motor, int16_t speed)
{
    if (!motor) {
        return;
    }

    motor->TargetSpeed = speed;
    Motor_SetDirectionPins(motor, speed);
    Motor_WriteDuty(motor, Motor_ClampDuty(speed));
}

void Motor_SetDirection(Motor_TypeDef *motor, int16_t speed)
{
    if (!motor) {
        return;
    }

    Motor_SetDirectionPins(motor, speed);
}

void Motor_Speed_Update(Motor_TypeDef *motor)
{
    if (!motor) {
        return;
    }

    motor->CurrentSpeed = (motor == &Motor1) ? CurrentSpeed1 : CurrentSpeed2;
    motor->EncoderCount += motor->CurrentSpeed;

    float output = PID_Calculate(&motor->PID, motor->TargetSpeed, motor->CurrentSpeed);
    Motor_SetDirectionPins(motor, (int16_t)output);
    Motor_WriteDuty(motor, Motor_ClampDuty((int16_t)output));
}

void Motor_Follow_Target(Motor_TypeDef *motor)
{
    if (!motor) {
        return;
    }

    motor->CurrentSpeed = (motor == &Motor1) ? CurrentSpeed1 : CurrentSpeed2;
    motor->EncoderCount += motor->CurrentSpeed;

    float output = PID_Calculate(&motor->PID, motor->TargetSpeed, motor->CurrentSpeed);
    Motor_SetDirectionPins(motor, (int16_t)output);
    Motor_WriteDuty(motor, Motor_ClampDuty((int16_t)output));
}
void Motor_Encoder_Update(Motor_TypeDef *motor)
{
    motor->CurrentSpeed = (motor == &Motor1) ? CurrentSpeed1 : CurrentSpeed2;
    motor->EncoderCount += motor->CurrentSpeed;
}
