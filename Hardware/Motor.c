#include "stm32f10x.h" // Device header
#include "Motor.h"
#include "PWM.h"
#include "Encoder.h"
#include "pid.h"

extern volatile int8_t TargetSpeed;
extern volatile int32_t CurrentSpeed1;
extern volatile int32_t CurrentSpeed2;
extern PID_TypeDef Motor1_PID;
extern PID_TypeDef Motor2_PID;

#define MOTOR_PWM_MAX 99

void Motor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);

    PWM_Init();
}

void Motor_SetPWM(uint32_t pwm)
{
    if (pwm > MOTOR_PWM_MAX) {
        pwm = MOTOR_PWM_MAX;
    }
    PWM_SetCompare3(pwm);
}

void Motor_UpdateSpeed(void)
{
    float output = PID_Calculate(&Motor1_PID, (float)TargetSpeed, (float)CurrentSpeed1);
    Motor1_PID.Output = output;
    Motor_SetPWM((uint32_t)(Motor1_PID.Output));
}
