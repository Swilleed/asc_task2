#include "stm32f10x.h" // Device header
#include "Motor.h"
#include "PWM.h"
#include "Encoder.h"
#include "pid.h"
#include "OLED.h"

extern volatile int32_t TargetSpeed;
extern volatile int32_t CurrentSpeed1;
extern volatile int32_t CurrentSpeed2;
extern volatile int64_t EncoderCount1;
extern volatile int64_t EncoderCount2;
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

    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    GPIO_SetBits(GPIOB, GPIO_Pin_13);

    PWM_Init();
}

void Motor_SetPWM(int32_t pwm)
{
    int32_t magnitude = pwm;

    if (magnitude >= 0) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
    }
    else {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        magnitude = -magnitude;
    }

    if (magnitude > MOTOR_PWM_MAX) {
        magnitude = MOTOR_PWM_MAX;
    }

    PWM_SetCompare3((uint16_t)magnitude);
}

void Motor_UpdateSpeed(void)
{
    float output = PID_Calculate(&Motor1_PID, (float)TargetSpeed, (float)CurrentSpeed1);
    Motor1_PID.Output = output;
    Motor_SetPWM((int32_t)Motor1_PID.Output);
    OLED_ShowSignedNum(4, 1, (int32_t)(Motor1_PID.Output), 4);
}

void Motor_Follow_Position(void)
{
    const float target = (float)EncoderCount2;
    const float actual = (float)EncoderCount1;

    float output = PID_Calculate(&Motor2_PID, target, actual);
    Motor2_PID.Output = output;

    if (output > -1.5f && output < 1.5f) {
        output = 0.0f;
    }

    Motor_SetPWM((int32_t)output);
    OLED_ShowSignedNum(4, 6, (int32_t)(output), 4);
}
