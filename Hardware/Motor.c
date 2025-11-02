#include "stm32f10x.h" // Device header
#include "PWM.h"
#include "pid.h"

typedef struct Motor {
    PID_TypeDef PID;

    int8_t TargetSpeed;
    int8_t CurrentSpeed;
    uint32_t EncoderCount;
} Motor_TypeDef;

Motor_TypeDef Motor1;
Motor_TypeDef Motor2;

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
}

// speed1: 电机1速度（-100~100），speed2: 电机2速度（-100~100）
void Motor_SetSpeed(int8_t speed1, int8_t speed2)
{
    // Motor1: PB12, PB13 控制方向，PA2(PWM3) 控制速度
    if (speed1 >= 0) {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    }
    else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        speed1 = -speed1;
    }
    if (speed1 > 100)
        speed1 = 100;
    PWM_SetCompare3(speed1);

    // Motor2: PB14, PB15 控制方向，PA3(PWM4) 控制速度
    if (speed2 >= 0) {
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    }
    else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        speed2 = -speed2;
    }
    if (speed2 > 100)
        speed2 = 100;
    PWM_SetCompare4(speed2);
}

void Motor_SetSpeed_PID(Motor_TypeDef *motor, float speed)
{
    motor->TargetSpeed = speed;
}

void Motor_Speed_Update(Motor_TypeDef *motor)
{
    float output = PID_Calculate(&motor->PID, motor->TargetSpeed, motor->CurrentSpeed);
    // 只转一个电机
    Motor_SetSpeed((int8_t)output, 0);
}

void Motor_Follow_Update(Motor_TypeDef *motor)
{

    float output = PID_Calculate(&motor->PID, motor->TargetSpeed, motor->CurrentSpeed);
    // 通过pid控制电机编码器计数值一致
}