#include "stm32f10x.h" // Device header
#include "PWM.h"
#include "pid.h"
#include "Encoder.h"

extern int8_t CurrentSpeed1;
extern int8_t CurrentSpeed2;

typedef struct Motor {
    PID_TypeDef PID;

    int8_t TargetSpeed;
    int8_t CurrentSpeed;
    // uint32_t EncoderCount;
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

void Motor_SetSpeed(Motor_TypeDef *motor, int8_t speed)
{
    motor->TargetSpeed = speed;
    PWM_SetCompare3(speed);
}

void Motor_Speed_Update(Motor_TypeDef *motor)
{
    motor->CurrentSpeed = (motor == &Motor1) ? CurrentSpeed1 : CurrentSpeed2;
    // 更新电机速度
    PID_Calculate(&motor->PID, motor->TargetSpeed, motor->CurrentSpeed);
    PWM_SetCompare3((uint16_t)motor->PID.Output);
}
