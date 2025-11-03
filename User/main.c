#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "Encoder.h"
#include "Timer.h"
#include "pid.h"
#include "Delay.h"

volatile uint8_t KeyNum;
volatile int8_t TargetSpeed = 20;
volatile int32_t CurrentSpeed1, CurrentSpeed2;
volatile uint8_t statu = 0;
volatile int64_t EncoderCount1 = 0;
volatile int64_t EncoderCount2 = 0;
volatile float kp = 2.0f;
volatile float ki = 0.5f;
volatile float kd = 0.1f;

PID_TypeDef Motor1_PID;
PID_TypeDef Motor2_PID;

int main(void)
{
    OLED_Init();
    Motor_Init();
    Key_Init();
    Encoder_Init();
    Timer_Init();

    PID_Init(&Motor1_PID);
    PID_Init(&Motor2_PID);
    //  OLED_ShowString(1, 1, "Speed:");

    while (1) {
        OLED_ShowSignedNum(1, 7, EncoderCount1, 6);
        OLED_ShowSignedNum(2, 7, EncoderCount2, 6);
        // OLED_ShowString(1, 1, "Speed:");
        OLED_ShowSignedNum(1, 1, CurrentSpeed1, 4);
        OLED_ShowSignedNum(2, 1, CurrentSpeed2, 4);

        KeyNum = Key_GetNum();
        if (statu == 0) {
            Motor_SetPWM(20);
            // 用pid控制电机速度
            // 控制速度
            // Motor_SetSpeed_PID(&Motor2, Speed2);

            if (KeyNum == 1) {
                statu = 1;
            }
        }
        else if (statu == 1) {
            // Motor_SetSpeed(Speed1, Speed2);
            // OLED_ShowSignedNum(1, 1, Speed1, 4);
            // OLED_ShowSignedNum(2, 1, Speed2, 4);
        }
    }
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        CurrentSpeed1 = Encoder1_Get();
        CurrentSpeed2 = Encoder2_Get();
        EncoderCount1 += CurrentSpeed1;
        EncoderCount2 += CurrentSpeed2;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        if (statu == 0) {
            Motor_UpdateSpeed();
        }
    }
}
