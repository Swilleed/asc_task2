#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "Encoder.h"
#include "Timer.h"
#include "pid.h"
#include "Delay.h"

uint8_t KeyNum;
int8_t TargetSpeed;
;
int8_t CurrentSpeed1, CurrentSpeed2;
uint8_t statu = 0;
int64_t EncoderCount1 = 0;
int64_t EncoderCount2 = 0;
extern Motor_TypeDef Motor1;

int main(void)
{
    OLED_Init();
    Motor_Init();
    Key_Init();
    Encoder_Init();
    Timer_Init();

    //  OLED_ShowString(1, 1, "Speed:");

    while (1) {
        OLED_ShowSignedNum(1, 7, EncoderCount1, 6);
        OLED_ShowSignedNum(2, 7, EncoderCount2, 6);
        // OLED_ShowString(1, 1, "Speed:");
        OLED_ShowSignedNum(1, 1, Speed1, 3);
        OLED_ShowSignedNum(2, 1, Speed2, 3);

        KeyNum = Key_GetNum();
        if (statu == 0) {
            Motor_SetSpeed(40, 0);
            // 用pid控制电机速度
            // 控制速度
            Motor_SetSpeed_PID(&Motor1, 40);
            Motor_Speed_Update(&Motor1);
            // Motor_SetSpeed_PID(&Motor2, Speed2);

            if (KeyNum == 1) {
                statu = 1;
            }
        }
        else if (statu == 1) {
            // Motor_SetSpeed(Speed1, Speed2);
            OLED_ShowSignedNum(1, 1, Speed1, 4);
            OLED_ShowSignedNum(2, 1, Speed2, 4);
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
    }
}
