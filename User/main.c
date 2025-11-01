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
int8_t Speed1, Speed2;
int8_t CurrentSpeed1, CurrentSpeed2;
uint8_t statu = 0;
int64_t EncoderCount1 = 0;
int64_t EncoderCount2 = 0;

int main(void)
{
    OLED_Init();
    Motor_Init();
    Key_Init();
    Encoder_Init();
    Timer_Init();

    OLED_ShowString(1, 1, "Speed:");

    while (1) {
        OLED_ShowNum(1, 13, EncoderCount1, 5);
        OLED_ShowNum(2, 13, EncoderCount2, 5);
        OLED_ShowString(1, 1, "Speed:");
        OLED_ShowNum(1, 7, Speed1, 3);
        OLED_ShowNum(2, 7, Speed2, 3);

        KeyNum = Key_GetNum();
        if (statu == 0) {
            Motor_SetSpeed(20, 100);
        }
        else if (statu == 1) {
            Motor_SetSpeed(Speed1, Speed2);
            OLED_ShowSignedNum(1, 7, Speed1, 3);
            OLED_ShowSignedNum(2, 7, Speed2, 3);
        }
    }
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        Speed1 = Encoder1_Get();
        Speed2 = Encoder2_Get();
        EncoderCount1 += Speed1;
        EncoderCount2 += Speed2;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
