#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "Encoder.h"
#include "Timer.h"
#include "pid.h"
#include "Serial.h"

volatile uint8_t KeyNum;
volatile int32_t TargetSpeed = 50;
volatile int32_t CurrentSpeed1, CurrentSpeed2;
volatile uint8_t statu = 0;
volatile int64_t EncoderCount1 = 0;
volatile int64_t EncoderCount2 = 0;
volatile float kp = 0.8f;
volatile float ki = 0.1f;
volatile float kd = 0.2f;

PID_TypeDef Motor1_PID;
PID_TypeDef Motor2_PID;
volatile uint8_t SpeedReportFlag = 0;

int main(void)
{
    OLED_Init();
    Motor_Init();
    Key_Init();
    Encoder_Init();
    Timer_Init();
    Serial_Init();

    PID_Init(&Motor1_PID);
    PID_Init(&Motor2_PID);

    while (1) {
        OLED_ShowSignedNum(1, 7, EncoderCount1, 6);
        OLED_ShowSignedNum(2, 7, EncoderCount2, 6);
        OLED_ShowSignedNum(1, 1, CurrentSpeed1, 4);
        OLED_ShowSignedNum(2, 1, CurrentSpeed2, 4);
        OLED_ShowSignedNum(3, 1, TargetSpeed, 4);

        int16_t requestedSpeed;
        if (Serial_TryParseTarget(&requestedSpeed)) {
            TargetSpeed = requestedSpeed;
            Serial_Printf("@ACK:%d\r\n", TargetSpeed);
        }

        if (SpeedReportFlag) {
            SpeedReportFlag = 0;
            Serial_Printf("@CUR:%ld\r\n", (long)CurrentSpeed1);
        }

        if (statu == 0) {
            OLED_ShowNum(3, 10, 0, 2);
            if (Key_Check(KEY_1, KEY_SINGLE)) {
                Motor_SetPWM(0);
                EncoderCount1 = 0;
                EncoderCount2 = 0;
                statu = 1;
            }
        }
        else if (statu == 1) {
            OLED_ShowNum(3, 10, 1, 2);
            // TODO: add additional runtime key handling if needed
        }
    }
}

void TIM1_UP_IRQHandler(void)
{
    Key_Tick();

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        CurrentSpeed1 = Encoder1_Get();
        CurrentSpeed2 = Encoder2_Get();
        EncoderCount1 += CurrentSpeed1;
        EncoderCount2 += CurrentSpeed2;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        if (statu == 0) {
            Motor_UpdateSpeed();
        }
        else if (statu == 1) {
            Motor_Follow_Position();
        }

        SpeedReportFlag = 1;
    }
}
