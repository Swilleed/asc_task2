#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "Encoder.h"
#include "Timer.h"
#include "pid.h"
#include "Serial.h"

#include <inttypes.h>

typedef enum {
    CTRL_MODE_SPEED = 0,
    CTRL_MODE_POSITION = 1
} control_mode_t;

volatile control_mode_t control_mode = CTRL_MODE_SPEED; // 当前任务状态

volatile int32_t TargetSpeed = 50;
volatile int32_t CurrentSpeed1, CurrentSpeed2;
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
        /* 原子读取 64-bit 编码器计数：短时间禁中断并拷贝到本地变量 */
        int64_t enc1_copy, enc2_copy;
        __disable_irq();
        enc1_copy = EncoderCount1;
        enc2_copy = EncoderCount2;
        __enable_irq();

        OLED_ShowSignedNum(1, 7, enc1_copy, 6);
        OLED_ShowSignedNum(2, 7, enc2_copy, 6);
        OLED_ShowSignedNum(1, 1, CurrentSpeed1, 4);
        OLED_ShowSignedNum(2, 1, CurrentSpeed2, 4);
        OLED_ShowSignedNum(3, 1, TargetSpeed, 4);

        // 处理串口命令
        int16_t requestedSpeed;
        if (Serial_TryParseTarget(&requestedSpeed)) {
            TargetSpeed = requestedSpeed;
            Serial_Printf("@ACK:%d\r\n", TargetSpeed);
        }

        if (SpeedReportFlag) {
            SpeedReportFlag = 0;
            Serial_Printf("@CUR:%ld\r\n", (long)CurrentSpeed1);
        }

        if (control_mode == CTRL_MODE_SPEED) {
            OLED_ShowString(3, 10, "SpeedControl");
            if (Key_Check(KEY_1, KEY_SINGLE)) {
                // 状态切换到位置控制
                Motor_SetPWM(0);
                kp = 0.4f;
                ki = 0.02f;
                kd = 0.1f;
                PID_Init(&Motor2_PID);
                EncoderCount1 = 0;
                EncoderCount2 = 0;
                control_mode = CTRL_MODE_POSITION;
            }
        }
        else if (control_mode == CTRL_MODE_POSITION) {
            OLED_ShowString(3, 10, "PositionControl");
            if (Key_Check(KEY_1, KEY_SINGLE)) {
                Motor_SetPWM(0);
                kp = 0.8f;
                ki = 0.1f;
                kd = 0.2f;
                PID_Init(&Motor1_PID);
                EncoderCount1 = 0;
                EncoderCount2 = 0;
                control_mode = CTRL_MODE_SPEED;
            }
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

        if (control_mode == CTRL_MODE_SPEED) {
            Motor_UpdateSpeed();
        }
        else if (control_mode == CTRL_MODE_POSITION) {
            Motor_Follow_Position();
        }

        SpeedReportFlag = 1;
    }
}
