#include "bsp_encoder.h"
#include "interrupt.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define WHEEL_DIAMETER_MM       65.0f       /* 轮胎直径，单位mm */
#define ENCODER_PPR_REDUCED     265.2f      /* 减速后编码器物理线数 */
#define SPEED_CALC_PERIOD_MS    20U         /* 速度计算周期，单位ms */

#define WHEEL_CIRCUMFERENCE_CM  (3.1416f * WHEEL_DIAMETER_MM / 10.0f)

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
volatile int32_t g_MotorA_Encoder_Count = 0;    /* 电机A编码器中断计数 */
volatile int32_t g_MotorB_Encoder_Count = 0;    /* 电机B编码器中断计数 */

static uint64_t g_last_calc_time = 0;            /* 上次速度计算时间 */

/*
*********************************************************************************************************
*   函 数 名: encoder_apply_sign
*   功能说明: 应用编码器方向修正
*   形    参: pulse — 原始脉冲；sign — 方向修正系数
*   返 回 值: 修正后的脉冲
*********************************************************************************************************
*/
static int32_t encoder_apply_sign(int32_t pulse, int8_t sign)
{
    return (sign < 0) ? -pulse : pulse;
}

/*
*********************************************************************************************************
*   函 数 名: Encoder_Init
*   功能说明: 初始化编码器数据和GPIO中断
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void Encoder_Init(void)
{
    NVIC_ClearPendingIRQ(BSP_ENCODER_GPIO_INT_IRQN);
    NVIC_EnableIRQ(BSP_ENCODER_GPIO_INT_IRQN);
    __disable_irq();
    g_MotorA_Encoder_Count = 0;
    g_MotorB_Encoder_Count = 0;
    __enable_irq();

    g_last_calc_time = Get_Tick();
}

/*
*********************************************************************************************************
*   函 数 名: Encoder_CalculateSpeed
*   功能说明: 计算电机实际速度，由20ms定时器调用
*   形    参: _encoder — 输出计算结果
*   返 回 值: 无
*********************************************************************************************************
*/
void Encoder_CalculateSpeed(Encoder_t *_encoder)
{
    int32_t pulseA_raw;
    int32_t pulseB_raw;
    int32_t pulseA;
    int32_t pulseB;
    uint64_t now;
    uint64_t dt_ms;
    float dt_s;

    if (_encoder == 0) {
        return;
    }

    __disable_irq();
    pulseA_raw = g_MotorA_Encoder_Count;
    pulseB_raw = g_MotorB_Encoder_Count;
    g_MotorA_Encoder_Count = 0;
    g_MotorB_Encoder_Count = 0;
    __enable_irq();

    pulseA_raw = encoder_apply_sign(pulseA_raw, BSP_ENCODER_A_SIGN);
    pulseB_raw = encoder_apply_sign(pulseB_raw, BSP_ENCODER_B_SIGN);
    
    now = Get_Tick();
    dt_ms = now - g_last_calc_time;
    g_last_calc_time = now;

    dt_s = (float)dt_ms / 1000.0f;
    if (dt_s < 0.001f) {
        dt_s = (float)SPEED_CALC_PERIOD_MS / 1000.0f;
    }

#if (BSP_ENCODER_DECODE_MODE == BSP_ENCODER_DECODE_X2)
    pulseA = pulseA_raw / (int32_t)BSP_ENCODER_DECODE_X2;
    pulseB = pulseB_raw / (int32_t)BSP_ENCODER_DECODE_X2;
#else
#error "Unsupported BSP_ENCODER_DECODE_MODE"
#endif

    _encoder->motorA.pulse_count = pulseA_raw;
    _encoder->motorA.real_pulse = pulseA;
    _encoder->motorA.direction = (pulseA >= 0) ? 0U : 1U;
    _encoder->motorA.speed_cm_s = ((float)pulseA / ENCODER_PPR_REDUCED) *
                                  WHEEL_CIRCUMFERENCE_CM / dt_s;

    _encoder->motorB.pulse_count = pulseB_raw;
    _encoder->motorB.real_pulse = pulseB;
    _encoder->motorB.direction = (pulseB >= 0) ? 0U : 1U;
    _encoder->motorB.speed_cm_s = ((float)pulseB / ENCODER_PPR_REDUCED) *
                                  WHEEL_CIRCUMFERENCE_CM / dt_s;
}
