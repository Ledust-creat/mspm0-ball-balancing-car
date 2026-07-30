#include "bsp_encoder.h"
#include "interrupt.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define WHEEL_DIAMETER_MM               (68.0f)     /* 轮胎直径，单位mm */
#define ENCODER_LINES_PER_WHEEL_REV     (330.0f)    /* 减速后编码器物理线数，单位line/rev */
#define ENCODER_DECODE_FACTOR           (2.0f)      /* A相双边沿二倍频计数系数 */
#define ENCODER_COUNTS_PER_WHEEL_REV    (ENCODER_LINES_PER_WHEEL_REV * ENCODER_DECODE_FACTOR)
#define SPEED_CALC_PERIOD_MS            (20U)       /* 速度计算备用周期，单位ms */

#define WHEEL_CIRCUMFERENCE_CM          (3.1416f * WHEEL_DIAMETER_MM / 10.0f)

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
volatile int32_t g_MotorA_Encoder_Count = 0;        /* 电机A编码器中断计数，单位count */
volatile int32_t g_MotorB_Encoder_Count = 0;        /* 电机B编码器中断计数，单位count */

static uint32_t g_last_calc_time = 0U;              /* 上次速度计算时刻，单位ms */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static int32_t encoder_apply_sign(int32_t pulse, int8_t sign);

/*
*********************************************************************************************************
*   函 数 名: encoder_apply_sign
*   功能说明: 应用编码器方向修正
*   形    参: pulse - 原始计数；sign - 方向修正系数，1或-1
*   返 回 值: 修正后的计数
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
    NVIC_ClearPendingIRQ(BSP_ENCODER_A_INT_IRQN);
    NVIC_EnableIRQ(BSP_ENCODER_A_INT_IRQN);
    NVIC_ClearPendingIRQ(BSP_ENCODER_B_INT_IRQN);
    NVIC_EnableIRQ(BSP_ENCODER_B_INT_IRQN);

    __disable_irq();
    g_MotorA_Encoder_Count = 0;
    g_MotorB_Encoder_Count = 0;
    __enable_irq();

    g_last_calc_time = Get_Tick();
}

/*
*********************************************************************************************************
*   函 数 名: Encoder_CalculateSpeed
*   功能说明: 计算电机实际速度，由20ms控制周期调用
*   形    参: _encoder - 输出编码器测速结果
*   返 回 值: 无
*********************************************************************************************************
*/
void Encoder_CalculateSpeed(Encoder_t *_encoder)
{
    int32_t count_a;
    int32_t count_b;
    uint32_t now;
    uint32_t dt_ms;
    float dt_s;

    if (_encoder == 0) {
        return;
    }

    __disable_irq();
    count_a = g_MotorA_Encoder_Count;
    count_b = g_MotorB_Encoder_Count;
    g_MotorA_Encoder_Count = 0;
    g_MotorB_Encoder_Count = 0;
    __enable_irq();

    count_a = encoder_apply_sign(count_a, BSP_ENCODER_A_SIGN);
    count_b = encoder_apply_sign(count_b, BSP_ENCODER_B_SIGN);

    now = Get_Tick();
    dt_ms = now - g_last_calc_time;
    g_last_calc_time = now;

    dt_s = (float)dt_ms / 1000.0f;
    if (dt_s < 0.001f) {
        dt_s = (float)SPEED_CALC_PERIOD_MS / 1000.0f;
    }

#if (BSP_ENCODER_DECODE_MODE != BSP_ENCODER_DECODE_X2)
#error "Unsupported BSP_ENCODER_DECODE_MODE"
#endif

    _encoder->motorA.sample_count = count_a;
    _encoder->motorA.direction = (count_a >= 0) ? 0U : 1U;
    _encoder->motorA.speed_cm_s = ((float)count_a / ENCODER_COUNTS_PER_WHEEL_REV) *
                                  WHEEL_CIRCUMFERENCE_CM / dt_s;

    _encoder->motorB.sample_count = count_b;
    _encoder->motorB.direction = (count_b >= 0) ? 0U : 1U;
    _encoder->motorB.speed_cm_s = ((float)count_b / ENCODER_COUNTS_PER_WHEEL_REV) *
                                  WHEEL_CIRCUMFERENCE_CM / dt_s;
}
