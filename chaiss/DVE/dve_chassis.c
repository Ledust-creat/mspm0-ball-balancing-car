#include "dve_chassis.h"
#include "bsp_chassis_motor.h"
#include "bsp_encoder.h"
#include "ti_msp_dl_config.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#ifndef DVE_CHASSIS_DEFAULT_LEFT_SIGN
#define DVE_CHASSIS_DEFAULT_LEFT_SIGN       (-1)        /* 左电机方向修正 */
#endif

#ifndef DVE_CHASSIS_DEFAULT_RIGHT_SIGN
#define DVE_CHASSIS_DEFAULT_RIGHT_SIGN      (-1)        /* 右电机方向修正 */
#endif

#define DVE_CHASSIS_LEFT_SPEED_KP           (36.0f)     /* 左轮速度环初始Kp */
#define DVE_CHASSIS_LEFT_SPEED_KI           (1.5f)      /* 左轮速度环初始Ki */
#define DVE_CHASSIS_LEFT_SPEED_KD           (0.0f)      /* 左轮速度环初始Kd */

#define DVE_CHASSIS_RIGHT_SPEED_KP          (35.0f)     /* 右轮速度环初始Kp */
#define DVE_CHASSIS_RIGHT_SPEED_KI          (1.5f)      /* 右轮速度环初始Ki */
#define DVE_CHASSIS_RIGHT_SPEED_KD          (0.0f)      /* 右轮速度环初始Kd */

#define DVE_CHASSIS_SPEED_MAX_CM_S          (80.0f)     /* 目标轮速限幅 */
#define DVE_CHASSIS_SPEED_PID_MAX_IOUT      (1000.0f)   /* 速度环积分限幅 */
#define DVE_CHASSIS_SPEED_PID_MAX_OUT       ((float)BSP_CHASSIS_MOTOR_PWM_MAX)

#define DVE_CHASSIS_ENCODER_MAP_NORMAL      (0U)        /* EncoderA=左轮，EncoderB=右轮 */
#define DVE_CHASSIS_ENCODER_MAP_SWAPPED     (1U)        /* EncoderA=右轮，EncoderB=左轮 */

#ifndef DVE_CHASSIS_ENCODER_MAP
#define DVE_CHASSIS_ENCODER_MAP             DVE_CHASSIS_ENCODER_MAP_SWAPPED
#endif

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static bool s_chassis_inited = false;          /* 底盘初始化标志 */
static bool s_chassis_enable = false;          /* 底盘使能标志 */
static volatile bool s_control_flag = false;   /* 20ms控制周期标志 */

static float s_left_pwm_ratio = 1.00f;         /* 左轮PWM补偿系数 */
static float s_right_pwm_ratio = 1.00f;        /* 右轮PWM补偿系数 */

static PID_t s_left_speed_pid;                 /* 左轮速度PID */
static PID_t s_right_speed_pid;                /* 右轮速度PID */
static Encoder_t s_encoder;                    /* 编码器测速数据 */

static float s_left_target_cm_s = 0.0f;        /* 左轮目标速度 */
static float s_right_target_cm_s = 0.0f;       /* 右轮目标速度 */
static float s_left_actual_cm_s = 0.0f;        /* 左轮实际速度 */
static float s_right_actual_cm_s = 0.0f;       /* 右轮实际速度 */
static float s_left_pwm_out = 0.0f;            /* 左轮PWM输出 */
static float s_right_pwm_out = 0.0f;           /* 右轮PWM输出 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static int16_t float_to_i16(float value);
static float limit_float(float value, float min, float max);
static void chassis_timer_init(void);
static void speed_pid_reset(void);

/*
*********************************************************************************************************
*   函 数 名: float_to_i16
*   功能说明: float转int16_t，带四舍五入
*   形    参: value — 输入浮点值
*   返 回 值: 转换后的int16_t
*********************************************************************************************************
*/
static int16_t float_to_i16(float value)
{
    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    } else {
        return (int16_t)(value - 0.5f);
    }
}

/*
*********************************************************************************************************
*   函 数 名: limit_float
*   功能说明: 浮点限幅
*   形    参: value — 输入值；min — 最小值；max — 最大值
*   返 回 值: 限幅后的值
*********************************************************************************************************
*/
static float limit_float(float value, float min, float max)
{
    if (value > max) {
        value = max;
    }

    if (value < min) {
        value = min;
    }

    return value;
}

/*
*********************************************************************************************************
*   函 数 名: chassis_timer_init
*   功能说明: 初始化底盘20ms控制定时器中断
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void chassis_timer_init(void)
{
    DL_TimerG_clearInterruptStatus(TIMER_Chassis_Motor_Encoder_Read_INST,
                                   DL_TIMER_INTERRUPT_ZERO_EVENT);

    NVIC_ClearPendingIRQ(TIMER_Chassis_Motor_Encoder_Read_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_Chassis_Motor_Encoder_Read_INST_INT_IRQN);

    DL_TimerG_startCounter(TIMER_Chassis_Motor_Encoder_Read_INST);
}

/*
*********************************************************************************************************
*   函 数 名: speed_pid_reset
*   功能说明: 清除左右速度环历史状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void speed_pid_reset(void)
{
    PID_Reset(&s_left_speed_pid);
    PID_Reset(&s_right_speed_pid);
    s_left_pwm_out = 0.0f;
    s_right_pwm_out = 0.0f;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_Init
*   功能说明: 初始化底盘设备层
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_Init(void)
{
    BSP_ChassisMotor_Init();
    Encoder_Init();

    PID_Init(&s_left_speed_pid,
             DVE_CHASSIS_LEFT_SPEED_KP,
             DVE_CHASSIS_LEFT_SPEED_KI,
             DVE_CHASSIS_LEFT_SPEED_KD,
             DVE_CHASSIS_SPEED_PID_MAX_IOUT,
             DVE_CHASSIS_SPEED_PID_MAX_OUT);
    PID_Init(&s_right_speed_pid,
             DVE_CHASSIS_RIGHT_SPEED_KP,
             DVE_CHASSIS_RIGHT_SPEED_KI,
             DVE_CHASSIS_RIGHT_SPEED_KD,
             DVE_CHASSIS_SPEED_PID_MAX_IOUT,
             DVE_CHASSIS_SPEED_PID_MAX_OUT);

    s_left_target_cm_s = 0.0f;
    s_right_target_cm_s = 0.0f;
    s_left_actual_cm_s = 0.0f;
    s_right_actual_cm_s = 0.0f;
    s_control_flag = false;
    s_chassis_enable = false;
    s_chassis_inited = true;

    chassis_timer_init();
    DVE_Chassis_Stop();
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_Enable
*   功能说明: 设置底盘使能状态
*   形    参: enable — true使能，false失能
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_Enable(bool enable)
{
    s_chassis_enable = enable;

    if (!enable) {
        DVE_Chassis_Stop();
    } else {
        speed_pid_reset();
    }
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_Stop
*   功能说明: 停止底盘左右电机
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_Stop(void)
{
    s_left_target_cm_s = 0.0f;
    s_right_target_cm_s = 0.0f;
    speed_pid_reset();
    BSP_ChassisMotor_StopAll();
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_SetPwmRatio
*   功能说明: 设置左右轮PWM补偿系数
*   形    参: left_ratio — 左轮补偿系数；right_ratio — 右轮补偿系数
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_SetPwmRatio(float left_ratio, float right_ratio)
{
    s_left_pwm_ratio = limit_float(left_ratio, 0.50f, 1.50f);
    s_right_pwm_ratio = limit_float(right_ratio, 0.50f, 1.50f);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_SetPwm
*   功能说明: 设置左右电机PWM，开环调试接口
*   形    参: left_pwm — 左电机PWM；right_pwm — 右电机PWM
*   返 回 值: DVE_CHASSIS_OK或错误码
*********************************************************************************************************
*/
DVE_ChassisStatus_t DVE_Chassis_SetPwm(int16_t left_pwm, int16_t right_pwm)
{
    if (!s_chassis_inited) {
        return DVE_CHASSIS_ERR_NOT_INIT;
    }

    if (!s_chassis_enable) {
        BSP_ChassisMotor_StopAll();
        return DVE_CHASSIS_OK;
    }

    left_pwm = (int16_t)((float)left_pwm * s_left_pwm_ratio);
    right_pwm = (int16_t)((float)right_pwm * s_right_pwm_ratio);

    left_pwm = (int16_t)(left_pwm * DVE_CHASSIS_DEFAULT_LEFT_SIGN);
    right_pwm = (int16_t)(right_pwm * DVE_CHASSIS_DEFAULT_RIGHT_SIGN);

    s_left_pwm_out = (float)left_pwm;
    s_right_pwm_out = (float)right_pwm;

    BSP_ChassisMotor_SetLeftPwm(left_pwm);
    BSP_ChassisMotor_SetRightPwm(right_pwm);

    return DVE_CHASSIS_OK;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_SetTracePwm
*   功能说明: 根据基础PWM和转向修正量输出左右轮差速，开环循迹接口
*   形    参: base_pwm — 基础PWM；turn_pwm — 转向修正量
*   返 回 值: DVE_CHASSIS_OK或错误码
*********************************************************************************************************
*/
DVE_ChassisStatus_t DVE_Chassis_SetTracePwm(float base_pwm, float turn_pwm)
{
    int16_t left_pwm;
    int16_t right_pwm;

    left_pwm = float_to_i16(base_pwm - turn_pwm);
    right_pwm = float_to_i16(base_pwm + turn_pwm);

    return DVE_Chassis_SetPwm(left_pwm, right_pwm);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_SetSpeedPid
*   功能说明: 设置左右轮速度环PID参数
*   形    参: kp, ki, kd — PID参数
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_SetSpeedPid(float kp, float ki, float kd)
{
    DVE_Chassis_SetLeftSpeedPid(kp, ki, kd);
    DVE_Chassis_SetRightSpeedPid(kp, ki, kd);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_SetLeftSpeedPid
*   功能说明: 设置左轮速度环PID参数
*   形    参: kp, ki, kd — PID参数
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_SetLeftSpeedPid(float kp, float ki, float kd)
{
    PID_SetParam(&s_left_speed_pid, kp, ki, kd,
                 DVE_CHASSIS_SPEED_PID_MAX_IOUT,
                 DVE_CHASSIS_SPEED_PID_MAX_OUT);
    PID_Reset(&s_left_speed_pid);
    s_left_pwm_out = 0.0f;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_SetRightSpeedPid
*   功能说明: 设置右轮速度环PID参数
*   形    参: kp, ki, kd — PID参数
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_SetRightSpeedPid(float kp, float ki, float kd)
{
    PID_SetParam(&s_right_speed_pid, kp, ki, kd,
                 DVE_CHASSIS_SPEED_PID_MAX_IOUT,
                 DVE_CHASSIS_SPEED_PID_MAX_OUT);
    PID_Reset(&s_right_speed_pid);
    s_right_pwm_out = 0.0f;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_SetSpeedTargetCmps
*   功能说明: 设置左右轮目标速度
*   形    参: left_cm_s — 左轮目标速度；right_cm_s — 右轮目标速度
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_SetSpeedTargetCmps(float left_cm_s, float right_cm_s)
{
    s_left_target_cm_s = limit_float(left_cm_s,
                                     -DVE_CHASSIS_SPEED_MAX_CM_S,
                                     DVE_CHASSIS_SPEED_MAX_CM_S);
    s_right_target_cm_s = limit_float(right_cm_s,
                                      -DVE_CHASSIS_SPEED_MAX_CM_S,
                                      DVE_CHASSIS_SPEED_MAX_CM_S);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_SetTraceSpeedCmps
*   功能说明: 根据基础速度和转向速度设置左右轮目标速度
*   形    参: base_cm_s — 基础速度；turn_cm_s — 转向修正速度
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_SetTraceSpeedCmps(float base_cm_s, float turn_cm_s)
{
    DVE_Chassis_SetSpeedTargetCmps(base_cm_s - turn_cm_s,
                                   base_cm_s + turn_cm_s);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_Update20ms
*   功能说明: 底盘速度环更新，20ms调用一次
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_Update20ms(void)
{
    if (!s_chassis_inited) {
        return;
    }

    Encoder_CalculateSpeed(&s_encoder);
#if (DVE_CHASSIS_ENCODER_MAP == DVE_CHASSIS_ENCODER_MAP_SWAPPED)
    s_left_actual_cm_s = s_encoder.motorB.speed_cm_s;
    s_right_actual_cm_s = s_encoder.motorA.speed_cm_s;
#else
    s_left_actual_cm_s = s_encoder.motorA.speed_cm_s;
    s_right_actual_cm_s = s_encoder.motorB.speed_cm_s;
#endif

    if (!s_chassis_enable) {
        BSP_ChassisMotor_StopAll();
        return;
    }

    s_left_pwm_out = PID_Calc(&s_left_speed_pid,
                              s_left_target_cm_s,
                              s_left_actual_cm_s);
    s_right_pwm_out = PID_Calc(&s_right_speed_pid,
                               s_right_target_cm_s,
                               s_right_actual_cm_s);

    (void)DVE_Chassis_SetPwm(float_to_i16(s_left_pwm_out),
                             float_to_i16(s_right_pwm_out));
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_GetSpeedPid
*   功能说明: 获取左右轮速度环PID快照
*   形    参: left_pid — 左轮PID输出；right_pid — 右轮PID输出，可传NULL
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_GetSpeedPid(PID_t *left_pid, PID_t *right_pid)
{
    if (left_pid != 0) {
        *left_pid = s_left_speed_pid;
    }

    if (right_pid != 0) {
        *right_pid = s_right_speed_pid;
    }
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_GetState
*   功能说明: 获取底盘速度环状态
*   形    参: state — 底盘状态输出
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_GetState(DVE_ChassisState_t *state)
{
    if (state == 0) {
        return;
    }

    state->left_target_cm_s = s_left_target_cm_s;
    state->right_target_cm_s = s_right_target_cm_s;
    state->left_actual_cm_s = s_left_actual_cm_s;
    state->right_actual_cm_s = s_right_actual_cm_s;
    state->left_pwm_out = s_left_pwm_out;
    state->right_pwm_out = s_right_pwm_out;
    state->left_pwm_ratio = s_left_pwm_ratio;
    state->right_pwm_ratio = s_right_pwm_ratio;
    state->is_enable = s_chassis_enable;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_ConsumeControlFlag
*   功能说明: 读取并清除20ms控制周期标志
*   形    参: 无
*   返 回 值: true表示到达控制周期，false表示未到
*********************************************************************************************************
*/
bool DVE_Chassis_ConsumeControlFlag(void)
{
    bool flag;

    __disable_irq();
    flag = s_control_flag;
    s_control_flag = false;
    __enable_irq();

    return flag;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_Chassis_Timer20msIRQ
*   功能说明: 底盘20ms定时器中断回调
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_Chassis_Timer20msIRQ(void)
{
    s_control_flag = true;
}
