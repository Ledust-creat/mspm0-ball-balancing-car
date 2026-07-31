#include "app_line_follow.h"
#include "dve_ir_sensor.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_LINE_FOLLOW_PERIOD_MS              (20U)       /* 循迹外环周期 */
#define APP_LINE_FOLLOW_LOST_TIMEOUT_MS        (3000U)      /* 丢线找线超时时间 */
#define APP_LINE_FOLLOW_LOST_TIMEOUT_TICKS     \
    (APP_LINE_FOLLOW_LOST_TIMEOUT_MS / APP_LINE_FOLLOW_PERIOD_MS)

#define APP_LINE_FOLLOW_DEFAULT_KP             (3.72f)      /* 循迹外环初始Kp */
#define APP_LINE_FOLLOW_DEFAULT_KI             (0.0f)      /* 循迹外环初始Ki */
#define APP_LINE_FOLLOW_DEFAULT_KD             (60.0f)      /* 循迹外环初始Kd */
#define APP_LINE_FOLLOW_MAX_CORRECTION_CM_S    (36.0f)     /* 转向修正速度限幅 */

#define APP_LINE_FOLLOW_ERROR_FILTER_ALPHA     (1.0f)      /* 误差低通滤波，新值占比 */
#define APP_LINE_FOLLOW_ERROR_DEADBAND         (1.10f)     /* 中心死区，覆盖0附近跳动 */
#define APP_LINE_FOLLOW_CENTER_I_DECAY         (0.50f)     /* 回到中心后释放积分 */
#define APP_LINE_FOLLOW_CENTER_I_EPSILON       (0.10f)     /* 小积分清零阈值 */
#define APP_LINE_FOLLOW_CORRECTION_SLEW_CM_S   (4.0f)      /* 每20ms转向修正最大变化量 */
#define APP_LINE_FOLLOW_SPEED_SCALE_SLEW       (0.05f)     /* 每20ms速度比例最大变化量 */
#define APP_LINE_FOLLOW_CURVE_GAIN             (1.18f)     /* 大误差附加转向增益 */

#define APP_LINE_FOLLOW_NORMAL_SPEED_SCALE     (1.00f)     /* 正常速度比例 */
#define APP_LINE_FOLLOW_INNER2_SPEED_SCALE     (0.98f)     /* 次次外侧传感器触发速度比例 */
#define APP_LINE_FOLLOW_INNER_SPEED_SCALE      (0.95f)     /* 次外侧传感器触发速度比例 */
#define APP_LINE_FOLLOW_OUTER_SPEED_SCALE      (0.90f)     /* 最外侧传感器触发速度比例 */
#define APP_LINE_FOLLOW_SEARCH_SPEED_SCALE     (0.60f)     /* 丢线找线速度比例 */
#define APP_LINE_FOLLOW_MIN_SEARCH_CORRECTION  (1.0f)      /* 有效转向记忆阈值 */

/*  
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static PID_t s_line_pid;                                      /* 循迹外环PID */
static float s_line_error = 0.0f;                             /* 最近一次循迹误差 */
static float s_steering_correction_cm_s = 0.0f;               /* 最近一次转向修正速度 */
static float s_filtered_line_error = 0.0f;                    /* 滤波后的循迹误差 */
static float s_last_search_correction_cm_s = 0.0f;            /* 丢线找线使用的转向修正 */
static float s_speed_scale = APP_LINE_FOLLOW_NORMAL_SPEED_SCALE;
static uint16_t s_lost_ticks = 0U;                            /* 连续丢线周期计数 */
static uint8_t s_black_count = 0U;                            /* 最近一次黑线通道数量 */
static bool s_is_lost = false;                                /* 丢线状态 */
static bool s_is_searching = false;                           /* 找线状态 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float abs_float(float value);
static float limit_float(float value, float min, float max);
static float slew_float(float current, float target, float max_step);
static float apply_deadband(float value, float deadband);
static float calc_speed_scale(const DVE_IRSensor_t *sensor);
static float calc_curve_gain(float line_error);
static void update_search_direction(float correction);
static void fill_output(APP_LineFollowOutput_t *output, bool should_stop);

/*
*********************************************************************************************************
*   函 数 名: abs_float
*   功能说明: 计算浮点绝对值
*   形    参: value - 输入值
*   返 回 值: 绝对值
*********************************************************************************************************
*/
static float abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/*
*********************************************************************************************************
*   函 数 名: limit_float
*   功能说明: 浮点限幅，用于限制循迹输出和中间增益
*   形    参: value - 输入值；min - 下限；max - 上限
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
*   函 数 名: slew_float
*   功能说明: 对目标值做斜率限制，避免速度比例或转向修正瞬间跳变
*   形    参: current - 当前值；target - 目标值；max_step - 单周期最大变化量
*   返 回 值: 本周期允许输出的值
*********************************************************************************************************
*/
static float slew_float(float current, float target, float max_step)
{
    float delta;

    delta = target - current;

    if (delta > max_step) {
        return current + max_step;
    }

    if (delta < -max_step) {
        return current - max_step;
    }

    return target;
}

/*
*********************************************************************************************************
*   函 数 名: apply_deadband
*   功能说明: 对中心附近误差做死区处理，减少直线中心抖动
*   形    参: value - 输入误差；deadband - 死区宽度
*   返 回 值: 扣除死区后的误差
*********************************************************************************************************
*/
static float apply_deadband(float value, float deadband)
{
    if (abs_float(value) <= deadband) {
        return 0.0f;
    }

    if (value > 0.0f) {
        return value - deadband;
    }

    return value + deadband;
}

/*
*********************************************************************************************************
*   函 数 名: calc_speed_scale
*   功能说明: 根据外侧传感器触发情况计算弯道速度比例
*   形    参: sensor - 循迹传感器数据
*   返 回 值: 速度比例
*********************************************************************************************************
*/
static float calc_speed_scale(const DVE_IRSensor_t *sensor)
{
    if ((sensor->ch[0] != 0U) || (sensor->ch[7] != 0U)) {
        return APP_LINE_FOLLOW_OUTER_SPEED_SCALE;
    }

    if ((sensor->ch[1] != 0U) || (sensor->ch[6] != 0U)) {
        return APP_LINE_FOLLOW_INNER_SPEED_SCALE;
    }

    if ((sensor->ch[2] != 0U) || (sensor->ch[5] != 0U)) {
        return APP_LINE_FOLLOW_INNER2_SPEED_SCALE;
    }
    return APP_LINE_FOLLOW_NORMAL_SPEED_SCALE;
}

/*
*********************************************************************************************************
*   函 数 名: calc_curve_gain
*   功能说明: 大误差时增加转向增益，提升直角弯进入阶段的转向力度
*   形    参: line_error - 死区处理后的循迹误差
*   返 回 值: 转向增益系数
*********************************************************************************************************
*/
static float calc_curve_gain(float line_error)
{
    if (abs_float(line_error) >= 2.5f) {
        return APP_LINE_FOLLOW_CURVE_GAIN;
    }

    return 1.0f;
}

/*
*********************************************************************************************************
*   函 数 名: update_search_direction
*   功能说明: 记录最近一次有效转向修正，用于丢线后继续按原方向找线
*   形    参: correction - 当前转向修正速度
*   返 回 值: 无
*********************************************************************************************************
*/
static void update_search_direction(float correction)
{
    if (abs_float(correction) >= APP_LINE_FOLLOW_MIN_SEARCH_CORRECTION) {
        s_last_search_correction_cm_s = correction;
    }
}

/*
*********************************************************************************************************
*   函 数 名: fill_output
*   功能说明: 填充循迹外环输出结构体
*   形    参: output - 输出结构体；should_stop - 是否需要停车
*   返 回 值: 无
*********************************************************************************************************
*/
static void fill_output(APP_LineFollowOutput_t *output, bool should_stop)
{
    if (output == 0) {
        return;
    }

    output->line_error = s_line_error;
    output->steering_correction_cm_s = s_steering_correction_cm_s;
    output->speed_scale = s_speed_scale;
    output->black_count = s_black_count;
    output->is_lost = s_is_lost;
    output->is_searching = s_is_searching;
    output->should_stop = should_stop;
}

/*
*********************************************************************************************************
*   函 数 名: APP_LineFollow_Init
*   功能说明: 初始化循迹外环模块
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_LineFollow_Init(void)
{
    DVE_IRSensor_Init();

    PID_Init(&s_line_pid,
             APP_LINE_FOLLOW_DEFAULT_KP,
             APP_LINE_FOLLOW_DEFAULT_KI,
             APP_LINE_FOLLOW_DEFAULT_KD,
             0.0f,
             APP_LINE_FOLLOW_MAX_CORRECTION_CM_S);

    APP_LineFollow_Reset();
}

/*
*********************************************************************************************************
*   函 数 名: APP_LineFollow_Reset
*   功能说明: 复位循迹外环状态和PID历史
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_LineFollow_Reset(void)
{
    PID_Reset(&s_line_pid);

    s_line_error = 0.0f;
    s_steering_correction_cm_s = 0.0f;
    s_filtered_line_error = 0.0f;
    s_last_search_correction_cm_s = 0.0f;
    s_speed_scale = APP_LINE_FOLLOW_NORMAL_SPEED_SCALE;
    s_lost_ticks = 0U;
    s_black_count = 0U;
    s_is_lost = false;
    s_is_searching = false;
}

/*
*********************************************************************************************************
*   函 数 名: APP_LineFollow_SetPid
*   功能说明: 设置循迹外环PID参数
*   形    参: kp, ki, kd - PID参数
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_LineFollow_SetPid(float kp, float ki, float kd)
{
    PID_SetParam(&s_line_pid, kp, ki, kd,
                 0.0f,
                 APP_LINE_FOLLOW_MAX_CORRECTION_CM_S);
    PID_Reset(&s_line_pid);
}

/*
*********************************************************************************************************
*   函 数 名: APP_LineFollow_Update
*   功能说明: 更新循迹外环，输出转向修正、弯道速度比例和找线状态
*   形    参: output - 循迹输出结构体
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_LineFollow_Update(APP_LineFollowOutput_t *output)
{
    DVE_IRSensor_t sensor;
    float raw_line_error;
    float pid_input_error;
    float target_correction_cm_s;
    float target_speed_scale;
    uint8_t i;

    /*
     * 只计算循迹策略，不直接控制电机。
     * 调用关系：APP_ChassisControl_Task() -> APP_LineFollow_Update() -> DVE_Chassis。
     */
    DVE_IRSensor_Read(&sensor);

    /*
     * error_f由DVE红外层给出；ERROR_SIGN只用于修正实车左右方向。
     * 若挡右侧传感器时车反而向左修，只改APP_LINE_FOLLOW_ERROR_SIGN。
     */
    raw_line_error = sensor.error_f * APP_LINE_FOLLOW_ERROR_SIGN;
    s_filtered_line_error += (raw_line_error - s_filtered_line_error) *
                             APP_LINE_FOLLOW_ERROR_FILTER_ALPHA;
    s_line_error = s_filtered_line_error;
    s_black_count = sensor.black_count;
    s_is_lost = sensor.is_lost ? true : false;

    if (s_is_lost) {
        /*
         * 丢线时复位PID历史，避免找回线后D项突跳。
         * 若还没超时且有最近一次有效转向修正，则继续按原方向找线。
         */
        PID_Reset(&s_line_pid);
        s_filtered_line_error = 0.0f;

        if ((s_lost_ticks < APP_LINE_FOLLOW_LOST_TIMEOUT_TICKS) &&(abs_float(s_last_search_correction_cm_s) >= APP_LINE_FOLLOW_MIN_SEARCH_CORRECTION)) {
            s_lost_ticks++;
            s_is_searching = true;
            s_speed_scale = slew_float(s_speed_scale,
                                       APP_LINE_FOLLOW_SEARCH_SPEED_SCALE,
                                       APP_LINE_FOLLOW_SPEED_SCALE_SLEW);
            s_steering_correction_cm_s = slew_float(s_steering_correction_cm_s,
                                                    s_last_search_correction_cm_s,
                                                    APP_LINE_FOLLOW_CORRECTION_SLEW_CM_S);
            fill_output(output, false);
        } else {
            /*
             * 没有可用方向记忆，或找线超时，通知上层停车。
             */
            s_is_searching = false;
            s_speed_scale = slew_float(s_speed_scale,
                                       APP_LINE_FOLLOW_NORMAL_SPEED_SCALE,
                                       APP_LINE_FOLLOW_SPEED_SCALE_SLEW);
            s_steering_correction_cm_s = slew_float(s_steering_correction_cm_s,
                                                    0.0f,
                                                    APP_LINE_FOLLOW_CORRECTION_SLEW_CM_S);
            fill_output(output, true);
        }

        return;
    }

    s_lost_ticks = 0U;
    s_is_searching = false;

    /*
     * 先根据外侧传感器决定基础速度比例。
     * 最外侧优先级高于次外侧，可用于直角弯提前降速或反向基础速度测试。
     */
    target_speed_scale = calc_speed_scale(&sensor);
    s_speed_scale = slew_float(s_speed_scale,
                               target_speed_scale,
                               APP_LINE_FOLLOW_SPEED_SCALE_SLEW);
    
    /*
     * line_error > 0 的实际左右方向由红外权重和 ERROR_SIGN 决定。
     * steering_correction_cm_s > 0 表示右轮目标速度高于左轮。
     * 如实车转向相反，只调整 APP_LINE_FOLLOW_ERROR_SIGN。
     */
    pid_input_error = apply_deadband(s_line_error, APP_LINE_FOLLOW_ERROR_DEADBAND);

    if (pid_input_error == 0.0f) {
        s_line_pid.Iout *= APP_LINE_FOLLOW_CENTER_I_DECAY;

        if (abs_float(s_line_pid.Iout) < APP_LINE_FOLLOW_CENTER_I_EPSILON) {
            s_line_pid.Iout = 0.0f;
        }

        for (i = 0U; i < PID_HIST_LEN; i++) {
            s_line_pid.error[i] = 0.0f;
            s_line_pid.Dbuf[i] = 0.0f;
        }
    }

    target_correction_cm_s = PID_Calc(&s_line_pid, 0.0f, pid_input_error);
    target_correction_cm_s *= calc_curve_gain(pid_input_error);
    target_correction_cm_s = limit_float(target_correction_cm_s,
                                         -APP_LINE_FOLLOW_MAX_CORRECTION_CM_S,
                                         APP_LINE_FOLLOW_MAX_CORRECTION_CM_S);
    s_steering_correction_cm_s = slew_float(s_steering_correction_cm_s,
                                            target_correction_cm_s,
                                            APP_LINE_FOLLOW_CORRECTION_SLEW_CM_S);
    /*
     * 记录最近一次有效转向，用于完全丢线后的短时间找线。
     */
    update_search_direction(s_steering_correction_cm_s);
    
    fill_output(output, false);
}

/*
*********************************************************************************************************
*   函 数 名: APP_LineFollow_GetState
*   功能说明: 获取循迹外环状态快照
*   形    参: state - 状态输出结构体
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_LineFollow_GetState(APP_LineFollowState_t *state)
{
    if (state == 0) {
        return;
    }

    state->line_pid = s_line_pid;
    state->line_error = s_line_error;
    state->steering_correction_cm_s = s_steering_correction_cm_s;
    state->speed_scale = s_speed_scale;
    state->lost_ticks = s_lost_ticks;
    state->black_count = s_black_count;
    state->is_lost = s_is_lost;
    state->is_searching = s_is_searching;
}
