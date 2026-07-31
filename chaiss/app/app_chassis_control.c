#include "app_chassis_control.h"
#include "app_gyro_control.h"
#include "app_command.h"    
#include "app_line_follow.h"
#include "dve_chassis.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_CHASSIS_CONTROL_DEFAULT_BASE_SPEED_CM_S    (25.0f)    /* 默认基础速度 */
#define APP_CHASSIS_CONTROL_MAX_BASE_SPEED_CM_S        (50.0f)    /* 基础速度上限 */
#define APP_CHASSIS_MOBILE_ACCEL_STEP_CM_S             (0.5f)     /* 移动控球每20ms加速步长 */
#define APP_CHASSIS_MOBILE_DECEL_STEP_CM_S             (0.4f)     /* 移动控球每20ms减速步长 */
#define APP_CHASSIS_BALL_TURN_LIMIT_ENABLE             (1U)       /* 移动控球转向限幅使能 */
#define APP_CHASSIS_BALL_TURN_LIMIT_RATIO              (0.8f)     /* 修正量占基础速度最大比例 */
#define APP_CHASSIS_CONTROL_PERIOD_S                    (0.02f)    /* 底盘控制周期，单位s */
#define APP_CHASSIS_SOFT_STOP_EPSILON_CM_S             (0.05f)    /* 软停车完成速度阈值 */

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static float s_base_speed_cm_s = APP_CHASSIS_CONTROL_DEFAULT_BASE_SPEED_CM_S; /* 普通模式基础速度 */
static float s_last_steering_correction_cm_s = 0.0f;                        /* 最近转向修正量 */
static bool s_is_running = false;                                            /* 底盘运行标志 */
static bool s_ball_priority_mode = false;                                    /* 移动控球专用模式 */
static bool s_soft_stop_requested = false;                                   /* 软停车请求标志 */
static bool s_soft_stopped = false;                                          /* 软停车完成标志 */
static float s_target_base_speed_cm_s = APP_CHASSIS_CONTROL_DEFAULT_BASE_SPEED_CM_S; /* 斜坡目标速度 */
static float s_planned_base_speed_cm_s = 0.0f;                               /* 斜坡规划速度 */
static float s_last_run_base_speed_cm_s = 0.0f;                              /* 上周期实际基础速度 */
static float s_planned_accel_cm_s2 = 0.0f;                                   /* 规划加速度 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float limit_float(float value, float min, float max);
static float approach_float(float current, float target, float rise_step, float fall_step);
static void stop_chassis_target(void);
static void reset_active_outer_loop(void);

/*
*********************************************************************************************************
*   函 数 名: limit_float
*   功能说明: 限制浮点数范围
*   形    参: value - 输入值；min/max - 上下限
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
*   函 数 名: approach_float
*   功能说明: 按不同加减速步长使当前值平滑逼近目标值
*   形    参: current - 当前值；target - 目标值；rise_step/fall_step - 单周期步长
*   返 回 值: 本周期规划值
*********************************************************************************************************
*/
static float approach_float(float current, float target, float rise_step, float fall_step)
{
    if (current < target) {
        current += rise_step;
        return (current > target) ? target : current;
    }

    if (current > target) {
        current -= fall_step;
        return (current < target) ? target : current;
    }

    return current;
}

/*
*********************************************************************************************************
*   函 数 名: stop_chassis_target
*   功能说明: 清除转向修正并将左右轮目标速度置零
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void stop_chassis_target(void)
{
    s_last_steering_correction_cm_s = 0.0f;
    DVE_Chassis_SetSpeedTargetCmps(0.0f, 0.0f);
}

/*
*********************************************************************************************************
*   函 数 名: reset_active_outer_loop
*   功能说明: 根据编译模式复位当前启用的底盘外环
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void reset_active_outer_loop(void)
{
#if (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_LINE_FOLLOW)

    APP_LineFollow_Reset();

#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_GYRO_YAW)

    APP_GyroControl_SetZero();

#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_SPEED_TEST)

    /* 纯速度环测试没有外环状态需要复位 */

#else
#error "Unsupported APP_CHASSIS_CONTROL_MODE"
#endif
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_Init
*   功能说明: 初始化底盘上层控制
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_Init(void)
{
    DVE_Chassis_Init();

#if (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_LINE_FOLLOW)
    APP_LineFollow_Init();
#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_GYRO_YAW)
    /* Gyro模块由系统单独初始化，避免与钢球IMU采样重复初始化。 */
#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_SPEED_TEST)
    /* 纯速度环测试无额外外环初始化。 */
#else
#error "Unsupported APP_CHASSIS_CONTROL_MODE"
#endif

    s_base_speed_cm_s = APP_CHASSIS_CONTROL_DEFAULT_BASE_SPEED_CM_S;
    s_last_steering_correction_cm_s = 0.0f;
    s_is_running = false;
    s_ball_priority_mode = false;
    s_soft_stop_requested = false;
    s_soft_stopped = false;
    s_target_base_speed_cm_s = s_base_speed_cm_s;
    s_planned_base_speed_cm_s = 0.0f;
    s_last_run_base_speed_cm_s = 0.0f;
    s_planned_accel_cm_s2 = 0.0f;

    DVE_Chassis_Enable(false);
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_Task
*   功能说明: 底盘20ms上层控制任务，统一消费控制周期标志
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_Task(void)
{
    float run_base_speed_cm_s;
    float steering_correction_cm_s;
    bool should_stop;

    if (!DVE_Chassis_ConsumeControlFlag()) {
        return;
    }

    if (!s_is_running) {
    #if (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_GYRO_YAW)
            {
                APP_GyroControlOutput_t gyro_output;

                APP_GyroControl_Update(&gyro_output); // 停车时也刷新IMU姿态，方便蓝牙观察
            }
    #endif
        stop_chassis_target();
        DVE_Chassis_Update20ms();
        return;
    }
    
#if (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_LINE_FOLLOW)
    {
        APP_LineFollowOutput_t line_output;

        APP_LineFollow_Update(&line_output); // 循迹外环只输出修正量
        if (s_ball_priority_mode) {
            // 移动控球模式先规划基础速度，再叠加循迹弯道速度比例
            s_planned_base_speed_cm_s = approach_float(s_planned_base_speed_cm_s,
                                                       s_target_base_speed_cm_s,
                                                       APP_CHASSIS_MOBILE_ACCEL_STEP_CM_S,
                                                       APP_CHASSIS_MOBILE_DECEL_STEP_CM_S);
            run_base_speed_cm_s = s_planned_base_speed_cm_s * line_output.speed_scale;
        } else {
            run_base_speed_cm_s = s_base_speed_cm_s * line_output.speed_scale; // 保持原循迹逻辑不变
        }
        steering_correction_cm_s = line_output.steering_correction_cm_s;
        should_stop = line_output.should_stop;
    }
#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_GYRO_YAW)
    {
        APP_GyroControlOutput_t gyro_output;

        APP_GyroControl_Update(&gyro_output); // 陀螺仪外环只输出修正量
        run_base_speed_cm_s = s_base_speed_cm_s * gyro_output.speed_scale;
        steering_correction_cm_s = gyro_output.steering_correction_cm_s;
        should_stop = gyro_output.should_stop;
    }
#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_SPEED_TEST)

    run_base_speed_cm_s = s_base_speed_cm_s; /* 使用蓝牙设置的基础速度 */
    steering_correction_cm_s = 0.0f;         /* 强制关闭所有外环差速 */
    should_stop = false;                     /* 不受IMU或循迹传感器影响 */
#else
#error "Unsupported APP_CHASSIS_CONTROL_MODE"
#endif

    if (s_ball_priority_mode) {
        float turn_limit_cm_s;

        // 记录最终运行基础速度变化率，供钢球加速度前馈接口使用
        s_planned_accel_cm_s2 = (run_base_speed_cm_s - s_last_run_base_speed_cm_s) /
                                APP_CHASSIS_CONTROL_PERIOD_S;
        s_last_run_base_speed_cm_s = run_base_speed_cm_s;

#if (APP_CHASSIS_BALL_TURN_LIMIT_ENABLE != 0U)
        // 修正量不超过基础速度的80%，避免低速时单轮突然反转冲击钢球
        turn_limit_cm_s = run_base_speed_cm_s * APP_CHASSIS_BALL_TURN_LIMIT_RATIO;
        steering_correction_cm_s = limit_float(steering_correction_cm_s,
                                               -turn_limit_cm_s,
                                               turn_limit_cm_s);
#else
        (void)turn_limit_cm_s;
#endif

        if (s_soft_stop_requested &&
            (s_planned_base_speed_cm_s <= APP_CHASSIS_SOFT_STOP_EPSILON_CM_S)) {
            s_planned_base_speed_cm_s = 0.0f;
            s_soft_stopped = true;
            s_is_running = false;
            DVE_Chassis_Enable(false); // 斜坡降到零后再真正关闭速度环输出
            stop_chassis_target();
            DVE_Chassis_Update20ms();
            return;
        }
    }

    if (should_stop) {
        stop_chassis_target();
        reset_active_outer_loop();
        DVE_Chassis_Enable(false);
        s_is_running = false;
    } else {
        s_last_steering_correction_cm_s = steering_correction_cm_s;
        #if (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_SPEED_TEST)
            /* 纯速度测试时，左右轮目标速度完全相同 */
            DVE_Chassis_SetSpeedTargetCmps(run_base_speed_cm_s,run_base_speed_cm_s);
        #else
            /* 循迹和航向模式通过差速控制方向 */
            DVE_Chassis_SetTraceSpeedCmps(run_base_speed_cm_s,s_last_steering_correction_cm_s);
        #endif
    }

    DVE_Chassis_Update20ms();
}

void APP_ChassisControl_Start(void)
{
    reset_active_outer_loop();
    s_last_steering_correction_cm_s = 0.0f;
    s_soft_stop_requested = false;
    s_soft_stopped = false;
    s_last_run_base_speed_cm_s = 0.0f;
    s_planned_accel_cm_s2 = 0.0f;
    if (!s_ball_priority_mode) {
        s_planned_base_speed_cm_s = s_base_speed_cm_s; // 普通模式不启用新增速度斜坡
    }
    s_is_running = true;
    DVE_Chassis_Enable(true);
}

void APP_ChassisControl_Stop(void)
{
    s_is_running = false;
    reset_active_outer_loop();
    DVE_Chassis_Enable(false);
    stop_chassis_target();
    s_soft_stop_requested = false;
    s_soft_stopped = true;
    s_planned_base_speed_cm_s = 0.0f;
    s_last_run_base_speed_cm_s = 0.0f;
    s_planned_accel_cm_s2 = 0.0f;
}

void APP_ChassisControl_SetBaseSpeedCmps(float base_speed_cm_s)
{
    s_base_speed_cm_s = limit_float(base_speed_cm_s,
                                    0.0f,
                                    APP_CHASSIS_CONTROL_MAX_BASE_SPEED_CM_S);
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_SetTargetBaseSpeed
*   功能说明: 设置移动控球模式的斜坡目标速度
*   形    参: speed_cm_s - 目标基础速度，单位cm/s
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_SetTargetBaseSpeed(float speed_cm_s)
{
    s_target_base_speed_cm_s = limit_float(speed_cm_s,
                                           0.0f,
                                           APP_CHASSIS_CONTROL_MAX_BASE_SPEED_CM_S);
    s_soft_stopped = false;
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_SetBallPriorityMode
*   功能说明: 启用或关闭移动控球专用速度斜坡和转向限制
*   形    参: enable - true启用，false关闭
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_SetBallPriorityMode(bool enable)
{
    s_ball_priority_mode = enable;
    s_soft_stop_requested = false;
    s_soft_stopped = false;
    s_planned_accel_cm_s2 = 0.0f;
    s_last_run_base_speed_cm_s = 0.0f;

    if (enable) {
        // 后三问从零速开始规划，防止启动瞬间扰动钢球
        s_target_base_speed_cm_s = 0.0f;
        s_planned_base_speed_cm_s = 0.0f;
    } else {
        // 退出后三问后恢复原基础速度语义
        s_target_base_speed_cm_s = s_base_speed_cm_s;
        s_planned_base_speed_cm_s = s_base_speed_cm_s;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_RequestSoftStop
*   功能说明: 请求底盘沿减速斜坡平滑停车
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_RequestSoftStop(void)
{
    if (!s_ball_priority_mode) {
        APP_ChassisControl_Stop(); // 普通模式保持原来的立即停车行为
        return;
    }

    s_target_base_speed_cm_s = 0.0f; // 20ms任务按减速步长逐渐逼近零速
    s_soft_stop_requested = true;
    s_soft_stopped = false;
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_IsSoftStopped
*   功能说明: 查询软停车是否完成
*   形    参: 无
*   返 回 值: true已停车，false仍在减速或未请求
*********************************************************************************************************
*/
bool APP_ChassisControl_IsSoftStopped(void)
{
    return s_soft_stopped;
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_GetPlannedAccelCmps2
*   功能说明: 获取最近一次规划基础速度对应的加速度
*   形    参: 无
*   返 回 值: 规划加速度，单位cm/s^2
*********************************************************************************************************
*/
float APP_ChassisControl_GetPlannedAccelCmps2(void)
{
    return s_planned_accel_cm_s2;
}

void APP_ChassisControl_GetState(APP_ChassisControlState_t *state)
{
    if (state == 0) {
        return;
    }
    
    state->base_speed_cm_s = s_base_speed_cm_s;
    state->target_base_speed_cm_s = s_target_base_speed_cm_s;
    state->planned_base_speed_cm_s = s_planned_base_speed_cm_s;
    state->planned_accel_cm_s2 = s_planned_accel_cm_s2;
    state->steering_correction_cm_s = s_last_steering_correction_cm_s;
    state->control_mode = APP_CHASSIS_CONTROL_MODE;
    state->is_running = s_is_running;
    state->ball_priority_mode = s_ball_priority_mode;
    state->soft_stopped = s_soft_stopped;
}
