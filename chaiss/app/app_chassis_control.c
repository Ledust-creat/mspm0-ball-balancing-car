#include "app_chassis_control.h"
#include "app_line_follow.h"
#include "dve_chassis.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_CHASSIS_CONTROL_DEFAULT_BASE_SPEED_CM_S    (25.0f)    /* 默认循迹基础速度 */
#define APP_CHASSIS_CONTROL_MAX_BASE_SPEED_CM_S        (50.0f)    /* 基础速度上限 */

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static float s_base_speed_cm_s = APP_CHASSIS_CONTROL_DEFAULT_BASE_SPEED_CM_S;
static float s_last_steering_correction_cm_s = 0.0f;
static bool s_is_running = false;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float limit_float(float value, float min, float max);
static void stop_chassis_target(void);

/*
*********************************************************************************************************
*   函 数 名: limit_float
*   功能说明: 浮点限幅
*   形    参: value - 输入值；min - 最小值；max - 最大值
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
*   函 数 名: stop_chassis_target
*   功能说明: 清零底盘速度目标和上层转向状态
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
*   函 数 名: APP_ChassisControl_Init
*   功能说明: 初始化底盘上层控制任务
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_Init(void)
{
    s_base_speed_cm_s = APP_CHASSIS_CONTROL_DEFAULT_BASE_SPEED_CM_S;
    s_last_steering_correction_cm_s = 0.0f;
    s_is_running = false;

    DVE_Chassis_Enable(false);
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_Task
*   功能说明: 底盘20ms上层控制任务，唯一消费控制周期标志
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_Task(void)
{
    APP_LineFollowOutput_t line_output;
    float run_base_speed_cm_s;

    if (!DVE_Chassis_ConsumeControlFlag()) {
        return;
    }

    if (!s_is_running) {
        stop_chassis_target();
        DVE_Chassis_Update20ms();
        return;
    }

    APP_LineFollow_Update(&line_output);

    if (line_output.should_stop) {
        stop_chassis_target();
        APP_LineFollow_Reset();
        DVE_Chassis_Enable(false);
        s_is_running = false;
    } else {
        run_base_speed_cm_s = s_base_speed_cm_s * line_output.speed_scale;
        s_last_steering_correction_cm_s = line_output.steering_correction_cm_s;
        DVE_Chassis_SetTraceSpeedCmps(run_base_speed_cm_s,
                                      line_output.steering_correction_cm_s);
    }

    DVE_Chassis_Update20ms();
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_Start
*   功能说明: 启动底盘循迹控制
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_Start(void)
{
    APP_LineFollow_Reset();
    s_last_steering_correction_cm_s = 0.0f;
    s_is_running = true;
    DVE_Chassis_Enable(true);
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_Stop
*   功能说明: 停止底盘循迹控制
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_Stop(void)
{
    s_is_running = false;
    APP_LineFollow_Reset();
    DVE_Chassis_Enable(false);
    stop_chassis_target();
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_SetBaseSpeedCmps
*   功能说明: 设置循迹基础速度
*   形    参: base_speed_cm_s - 基础速度，单位cm/s
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_SetBaseSpeedCmps(float base_speed_cm_s)
{
    s_base_speed_cm_s = limit_float(base_speed_cm_s,
                                    0.0f,
                                    APP_CHASSIS_CONTROL_MAX_BASE_SPEED_CM_S);
}

/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_GetState
*   功能说明: 获取底盘上层控制状态
*   形    参: state - 状态输出结构体
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_ChassisControl_GetState(APP_ChassisControlState_t *state)
{
    if (state == 0) {
        return;
    }

    state->base_speed_cm_s = s_base_speed_cm_s;
    state->steering_correction_cm_s = s_last_steering_correction_cm_s;
    state->is_running = s_is_running;
}
