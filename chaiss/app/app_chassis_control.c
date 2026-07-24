#include "app_chassis_control.h"
#include "app_gyro_control.h"
#include "app_line_follow.h"
#include "dve_chassis.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_CHASSIS_CONTROL_DEFAULT_BASE_SPEED_CM_S    (25.0f)    /* 默认基础速度 */
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
static void reset_active_outer_loop(void);

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

static void stop_chassis_target(void)
{
    s_last_steering_correction_cm_s = 0.0f;
    DVE_Chassis_SetSpeedTargetCmps(0.0f, 0.0f);
}

static void reset_active_outer_loop(void)
{
#if (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_LINE_FOLLOW)
    APP_LineFollow_Reset();
#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_GYRO_YAW)
    APP_GyroControl_SetZero();
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
    s_base_speed_cm_s = APP_CHASSIS_CONTROL_DEFAULT_BASE_SPEED_CM_S;
    s_last_steering_correction_cm_s = 0.0f;
    s_is_running = false;

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
        run_base_speed_cm_s = s_base_speed_cm_s * line_output.speed_scale;
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
#else
#error "Unsupported APP_CHASSIS_CONTROL_MODE"
#endif

    if (should_stop) {
        stop_chassis_target();
        reset_active_outer_loop();
        DVE_Chassis_Enable(false);
        s_is_running = false;
    } else {
        s_last_steering_correction_cm_s = steering_correction_cm_s;

        DVE_Chassis_SetTraceSpeedCmps(run_base_speed_cm_s, s_last_steering_correction_cm_s);
    }
    
    DVE_Chassis_Update20ms();
}

void APP_ChassisControl_Start(void)
{
    reset_active_outer_loop();
    s_last_steering_correction_cm_s = 0.0f;
    s_is_running = true;
    DVE_Chassis_Enable(true);
}

void APP_ChassisControl_Stop(void)
{
    s_is_running = false;
    reset_active_outer_loop();
    DVE_Chassis_Enable(false);
    stop_chassis_target();
}

void APP_ChassisControl_SetBaseSpeedCmps(float base_speed_cm_s)
{
    s_base_speed_cm_s = limit_float(base_speed_cm_s,
                                    0.0f,
                                    APP_CHASSIS_CONTROL_MAX_BASE_SPEED_CM_S);
}

void APP_ChassisControl_GetState(APP_ChassisControlState_t *state)
{
    if (state == 0) {
        return;
    }

    state->base_speed_cm_s = s_base_speed_cm_s;
    state->steering_correction_cm_s = s_last_steering_correction_cm_s;
    state->control_mode = APP_CHASSIS_CONTROL_MODE;
    state->is_running = s_is_running;
}
