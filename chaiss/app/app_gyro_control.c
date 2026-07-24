#include "app_gyro_control.h"
#include "dve_icm20602.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#ifndef APP_GYRO_CONTROL_PERIOD_S
#define APP_GYRO_CONTROL_PERIOD_S                 (0.020f)    /* 姿态更新周期，当前跟随底盘20ms */
#endif
#define APP_GYRO_CONTROL_DEFAULT_KP               (1.0f)      /* yaw外环Kp */
#define APP_GYRO_CONTROL_DEFAULT_KI               (0.0f)      /* yaw外环Ki */
#define APP_GYRO_CONTROL_DEFAULT_KD               (0.0f)      /* yaw外环Kd */
#define APP_GYRO_CONTROL_MAX_IOUT                 (0.0f)      /* 第一版不用积分 */
#define APP_GYRO_CONTROL_MAX_STEERING_CM_S        (30.0f)     /* 转向修正速度限幅 */
#define APP_GYRO_CONTROL_SPEED_SCALE              (1.0f)      /* 陀螺仪外环默认不减速 */

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static PID_t s_yaw_pid;                           /* yaw外环PID */
static float s_target_yaw_deg = 0.0f;             /* 目标yaw */
static float s_yaw_error_deg = 0.0f;              /* yaw误差 */
static float s_steering_cm_s = 0.0f;              /* 转向修正速度 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float angle_error(float target, float feedback);
static void fill_output(APP_GyroControlOutput_t *output, bool should_stop);

static float angle_error(float target, float feedback)
{
    float err = target - feedback;

    while (err > 180.0f) {
        err -= 360.0f;
    }

    while (err < -180.0f) {
        err += 360.0f;
    }

    return err;
}

static void fill_output(APP_GyroControlOutput_t *output, bool should_stop)
{
    DVE_ICM20602State_t imu_state;

    if (output == 0) {
        return;
    }

    DVE_ICM20602_GetState(&imu_state);
    output->yaw_error_deg = s_yaw_error_deg;
    output->steering_correction_cm_s = s_steering_cm_s;
    output->speed_scale = APP_GYRO_CONTROL_SPEED_SCALE;
    output->imu_ready = imu_state.is_ready;
    output->should_stop = should_stop;
}

/*
*********************************************************************************************************
*   函 数 名: APP_GyroControl_Init
*   功能说明: 初始化陀螺仪yaw外环
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_GyroControl_Init(void)
{
    PID_Init(&s_yaw_pid,
             APP_GYRO_CONTROL_DEFAULT_KP,
             APP_GYRO_CONTROL_DEFAULT_KI,
             APP_GYRO_CONTROL_DEFAULT_KD,
             APP_GYRO_CONTROL_MAX_IOUT,
             APP_GYRO_CONTROL_MAX_STEERING_CM_S);

    (void)DVE_ICM20602_Init(); // 只初始化陀螺仪设备，不控制底盘
    APP_GyroControl_Reset();
}

void APP_GyroControl_Reset(void)
{
    PID_Reset(&s_yaw_pid);
    s_target_yaw_deg = 0.0f;
    s_yaw_error_deg = 0.0f;
    s_steering_cm_s = 0.0f;
}

/*
*********************************************************************************************************
*   函 数 名: APP_GyroControl_Update
*   功能说明: 更新陀螺仪yaw外环，只输出速度修正量
*   形    参: output - 外环输出
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_GyroControl_Update(APP_GyroControlOutput_t *output)
{
    DVE_ICM20602State_t imu_state;

    DVE_ICM20602_Update(APP_GYRO_CONTROL_PERIOD_S); // 复用底盘20ms周期，不重配SysTick
    DVE_ICM20602_GetState(&imu_state);

    if (!imu_state.is_ready) {
        s_yaw_error_deg = 0.0f;
        s_steering_cm_s = 0.0f;
        fill_output(output, true);
        return;
    }

    s_yaw_error_deg = angle_error(s_target_yaw_deg, imu_state.yaw_deg);
    s_steering_cm_s = PID_Calc(&s_yaw_pid, 0.0f, -s_yaw_error_deg); // 正修正表示右轮目标速度更高
    fill_output(output, false);
}

void APP_GyroControl_SetYawPid(float kp, float ki, float kd)
{
    PID_SetParam(&s_yaw_pid, kp, ki, kd, APP_GYRO_CONTROL_MAX_IOUT, APP_GYRO_CONTROL_MAX_STEERING_CM_S);
    PID_Reset(&s_yaw_pid);
}

void APP_GyroControl_SetTargetYaw(float yaw_deg)
{
    s_target_yaw_deg = yaw_deg;
}

void APP_GyroControl_SetZero(void)
{
    DVE_ICM20602_SetZero();
    APP_GyroControl_Reset();
}

void APP_GyroControl_GetState(APP_GyroControlState_t *state)
{
    DVE_ICM20602State_t imu_state;

    if (state == 0) {
        return;
    }

    DVE_ICM20602_GetState(&imu_state);
    state->yaw_pid = s_yaw_pid;
    state->target_yaw_deg = s_target_yaw_deg;
    state->roll_deg = imu_state.roll_deg;
    state->pitch_deg = imu_state.pitch_deg;
    state->yaw_deg = imu_state.yaw_deg;
    state->yaw_error_deg = s_yaw_error_deg;
    state->steering_correction_cm_s = s_steering_cm_s;
    state->imu_ready = imu_state.is_ready;
}
