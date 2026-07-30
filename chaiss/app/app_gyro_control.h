#ifndef APP_GYRO_CONTROL_H_
#define APP_GYRO_CONTROL_H_

#include "PID.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_IMU_SAMPLE_PERIOD_MS        (10U)       /* ICM20602独立采样周期，单位ms */
#define APP_IMU_DATA_TIMEOUT_MS         (30U)       /* IMU缓存最大允许数据年龄，单位ms */
#define APP_IMU_ACCEL_LPF_ALPHA         (0.25f)     /* APP层加速度低通新数据权重，范围0~1 */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    float accel_forward_g;                      /* 车体前向动态加速度，单位g，车头方向为正 */
    float accel_lateral_g;                      /* 车体横向动态加速度，单位g，右侧为正 */
    uint32_t update_tick_ms;                    /* 最近一次有效采样时刻，单位ms */
    bool valid;                                 /* true表示IMU加速度缓存有效 */
} APP_GyroAccelData_t;

typedef struct
{
    float yaw_error_deg;                        /* yaw误差，单位deg */
    float steering_correction_cm_s;             /* 转向修正速度，单位cm/s */
    float speed_scale;                          /* 速度比例，1.0表示不减速 */
    bool imu_ready;                             /* true表示陀螺仪就绪 */
    bool should_stop;                           /* true表示需要保护停车 */
} APP_GyroControlOutput_t;

typedef struct
{
    PID_t yaw_pid;                              /* yaw外环PID参数和内部状态 */
    float target_yaw_deg;                       /* 目标yaw，单位deg */
    float roll_deg;                             /* 当前roll角，单位deg */
    float pitch_deg;                            /* 当前pitch角，单位deg */
    float yaw_deg;                              /* 当前yaw角，单位deg */
    float yaw_error_deg;                        /* yaw误差，单位deg */
    float steering_correction_cm_s;             /* 转向修正速度，单位cm/s */
    bool imu_ready;                             /* true表示陀螺仪就绪 */
} APP_GyroControlState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_GyroControl_Init(void);
void APP_GyroControl_Reset(void);
void APP_GyroControl_SensorTask10ms(uint32_t now_ms);
bool APP_GyroControl_GetLatestAccel(APP_GyroAccelData_t *data);
void APP_GyroControl_Update(APP_GyroControlOutput_t *output);
void APP_GyroControl_SetYawPid(float kp, float ki, float kd);
void APP_GyroControl_SetTargetYaw(float yaw_deg);
void APP_GyroControl_SetZero(void);
void APP_GyroControl_GetState(APP_GyroControlState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* APP_GYRO_CONTROL_H_ */
