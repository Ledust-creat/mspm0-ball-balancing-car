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
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    float yaw_error_deg;                       /* yaw误差 */
    float steering_correction_cm_s;            /* 转向修正速度 */
    float speed_scale;                         /* 速度比例，1.0表示不减速 */
    bool imu_ready;                            /* 陀螺仪就绪 */
    bool should_stop;                          /* true表示需要停车 */
} APP_GyroControlOutput_t;

typedef struct
{
    PID_t yaw_pid;                             /* yaw外环PID */
    float target_yaw_deg;                      /* 目标yaw */
    float roll_deg;                            /* 当前roll */
    float pitch_deg;                           /* 当前pitch */
    float yaw_deg;                             /* 当前yaw */
    float yaw_error_deg;                       /* yaw误差 */
    float steering_correction_cm_s;            /* 转向修正速度 */
    bool imu_ready;                            /* 陀螺仪就绪 */
} APP_GyroControlState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_GyroControl_Init(void);
void APP_GyroControl_Reset(void);
void APP_GyroControl_Update(APP_GyroControlOutput_t *output);
void APP_GyroControl_SetYawPid(float kp, float ki, float kd);
void APP_GyroControl_SetTargetYaw(float yaw_deg);
void APP_GyroControl_SetZero(void);
void APP_GyroControl_GetState(APP_GyroControlState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* APP_GYRO_CONTROL_H_ */
