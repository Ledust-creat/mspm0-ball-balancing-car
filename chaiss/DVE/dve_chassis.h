#ifndef DVE_CHASSIS_H_
#define DVE_CHASSIS_H_

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
typedef enum
{
    DVE_CHASSIS_OK = 0,
    DVE_CHASSIS_ERR_NOT_INIT,
    DVE_CHASSIS_ERR_PARAM
} DVE_ChassisStatus_t;

typedef struct
{
    float left_target_cm_s;               /* 左轮目标速度 */
    float right_target_cm_s;              /* 右轮目标速度 */
    float left_actual_cm_s;               /* 左轮实际速度 */
    float right_actual_cm_s;              /* 右轮实际速度 */
    float left_pwm_out;                   /* 左轮PWM输出 */
    float right_pwm_out;                  /* 右轮PWM输出 */
    float left_pwm_ratio;                 /* 左轮PWM补偿系数 */
    float right_pwm_ratio;                /* 右轮PWM补偿系数 */
    bool is_enable;                       /* 底盘使能状态 */
} DVE_ChassisState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void DVE_Chassis_Init(void);

void DVE_Chassis_Enable(bool enable);
void DVE_Chassis_Stop(void);

void DVE_Chassis_SetPwmRatio(float left_ratio, float right_ratio);

DVE_ChassisStatus_t DVE_Chassis_SetPwm(int16_t left_pwm, int16_t right_pwm);
DVE_ChassisStatus_t DVE_Chassis_SetTracePwm(float base_pwm, float turn_pwm);

void DVE_Chassis_SetSpeedPid(float kp, float ki, float kd);
void DVE_Chassis_SetLeftSpeedPid(float kp, float ki, float kd);
void DVE_Chassis_SetRightSpeedPid(float kp, float ki, float kd);
void DVE_Chassis_SetSpeedTargetCmps(float left_cm_s, float right_cm_s);
void DVE_Chassis_SetTraceSpeedCmps(float base_cm_s, float turn_cm_s);
void DVE_Chassis_Update20ms(void);

void DVE_Chassis_GetSpeedPid(PID_t *left_pid, PID_t *right_pid);
void DVE_Chassis_GetState(DVE_ChassisState_t *state);

bool DVE_Chassis_ConsumeControlFlag(void);
void DVE_Chassis_Timer20msIRQ(void);

#ifdef __cplusplus
}
#endif

#endif /* DVE_CHASSIS_H_ */
