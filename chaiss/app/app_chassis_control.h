#ifndef APP_CHASSIS_CONTROL_H_
#define APP_CHASSIS_CONTROL_H_

#include <stdbool.h>

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
    float base_speed_cm_s;                  /* 循迹基础速度 */
    float steering_correction_cm_s;         /* 转向修正速度 */
    bool is_running;                        /* 底盘运行状态 */
} APP_ChassisControlState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_ChassisControl_Init(void);
void APP_ChassisControl_Task(void);
void APP_ChassisControl_Start(void);
void APP_ChassisControl_Stop(void);
void APP_ChassisControl_SetBaseSpeedCmps(float base_speed_cm_s);
void APP_ChassisControl_GetState(APP_ChassisControlState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* APP_CHASSIS_CONTROL_H_ */
