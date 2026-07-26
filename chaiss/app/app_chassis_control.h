#ifndef APP_CHASSIS_CONTROL_H_
#define APP_CHASSIS_CONTROL_H_

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
#define APP_CHASSIS_CONTROL_MODE_LINE_FOLLOW    (0U)      /* 循迹外环 */
#define APP_CHASSIS_CONTROL_MODE_GYRO_YAW       (1U)      /* 陀螺仪yaw外环 */
#define APP_CHASSIS_CONTROL_MODE_SPEED_TEST     (2U)      /* 纯速度环测试 */

#ifndef APP_CHASSIS_CONTROL_MODE
#define APP_CHASSIS_CONTROL_MODE                APP_CHASSIS_CONTROL_MODE_LINE_FOLLOW
#endif

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    float base_speed_cm_s;                  /* 基础速度 */
    float steering_correction_cm_s;         /* 转向修正速度 */
    uint8_t control_mode;                   /* 当前外环模式 */
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
