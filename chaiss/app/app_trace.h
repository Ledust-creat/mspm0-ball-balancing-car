#ifndef APP_TRACE_H_
#define APP_TRACE_H_

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
#define APP_TRACE_MODE_OPEN_PWM          (0U)      /* 外环输出PWM修正量 */
#define APP_TRACE_MODE_SPEED_LOOP        (1U)      /* 外环输出速度修正量，DVE执行速度内环 */

#ifndef APP_TRACE_CONTROL_MODE
#define APP_TRACE_CONTROL_MODE           APP_TRACE_MODE_SPEED_LOOP
#endif

/*
 * 0：只调速度内环，不读取红外，不做循迹外环
 * 1：启用红外循迹外环，外环输出左右轮目标速度差
 */
#ifndef APP_TRACE_ENABLE_IR_OUTER_LOOP
#define APP_TRACE_ENABLE_IR_OUTER_LOOP   (0U)
#endif

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    float base_pwm;                         /* 开环基础PWM */
    float base_speed_cm_s;                  /* 速度环基础速度 */
    PID_t trace_pid;                        /* 循迹外环PID快照 */
    float last_error;                       /* 最近一次循迹偏差 */
    float last_turn_pwm;                    /* 最近一次PWM转向输出 */
    float last_turn_speed;                  /* 最近一次速度转向输出 */
    uint8_t last_black_count;               /* 最近一次黑线通道数量 */
    bool is_lost;                           /* 是否丢线 */
    bool is_running;                        /* 是否运行 */
} APP_TraceState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_Trace_Init(void);
void APP_Trace_Task(void);

void APP_Trace_Start(void);
void APP_Trace_Stop(void);

void APP_Trace_SetBasePwm(float base_pwm);
void APP_Trace_SetBaseSpeedCmps(float base_cm_s);
void APP_Trace_SetTestSpeedCmps(float left_cm_s, float right_cm_s);
void APP_Trace_SetPid(float kp, float ki, float kd);

void APP_Trace_GetState(APP_TraceState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* APP_TRACE_H_ */
