#ifndef APP_LINE_FOLLOW_H_
#define APP_LINE_FOLLOW_H_

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
#define APP_LINE_FOLLOW_ERROR_SIGN             (1.0f)     /* 循迹误差方向修正 */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    float line_error;                          /* 循迹误差 */
    float steering_correction_cm_s;            /* 转向修正速度，正值表示右轮目标速度高于左轮 */
    float speed_scale;                         /* 弯道速度比例，1.0表示不减速 */
    uint8_t black_count;                       /* 检测到黑线的通道数量 */
    bool is_lost;                              /* true表示当前丢线 */
    bool is_searching;                         /* true表示丢线后正在按原方向找线 */
    bool should_stop;                          /* true表示找线超时，需要停车 */
} APP_LineFollowOutput_t;

typedef struct
{
    PID_t line_pid;                            /* 循迹外环PID快照 */
    float line_error;                          /* 最近一次循迹误差 */
    float steering_correction_cm_s;            /* 最近一次转向修正速度 */
    float speed_scale;                         /* 最近一次速度比例 */
    uint16_t lost_ticks;                       /* 连续丢线周期计数 */
    uint8_t black_count;                       /* 最近一次黑线通道数量 */
    bool is_lost;                              /* 是否丢线 */
    bool is_searching;                         /* 是否正在找线 */
} APP_LineFollowState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_LineFollow_Init(void);
void APP_LineFollow_Reset(void);
void APP_LineFollow_SetPid(float kp, float ki, float kd);
void APP_LineFollow_Update(APP_LineFollowOutput_t *output);
void APP_LineFollow_GetState(APP_LineFollowState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_FOLLOW_H_ */
