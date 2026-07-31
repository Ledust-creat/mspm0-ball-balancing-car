#ifndef APP_TASK_STATIC_BALL_H_
#define APP_TASK_STATIC_BALL_H_

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
    APP_STATIC_BALL_IDLE = 0,              /* 任务空闲 */
    APP_STATIC_BALL_ACCEL,                 /* 固定角度加速 */
    APP_STATIC_BALL_BRAKE,                 /* 反向固定角度制动 */
    APP_STATIC_BALL_SETTLE,                /* 回水平等待机械和球速缓和 */
    APP_STATIC_BALL_HOLD,                  /* PID保持当前目标 */
    APP_STATIC_BALL_NEXT_TARGET,           /* 切换到下一个目标 */
    APP_STATIC_BALL_FINISHED,              /* +5cm和-5cm均完成 */
    APP_STATIC_BALL_FAULT                  /* 视觉超时或控球故障 */
} APP_StaticBallState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_TaskStaticBall_Init(void);
void APP_TaskStaticBall_Start(void);
void APP_TaskStaticBall_Task(uint32_t now_ms);
void APP_TaskStaticBall_Stop(void);
bool APP_TaskStaticBall_IsFinished(void);
APP_StaticBallState_t APP_TaskStaticBall_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_STATIC_BALL_H_ */
