#ifndef APP_TASK_MOBILE_BALL_H_
#define APP_TASK_MOBILE_BALL_H_

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
    APP_MOBILE_MODE_AB_CENTER = 0,          /* AB段行驶，钢球保持中心 */
    APP_MOBILE_MODE_LAP_CENTER,             /* 整圈行驶，钢球保持中心 */
    APP_MOBILE_MODE_LAP_TARGET              /* 整圈行驶，钢球保持指定位置 */
} APP_MobileMode_t;

typedef enum
{
    APP_MOBILE_BALL_IDLE = 0,
    APP_MOBILE_BALL_WAIT_VISION,
    APP_MOBILE_BALL_PRE_HOLD,
    APP_MOBILE_BALL_RAMP_UP,
    APP_MOBILE_BALL_RUNNING,
    APP_MOBILE_BALL_RAMP_DOWN,
    APP_MOBILE_BALL_FINAL_HOLD,
    APP_MOBILE_BALL_FINISHED,
    APP_MOBILE_BALL_FAULT
} APP_MobileBallState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_TaskMobileBall_Init(void);
void APP_TaskMobileBall_Start(APP_MobileMode_t mode, float target_position_cm);
void APP_TaskMobileBall_Task(uint32_t now_ms);
void APP_TaskMobileBall_Stop(void);
bool APP_TaskMobileBall_IsFinished(void);
bool APP_TaskMobileBall_IsFault(void);
APP_MobileBallState_t APP_TaskMobileBall_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_MOBILE_BALL_H_ */
