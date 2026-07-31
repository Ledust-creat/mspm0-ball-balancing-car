#ifndef APP_TASK_TRACK_LAP_H_
#define APP_TASK_TRACK_LAP_H_

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
    APP_TRACK_LAP_STATE_IDLE = 0,          /* 未运行 */
    APP_TRACK_LAP_STATE_RUNNING,           /* 正在循迹一圈 */
    APP_TRACK_LAP_STATE_FINISHED,          /* 已检测终点并停车 */
    APP_TRACK_LAP_STATE_FAULT              /* 运行故障 */
} APP_TrackLapState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_TaskTrackLap_Init(void);
void APP_TaskTrackLap_Start(void);
void APP_TaskTrackLap_Task(uint32_t now_ms);
void APP_TaskTrackLap_Stop(void);
bool APP_TaskTrackLap_IsFinished(void);
APP_TrackLapState_t APP_TaskTrackLap_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_TRACK_LAP_H_ */
