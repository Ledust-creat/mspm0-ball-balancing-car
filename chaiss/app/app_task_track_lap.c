#include "app_task_track_lap.h"
#include "app_chassis_control.h"
#include "app_line_follow.h"
#include "interrupt.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_TRACK_LAP_BASE_SPEED_CM_S          (50.0f)    /* 第二问底盘基础速度，单位cm/s */
#define APP_TRACK_LAP_LINE_KP                  (3.72f)    /* 第二问循迹外环Kp */
#define APP_TRACK_LAP_LINE_KI                  (0.0f)     /* 第二问循迹外环Ki */
#define APP_TRACK_LAP_LINE_KD                  (60.0f)    /* 第二问循迹外环Kd */
#define APP_TRACK_LAP_FINISH_IGNORE_MS         (5000U)    /* 发车后忽略起点黑线时间，单位ms */
#define APP_TRACK_LAP_FINISH_BLACK_COUNT_MIN   (4U)       /* 终点停车最少黑线通道数，单位路 */
#define APP_TRACK_LAP_FINISH_CONFIRM_MS        (60U)      /* 终点黑线连续确认时间，单位ms */

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static APP_TrackLapState_t s_state;             /* 第二问任务状态 */
static uint32_t s_start_tick_ms;                /* 第二问发车时刻，单位ms */
static uint32_t s_finish_detect_start_ms;       /* 终点线开始满足条件时刻，单位ms */
static bool s_finish_detecting;                 /* true表示正在连续确认终点线 */

/*
*********************************************************************************************************
*   函 数 名: APP_TaskTrackLap_Init
*   功能说明: 初始化第二问循迹一圈任务状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskTrackLap_Init(void)
{
    s_state = APP_TRACK_LAP_STATE_IDLE;
    s_start_tick_ms = 0U;
    s_finish_detect_start_ms = 0U;
    s_finish_detecting = false;
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskTrackLap_Start
*   功能说明: 启动第二问循迹一圈任务
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskTrackLap_Start(void)
{
    s_state = APP_TRACK_LAP_STATE_RUNNING;
    s_start_tick_ms = Get_Tick();
    s_finish_detect_start_ms = 0U;
    s_finish_detecting = false;

    APP_LineFollow_SetPid(APP_TRACK_LAP_LINE_KP,
                          APP_TRACK_LAP_LINE_KI,
                          APP_TRACK_LAP_LINE_KD);
    APP_ChassisControl_SetBaseSpeedCmps(APP_TRACK_LAP_BASE_SPEED_CM_S);
    APP_ChassisControl_Start();
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskTrackLap_Task
*   功能说明: 检测第二问终点线，满足条件后停止底盘并标记完成
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskTrackLap_Task(uint32_t now_ms)
{
    APP_LineFollowState_t line_state;

    if (s_state != APP_TRACK_LAP_STATE_RUNNING) {
        return;
    }

    if ((uint32_t)(now_ms - s_start_tick_ms) < APP_TRACK_LAP_FINISH_IGNORE_MS) {
        s_finish_detecting = false;
        return;
    }

    APP_LineFollow_GetState(&line_state);
    if (line_state.black_count >= APP_TRACK_LAP_FINISH_BLACK_COUNT_MIN) {
        if (!s_finish_detecting) {
            s_finish_detecting = true;
            s_finish_detect_start_ms = now_ms;
        }

        if ((uint32_t)(now_ms - s_finish_detect_start_ms) >= APP_TRACK_LAP_FINISH_CONFIRM_MS) {
            APP_ChassisControl_Stop();
            s_state = APP_TRACK_LAP_STATE_FINISHED;
        }
    } else {
        s_finish_detecting = false;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskTrackLap_Stop
*   功能说明: 停止第二问任务并复位为空闲状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskTrackLap_Stop(void)
{
    APP_ChassisControl_Stop();
    s_state = APP_TRACK_LAP_STATE_IDLE;
    s_finish_detect_start_ms = 0U;
    s_finish_detecting = false;
}

bool APP_TaskTrackLap_IsFinished(void)
{
    return (s_state == APP_TRACK_LAP_STATE_FINISHED);
}

APP_TrackLapState_t APP_TaskTrackLap_GetState(void)
{
    return s_state;
}
