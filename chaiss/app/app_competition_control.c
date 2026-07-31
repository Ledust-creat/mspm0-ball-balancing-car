#include "app_competition_control.h"
#include "app_ball_control.h"
#include "app_chassis_control.h"
#include "app_task_static_ball.h"
#include "app_task_mobile_ball.h"
#include "app_task_track_lap.h"

#define APP_COMPETITION_LAP_TARGET_CM          (5.0f)     /* 整圈指定位置控球目标 */

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static APP_CompetitionMode_t s_current_mode;     /* 当前正在运行的小题模式 */
static bool s_running;                           /* true表示已有小题正在运行 */

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionControl_Init
*   功能说明: 初始化统一比赛控制层
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionControl_Init(void)
{
    s_current_mode = APP_COMP_MODE_TRACK_LAP;
    s_running = false;
    APP_TaskTrackLap_Init();
    APP_TaskStaticBall_Init();
    APP_TaskMobileBall_Init();
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionControl_Start
*   功能说明: 根据菜单模式启动对应小题
*   形    参: mode - 菜单选择的小题模式
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionControl_Start(APP_CompetitionMode_t mode)
{
    APP_CompetitionControl_Stop();
    s_current_mode = mode;
    s_running = true;

    switch (mode) {
        case APP_COMP_MODE_TRACK_LAP:
            APP_TaskTrackLap_Start();
            break;

        case APP_COMP_MODE_STATIC_RECIPROCATE:
            APP_TaskStaticBall_Start();
            break;

        case APP_COMP_MODE_AB_CENTER:
            APP_TaskMobileBall_Start(APP_MOBILE_MODE_AB_CENTER, 0.0f);
            break;

        case APP_COMP_MODE_LAP_CENTER:
            APP_TaskMobileBall_Start(APP_MOBILE_MODE_LAP_CENTER, 0.0f);
            break;

        case APP_COMP_MODE_LAP_TARGET:
            APP_TaskMobileBall_Start(APP_MOBILE_MODE_LAP_TARGET,
                                     APP_COMPETITION_LAP_TARGET_CM);
            break;

        default:
            s_running = false;
            APP_CompetitionMenu_NotifyFault(APP_COMPETITION_FAULT_MODE_NOT_IMPLEMENTED);
            break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionControl_Task
*   功能说明: 周期调用当前小题状态机，并统一通知菜单完成状态
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionControl_Task(uint32_t now_ms)
{
    if (!s_running) {
        return;
    }

    switch (s_current_mode) {
        case APP_COMP_MODE_TRACK_LAP:
            APP_TaskTrackLap_Task(now_ms);
            if (APP_TaskTrackLap_IsFinished()) {
                s_running = false;
                APP_CompetitionMenu_NotifyFinished();
            }
            break;
                
        case APP_COMP_MODE_STATIC_RECIPROCATE:
            APP_TaskStaticBall_Task(now_ms);
            if (APP_TaskStaticBall_IsFinished()) {
                s_running = false;
                APP_CompetitionMenu_NotifyFinished();
            }
            break;

        case APP_COMP_MODE_AB_CENTER:
        case APP_COMP_MODE_LAP_CENTER:
        case APP_COMP_MODE_LAP_TARGET:
            APP_TaskMobileBall_Task(now_ms);
            if (APP_TaskMobileBall_IsFault()) {
                s_running = false;
                APP_CompetitionMenu_NotifyFault(APP_COMPETITION_FAULT_MOBILE_BALL);
            } else if (APP_TaskMobileBall_IsFinished()) {
                s_running = false;
                APP_CompetitionMenu_NotifyFinished();
            }
            break;

        default:
            break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionControl_Stop
*   功能说明: 统一停止当前比赛任务、底盘和钢球控制
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionControl_Stop(void)
{
    APP_TaskTrackLap_Stop();
    APP_TaskStaticBall_Stop();
    APP_TaskMobileBall_Stop();
    APP_ChassisControl_Stop();
    APP_BallControl_SetEnable(0U);
    s_running = false;
}
