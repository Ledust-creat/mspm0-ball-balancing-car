#include "app_task_track_lap.h"
#include "app_chassis_control.h"
#include "app_line_follow.h"
#include "interrupt.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_TRACK_LAP_BASE_SPEED_CM_S          (45.0f)    /* 第二问底盘基础速度，单位cm/s */
#define APP_TRACK_LAP_LINE_KP                  (3.72f)    /* 第二问循迹外环Kp */
#define APP_TRACK_LAP_LINE_KI                  (0.0f)     /* 第二问循迹外环Ki */
#define APP_TRACK_LAP_LINE_KD                  (60.0f)    /* 第二问循迹外环Kd */
#define APP_TRACK_LAP_ERROR_FILTER_ALPHA       (1.0f)     /* 第二问使用原始误差响应，不引入额外滞后 */
#define APP_TRACK_LAP_FINISH_IGNORE_MS         (5000U)    /* 发车后忽略起点黑线时间，单位ms */
#define APP_TRACK_LAP_FINISH_BLACK_COUNT_MIN   (3U)       /* 第二问沿用原终点阈值，避免普通弯道误判停车 */
#define APP_TRACK_LAP_FINISH_CONFIRM_MS        (10U)      /* 低速接近时仅确认一个控制周期，减少停车距离 */
#define APP_TRACK_LAP_CORNER_COUNT              (3U)       /* 减速规划使用的弯道计数上限，不限制停车 */
#define APP_TRACK_LAP_CORNER_ENTER_ERROR        (3.0f)     /* 判定进入弯道的误差绝对值 */
#define APP_TRACK_LAP_CORNER_EXIT_ERROR         (1.5f)     /* 判定回到中心的误差绝对值 */
#define APP_TRACK_LAP_CORNER_EXIT_CONFIRM_MS    (5U)     /* 出弯后中心状态连续确认时间 */
#define APP_TRACK_LAP_FINISH_APPROACH_CM_S      (20.0f)    /* 最后出弯后的停止线接近速度 */
#define APP_TRACK_LAP_SLOWDOWN_TIME_MS          (5000U)    /* 进入减速段后均匀减到接近速度的时间 */

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static APP_TrackLapState_t s_state;             /* 第二问任务状态 */
static uint32_t s_start_tick_ms;                /* 第二问发车时刻，单位ms */
static uint32_t s_finish_detect_start_ms;       /* 终点线开始满足条件时刻，单位ms */
static bool s_finish_detecting;                 /* true表示正在连续确认终点线 */
static uint32_t s_corner_exit_start_ms;         /* 当前弯道出弯稳定计时起点 */
static uint8_t s_corner_count;                  /* 已确认通过的弯道数量 */
static bool s_in_corner;                        /* true表示当前处于弯道 */
static bool s_corner_exit_detecting;            /* true表示正在确认出弯回正 */
static bool s_finish_slowdown;                  /* true表示进入第三个弯后已经开始降速 */
static uint32_t s_slowdown_start_ms;            /* 均匀减速过程起始时刻 */

/*
*********************************************************************************************************
*   函 数 名: abs_float
*   功能说明: 计算浮点数绝对值
*   形    参: value - 输入值
*   返 回 值: 绝对值
*********************************************************************************************************
*/
static float abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

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
    s_corner_exit_start_ms = 0U;
    s_corner_count = 0U;
    s_in_corner = false;
    s_corner_exit_detecting = false;
    s_finish_slowdown = false;
    s_slowdown_start_ms = 0U;
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
    s_corner_exit_start_ms = 0U;
    s_corner_count = 0U;
    s_in_corner = false;
    s_corner_exit_detecting = false;
    s_finish_slowdown = false;
    s_slowdown_start_ms = 0U;


    APP_LineFollow_SetErrorFilterAlpha(APP_TRACK_LAP_ERROR_FILTER_ALPHA);
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
    float error_abs;

    if (s_state != APP_TRACK_LAP_STATE_RUNNING) {
        return;
    }

    APP_LineFollow_GetState(&line_state);
    error_abs = abs_float(line_state.line_error);

    // 大误差只锁存一次入弯，避免同一个弯道被重复计数
    if (!s_in_corner && (error_abs >= APP_TRACK_LAP_CORNER_ENTER_ERROR)) {
        s_in_corner = true;
        s_corner_exit_detecting = false;

        // 已完成两个弯后，本次即为第三个弯；从入弯处提前降速，为第四弯出口停车做准备
        if ((s_corner_count >= 2U) && !s_finish_slowdown) {
            s_finish_slowdown = true;
            s_slowdown_start_ms = now_ms; // 从当前入弯时刻开始按宏定义时间均匀减速
        }
    }

    if (s_finish_slowdown) {
        uint32_t slowdown_elapsed_ms;
        float slowdown_ratio;
        float planned_speed_cm_s;

        slowdown_elapsed_ms = (uint32_t)(now_ms - s_slowdown_start_ms);
        if (slowdown_elapsed_ms >= APP_TRACK_LAP_SLOWDOWN_TIME_MS) {
            planned_speed_cm_s = APP_TRACK_LAP_FINISH_APPROACH_CM_S;
        } else {
            // 从巡航速度线性插值到接近速度，避免目标速度一次性突变
            slowdown_ratio = (float)slowdown_elapsed_ms /
                             (float)APP_TRACK_LAP_SLOWDOWN_TIME_MS;
            planned_speed_cm_s = APP_TRACK_LAP_BASE_SPEED_CM_S -
                                 ((APP_TRACK_LAP_BASE_SPEED_CM_S -
                                   APP_TRACK_LAP_FINISH_APPROACH_CM_S) *
                                  slowdown_ratio);
        }
        APP_ChassisControl_SetBaseSpeedCmps(planned_speed_cm_s);
    }

    if (s_in_corner) {
        if (error_abs <= APP_TRACK_LAP_CORNER_EXIT_ERROR) {
            if (!s_corner_exit_detecting) {
                s_corner_exit_detecting = true;
                s_corner_exit_start_ms = now_ms;
            } else if ((uint32_t)(now_ms - s_corner_exit_start_ms) >=
                       APP_TRACK_LAP_CORNER_EXIT_CONFIRM_MS) {
                s_in_corner = false;
                s_corner_exit_detecting = false;
                if (s_corner_count < APP_TRACK_LAP_CORNER_COUNT) {
                    s_corner_count++;
                }
            }
        } else {
            s_corner_exit_detecting = false; // 尚未稳定回中，重新等待连续中心状态
        }
    }

    // 起点屏蔽结束后独立确认停车线；弯道计数仅用于提前减速，漏计不能阻止停车
    if ((uint32_t)(now_ms - s_start_tick_ms) < APP_TRACK_LAP_FINISH_IGNORE_MS) {
        s_finish_detecting = false;
        return;
    }

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
    s_corner_exit_start_ms = 0U;
    s_corner_count = 0U;
    s_in_corner = false;
    s_corner_exit_detecting = false;
    s_finish_slowdown = false;
    s_slowdown_start_ms = 0U;
}

bool APP_TaskTrackLap_IsFinished(void)
{
    return (s_state == APP_TRACK_LAP_STATE_FINISHED);
}

APP_TrackLapState_t APP_TaskTrackLap_GetState(void)
{
    return s_state;
}
