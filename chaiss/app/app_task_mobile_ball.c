#include "app_task_mobile_ball.h"
#include "app_ball_control.h"
#include "app_chassis_control.h"
#include "app_line_follow.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_MOBILE_PRE_HOLD_MS                 (500U)     /* 发车前控球稳定时间 */
#define APP_MOBILE_FINAL_HOLD_MS               (500U)     /* 停车后继续控球时间 */
#define APP_MOBILE_VISION_WAIT_TIMEOUT_MS      (1000U)    /* 等待首帧视觉超时 */
#define APP_MOBILE_AB_CURVE_IGNORE_MS          (1000U)    /* AB模式发车后弯道检测屏蔽时间 */
#define APP_MOBILE_LAP_FINISH_IGNORE_MS        (5000U)    /* 整圈模式起点黑线屏蔽时间 */
#define APP_MOBILE_FINISH_BLACK_COUNT_MIN      (4U)       /* 终点黑线最少触发通道 */
#define APP_MOBILE_FINISH_CONFIRM_MS           (60U)      /* 结束条件连续确认时间 */
#define APP_MOBILE_AB_CURVE_ERROR_MIN          (3.0f)     /* AB模式进入首个弯道误差阈值 */
#define APP_MOBILE_RAMP_READY_RATIO            (0.80f)    /* 规划速度达到目标比例后进入运行 */
#define APP_MOBILE_CRUISE_SPEED_CENTER_CM_S    (20.0f)    /* 中心控球巡航速度 */
#define APP_MOBILE_CRUISE_SPEED_TARGET_CM_S    (15.0f)    /* 指定位置控球巡航速度 */

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static APP_MobileBallState_t s_state;           /* 移动控球状态机当前状态 */
static APP_MobileMode_t s_mode;                 /* 当前后三问任务模式 */
static float s_target_position_cm;              /* 当前钢球目标位置，单位cm */
static float s_cruise_speed_cm_s;               /* 当前任务巡航速度，单位cm/s */
static uint32_t s_start_tick_ms;                /* 底盘实际发车时刻，单位ms */
static uint32_t s_state_start_ms;               /* 当前状态进入时刻，单位ms */
static uint32_t s_finish_detect_start_ms;       /* 结束条件首次成立时刻，单位ms */
static bool s_finish_detecting;                 /* true表示正在连续确认结束条件 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float abs_float(float value);
static void enter_state(APP_MobileBallState_t state, uint32_t now_ms);
static void enter_fault(void);
static bool update_finish_detection(bool condition, uint32_t now_ms);

/*
*********************************************************************************************************
*   函 数 名: abs_float
*   功能说明: 计算浮点绝对值
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
*   函 数 名: enter_state
*   功能说明: 切换移动控球状态并复位该状态的计时与确认标志
*   形    参: state - 新状态；now_ms - 当前时间
*   返 回 值: 无
*********************************************************************************************************
*/
static void enter_state(APP_MobileBallState_t state, uint32_t now_ms)
{
    s_state = state;
    s_state_start_ms = now_ms;           // 每个状态使用独立进入时间
    s_finish_detect_start_ms = 0U;       // 状态切换后重新确认结束条件
    s_finish_detecting = false;
}

/*
*********************************************************************************************************
*   函 数 名: enter_fault
*   功能说明: 执行移动控球故障保护并进入FAULT状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void enter_fault(void)
{
    APP_ChassisControl_Stop();                         // 故障使用硬停车，不等待速度斜坡
    APP_ChassisControl_SetBallPriorityMode(false);    // 关闭后三问专用速度限制
    APP_BallControl_SetMode(APP_BALL_MODE_FAULT);     // 横梁进入钢球安全故障状态
    s_state = APP_MOBILE_BALL_FAULT;
}

/*
*********************************************************************************************************
*   函 数 名: update_finish_detection
*   功能说明: 对终点或入弯条件进行连续时间确认，过滤单次传感器跳变
*   形    参: condition - 当前条件；now_ms - 当前时间
*   返 回 值: true表示条件已持续达到确认时间
*********************************************************************************************************
*/
static bool update_finish_detection(bool condition, uint32_t now_ms)
{
    if (!condition) {
        s_finish_detecting = false;       // 条件中断后必须重新计时
        return false;
    }

    if (!s_finish_detecting) {
        s_finish_detecting = true;
        s_finish_detect_start_ms = now_ms; // 记录本次连续有效起点
    }

    return ((uint32_t)(now_ms - s_finish_detect_start_ms) >=
            APP_MOBILE_FINISH_CONFIRM_MS);
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskMobileBall_Init
*   功能说明: 初始化后三问移动控球任务
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskMobileBall_Init(void)
{
    s_state = APP_MOBILE_BALL_IDLE;
    s_mode = APP_MOBILE_MODE_AB_CENTER;
    s_target_position_cm = 0.0f;
    s_cruise_speed_cm_s = APP_MOBILE_CRUISE_SPEED_CENTER_CM_S;
    s_start_tick_ms = 0U;
    s_state_start_ms = 0U;
    s_finish_detect_start_ms = 0U;
    s_finish_detecting = false;
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskMobileBall_Start
*   功能说明: 启动指定的后三问移动控球任务
*   形    参: mode - 任务模式；target_position_cm - 钢球目标位置
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskMobileBall_Start(APP_MobileMode_t mode, float target_position_cm)
{
    s_mode = mode;
    s_target_position_cm = target_position_cm;
    s_cruise_speed_cm_s = (mode == APP_MOBILE_MODE_LAP_TARGET) ? // 偏置控球降低巡航速度
                          APP_MOBILE_CRUISE_SPEED_TARGET_CM_S :
                          APP_MOBILE_CRUISE_SPEED_CENTER_CM_S;
    s_start_tick_ms = 0U;
    APP_ChassisControl_Stop();                         // 启动状态机前保证底盘静止
    APP_ChassisControl_SetBallPriorityMode(true);     // 仅后三问启用速度斜坡和转向限制
    APP_ChassisControl_SetTargetBaseSpeed(0.0f);      // 收到有效视觉前禁止发车
    APP_BallControl_Reset();                          // 清除上一次任务的PID动态量
    enter_state(APP_MOBILE_BALL_WAIT_VISION, 0U);     // 首次Task调用时补写真实进入时间
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskMobileBall_Task
*   功能说明: 执行移动控球状态机、结束检测和软停车
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskMobileBall_Task(uint32_t now_ms)
{
    APP_BallControlStatus_t ball;
    APP_ChassisControlState_t chassis;
    APP_LineFollowState_t line;
    bool finish_condition;

    if ((s_state == APP_MOBILE_BALL_IDLE) ||
        (s_state == APP_MOBILE_BALL_FINISHED) ||
        (s_state == APP_MOBILE_BALL_FAULT)) {
        return; // 空闲、完成和故障状态不再执行控制流程
    }

    if (s_state_start_ms == 0U) {
        s_state_start_ms = now_ms; // Start接口无时间参数，在这里建立准确计时基准
    }

    APP_BallControl_GetStatus(&ball);
    if (APP_BallControl_IsFault()) {
        enter_fault(); // K230超时等钢球故障立即联动停车
        return;
    }

    if ((s_state == APP_MOBILE_BALL_RAMP_UP) ||
        (s_state == APP_MOBILE_BALL_RUNNING)) {
        APP_ChassisControl_GetState(&chassis);
        if (!chassis.is_running) {
            enter_fault(); // 循迹丢线导致底盘异常停机时同步转为任务故障
            return;
        }
    }

    switch (s_state) {
        case APP_MOBILE_BALL_WAIT_VISION:
            if (ball.vision_valid) {
                APP_BallControl_StartHold(s_target_position_cm); // 首帧有效后才启动位置闭环
                enter_state(APP_MOBILE_BALL_PRE_HOLD, now_ms);   // 先稳定钢球，再启动底盘
            } else if ((uint32_t)(now_ms - s_state_start_ms) >=
                       APP_MOBILE_VISION_WAIT_TIMEOUT_MS) {
                enter_fault();
            }
            break;

        case APP_MOBILE_BALL_PRE_HOLD:
            if ((uint32_t)(now_ms - s_state_start_ms) >= APP_MOBILE_PRE_HOLD_MS) {
                APP_ChassisControl_SetTargetBaseSpeed(s_cruise_speed_cm_s); // 设置斜坡最终速度
                APP_ChassisControl_Start();                                // 从0开始平滑发车
                s_start_tick_ms = now_ms;                                  // 结束检测从实际发车计时
                enter_state(APP_MOBILE_BALL_RAMP_UP, now_ms);
            }
            break;

        case APP_MOBILE_BALL_RAMP_UP:
            if (chassis.planned_base_speed_cm_s >=
                (s_cruise_speed_cm_s * APP_MOBILE_RAMP_READY_RATIO)) {
                enter_state(APP_MOBILE_BALL_RUNNING, now_ms); // 达到80%巡航速度后开放终点检测
            }
            break;

        case APP_MOBILE_BALL_RUNNING:
            APP_LineFollow_GetState(&line);
            if (s_mode == APP_MOBILE_MODE_AB_CENTER) {
                // AB模式用明显循迹误差近似判断进入第一个弯道
                finish_condition = ((uint32_t)(now_ms - s_start_tick_ms) >=
                                    APP_MOBILE_AB_CURVE_IGNORE_MS) &&
                                   (abs_float(line.line_error) >=
                                    APP_MOBILE_AB_CURVE_ERROR_MIN);
            } else {
                // 整圈模式复用第二问的宽黑线终点判断
                finish_condition = ((uint32_t)(now_ms - s_start_tick_ms) >=
                                    APP_MOBILE_LAP_FINISH_IGNORE_MS) &&
                                   (line.black_count >=
                                    APP_MOBILE_FINISH_BLACK_COUNT_MIN);
            }

            if (update_finish_detection(finish_condition, now_ms)) {
                APP_ChassisControl_RequestSoftStop(); // 正常完成必须减速停车，避免钢球受冲击
                enter_state(APP_MOBILE_BALL_RAMP_DOWN, now_ms);
            }
            break;

        case APP_MOBILE_BALL_RAMP_DOWN:
            if (APP_ChassisControl_IsSoftStopped()) {
                enter_state(APP_MOBILE_BALL_FINAL_HOLD, now_ms); // 停车后继续闭环稳定钢球
            }
            break;

        case APP_MOBILE_BALL_FINAL_HOLD:
            if ((uint32_t)(now_ms - s_state_start_ms) >= APP_MOBILE_FINAL_HOLD_MS) {
                s_state = APP_MOBILE_BALL_FINISHED; // 保留钢球闭环输出，仅通知菜单任务完成
            }
            break;

        default:
            enter_fault();
            break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskMobileBall_Stop
*   功能说明: 人工停止移动控球任务并恢复普通底盘控制
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskMobileBall_Stop(void)
{
    APP_ChassisControl_Stop();                         // 用户停止直接执行安全硬停车
    APP_ChassisControl_SetBallPriorityMode(false);    // 恢复普通底盘控制语义
    APP_BallControl_SetEnable(0U);                    // 人工停止时关闭控球并回安全中位
    s_state = APP_MOBILE_BALL_IDLE;
    s_finish_detecting = false;
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskMobileBall_IsFinished
*   功能说明: 查询移动控球任务是否正常完成
*   形    参: 无
*   返 回 值: true已完成，false未完成
*********************************************************************************************************
*/
bool APP_TaskMobileBall_IsFinished(void)
{
    return (s_state == APP_MOBILE_BALL_FINISHED);
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskMobileBall_IsFault
*   功能说明: 查询移动控球任务是否进入故障状态
*   形    参: 无
*   返 回 值: true故障，false无故障
*********************************************************************************************************
*/
bool APP_TaskMobileBall_IsFault(void)
{
    return (s_state == APP_MOBILE_BALL_FAULT);
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskMobileBall_GetState
*   功能说明: 获取移动控球状态机当前状态
*   形    参: 无
*   返 回 值: 当前状态
*********************************************************************************************************
*/
APP_MobileBallState_t APP_TaskMobileBall_GetState(void)
{
    return s_state;
}
