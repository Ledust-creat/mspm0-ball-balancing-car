#include "app_task_static_ball.h"
#include "app_ball_control.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_STATIC_BALL_TARGET_POS_CM             (5.0f)      /* 第一个目标位置，单位cm */
#define APP_STATIC_BALL_TARGET_NEG_CM             (-5.0f)     /* 第二个目标位置，单位cm */

#define APP_STATIC_BALL_ACCEL_ANGLE_DEG           (4.0f)      /* 加速固定倾角，单位deg */
#define APP_STATIC_BALL_BRAKE_ANGLE_DEG           (4.0f)      /* 制动固定倾角，单位deg */
#define APP_STATIC_BALL_SERVO_MOVE_TIME_MS        (30U)       /* 固定倾角舵机移动时间，单位ms */

#define APP_STATIC_BALL_ACCEL_MIN_MS              (120U)      /* 最短加速时间，避免刚启动即制动 */
#define APP_STATIC_BALL_ACCEL_MAX_MS              (650U)      /* 最长加速时间，防止越过目标太多 */
#define APP_STATIC_BALL_BRAKE_TIME_MS             (180U)      /* 反向制动持续时间 */
#define APP_STATIC_BALL_SETTLE_TIME_MS            (120U)      /* 切入PID前水平等待时间 */
#define APP_STATIC_BALL_TARGET_TIMEOUT_MS         (5000U)     /* 单个目标最大尝试时间 */

#define APP_STATIC_POSITION_ERROR_CM              (0.5f)      /* 位置稳定误差阈值 */
#define APP_STATIC_SPEED_LIMIT_CM_S               (1.0f)      /* 速度稳定阈值 */
#define APP_STATIC_HOLD_TIME_MS                   (300U)      /* 稳定条件持续时间 */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    float speed_cm_s;                            /* 球速绝对值，单位cm/s */
    float brake_distance_cm;                     /* 建议提前制动距离，单位cm */
} APP_StaticBallBrakePoint_t;

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static APP_StaticBallState_t s_state;            /* 第三问任务状态 */
static uint8_t s_target_index;                   /* 0表示+5cm，1表示-5cm */
static uint32_t s_state_start_ms;                /* 当前状态进入时刻 */
static uint32_t s_target_start_ms;               /* 当前目标开始时刻 */
static uint32_t s_hold_start_ms;                 /* 稳定条件开始满足时刻 */
static bool s_hold_timing;                       /* true表示正在累计稳定时间 */

static const APP_StaticBallBrakePoint_t s_brake_table[] = {
    { 2.0f,  0.8f },
    { 5.0f,  1.5f },
    { 10.0f, 2.6f },
    { 15.0f, 3.8f },
    { 20.0f, 5.0f }
};

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float abs_float(float value);
static float get_target_cm(void);
static float get_target_sign(void);
static float calc_brake_distance_cm(float speed_cm_s);
static void enter_accel(uint32_t now_ms);
static void enter_brake(uint32_t now_ms);
static void enter_settle(uint32_t now_ms);
static void enter_hold(uint32_t now_ms);
static bool is_vision_timeout(const APP_BallControlStatus_t *ball, uint32_t now_ms);

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
*   函 数 名: get_target_cm
*   功能说明: 获取当前阶段目标位置
*   形    参: 无
*   返 回 值: 目标位置，单位cm
*********************************************************************************************************
*/
static float get_target_cm(void)
{
    return (s_target_index == 0U) ? APP_STATIC_BALL_TARGET_POS_CM : APP_STATIC_BALL_TARGET_NEG_CM;
}

/*
*********************************************************************************************************
*   函 数 名: get_target_sign
*   功能说明: 获取当前目标方向符号，用于固定倾角方向
*   形    参: 无
*   返 回 值: +1或-1
*********************************************************************************************************
*/
static float get_target_sign(void)
{
    return (get_target_cm() >= 0.0f) ? 1.0f : -1.0f;
}

/*
*********************************************************************************************************
*   函 数 名: calc_brake_distance_cm
*   功能说明: 根据球速查表得到提前制动距离
*   形    参: speed_cm_s - 球速绝对值，单位cm/s
*   返 回 值: 制动距离，单位cm
*********************************************************************************************************
*/
static float calc_brake_distance_cm(float speed_cm_s)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)(sizeof(s_brake_table) / sizeof(s_brake_table[0])); i++) {
        if (speed_cm_s <= s_brake_table[i].speed_cm_s) {
            return s_brake_table[i].brake_distance_cm;
        }
    }

    return s_brake_table[(sizeof(s_brake_table) / sizeof(s_brake_table[0])) - 1U].brake_distance_cm;
}

static void enter_accel(uint32_t now_ms)
{
    s_state = APP_STATIC_BALL_ACCEL;
    s_state_start_ms = now_ms;
    s_target_start_ms = now_ms;
    s_hold_timing = false;
    APP_BallControl_SetMode(APP_BALL_MODE_MANUAL_TILT);
    APP_BallControl_SetManualTiltDeg(get_target_sign() * APP_STATIC_BALL_ACCEL_ANGLE_DEG,
                                     APP_STATIC_BALL_SERVO_MOVE_TIME_MS);
}

static void enter_brake(uint32_t now_ms)
{
    s_state = APP_STATIC_BALL_BRAKE;
    s_state_start_ms = now_ms;
    APP_BallControl_SetManualTiltDeg(-get_target_sign() * APP_STATIC_BALL_BRAKE_ANGLE_DEG,
                                     APP_STATIC_BALL_SERVO_MOVE_TIME_MS);
}

static void enter_settle(uint32_t now_ms)
{
    s_state = APP_STATIC_BALL_SETTLE;
    s_state_start_ms = now_ms;
    APP_BallControl_SetManualTiltDeg(0.0f, APP_STATIC_BALL_SERVO_MOVE_TIME_MS);
}

static void enter_hold(uint32_t now_ms)
{
    s_state = APP_STATIC_BALL_HOLD;
    s_state_start_ms = now_ms;
    s_hold_timing = false;
    APP_BallControl_StartHold(get_target_cm());
}

static bool is_vision_timeout(const APP_BallControlStatus_t *ball, uint32_t now_ms)
{
    if (!ball->vision_valid) {
        return ((uint32_t)(now_ms - s_target_start_ms) > APP_BALL_VISION_TIMEOUT_MS);
    }

    return ((uint32_t)(now_ms - ball->last_vision_tick_ms) > APP_BALL_VISION_TIMEOUT_MS);
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskStaticBall_Init
*   功能说明: 初始化第三问静态往返控球任务
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskStaticBall_Init(void)
{
    s_state = APP_STATIC_BALL_IDLE;
    s_target_index = 0U;
    s_state_start_ms = 0U;
    s_target_start_ms = 0U;
    s_hold_start_ms = 0U;
    s_hold_timing = false;
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskStaticBall_Start
*   功能说明: 启动第三问任务，目标顺序为+5cm到-5cm
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskStaticBall_Start(void)
{
    s_target_index = 0U;
    APP_BallControl_Reset();
    enter_accel(0U);
    s_state_start_ms = 0U;
    s_target_start_ms = 0U;
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskStaticBall_Task
*   功能说明: 第三问静态往返控球状态机
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskStaticBall_Task(uint32_t now_ms)
{
    APP_BallControlStatus_t ball;
    float target_cm;
    float remain_cm;
    float speed_abs_cm_s;
    float brake_distance_cm;

    if ((s_state == APP_STATIC_BALL_IDLE) ||
        (s_state == APP_STATIC_BALL_FINISHED) ||
        (s_state == APP_STATIC_BALL_FAULT)) {
        return;
    }

    if (s_target_start_ms == 0U) {
        s_target_start_ms = now_ms;
    }

    if (s_state_start_ms == 0U) {
        s_state_start_ms = now_ms;
    }

    APP_BallControl_GetStatus(&ball);
    if (APP_BallControl_IsFault() || is_vision_timeout(&ball, now_ms) ||
        ((uint32_t)(now_ms - s_target_start_ms) > APP_STATIC_BALL_TARGET_TIMEOUT_MS)) {
        s_state = APP_STATIC_BALL_FAULT;
        APP_BallControl_SetMode(APP_BALL_MODE_FAULT);
        return;
    }

    target_cm = get_target_cm();
    remain_cm = abs_float(target_cm - ball.measured_position_cm);
    speed_abs_cm_s = abs_float(ball.estimated_velocity_cm_s);
    brake_distance_cm = calc_brake_distance_cm(speed_abs_cm_s);

    switch (s_state) {
        case APP_STATIC_BALL_ACCEL:
            if ((((uint32_t)(now_ms - s_state_start_ms) >= APP_STATIC_BALL_ACCEL_MIN_MS) &&
                 (remain_cm <= brake_distance_cm)) ||
                ((uint32_t)(now_ms - s_state_start_ms) >= APP_STATIC_BALL_ACCEL_MAX_MS)) {
                enter_brake(now_ms);
            }
            break;

        case APP_STATIC_BALL_BRAKE:
            if ((uint32_t)(now_ms - s_state_start_ms) >= APP_STATIC_BALL_BRAKE_TIME_MS) {
                enter_settle(now_ms);
            }
            break;

        case APP_STATIC_BALL_SETTLE:
            if ((uint32_t)(now_ms - s_state_start_ms) >= APP_STATIC_BALL_SETTLE_TIME_MS) {
                enter_hold(now_ms);
            }
            break;

        case APP_STATIC_BALL_HOLD:
            if ((abs_float(ball.error_cm) <= APP_STATIC_POSITION_ERROR_CM) &&
                (speed_abs_cm_s <= APP_STATIC_SPEED_LIMIT_CM_S)) {
                if (!s_hold_timing) {
                    s_hold_timing = true;
                    s_hold_start_ms = now_ms;
                }

                if ((uint32_t)(now_ms - s_hold_start_ms) >= APP_STATIC_HOLD_TIME_MS) {
                    s_state = APP_STATIC_BALL_NEXT_TARGET;
                }
            } else {
                s_hold_timing = false;
            }
            break;

        case APP_STATIC_BALL_NEXT_TARGET:
            if (s_target_index == 0U) {
                s_target_index = 1U;
                enter_accel(now_ms);
            } else {
                s_state = APP_STATIC_BALL_FINISHED;
            }
            break;

        default:
            break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_TaskStaticBall_Stop
*   功能说明: 停止第三问任务并关闭钢球控制
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_TaskStaticBall_Stop(void)
{
    APP_BallControl_SetMode(APP_BALL_MODE_DISABLE);
    s_state = APP_STATIC_BALL_IDLE;
    s_hold_timing = false;
}

bool APP_TaskStaticBall_IsFinished(void)
{
    return (s_state == APP_STATIC_BALL_FINISHED);
}

APP_StaticBallState_t APP_TaskStaticBall_GetState(void)
{
    return s_state;
}
