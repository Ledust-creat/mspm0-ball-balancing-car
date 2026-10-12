#include "app_task_static_ball.h"
#include "app_ball_control.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_STATIC_BALL_TARGET_POS_CM             (5.0f)      /* 第一个目标位置，单位cm */
#define APP_STATIC_BALL_TARGET_NEG_CM             (-5.0f)     /* 第二个目标位置，单位cm */

#define APP_STATIC_BALL_PID_KP                    (0.25f)     /* 第三问静态往返专用位置环Kp */
#define APP_STATIC_BALL_PID_KI                    (0.01f)     /* 第三问静态往返专用位置环Ki */
#define APP_STATIC_BALL_PID_KD                    (0.8f)      /* 第三问静态往返专用位置环Kd */
#define APP_STATIC_BALL_SPEED_WINDOW_FRAME         (2U)        /* 第三问保持原相邻帧速度差分 */
#define APP_STATIC_BALL_SPEED_FILTER_ALPHA         (0.30f)     /* 第三问保持原速度低通系数 */

#define APP_STATIC_BALL_BRAKE_TO_SERVO_OFFSET     (60)        /* 球向舵机侧运动时制动偏移 */
#define APP_STATIC_BALL_ACCEL_SERVO_OFFSET        (-45)       /* 向舵机侧加速的零点偏移 */
#define APP_STATIC_BALL_ACCEL_HINGE_OFFSET        (12)       /* 向铰链侧加速的零点偏移 */
#define APP_STATIC_BALL_BRAKE_TO_HINGE_OFFSET     (-20)       /* 球向铰链侧运动时制动偏移 */
#define APP_STATIC_BALL_NEG_SETTLE_OFFSET         (5)         /* 到达-5cm后使用独立水平零点偏移 */
#define APP_STATIC_BALL_ACCEL_MIN_MS              (100U)      /* 最短加速时间，避免刚启动即制动 */
#define APP_STATIC_BALL_ACCEL_MAX_POS_MS          (1800U)     /* 0到+5cm阶段最长加速时间 */
#define APP_STATIC_BALL_ACCEL_MAX_NEG_MS          (120U)     /* +5到-5cm阶段最长加速时间 */
#define APP_STATIC_BALL_BRAKE_POS_TIME_MS         (120U)      /* 0到+5cm阶段反向制动时间 */
#define APP_STATIC_BALL_BRAKE_NEG_TIME_MS         (80U)      /* +5到-5cm阶段反向制动时间 */
#define APP_STATIC_BALL_SETTLE_POS_TIME_MS        (120U)      /* +5cm阶段切入PID前水平等待时间 */
#define APP_STATIC_BALL_SETTLE_NEG_TIME_MS        (100U)      /* -5cm阶段切入PID前水平等待时间 */
#define APP_STATIC_BALL_NEG_PID_RELEASE_MIN_MS     (20U)       /* 负向制动至少保持时间，避免速度噪声误触发 */
#define APP_STATIC_BALL_NEG_PID_RELEASE_ERROR_CM   (0.4f)      /* 距离-5cm目标0.4cm内允许PID接管 */
#define APP_STATIC_BALL_TARGET_TIMEOUT_MS         (8000U)     /* 单个目标最大尝试时间 */

#define APP_STATIC_POSITION_ERROR_CM              (0.85f)      /* 位置稳定误差阈值 */
#define APP_STATIC_SPEED_LIMIT_CM_S               (1.0f)      /* 速度稳定阈值 */
#define APP_STATIC_HOLD_POS_TIME_MS               (50U)      /* +5cm稳定条件需要持续的时间 */
#define APP_STATIC_HOLD_NEG_TIME_MS               (100U)      /* -5cm稳定条件需要持续的时间 */

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

static const APP_StaticBallBrakePoint_t s_brake_pos_table[] = {
    {  2.0f, 0.8f },  /* 低速时距离目标0.4cm开始制动 */
    {  5.0f, 1.5f },  /* 5cm/s以内提前0.8cm制动 */
    { 10.0f, 2.6f },  /* 10cm/s以内提前1.3cm制动 */
    { 15.0f, 3.8f },  /* 15cm/s以内提前1.9cm制动 */
    { 20.0f, 5.0f }   /* 高速时最多提前2.6cm制动 */
};

static const APP_StaticBallBrakePoint_t s_brake_neg_table[] = {
    {  2.0f, 1.2f },  /* 低速时距离目标0.4cm开始制动 */
    {  5.0f, 1.7f },  /* 5cm/s以内提前0.8cm制动 */
    { 10.0f, 2.8f },  /* 10cm/s以内提前1.3cm制动 */
    { 15.0f, 4.0f },  /* 15cm/s以内提前1.9cm制动 */
    { 20.0f, 5.2f }   /* 高速时最多提前2.6cm制动 */
};

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float abs_float(float value);
static float get_target_cm(void);
static int16_t get_accel_offset(void);
static int16_t get_brake_offset(void);
static uint32_t get_accel_max_ms(void);
static uint32_t get_brake_time_ms(void);
static uint32_t get_settle_time_ms(void);
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
*   函 数 名: get_accel_offset
*   功能说明: 获取当前目标方向对应的加速位置偏移
*   形    参: 无
*   返 回 值: 相对水平零点的有符号偏移
*********************************************************************************************************
*/
static int16_t get_accel_offset(void)
{
    // 默认视觉+5cm朝铰链侧；若实车相反，只交换这里两个宏的返回关系
    return (get_target_cm() >= 0.0f) ? APP_STATIC_BALL_ACCEL_SERVO_OFFSET :
                                      APP_STATIC_BALL_ACCEL_HINGE_OFFSET;
}

/*
*********************************************************************************************************
*   函 数 名: get_brake_offset
*   功能说明: 获取当前运动方向对应的反向制动位置偏移
*   形    参: 无
*   返 回 值: 相对水平零点的有符号偏移
*********************************************************************************************************
*/
static int16_t get_brake_offset(void)
{
    // 制动方向与当前钢球运动方向相反
    return (get_target_cm() >= 0.0f) ? APP_STATIC_BALL_BRAKE_TO_SERVO_OFFSET :
                                      APP_STATIC_BALL_BRAKE_TO_HINGE_OFFSET;
}

/*
*********************************************************************************************************
*   函 数 名: get_accel_max_ms
*   功能说明: 根据当前目标阶段获取最长加速时间
*   形    参: 无
*   返 回 值: 当前阶段最长加速时间，单位ms
*********************************************************************************************************
*/
static uint32_t get_accel_max_ms(void)
{
    // 两个目标阶段分别使用独立的最长加速时间
    return (s_target_index == 0U) ? APP_STATIC_BALL_ACCEL_MAX_POS_MS :
                                   APP_STATIC_BALL_ACCEL_MAX_NEG_MS;
}

/*
*********************************************************************************************************
*   函 数 名: get_brake_time_ms
*   功能说明: 根据当前目标阶段获取反向制动持续时间
*   形    参: 无
*   返 回 值: 当前阶段制动时间，单位ms
*********************************************************************************************************
*/
static uint32_t get_brake_time_ms(void)
{
    return (s_target_index == 0U) ? APP_STATIC_BALL_BRAKE_POS_TIME_MS :
                                   APP_STATIC_BALL_BRAKE_NEG_TIME_MS;
}

/*
*********************************************************************************************************
*   函 数 名: get_settle_time_ms
*   功能说明: 根据当前目标阶段获取切入PID前的水平等待时间
*   形    参: 无
*   返 回 值: 当前阶段水平等待时间，单位ms
*********************************************************************************************************
*/
static uint32_t get_settle_time_ms(void)
{
    return (s_target_index == 0U) ? APP_STATIC_BALL_SETTLE_POS_TIME_MS :
                                   APP_STATIC_BALL_SETTLE_NEG_TIME_MS;
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
    const APP_StaticBallBrakePoint_t *table;
    uint8_t table_count;
    uint8_t i;

    if (s_target_index == 0U) {
        table = s_brake_pos_table; // 0到+5cm阶段使用正向独立制动表
        table_count = (uint8_t)(sizeof(s_brake_pos_table) / sizeof(s_brake_pos_table[0]));
    } else {
        table = s_brake_neg_table; // +5到-5cm阶段使用负向独立制动表
        table_count = (uint8_t)(sizeof(s_brake_neg_table) / sizeof(s_brake_neg_table[0]));
    }

    for (i = 0U; i < table_count; i++) {
        if (speed_cm_s <= table[i].speed_cm_s) {
            return table[i].brake_distance_cm;
        }
    }

    return table[table_count - 1U].brake_distance_cm;
}

static void enter_accel(uint32_t now_ms)
{
    s_state = APP_STATIC_BALL_ACCEL;
    s_state_start_ms = now_ms;
    s_target_start_ms = now_ms;
    s_hold_timing = false;
    APP_BallControl_SetMode(APP_BALL_MODE_MANUAL_TILT);
    APP_BallControl_SetServoTestPosition(
        APP_BallControl_PositionFromLevelOffset(get_accel_offset()));
}

static void enter_brake(uint32_t now_ms)
{
    s_state = APP_STATIC_BALL_BRAKE;
    s_state_start_ms = now_ms;
    APP_BallControl_SetServoTestPosition(
        APP_BallControl_PositionFromLevelOffset(get_brake_offset()));
}

static void enter_settle(uint32_t now_ms)
{
    int16_t settle_offset;

    s_state = APP_STATIC_BALL_SETTLE;
    s_state_start_ms = now_ms;
    // +5cm阶段保持水平，-5cm阶段允许通过独立宏微调到达后的横梁姿态
    settle_offset = (s_target_index == 0U) ? 0 : APP_STATIC_BALL_NEG_SETTLE_OFFSET;
    APP_BallControl_SetServoControlOffset(settle_offset); // 后续HOLD闭环继续沿用该水平基准偏移
    APP_BallControl_SetServoTestPosition(
        APP_BallControl_PositionFromLevelOffset(settle_offset));
}

static void enter_hold(uint32_t now_ms)
{
    int16_t hold_offset;

    s_state = APP_STATIC_BALL_HOLD;
    s_state_start_ms = now_ms;
    s_hold_timing = false;
    // 提前PID路径可能跳过SETTLE，因此在HOLD入口再次设置对应目标的持续偏移
    hold_offset = (s_target_index == 0U) ? 0 : APP_STATIC_BALL_NEG_SETTLE_OFFSET;
    APP_BallControl_SetServoControlOffset(hold_offset);
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
    APP_BallControl_SetPid(APP_STATIC_BALL_PID_KP,
                           APP_STATIC_BALL_PID_KI,
                           APP_STATIC_BALL_PID_KD); // 每次启动第三问都恢复本任务已调好的专用PID
    APP_BallControl_SetMotionCompensationEnabled(false); // 静态任务仅屏蔽Kv/Ka，保留移动任务调参值
    APP_BallControl_SetMotionEstimator(APP_STATIC_BALL_SPEED_WINDOW_FRAME,
                                       APP_STATIC_BALL_SPEED_FILTER_ALPHA); // 保持第三问已调好的速度估计动态
    APP_BallControl_SetServoControlOffset(0); // 每次新任务从现场校准水平零点开始
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
    uint32_t hold_time_ms;
    bool neg_target_reached;

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
    hold_time_ms = (s_target_index == 0U) ? APP_STATIC_HOLD_POS_TIME_MS :
                                           APP_STATIC_HOLD_NEG_TIME_MS; // 两个目标分别使用独立稳定时间
    neg_target_reached = (remain_cm <= APP_STATIC_BALL_NEG_PID_RELEASE_ERROR_CM) ||
                         (ball.measured_position_cm <= target_cm); // 接近或越过-5cm都必须交给PID

    switch (s_state) {
        case APP_STATIC_BALL_ACCEL:
            if ((((uint32_t)(now_ms - s_state_start_ms) >= APP_STATIC_BALL_ACCEL_MIN_MS) &&
                 (remain_cm <= brake_distance_cm)) ||
                ((uint32_t)(now_ms - s_state_start_ms) >= get_accel_max_ms())) {
                enter_brake(now_ms);
            }
            break;

        case APP_STATIC_BALL_BRAKE:
            /*
             * +5到-5cm阶段只有接近目标后才允许PID接管，避免速度符号变化在中心附近误触发。
             * 如果没有进入目标窗口，仍按固定制动时间结束并进入SETTLE/HOLD。
             */
            if ((s_target_index != 0U) &&
                ((uint32_t)(now_ms - s_state_start_ms) >= APP_STATIC_BALL_NEG_PID_RELEASE_MIN_MS) &&
                neg_target_reached) {
                enter_hold(now_ms);
            } else if ((uint32_t)(now_ms - s_state_start_ms) >= get_brake_time_ms()) {
                enter_settle(now_ms);
            }
            break;

        case APP_STATIC_BALL_SETTLE:
            // -5cm阶段等待结束后仍需进入目标窗口，避免在中心附近过早切入PID
            if (((uint32_t)(now_ms - s_state_start_ms) >= get_settle_time_ms()) &&
                ((s_target_index == 0U) ||
                 neg_target_reached)) {
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

                if ((uint32_t)(now_ms - s_hold_start_ms) >= hold_time_ms) {
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
    APP_BallControl_SetServoControlOffset(0); // 手动停止时恢复现场校准水平零点
    APP_BallControl_SetMode(APP_BALL_MODE_DISABLE);
    APP_BallControl_SetMotionCompensationEnabled(true); // 已关闭控球，退出静态任务后恢复补偿可用状态
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
