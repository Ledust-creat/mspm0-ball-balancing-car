#include "app_ball_control.h"
#include "dve_hts25l.h"
#include "interrupt.h"

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static APP_BallControl_t s_ball;
static APP_BallVisionCalibration_t s_vision_calibration;
static uint32_t s_last_task_ms;
static uint32_t s_enable_tick_ms;
static uint32_t s_measurement_tick_ms;
static uint32_t s_previous_measurement_tick_ms;
static float s_previous_measurement_cm;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float limit_float(float value, float min, float max);
static void reset_control(void);
static void command_center(void);
static float get_feedforward_deg(void);
static uint16_t angle_to_servo_position(float angle_deg, uint8_t *saturated);

/*
*********************************************************************************************************
*   函 数 名: limit_float
*   功能说明: 浮点数限幅
*   形    参: value - 输入值；min - 下限；max - 上限
*   返 回 值: 限幅后的值
*********************************************************************************************************
*/
static float limit_float(float value, float min, float max)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

/*
*********************************************************************************************************
*   函 数 名: reset_control
*   功能说明: 清除钢球PID与动态状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void reset_control(void)
{
    PID_Reset(&s_ball.ball_pid);
    s_ball.error_cm = 0.0f;
    s_ball.ball_speed_cm_s = 0.0f;
    s_ball.angle_cmd_deg = 0.0f;
    s_ball.saturated = 0U;
}

/*
*********************************************************************************************************
*   函 数 名: command_center
*   功能说明: 命令舵机回到安全中位
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void command_center(void)
{
    (void)DVE_HTS25L_SetPosition(DVE_HTS25L_POS_CENTER, DVE_HTS25L_MOVE_TIME_MS);
    s_ball.servo_position = DVE_HTS25L_POS_CENTER;
    s_ball.angle_cmd_deg = 0.0f;
}

/*
*********************************************************************************************************
*   函 数 名: get_feedforward_deg
*   功能说明: 获取车体姿态前馈，第一版固定为0
*   形    参: 无
*   返 回 值: 前馈角度
*********************************************************************************************************
*/
static float get_feedforward_deg(void)
{
    return 0.0f;
}

/*
*********************************************************************************************************
*   函 数 名: angle_to_servo_position
*   功能说明: 将横梁角度映射为舵机安全位置
*   形    参: angle_deg - 横梁角度；saturated - 限幅标志输出
*   返 回 值: 舵机位置
*********************************************************************************************************
*/
static uint16_t angle_to_servo_position(float angle_deg, uint8_t *saturated)
{
    float position;
    int32_t rounded_position;

    position = (float)DVE_HTS25L_POS_CENTER +
               APP_BALL_SERVO_DIR * angle_deg * APP_BALL_SERVO_POS_PER_DEG;
    rounded_position = (int32_t)((position >= 0.0f) ? (position + 0.5f) : (position - 0.5f));

    if (rounded_position < (int32_t)DVE_HTS25L_POS_MIN) {
        rounded_position = (int32_t)DVE_HTS25L_POS_MIN;
        *saturated = 1U;
    } else if (rounded_position > (int32_t)DVE_HTS25L_POS_MAX) {
        rounded_position = (int32_t)DVE_HTS25L_POS_MAX;
        *saturated = 1U;
    }

    return (uint16_t)rounded_position;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_Init
*   功能说明: 初始化钢球控制层和HTS-25L舵机
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_Init(void)
{
    uint32_t now = (uint32_t)Get_Tick();

    PID_Init(&s_ball.ball_pid,
             APP_BALL_DEFAULT_KP,
             APP_BALL_DEFAULT_KI,
             APP_BALL_DEFAULT_KD,
             APP_BALL_INTEGRAL_LIMIT_DEG,
             APP_BALL_ANGLE_LIMIT_DEG);

    s_ball.state = APP_BALL_STATE_STOP;
    s_ball.target_cm = 0.0f;
    s_ball.measured_cm = 0.0f;
    s_ball.servo_position = DVE_HTS25L_POS_CENTER;
    s_ball.enabled = 0U;
    s_ball.measurement_valid = 0U;

    s_vision_calibration.center_x_px = 0.0f;
    s_vision_calibration.cm_per_px = 0.0f;
    s_vision_calibration.valid_y_min_px = 0.0f;
    s_vision_calibration.valid_y_max_px = 0.0f;
    s_vision_calibration.is_calibrated = 0U;

    s_last_task_ms = now;
    s_enable_tick_ms = now;
    s_measurement_tick_ms = now;
    s_previous_measurement_tick_ms = now;
    s_previous_measurement_cm = 0.0f;

    reset_control();
    DVE_HTS25L_Init();
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_Task
*   功能说明: 按33ms周期执行钢球位置闭环和超时保护
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_Task(void)
{
    uint32_t now = (uint32_t)Get_Tick();
    uint32_t elapsed_ms;
    float dt_s;
    float angle_raw;
    float angle_limited;

    elapsed_ms = now - s_last_task_ms; // 主循环非阻塞运行，只在到达33ms周期时执行一次闭环
    if (elapsed_ms < APP_BALL_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_task_ms = now;

    // 停止、开环测试和故障状态均禁止执行位置闭环。
    if ((s_ball.state == APP_BALL_STATE_STOP) ||
        (s_ball.state == APP_BALL_STATE_OPEN_LOOP_TEST) ||
        (s_ball.state == APP_BALL_STATE_FAULT)) {
        return;
    }

    // 使能后长时间没有首帧，或运行中位置数据超时，都进入故障并回中位。
    if ((!s_ball.measurement_valid && ((now - s_enable_tick_ms) > APP_BALL_VISION_TIMEOUT_MS)) ||
        (s_ball.measurement_valid && ((now - s_measurement_tick_ms) > APP_BALL_VISION_TIMEOUT_MS))) {
        s_ball.measurement_valid = 0U;
        s_ball.state = APP_BALL_STATE_FAULT;
        reset_control();
        command_center();
        return;
    }

    if (!s_ball.measurement_valid) {
        return;
    }

    dt_s = (float)elapsed_ms * 0.001f; // 使用实际任务间隔，改变任务周期后积分量仍保持一致
    s_ball.error_cm = s_ball.target_cm - s_ball.measured_cm; // 正误差表示钢球位于目标负方向

    // 钢球D项直接使用物理速度反馈，通用PID_Calc的误差差分不适合这里，因此在本层显式计算。
    s_ball.ball_pid.error[0] = s_ball.error_cm; // 保存当前误差，便于状态查看
    s_ball.ball_pid.Pout = s_ball.ball_pid.Kp * s_ball.error_cm; // P项决定位置偏差对应的倾斜角
    s_ball.ball_pid.Iout += s_ball.ball_pid.Ki * s_ball.error_cm * dt_s; // I项消除静态摩擦造成的残余误差
    s_ball.ball_pid.Iout = limit_float(s_ball.ball_pid.Iout,
                                       -s_ball.ball_pid.max_iout,
                                       s_ball.ball_pid.max_iout); // 限制积分，防止长时间偏差导致积分饱和
    s_ball.ball_pid.Dout = -s_ball.ball_pid.Kd * s_ball.ball_speed_cm_s; // 速度方向与目标运动一致时提供阻尼

    angle_raw = s_ball.ball_pid.Pout +
                s_ball.ball_pid.Iout +
                s_ball.ball_pid.Dout +
                get_feedforward_deg();
    angle_limited = limit_float(angle_raw,
                                -APP_BALL_ANGLE_LIMIT_DEG,
                                APP_BALL_ANGLE_LIMIT_DEG); // 横梁命令强制限制在机械安全角度内

    s_ball.saturated = (angle_raw != angle_limited) ? 1U : 0U;
    s_ball.angle_cmd_deg = angle_limited;
    s_ball.ball_pid.out = angle_limited;
    s_ball.servo_position = angle_to_servo_position(angle_limited, &s_ball.saturated); // 角度映射为430~570舵机位置

    (void)DVE_HTS25L_SetPosition(s_ball.servo_position, DVE_HTS25L_MOVE_TIME_MS);
    s_ball.state = (s_ball.target_cm == 0.0f) ?
                   APP_BALL_STATE_CENTER_HOLD :
                   APP_BALL_STATE_TARGET_HOLD;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetEnable
*   功能说明: 使能或失能钢球闭环，失能时回中位
*   形    参: enable - 1使能，0失能
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetEnable(uint8_t enable)
{
    reset_control();

    if (enable == 0U) {
        s_ball.enabled = 0U;
        s_ball.state = APP_BALL_STATE_STOP;
        command_center();
        return;
    }

    s_ball.enabled = 1U;
    s_ball.state = APP_BALL_STATE_CENTER_HOLD;
    s_enable_tick_ms = (uint32_t)Get_Tick();
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetTargetCm
*   功能说明: 设置钢球目标位置
*   形    参: target_cm - 目标位置，单位cm
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetTargetCm(float target_cm)
{
    s_ball.target_cm = limit_float(target_cm,
                                   -APP_BALL_TARGET_LIMIT_CM,
                                   APP_BALL_TARGET_LIMIT_CM);

    if (s_ball.enabled && (s_ball.state != APP_BALL_STATE_FAULT)) {
        s_ball.state = (s_ball.target_cm == 0.0f) ?
                       APP_BALL_STATE_CENTER_HOLD :
                       APP_BALL_STATE_TARGET_HOLD;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetMeasuredPositionCm
*   功能说明: 写入钢球实测位置并估算低通速度
*   形    参: x_cm - 实测位置；tick_ms - 数据到达时间
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetMeasuredPositionCm(float x_cm, uint32_t tick_ms)
{
    uint32_t sample_ms;
    float raw_speed;

    x_cm = limit_float(x_cm,
                       -APP_BALL_MEASUREMENT_LIMIT_CM,
                       APP_BALL_MEASUREMENT_LIMIT_CM);

    if (s_ball.measurement_valid) {
        sample_ms = tick_ms - s_previous_measurement_tick_ms;
        if (sample_ms > 0U) {
            raw_speed = (x_cm - s_previous_measurement_cm) * 1000.0f / (float)sample_ms; // 位置差分得到cm/s
            s_ball.ball_speed_cm_s += APP_BALL_SPEED_LPF_ALPHA *
                                      (raw_speed - s_ball.ball_speed_cm_s); // 一阶低通抑制视觉位置跳动
        }
    } else {
        s_ball.ball_speed_cm_s = 0.0f;
    }

    s_previous_measurement_cm = x_cm;
    s_previous_measurement_tick_ms = tick_ms;
    s_measurement_tick_ms = tick_ms;
    s_ball.measured_cm = x_cm;
    s_ball.measurement_valid = 1U;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetVisionPoint
*   功能说明: 将已标定的视觉像素坐标转换为钢球位置
*   形    参: x_px,y_px - 像素坐标；tick_ms - 数据到达时间
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetVisionPoint(float x_px, float y_px, uint32_t tick_ms)
{
    float x_cm;

    if (!s_vision_calibration.is_calibrated) {
        return;
    }

    if ((y_px < s_vision_calibration.valid_y_min_px) ||
        (y_px > s_vision_calibration.valid_y_max_px)) {
        return;
    }

    x_cm = (x_px - s_vision_calibration.center_x_px) *
           s_vision_calibration.cm_per_px;
    APP_BallControl_SetMeasuredPositionCm(x_cm, tick_ms);
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetVisionCalibration
*   功能说明: 设置视觉像素到钢球位置的投影参数
*   形    参: calibration - 标定参数
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetVisionCalibration(const APP_BallVisionCalibration_t *calibration)
{
    if (calibration == 0) {
        s_vision_calibration.is_calibrated = 0U; // 空指针用于关闭像素投影入口
        return;
    }

    s_vision_calibration = *calibration; // 保存像素中心、厘米比例和有效Y区域
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetPid
*   功能说明: 设置钢球位置PID参数并清除历史状态
*   形    参: kp,ki,kd - PID参数
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetPid(float kp, float ki, float kd)
{
    PID_SetParam(&s_ball.ball_pid,
                 kp,
                 ki,
                 kd,
                 APP_BALL_INTEGRAL_LIMIT_DEG,
                 APP_BALL_ANGLE_LIMIT_DEG);
    reset_control();
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetServoTestPosition
*   功能说明: 进入开环测试并发送安全舵机位置
*   形    参: pos - 目标舵机位置
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetServoTestPosition(uint16_t pos)
{
    s_ball.enabled = 0U;
    s_ball.state = APP_BALL_STATE_OPEN_LOOP_TEST;
    reset_control();
    (void)DVE_HTS25L_SetPosition(pos, DVE_HTS25L_MOVE_TIME_MS);
    s_ball.servo_position = DVE_HTS25L_GetCommandPosition();
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_GetStatus
*   功能说明: 获取钢球控制状态只读指针
*   形    参: 无
*   返 回 值: 钢球控制状态
*********************************************************************************************************
*/
const APP_BallControl_t *APP_BallControl_GetStatus(void)
{
    return &s_ball;
}
