#include "app_ball_control.h"
#include "app_chassis_control.h"
#include "app_gyro_control.h"
#include "dve_hts25l.h"
#include "interrupt.h"

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static PID_t s_position_pid;                              /* 钢球位置闭环PID对象 */
static APP_BallControlConfig_t s_config;                  /* 当前控球配置参数 */
static APP_BallControlStatus_t s_status;                  /* 当前对外状态快照 */
static APP_BallVisionCalibration_t s_vision_calibration;  /* 视觉像素投影标定 */
static uint16_t s_servo_level_position = APP_BALL_SERVO_LEVEL_DEFAULT; /* 当前水平零点，仅RAM保存 */
static int16_t s_servo_control_offset;                  /* 当前任务相对校准零点的持续控制偏移 */

static uint32_t s_last_control_tick_ms;                   /* 上一次新视觉帧控球时刻，单位ms */
static uint32_t s_last_safety_ms;                         /* 最近一次安全检查时刻，单位ms */
static uint32_t s_enable_tick_ms;                         /* 闭环使能时刻，单位ms */
static uint32_t s_previous_measurement_tick_ms;           /* 上一次测量时刻，单位ms */
static float s_position_window[APP_BALL_SPEED_WINDOW_FRAME]; /* 速度窗口位置缓存 */
static uint32_t s_tick_window[APP_BALL_SPEED_WINDOW_FRAME];  /* 速度窗口时间缓存 */
static uint8_t s_window_write_index;                       /* 下一帧窗口写入位置 */
static uint8_t s_window_count;                             /* 当前有效窗口帧数 */
static uint8_t s_speed_window_frame;                       /* 当前任务使用的速度窗口帧数 */
static float s_speed_filter_alpha;                         /* 当前任务使用的速度低通系数 */
static float s_motion_velocity_gain;                       /* 移动控球附加速度反馈Kv */
static float s_motion_acceleration_gain;                   /* 车辆加速度前馈Ka */
static float s_imu_accel_gain;                             /* IMU实测加速度参与比例 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float limit_float(float value, float min, float max);
static void reset_control(void);
static void command_center(void);
static float get_feedforward_deg(uint32_t frame_tick_ms);
static uint16_t angle_to_servo_position(float angle_deg, bool *saturated);

/*
*********************************************************************************************************
*   函 数 名: limit_float
*   功能说明: 浮点限幅
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
*   功能说明: 清除钢球PID和动态控制状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void reset_control(void)
{
    uint8_t i;

    PID_Reset(&s_position_pid);
    s_status.error_cm = 0.0f;
    s_status.estimated_velocity_cm_s = 0.0f;
    s_status.estimated_acceleration_cm_s2 = 0.0f;
    s_status.angle_cmd_deg = 0.0f;
    s_status.velocity_feedback_deg = 0.0f;
    s_status.vehicle_accel_feedforward_deg = 0.0f;
    s_status.accel_feedforward_deg = 0.0f;
    s_status.imu_accel_valid = false;
    s_status.saturated = false;
    s_window_write_index = 0U;
    s_window_count = 0U;
    for (i = 0U; i < APP_BALL_SPEED_WINDOW_FRAME; i++) {
        s_position_window[i] = 0.0f;
        s_tick_window[i] = 0U;
    }
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
    s_status.servo_command = APP_BallControl_PositionFromLevelOffset(s_servo_control_offset);
    (void)DVE_HTS25L_SetPosition(s_status.servo_command, DVE_HTS25L_MOVE_TIME_MS);
    s_status.angle_cmd_deg = 0.0f;
}

/*
*********************************************************************************************************
*   函 数 名: get_feedforward_deg
*   功能说明: 根据最新IMU缓存计算加速度前馈角度，不触发SPI读取
*   形    参: frame_tick_ms - 当前视觉帧真实收帧时刻，单位ms
*   返 回 值: 前馈角度，单位deg
*********************************************************************************************************
*/
static float get_feedforward_deg(uint32_t frame_tick_ms)
{
#if (APP_BALL_ACCEL_FF_ENABLE != 0U)
    float a_forward_g;
    float a_lateral_g = 0.0f;
    float a_tube_g;
    float feedforward;
    APP_ChassisControlState_t chassis_control;

    a_forward_g = 0.0f;

#if (APP_BALL_IMU_ACCEL_FF_ENABLE != 0U)
    {
        APP_GyroAccelData_t imu_data;

        if (APP_GyroControl_GetLatestAccel(&imu_data) &&
            ((uint32_t)(frame_tick_ms - imu_data.update_tick_ms) <= APP_IMU_DATA_TIMEOUT_MS)) {
            // DVE层已完成安装轴向和Y轴反向修正，此处只保留有效动态加速度
            a_forward_g = s_imu_accel_gain *
                          imu_data.accel_forward_g;
            a_lateral_g = s_imu_accel_gain *
                          imu_data.accel_lateral_g;
            if ((a_forward_g < (APP_BALL_IMU_ACCEL_DEADBAND_G * s_imu_accel_gain)) &&
                (a_forward_g > -(APP_BALL_IMU_ACCEL_DEADBAND_G * s_imu_accel_gain))) {
                a_forward_g = 0.0f; // 忽略静止零偏和轻微机械振动
            }
            if ((a_lateral_g < (APP_BALL_IMU_ACCEL_DEADBAND_G * s_imu_accel_gain)) &&
                (a_lateral_g > -(APP_BALL_IMU_ACCEL_DEADBAND_G * s_imu_accel_gain))) {
                a_lateral_g = 0.0f;
            }
            s_status.imu_accel_valid = true;
        } else {
            s_status.imu_accel_valid = false;
        }
    }
#else
    s_status.imu_accel_valid = false;
#endif

    APP_ChassisControl_GetState(&chassis_control);
    if (!chassis_control.is_running ||
        (chassis_control.planned_base_speed_cm_s <=
         APP_BALL_IMU_FF_MIN_CHASSIS_SPEED_CM_S)) {
        // 底盘停稳和极低速阶段不采用IMU噪声，但不修改串口设置的参与比例
        a_forward_g = 0.0f;
        a_lateral_g = 0.0f;
    }

    // 将车体前向和横向加速度投影到钢球导轨轴向
    a_tube_g = APP_BALL_TUBE_FORWARD_GAIN * a_forward_g +
               APP_BALL_TUBE_LATERAL_GAIN * a_lateral_g;
    // 横梁需要向惯性运动的反方向倾斜，因此在APP层统一反向映射
    feedforward = APP_BALL_IMU_ACCEL_FF_DIR *
                  APP_BALL_ACCEL_FF_DEG_PER_G * a_tube_g;
    feedforward = limit_float(feedforward,
                              -APP_BALL_ACCEL_FF_LIMIT_DEG,
                              APP_BALL_ACCEL_FF_LIMIT_DEG);
    s_status.accel_feedforward_deg = feedforward;
    return feedforward;
#else
    (void)frame_tick_ms;
    s_status.imu_accel_valid = false;
    s_status.accel_feedforward_deg = 0.0f;
    return 0.0f;
#endif
}

/*
*********************************************************************************************************
*   函 数 名: angle_to_servo_position
*   功能说明: 将横梁角度映射为舵机安全位置
*   形    参: angle_deg - 横梁角度，单位deg；saturated - 限幅标志输出
*   返 回 值: 舵机位置命令，范围0~1000
*********************************************************************************************************
*/
static uint16_t angle_to_servo_position(float angle_deg, bool *saturated)
{
    float position;
    int32_t rounded_position;

    position = (float)APP_BallControl_PositionFromLevelOffset(s_servo_control_offset) +
               APP_BALL_SERVO_DIR * angle_deg * APP_BALL_SERVO_POS_PER_DEG;
    rounded_position = (int32_t)((position >= 0.0f) ? (position + 0.5f) : (position - 0.5f));

    if (rounded_position < (int32_t)s_config.servo_position_min) {
        rounded_position = (int32_t)s_config.servo_position_min;
        *saturated = true;
    } else if (rounded_position > (int32_t)s_config.servo_position_max) {
        rounded_position = (int32_t)s_config.servo_position_max;
        *saturated = true;
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
    uint32_t now = Get_Tick();

    s_config.position_kp = APP_BALL_DEFAULT_KP;
    s_config.position_ki = APP_BALL_DEFAULT_KI;
    s_config.position_kd = APP_BALL_DEFAULT_KD;
    s_config.target_position_cm = 0.0f;
    s_config.accel_forward_ff_gain = 0.0f;
    s_servo_level_position = APP_BALL_SERVO_LEVEL_DEFAULT;
    s_servo_control_offset = 0;
    s_motion_velocity_gain = 0.0f;
    s_motion_acceleration_gain = 0.0f;
    s_imu_accel_gain = APP_BALL_ACCEL_RESIDUAL_GAIN;
    s_status.imu_accel_gain = s_imu_accel_gain;
    s_speed_window_frame = 2U;                             // 默认保持第三问原单帧差分行为
    s_speed_filter_alpha = 0.30f;
    s_config.servo_position_min = DVE_HTS25L_POS_MIN;
    s_config.servo_position_max = DVE_HTS25L_POS_MAX;

    PID_Init(&s_position_pid,
             s_config.position_kp,
             s_config.position_ki,
             s_config.position_kd,
             APP_BALL_INTEGRAL_LIMIT_DEG,
             APP_BALL_ANGLE_LIMIT_DEG);

    s_status.state = APP_BALL_STATE_STOP;
    s_status.fault = APP_BALL_FAULT_NONE;
    s_status.target_position_cm = s_config.target_position_cm;
    s_status.measured_position_cm = 0.0f;
    s_status.servo_command = s_servo_level_position;
    s_status.last_vision_tick_ms = now;
    s_status.vision_valid = false;
    s_status.enabled = false;
    s_status.motion_compensation_enabled = true; // 默认沿用原补偿行为，静态任务可临时屏蔽

    s_vision_calibration.center_x_px = 0.0f;
    s_vision_calibration.cm_per_px = 0.0f;
    s_vision_calibration.valid_y_min_px = 0.0f;
    s_vision_calibration.valid_y_max_px = 0.0f;
    s_vision_calibration.is_calibrated = 0U;

    s_last_control_tick_ms = now;
    s_last_safety_ms = now;
    s_enable_tick_ms = now;
    s_previous_measurement_tick_ms = now;

    reset_control();
    DVE_HTS25L_Init();
    command_center(); // APP层统一发送运行时水平零点，避免DVE固定中位覆盖校准基准
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_Task
*   功能说明: 钢球控制兼容任务，当前只执行安全检查
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_Task(void)
{
    APP_BallControl_SafetyTask();
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_UpdateFromVision
*   功能说明: K230新位置帧到达后执行一次钢球闭环并发送舵机位置
*   形    参: frame - K230钢球位置帧
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_UpdateFromVision(const APP_K230BallFrame_t *frame)
{
    uint32_t elapsed_ms;
    float dt_s;
    float angle_raw;
    float angle_limited;

    if (frame == 0) {
        return;
    }
    elapsed_ms = frame->tick_ms - s_last_control_tick_ms;
    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    }
    s_last_control_tick_ms = frame->tick_ms;

    APP_BallControl_SetMeasuredPositionCm(frame->x_cm, frame->tick_ms);
    
    if ((s_status.state == APP_BALL_STATE_STOP) ||
        (s_status.state == APP_BALL_STATE_OPEN_LOOP_TEST) ||
        (s_status.state == APP_BALL_STATE_FAULT)) {
        return;
    }

    

    dt_s = (float)elapsed_ms * 0.001f;
    s_status.error_cm = s_status.target_position_cm - s_status.measured_position_cm;
    
    s_position_pid.error[2] = s_position_pid.error[1];
    s_position_pid.error[1] = s_position_pid.error[0];
    s_position_pid.error[0] = s_status.error_cm;
    s_position_pid.Pout = s_position_pid.Kp * s_status.error_cm;
    s_position_pid.Iout += s_position_pid.Ki * s_status.error_cm * dt_s;
    s_position_pid.Iout = limit_float(s_position_pid.Iout,
                                      -s_position_pid.max_iout,
                                      s_position_pid.max_iout);
    s_position_pid.Dout = -s_position_pid.Kd * s_status.estimated_velocity_cm_s;

    angle_raw = s_position_pid.Pout +
                s_position_pid.Iout +
                s_position_pid.Dout +
                get_feedforward_deg(frame->tick_ms);

    s_status.velocity_feedback_deg = 0.0f;
    s_status.vehicle_accel_feedforward_deg = 0.0f;
    if (s_status.motion_compensation_enabled) { // 静态任务只屏蔽补偿，不清空已调好的Kv/Ka
        s_status.velocity_feedback_deg =
            -s_motion_velocity_gain * s_status.estimated_velocity_cm_s; // Kv抵消钢球惯性
        // 实车安装方向与计划加速度坐标相反，正Ka反向映射到横梁倾角
        s_status.vehicle_accel_feedforward_deg =
            -s_motion_acceleration_gain * APP_ChassisControl_GetPlannedAccelCmps2();
    }
    angle_raw += s_status.velocity_feedback_deg +
                 s_status.vehicle_accel_feedforward_deg;

    // 小球存在明显位置误差但基本静止时，补足最低有效倾角以克服静摩擦和机构死区
    if (((s_status.error_cm > APP_BALL_STATIC_ERROR_CM) ||
         (s_status.error_cm < -APP_BALL_STATIC_ERROR_CM)) &&
        (s_status.estimated_velocity_cm_s < APP_BALL_STATIC_SPEED_CM_S) &&
        (s_status.estimated_velocity_cm_s > -APP_BALL_STATIC_SPEED_CM_S) &&
        (angle_raw < APP_BALL_STATIC_MIN_ANGLE_DEG) &&
        (angle_raw > -APP_BALL_STATIC_MIN_ANGLE_DEG)) {
        angle_raw = (s_status.error_cm > 0.0f) ? APP_BALL_STATIC_MIN_ANGLE_DEG :
                                                -APP_BALL_STATIC_MIN_ANGLE_DEG;
    }

    angle_limited = limit_float(angle_raw,
                                -APP_BALL_ANGLE_LIMIT_DEG,
                                APP_BALL_ANGLE_LIMIT_DEG);

    s_status.saturated = (angle_raw != angle_limited);
    s_status.angle_cmd_deg = angle_limited;
    s_position_pid.out = angle_limited;
    s_status.servo_command = angle_to_servo_position(angle_limited, &s_status.saturated);

    (void)DVE_HTS25L_SetPosition(s_status.servo_command, DVE_HTS25L_MOVE_TIME_MS);
    s_status.state = (s_status.target_position_cm == 0.0f) ?
                     APP_BALL_STATE_CENTER_HOLD :
                     APP_BALL_STATE_TARGET_HOLD;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SafetyTask
*   功能说明: 低频检查视觉超时，超时后进入故障并命令舵机回中
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SafetyTask(void)
{
    uint32_t now = Get_Tick();

    if ((uint32_t)(now - s_last_safety_ms) < 5U) {
        return;
    }
    s_last_safety_ms = now;

    if ((s_status.state == APP_BALL_STATE_STOP) ||
        (s_status.state == APP_BALL_STATE_OPEN_LOOP_TEST) ||
        (s_status.state == APP_BALL_STATE_FAULT)) {
        return;
    }

    if ((!s_status.vision_valid && ((uint32_t)(now - s_enable_tick_ms) > APP_BALL_VISION_TIMEOUT_MS)) ||
        (s_status.vision_valid && ((uint32_t)(now - s_status.last_vision_tick_ms) > APP_BALL_VISION_TIMEOUT_MS))) {
        s_status.vision_valid = false;
        s_status.state = APP_BALL_STATE_FAULT;
        s_status.fault = APP_BALL_FAULT_VISION_TIMEOUT;
        reset_control();
        command_center();
    }
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
        s_status.enabled = false;
        s_status.state = APP_BALL_STATE_STOP;
        command_center();
        return;
    }

    s_status.enabled = true;
    s_status.state = APP_BALL_STATE_CENTER_HOLD;
    s_status.fault = APP_BALL_FAULT_NONE;
    s_status.vision_valid = false;
    s_enable_tick_ms = Get_Tick();
    s_last_control_tick_ms = s_enable_tick_ms;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetMode
*   功能说明: 按任务层要求切换钢球控制模式
*   形    参: mode - 控球模式
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetMode(APP_BallControlMode_t mode)
{
    switch (mode) {
        case APP_BALL_MODE_DISABLE:
            APP_BallControl_SetEnable(0U);
            break;

        case APP_BALL_MODE_MANUAL_TILT:
            s_status.enabled = true;
            s_status.state = APP_BALL_STATE_OPEN_LOOP_TEST;
            s_status.fault = APP_BALL_FAULT_NONE;
            reset_control();
            break;

        case APP_BALL_MODE_POSITION_HOLD:
            APP_BallControl_StartHold(s_status.target_position_cm);
            break;

        case APP_BALL_MODE_FAULT:
        default:
            s_status.enabled = false;
            s_status.state = APP_BALL_STATE_FAULT;
            reset_control();
            command_center();
            break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetManualTiltDeg
*   功能说明: 固定横梁角度开环输出，用于第三问加速和反向制动
*   形    参: angle_deg - 横梁角度，单位deg；move_time_ms - 舵机移动时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetManualTiltDeg(float angle_deg, uint16_t move_time_ms)
{
    bool saturated = false;

    s_status.enabled = true;
    s_status.state = APP_BALL_STATE_OPEN_LOOP_TEST;
    s_status.fault = APP_BALL_FAULT_NONE;
    s_status.angle_cmd_deg = limit_float(angle_deg,
                                         -APP_BALL_ANGLE_LIMIT_DEG,
                                         APP_BALL_ANGLE_LIMIT_DEG);
    s_status.servo_command = angle_to_servo_position(s_status.angle_cmd_deg, &saturated);
    s_status.saturated = saturated;
    (void)DVE_HTS25L_SetPosition(s_status.servo_command, move_time_ms);
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_StartHold
*   功能说明: 启动钢球位置保持闭环
*   形    参: target_cm - 目标位置，单位cm
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_StartHold(float target_cm)
{
    reset_control();
    s_status.enabled = true;
    s_status.fault = APP_BALL_FAULT_NONE;
    s_enable_tick_ms = Get_Tick();
    s_last_control_tick_ms = s_enable_tick_ms;
    APP_BallControl_SetTargetCm(target_cm);
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_Reset
*   功能说明: 清除钢球PID、速度估计和历史位置，不改变当前目标位置
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_Reset(void)
{
    reset_control();
    s_previous_measurement_tick_ms = Get_Tick();
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
    s_status.target_position_cm = limit_float(target_cm,
                                              -APP_BALL_TARGET_LIMIT_CM,
                                              APP_BALL_TARGET_LIMIT_CM);

    if (s_status.enabled && (s_status.state != APP_BALL_STATE_FAULT)) {
        s_status.state = (s_status.target_position_cm == 0.0f) ?
                         APP_BALL_STATE_CENTER_HOLD :
                         APP_BALL_STATE_TARGET_HOLD;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetMeasuredPositionCm
*   功能说明: 写入钢球实测位置并估算低通速度
*   形    参: x_cm - 实测位置，单位cm；tick_ms - 数据到达时刻，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetMeasuredPositionCm(float x_cm, uint32_t tick_ms)
{
    uint8_t newest_index;
    uint8_t oldest_index;
    uint32_t window_ms;
    uint32_t accel_ms;
    float raw_speed;
    float previous_velocity;

    x_cm = limit_float(x_cm,
                       -APP_BALL_MEASUREMENT_LIMIT_CM,
                       APP_BALL_MEASUREMENT_LIMIT_CM);

    previous_velocity = s_status.estimated_velocity_cm_s;
    accel_ms = tick_ms - s_previous_measurement_tick_ms;

    s_position_window[s_window_write_index] = x_cm;
    s_tick_window[s_window_write_index] = tick_ms;
    newest_index = s_window_write_index;
    s_window_write_index = (uint8_t)((s_window_write_index + 1U) % s_speed_window_frame);
    if (s_window_count < s_speed_window_frame) {
        s_window_count++;
    }

    if (s_window_count >= 2U) {
        oldest_index = (s_window_count < s_speed_window_frame) ? 0U :
                       s_window_write_index;
        window_ms = s_tick_window[newest_index] - s_tick_window[oldest_index];
        if (window_ms > 0U) {
            raw_speed = (x_cm - s_position_window[oldest_index]) * 1000.0f /
                        (float)window_ms;
            s_status.estimated_velocity_cm_s += s_speed_filter_alpha *
                                                (raw_speed - s_status.estimated_velocity_cm_s);
        }
    }

    if (accel_ms > 0U) {
        s_status.estimated_acceleration_cm_s2 =
            (s_status.estimated_velocity_cm_s - previous_velocity) * 1000.0f /
            (float)accel_ms;
    }

    s_previous_measurement_tick_ms = tick_ms;
    s_status.last_vision_tick_ms = tick_ms;
    s_status.measured_position_cm = x_cm;
    s_status.vision_valid = true;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetVisionPoint
*   功能说明: 将已标定的视觉像素坐标转换为钢球位置
*   形    参: x_px,y_px - 像素坐标，单位px；tick_ms - 数据到达时刻，单位ms
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
*   形    参: calibration - 标定参数，NULL表示关闭像素投影入口
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetVisionCalibration(const APP_BallVisionCalibration_t *calibration)
{
    if (calibration == 0) {
        s_vision_calibration.is_calibrated = 0U;
        return;
    }

    s_vision_calibration = *calibration;
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
    PID_SetParam(&s_position_pid,
                 kp,
                 ki,
                 kd,
                 APP_BALL_INTEGRAL_LIMIT_DEG,
                 APP_BALL_ANGLE_LIMIT_DEG);
    s_config.position_kp = kp;
    s_config.position_ki = ki;
    s_config.position_kd = kd;
    reset_control();
}

void APP_BallControl_GetPid(PID_t *pid)
{
    if (pid == 0) {
        return;
    }

    *pid = s_position_pid;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetMotionCompensation
*   功能说明: 保存移动控球Kv/Ka，补偿关闭时仍保留新值供下次启用
*   形    参: velocity_gain - Kv；acceleration_gain - Ka
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetMotionCompensation(float velocity_gain,
                                           float acceleration_gain)
{
    s_motion_velocity_gain = velocity_gain;
    s_motion_acceleration_gain = acceleration_gain;
    s_status.motion_velocity_gain = velocity_gain;         // 同步Kv状态，供串口独立调参和回传
    s_status.motion_acceleration_gain = acceleration_gain; // 同步Ka状态，供串口独立调参和回传
    s_status.velocity_feedback_deg = 0.0f;
    s_status.vehicle_accel_feedforward_deg = 0.0f;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetMotionCompensationEnabled
*   功能说明: 切换Kv/Ka补偿输出，不改变已保存的增益和位置PID
*   形    参: enable - true启用，false屏蔽附加速度反馈及计划加速度前馈
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetMotionCompensationEnabled(bool enable)
{
    s_status.motion_compensation_enabled = enable;
    s_status.velocity_feedback_deg = 0.0f;         // 模式切换后清除上一帧补偿角，新视觉帧重新计算
    s_status.vehicle_accel_feedforward_deg = 0.0f;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetImuAccelGain
*   功能说明: 设置IMU实测动态加速度参与比例
*   形    参: gain - 参与比例，范围0~1
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetImuAccelGain(float gain)
{
    s_imu_accel_gain = limit_float(gain, 0.0f, 1.0f);
    s_status.imu_accel_gain = s_imu_accel_gain;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetMotionEstimator
*   功能说明: 按任务设置球速窗口帧数和低通系数，并清空旧估算历史
*   形    参: window_frame - 2~APP_BALL_SPEED_WINDOW_FRAME；filter_alpha - 0~1
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetMotionEstimator(uint8_t window_frame, float filter_alpha)
{
    if (window_frame < 2U) window_frame = 2U;
    if (window_frame > APP_BALL_SPEED_WINDOW_FRAME) {
        window_frame = APP_BALL_SPEED_WINDOW_FRAME;
    }
    if (filter_alpha < 0.0f) filter_alpha = 0.0f;
    if (filter_alpha > 1.0f) filter_alpha = 1.0f;

    s_speed_window_frame = window_frame;
    s_speed_filter_alpha = filter_alpha;
    reset_control(); // 防止切换任务后混用上一任务的位置和速度历史
    s_previous_measurement_tick_ms = Get_Tick();
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetServoTestPosition
*   功能说明: 进入开环测试并发送安全舵机位置
*   形    参: pos - 目标舵机位置，范围0~1000
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetServoTestPosition(uint16_t pos)
{
    s_status.enabled = false;
    s_status.state = APP_BALL_STATE_OPEN_LOOP_TEST;
    s_status.fault = APP_BALL_FAULT_NONE;
    reset_control();
    (void)DVE_HTS25L_SetPosition(pos, DVE_HTS25L_MOVE_TIME_MS);
    s_status.servo_command = DVE_HTS25L_GetCommandPosition();
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetServoLevelPosition
*   功能说明: 设置RAM中的舵机水平零点
*   形    参: position - 水平零点位置
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetServoLevelPosition(uint16_t position)
{
    if (position < APP_BALL_SERVO_LEVEL_MIN) {
        position = APP_BALL_SERVO_LEVEL_MIN;
    } else if (position > APP_BALL_SERVO_LEVEL_MAX) {
        position = APP_BALL_SERVO_LEVEL_MAX;
    }

    s_servo_level_position = position;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_GetServoLevelPosition
*   功能说明: 获取当前RAM舵机水平零点
*   形    参: 无
*   返 回 值: 当前水平零点
*********************************************************************************************************
*/
uint16_t APP_BallControl_GetServoLevelPosition(void)
{
    return s_servo_level_position;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_PositionFromLevelOffset
*   功能说明: 将水平零点相对偏移换算为安全绝对位置
*   形    参: offset - 相对水平零点的有符号偏移
*   返 回 值: 限幅后的舵机绝对位置
*********************************************************************************************************
*/
uint16_t APP_BallControl_PositionFromLevelOffset(int16_t offset)
{
    int32_t position = (int32_t)s_servo_level_position + (int32_t)offset;

    if (position < (int32_t)DVE_HTS25L_POS_MIN) {
        position = (int32_t)DVE_HTS25L_POS_MIN;
    } else if (position > (int32_t)DVE_HTS25L_POS_MAX) {
        position = (int32_t)DVE_HTS25L_POS_MAX;
    }

    return (uint16_t)position;
}

/*
*********************************************************************************************************
*   函 数 名: APP_BallControl_SetServoControlOffset
*   功能说明: 设置当前任务相对现场校准零点的持续控制偏移
*   形    参: offset - 有符号舵机位置偏移
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_BallControl_SetServoControlOffset(int16_t offset)
{
    // 这里只改变任务控制基准，不修改RAM中保存的现场水平零点
    s_servo_control_offset = offset;
}

bool APP_BallControl_IsFault(void)
{
    return (s_status.state == APP_BALL_STATE_FAULT);
}

APP_BallControlFault_t APP_BallControl_GetFault(void)
{
    return s_status.fault;
}

void APP_BallControl_GetStatus(APP_BallControlStatus_t *status)
{
    if (status == 0) {
        return;
    }

    *status = s_status;
}
