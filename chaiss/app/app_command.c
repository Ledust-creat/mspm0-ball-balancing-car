#include "app_command.h"
#include "app_ball_control.h"
#include "app_chassis_control.h"
#include "app_gyro_control.h"
#include "app_line_follow.h"
#include "bsp_bluetooth.h"
#include "dve_chassis.h"
#include "dve_hts25l.h"
#include "interrupt.h"
#include <stdbool.h>
#include <string.h>

#if (BLUETOOTH_RX_BUF_SIZE != APP_CMD_PACKET_LEN)
#error "Bluetooth RX buffer size must match command packet length"
#endif

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static uint8_t s_rx_buf[APP_CMD_PACKET_LEN];
static uint8_t s_rx_index;
static bool s_receiving;
static char s_text_buf[512];
static uint32_t s_last_wave_ms;
static uint32_t s_last_text_ms;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static uint8_t calc_checksum(const uint8_t *buf);
static void send_ack(uint8_t cmd);
static void send_error(float code);
static bool value_in_range(float value, float min, float max);
static void feed_byte(uint8_t data);
static void get_tune_pid(PID_t *pid);
static void set_tune_pid(float kp, float ki, float kd);
static float get_tune_error(void);
static float get_tune_output(void);
static void set_one_pid_param(uint8_t cmd, float value);
static void send_wave_status(void);
static void send_text_status(void);
static char *append_str(char *dst, const char *src);
static char *append_i32(char *dst, int32_t value);
static char *append_f2(char *dst, float value);

/*
*********************************************************************************************************
*   函 数 名: calc_checksum
*   功能说明: 计算A5/5A命令帧校验和
*   形    参: buf - 命令帧
*   返 回 值: 校验和
*********************************************************************************************************
*/
static uint8_t calc_checksum(const uint8_t *buf)
{
    return (uint8_t)(buf[1] + buf[2] + buf[3] + buf[4] + buf[5]);
}

/*
*********************************************************************************************************
*   函 数 名: Command_PackAndSend
*   功能说明: 打包并发送A5/5A浮点数据帧
*   形    参: cmd - 命令字；data - 浮点数据
*   返 回 值: 无
*********************************************************************************************************
*/
void Command_PackAndSend(uint8_t cmd, float data)
{
    uint8_t frame[APP_CMD_PACKET_LEN];

    frame[0] = APP_CMD_HEAD;
    frame[1] = cmd;
    memcpy(&frame[2], &data, 4U);
    frame[6] = calc_checksum(frame);
    frame[7] = APP_CMD_TAIL;
    Bluetooth_SendData(frame, APP_CMD_PACKET_LEN);
}

static void send_ack(uint8_t cmd)
{
#if (APP_CMD_ENABLE_ACK_FRAME != 0U)
    Command_PackAndSend(CMD_ACK, (float)cmd);
#else
    (void)cmd;
#endif
}

static void send_error(float code)
{
#if (APP_CMD_ENABLE_ERR_FRAME != 0U)
    Command_PackAndSend(CMD_ERR, code);
#else
    (void)code;
#endif
}

/*
*********************************************************************************************************
*   函 数 名: value_in_range
*   功能说明: 检查浮点参数是否有效并位于指定范围
*   形    参: value - 输入值；min - 下限；max - 上限
*   返 回 值: true参数有效，false参数无效
*********************************************************************************************************
*/
static bool value_in_range(float value, float min, float max)
{
    return ((value == value) && (value >= min) && (value <= max)); // value==value用于排除NaN
}

static void get_tune_pid(PID_t *pid)
{
    if (pid == 0) {
        return;
    }

#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LINE)
    {
        APP_LineFollowState_t state;
        APP_LineFollow_GetState(&state);
        *pid = state.line_pid;
    }
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_GYRO)
    {
        APP_GyroControlState_t state;
        APP_GyroControl_GetState(&state);
        *pid = state.yaw_pid;
    }
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LEFT_SPEED)
    DVE_Chassis_GetSpeedPid(pid, 0);
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_RIGHT_SPEED)
    DVE_Chassis_GetSpeedPid(0, pid);
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_BALL)
    APP_BallControl_GetPid(pid);
#else
#error "Unsupported APP_CMD_TUNE_TARGET"
#endif
}

static void set_tune_pid(float kp, float ki, float kd)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LINE)
    APP_LineFollow_SetPid(kp, ki, kd);
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_GYRO)
    APP_GyroControl_SetYawPid(kp, ki, kd);
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LEFT_SPEED)
    DVE_Chassis_SetLeftSpeedPid(kp, ki, kd);
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_RIGHT_SPEED)
    DVE_Chassis_SetRightSpeedPid(kp, ki, kd);
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_BALL)
    APP_BallControl_SetPid(kp, ki, kd);
#else
#error "Unsupported APP_CMD_TUNE_TARGET"
#endif
}

static float get_tune_error(void)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LINE)
    APP_LineFollowState_t state;
    APP_LineFollow_GetState(&state);
    return state.line_error;
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_GYRO)
    APP_GyroControlState_t state;
    APP_GyroControl_GetState(&state);
    return state.yaw_error_deg;
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LEFT_SPEED)
    DVE_ChassisState_t state;
    DVE_Chassis_GetState(&state);
    return state.left_actual_cm_s;
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_RIGHT_SPEED)
    DVE_ChassisState_t state;
    DVE_Chassis_GetState(&state);
    return state.right_actual_cm_s;
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_BALL)
    APP_BallControlStatus_t status;
    APP_BallControl_GetStatus(&status);
    return status.error_cm;
#else
#error "Unsupported APP_CMD_TUNE_TARGET"
#endif
}

static float get_tune_output(void)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_BALL)
    APP_BallControlStatus_t status;
    APP_BallControl_GetStatus(&status);
    return status.angle_cmd_deg;
#else
    APP_ChassisControlState_t state;
    APP_ChassisControl_GetState(&state);
    return state.steering_correction_cm_s;
#endif
}

static void set_one_pid_param(uint8_t cmd, float value)
{
    PID_t pid;

    get_tune_pid(&pid);
    if (cmd == CMD_SET_KP) {
        pid.Kp = value;
    } else if (cmd == CMD_SET_KI) {
        pid.Ki = value;
    } else {
        pid.Kd = value;
    }
    set_tune_pid(pid.Kp, pid.Ki, pid.Kd);
}

/*
*********************************************************************************************************
*   函 数 名: Command_SendStatus
*   功能说明: 发送当前调参对象和底盘状态帧
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void Command_SendStatus(void)
{
    DVE_ChassisState_t chassis;
    APP_ChassisControlState_t control;
    PID_t pid;

    DVE_Chassis_GetState(&chassis);
    APP_ChassisControl_GetState(&control);
    get_tune_pid(&pid);

    Command_PackAndSend(CMD_STATUS_ERROR, get_tune_error());
    Command_PackAndSend(CMD_STATUS_TURN, get_tune_output());
    Command_PackAndSend(CMD_STATUS_BASE, control.base_speed_cm_s);
    Command_PackAndSend(CMD_STATUS_KP, pid.Kp);
    Command_PackAndSend(CMD_STATUS_KD, pid.Kd);
    Command_PackAndSend(CMD_STATUS_RUN, control.is_running ? 1.0f : 0.0f);
    Command_PackAndSend(CMD_STATUS_LEFT_SPEED, chassis.left_actual_cm_s);
    Command_PackAndSend(CMD_STATUS_RIGHT_SPEED, chassis.right_actual_cm_s);
    Command_PackAndSend(CMD_STATUS_LEFT_PWM, chassis.left_pwm_out);
    Command_PackAndSend(CMD_STATUS_RIGHT_PWM, chassis.right_pwm_out);
}

/*
*********************************************************************************************************
*   函 数 名: Command_Parse
*   功能说明: 校验并执行A5/5A固定命令帧
*   形    参: buf - 接收数据；len - 数据长度
*   返 回 值: 无
*********************************************************************************************************
*/
void Command_Parse(uint8_t *buf, uint8_t len)
{
    DVE_ChassisState_t chassis;
    PID_t pid;
    uint8_t cmd;
    float data;

    if ((buf == 0) || (len < APP_CMD_PACKET_LEN) ||
        (buf[0] != APP_CMD_HEAD) || (buf[7] != APP_CMD_TAIL)) {
        send_error(APP_CMD_ERR_FORMAT);
        return;
    }

    if (calc_checksum(buf) != buf[6]) {
        send_error(APP_CMD_ERR_CHECKSUM);
        return;
    }

    cmd = buf[1];
    memcpy(&data, &buf[2], 4U);

    switch (cmd) {
        case CMD_START:
            APP_ChassisControl_Start();
            break;
        case CMD_STOP:
            APP_ChassisControl_Stop();
            break;
        case CMD_SET_SPEED:
            APP_ChassisControl_SetBaseSpeedCmps(data);
            break;
        case CMD_SET_KP:
        case CMD_SET_KI:
        case CMD_SET_KD:
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_BALL)
            if (!value_in_range(data, 0.0f, 100.0f)) {
                send_error(APP_CMD_ERR_PARAM);
                return;
            }
#endif
            set_one_pid_param(cmd, data);
            break;
        case CMD_SET_LEFT_RATIO:
            DVE_Chassis_GetState(&chassis);
            DVE_Chassis_SetPwmRatio(data, chassis.right_pwm_ratio);
            break;
        case CMD_SET_RIGHT_RATIO:
            DVE_Chassis_GetState(&chassis);
            DVE_Chassis_SetPwmRatio(chassis.left_pwm_ratio, data);
            break;
        case CMD_QUERY_STATUS:
            Command_SendStatus();
            break;
        case CMD_SET_LEFT_SPEED:
            DVE_Chassis_GetState(&chassis);
            DVE_Chassis_SetSpeedTargetCmps(data, chassis.right_target_cm_s);
            break;
        case CMD_SET_RIGHT_SPEED:
            DVE_Chassis_GetState(&chassis);
            DVE_Chassis_SetSpeedTargetCmps(chassis.left_target_cm_s, data);
            break;
        case CMD_SET_LEFT_KP:
        case CMD_SET_LEFT_KI:
        case CMD_SET_LEFT_KD:
            DVE_Chassis_GetSpeedPid(&pid, 0);
            if (cmd == CMD_SET_LEFT_KP) pid.Kp = data;
            if (cmd == CMD_SET_LEFT_KI) pid.Ki = data;
            if (cmd == CMD_SET_LEFT_KD) pid.Kd = data;
            DVE_Chassis_SetLeftSpeedPid(pid.Kp, pid.Ki, pid.Kd);
            break;
        case CMD_SET_RIGHT_KP:
        case CMD_SET_RIGHT_KI:
        case CMD_SET_RIGHT_KD:
            DVE_Chassis_GetSpeedPid(0, &pid);
            if (cmd == CMD_SET_RIGHT_KP) pid.Kp = data;
            if (cmd == CMD_SET_RIGHT_KI) pid.Ki = data;
            if (cmd == CMD_SET_RIGHT_KD) pid.Kd = data;
            DVE_Chassis_SetRightSpeedPid(pid.Kp, pid.Ki, pid.Kd);
            break;
        case CMD_BALL_START:
            APP_BallControl_SetEnable(1U);
            break;
        case CMD_BALL_STOP:
            APP_BallControl_SetEnable(0U);
            break;
        case CMD_SET_BALL_TARGET:
            if (!value_in_range(data,
                                -APP_BALL_TARGET_LIMIT_CM,
                                APP_BALL_TARGET_LIMIT_CM)) {
                send_error(APP_CMD_ERR_PARAM);
                return;
            }
            APP_BallControl_SetTargetCm(data);
            break;
        case CMD_SET_BALL_POSITION:
            if (!value_in_range(data,
                                -APP_BALL_MEASUREMENT_LIMIT_CM,
                                APP_BALL_MEASUREMENT_LIMIT_CM)) {
                send_error(APP_CMD_ERR_PARAM);
                return;
            }
            APP_BallControl_SetMeasuredPositionCm(data, (uint32_t)Get_Tick());
            break;
        case CMD_SET_SERVO_POSITION:
            if (!value_in_range(data,
                                (float)DVE_HTS25L_POS_MIN,
                                (float)DVE_HTS25L_POS_MAX)) {
                send_error(APP_CMD_ERR_PARAM);
                return;
            }
            APP_BallControl_SetServoTestPosition((uint16_t)(data + 0.5f));
            break;
        default:
            send_error(APP_CMD_ERR_UNKNOWN);
            return;
    }

    send_ack(cmd);
}

static void feed_byte(uint8_t data)
{
    if (!s_receiving) {
        if (data != APP_CMD_HEAD) {
            return;
        }
        s_receiving = true;
        s_rx_index = 0U;
    }

    s_rx_buf[s_rx_index++] = data;
    if (s_rx_index >= APP_CMD_PACKET_LEN) {
        Command_Parse(s_rx_buf, APP_CMD_PACKET_LEN);
        s_rx_index = 0U;
        s_receiving = false;
    }
}

static char *append_str(char *dst, const char *src)
{
    while (*src != '\0') {
        *dst++ = *src++;
    }
    *dst = '\0';
    return dst;
}

static char *append_i32(char *dst, int32_t value)
{
    char temp[12];
    uint8_t index = 0U;
    uint32_t magnitude;

    if (value < 0) {
        *dst++ = '-';
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        temp[index++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude > 0U);

    while (index > 0U) {
        *dst++ = temp[--index];
    }
    *dst = '\0';
    return dst;
}

static char *append_f2(char *dst, float value)
{
    int32_t scaled;
    int32_t integer;
    int32_t decimal;

    scaled = (int32_t)((value >= 0.0f) ?
             (value * 100.0f + 0.5f) :
             (value * 100.0f - 0.5f));
    integer = scaled / 100;
    decimal = scaled % 100;
    if (decimal < 0) decimal = -decimal;

    dst = append_i32(dst, integer);
    *dst++ = '.';
    *dst++ = (char)('0' + decimal / 10);
    *dst++ = (char)('0' + decimal % 10);
    *dst = '\0';
    return dst;
}

static void send_wave_status(void)
{
    Command_PackAndSend(CMD_STATUS_ERROR, get_tune_error());
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_BALL)
    Command_PackAndSend(CMD_STATUS_TURN, get_tune_output());
#endif
}

static void send_text_status(void)
{
    APP_BallControlStatus_t ball;
    DVE_ChassisState_t chassis;
    APP_ChassisControlState_t control;
    PID_t pid;
    char *p = s_text_buf;

    DVE_Chassis_GetState(&chassis);
    APP_ChassisControl_GetState(&control);
    APP_BallControl_GetStatus(&ball);
    get_tune_pid(&pid);

    p = append_str(p, "STAT,mode=");
#if (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_LINE_FOLLOW)
    p = append_str(p, "line_follow");
#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_GYRO_YAW)
    p = append_str(p, "gyro");
#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_SPEED_TEST)
    p = append_str(p, "speed_test");
#else
#error "Unsupported APP_CHASSIS_CONTROL_MODE"
#endif
    p = append_str(p, ",tune=");
    p = append_str(p, APP_CMD_TUNE_TARGET_NAME);
    p = append_str(p, ",base=");
    p = append_f2(p, control.base_speed_cm_s);
    p = append_str(p, ",kp=");
    p = append_f2(p, pid.Kp);
    p = append_str(p, ",ki=");
    p = append_f2(p, pid.Ki);
    p = append_str(p, ",kd=");
    p = append_f2(p, pid.Kd);

#if (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_LINE_FOLLOW)
    {
        APP_LineFollowState_t state;

        APP_LineFollow_GetState(&state);
        p = append_str(p, "\r\nline_err=");
        p = append_f2(p, state.line_error);
        p = append_str(p, ",black=");
        p = append_i32(p, state.black_count);
        p = append_str(p, ",lost=");
        p = append_i32(p, state.is_lost ? 1 : 0);
        p = append_str(p, ",search=");
        p = append_i32(p, state.is_searching ? 1 : 0);
        p = append_str(p, ",scale=");
        p = append_f2(p, state.speed_scale);
    }
#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_GYRO_YAW)
    {
        APP_GyroControlState_t state;

        APP_GyroControl_GetState(&state);
        p = append_str(p, "\r\nyaw=");
        p = append_f2(p, state.yaw_deg);
        p = append_str(p, ",roll=");
        p = append_f2(p, state.roll_deg);
        p = append_str(p, ",pitch=");
        p = append_f2(p, state.pitch_deg);
        p = append_str(p, ",target=");
        p = append_f2(p, state.target_yaw_deg);
        p = append_str(p, ",yaw_err=");
        p = append_f2(p, state.yaw_error_deg);
        p = append_str(p, ",ready=");
        p = append_i32(p, state.imu_ready ? 1 : 0);
    }
#elif (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_SPEED_TEST)
#else
#error "Unsupported APP_CHASSIS_CONTROL_MODE"
#endif

    p = append_str(p, "\r\nBALL st=");
    p = append_i32(p, (int32_t)ball.state);
    p = append_str(p, ",en=");
    p = append_i32(p, ball.enabled ? 1 : 0);
    p = append_str(p, ",valid=");
    p = append_i32(p, ball.vision_valid ? 1 : 0);
    p = append_str(p, ",x=");
    p = append_f2(p, ball.measured_position_cm);
    p = append_str(p, ",tgt=");
    p = append_f2(p, ball.target_position_cm);
    p = append_str(p, ",err=");
    p = append_f2(p, ball.error_cm);
    p = append_str(p, ",v=");
    p = append_f2(p, ball.estimated_velocity_cm_s);
    p = append_str(p, ",pid=");
    p = append_f2(p, pid.Kp);
    p = append_str(p, "/");
    p = append_f2(p, pid.Ki);
    p = append_str(p, "/");
    p = append_f2(p, pid.Kd);
    p = append_str(p, ",ang=");
    p = append_f2(p, ball.angle_cmd_deg);
    p = append_str(p, ",pos=");
    p = append_i32(p, ball.servo_command);
    p = append_str(p, ",sat=");
    p = append_i32(p, ball.saturated ? 1 : 0);

    p = append_str(p, "\r\nlt=");
    p = append_f2(p, chassis.left_target_cm_s);
    p = append_str(p, ",rt=");
    p = append_f2(p, chassis.right_target_cm_s);
    p = append_str(p, ",la=");
    p = append_f2(p, chassis.left_actual_cm_s);
    p = append_str(p, ",ra=");
    p = append_f2(p, chassis.right_actual_cm_s);
    p = append_str(p, ",lpwm=");
    p = append_f2(p, chassis.left_pwm_out);
    p = append_str(p, ",rpwm=");
    p = append_f2(p, chassis.right_pwm_out);
    p = append_str(p, ",run=");
    p = append_i32(p, control.is_running ? 1 : 0);
    p = append_str(p, "\r\n");

    Bluetooth_SendString(s_text_buf);
}

/*
*********************************************************************************************************
*   函 数 名: APP_Command_Task
*   功能说明: 处理蓝牙命令并按低频回传状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_Command_Task(void)
{
    BLE_RxData_t rx;
    uint8_t i;
    uint32_t now;

    Bluetooth_ReceiveData(&rx);
    if (rx.flag) {
        for (i = 0U; i < rx.len; i++) {
            feed_byte(rx.buff[i]);
        }
    }

    now = (uint32_t)Get_Tick();
#if (APP_CMD_ENABLE_WAVE_FRAME != 0U)
    if ((now - s_last_wave_ms) >= APP_CMD_WAVE_PERIOD_MS) {
        s_last_wave_ms = now;
        send_wave_status();
    }
#endif
#if (APP_CMD_ENABLE_TEXT_STATUS != 0U)
    if ((now - s_last_text_ms) >= APP_CMD_TEXT_PERIOD_MS) {
        s_last_text_ms = now;
        send_text_status();
    }
#endif
}
