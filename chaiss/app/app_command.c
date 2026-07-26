#include "app_command.h"
#include "app_chassis_control.h"
#include "app_gyro_control.h"
#include "app_line_follow.h"
#include "bsp_bluetooth.h"
#include "dve_chassis.h"
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
static uint8_t s_cmd_rx_buf[APP_CMD_PACKET_LEN];        /* 协议接收缓存 */
static uint8_t s_cmd_rx_index = 0U;                     /* 当前接收位置 */
static bool s_cmd_receiving = false;                    /* 协议接收状态 */

static char s_text_tx_buf[380];                         /* 文本状态回传缓存 */
static uint32_t s_last_wave_ms = 0U;                    /* 上一次波形回传时间 */
static uint32_t s_last_text_ms = 0U;                    /* 上一次文本回传时间 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static uint8_t Command_CalcCheckSum(uint8_t *buf);
static void Command_SendAck(uint8_t cmd);
static void Command_SendError(float err_code);
static void Command_FeedByte(uint8_t data);
static void command_get_tune_pid(PID_t *pid);
static void command_set_tune_pid(float kp, float ki, float kd);
static float command_get_tune_error(void);
static void command_set_outer_kp(float value);
static void command_set_outer_ki(float value);
static void command_set_outer_kd(float value);
static void send_wave_status(void);
static void send_text_status(void);
static char *append_str(char *dst, const char *src);
static char *append_i32(char *dst, int32_t value);
static char *append_f2(char *dst, float value);

static uint8_t Command_CalcCheckSum(uint8_t *buf)
{
    return (uint8_t)(buf[1] + buf[2] + buf[3] + buf[4] + buf[5]);
}

void Command_PackAndSend(uint8_t cmd, float data)
{
    uint8_t tx_buf[APP_CMD_PACKET_LEN];

    tx_buf[0] = APP_CMD_HEAD;
    tx_buf[1] = cmd;
    memcpy(&tx_buf[2], &data, 4U);
    tx_buf[6] = Command_CalcCheckSum(tx_buf);
    tx_buf[7] = APP_CMD_TAIL;

    Bluetooth_SendData(tx_buf, APP_CMD_PACKET_LEN);
}

static void Command_SendAck(uint8_t cmd)
{
#if (APP_CMD_ENABLE_ACK_FRAME != 0U)
    Command_PackAndSend(CMD_ACK, (float)cmd);
#else
    (void)cmd;
#endif
}

static void Command_SendError(float err_code)
{
#if (APP_CMD_ENABLE_ERR_FRAME != 0U)
    Command_PackAndSend(CMD_ERR, err_code);
#else
    (void)err_code;
#endif
}

static void command_get_tune_pid(PID_t *pid)
{
    if (pid == 0) return; /* 防止空指针写入 */

#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LINE)
    APP_LineFollowState_t state;

    APP_LineFollow_GetState(&state);
    *pid = state.line_pid; /* 复制循迹PID参数 */
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_GYRO)
    APP_GyroControlState_t state;

    APP_GyroControl_GetState(&state);
    *pid = state.yaw_pid; /* 复制航向PID参数 */
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LEFT_SPEED)
    DVE_Chassis_GetSpeedPid(pid, 0); /* 读取左轮速度PID */
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_RIGHT_SPEED)
    DVE_Chassis_GetSpeedPid(0, pid); /* 读取右轮速度PID */
#else
#error "Unsupported APP_CMD_TUNE_TARGET"
#endif
}

static void command_set_tune_pid(float kp, float ki, float kd)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LINE)
    APP_LineFollow_SetPid(kp, ki, kd); /* 写入循迹PID */
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_GYRO)
    APP_GyroControl_SetYawPid(kp, ki, kd); /* 写入航向PID */
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LEFT_SPEED)
    DVE_Chassis_SetLeftSpeedPid(kp, ki, kd); /* 写入左轮速度PID */
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_RIGHT_SPEED)
    DVE_Chassis_SetRightSpeedPid(kp, ki, kd); /* 写入右轮速度PID */
#else
#error "Unsupported APP_CMD_TUNE_TARGET"
#endif
}

static float command_get_tune_error(void)
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
#else
#error "Unsupported APP_CMD_TUNE_TARGET"
#endif
}

static void command_set_outer_kp(float value)
{
    PID_t pid;

    command_get_tune_pid(&pid); /* 保留Ki、Kd，仅更新Kp */
    command_set_tune_pid(value, pid.Ki, pid.Kd);
}

static void command_set_outer_ki(float value)
{
    PID_t pid;

    command_get_tune_pid(&pid); /* 保留Kp、Kd，仅更新Ki */
    command_set_tune_pid(pid.Kp, value, pid.Kd);
}

static void command_set_outer_kd(float value)
{
    PID_t pid;

    command_get_tune_pid(&pid); /* 保留Kp、Ki，仅更新Kd */
    command_set_tune_pid(pid.Kp, pid.Ki, value);
}

void Command_SendStatus(void)
{
    DVE_ChassisState_t chassis_state;
    APP_ChassisControlState_t control_state;
    PID_t tune_pid;

    DVE_Chassis_GetState(&chassis_state);
    APP_ChassisControl_GetState(&control_state);
    command_get_tune_pid(&tune_pid); /* PID参数只读取一次 */

    Command_PackAndSend(CMD_STATUS_ERROR, command_get_tune_error());
    Command_PackAndSend(CMD_STATUS_TURN, control_state.steering_correction_cm_s);
    Command_PackAndSend(CMD_STATUS_BASE, control_state.base_speed_cm_s);
    Command_PackAndSend(CMD_STATUS_KP, tune_pid.Kp);
    Command_PackAndSend(CMD_STATUS_KD, tune_pid.Kd);
    Command_PackAndSend(CMD_STATUS_RUN, control_state.is_running ? 1.0f : 0.0f);
    Command_PackAndSend(CMD_STATUS_LEFT_SPEED, chassis_state.left_actual_cm_s);
    Command_PackAndSend(CMD_STATUS_RIGHT_SPEED, chassis_state.right_actual_cm_s);
    Command_PackAndSend(CMD_STATUS_LEFT_PWM, chassis_state.left_pwm_out);
    Command_PackAndSend(CMD_STATUS_RIGHT_PWM, chassis_state.right_pwm_out);
}

void Command_Parse(uint8_t *buf, uint8_t len)
{
    uint8_t cmd;
    uint8_t sum;
    float data;

    if ((buf == 0) || (len < APP_CMD_PACKET_LEN)) {
        Command_SendError(APP_CMD_ERR_FORMAT);
        return;
    }

    if ((buf[0] != APP_CMD_HEAD) || (buf[7] != APP_CMD_TAIL)) {
        Command_SendError(APP_CMD_ERR_FORMAT);
        return;
    }

    sum = Command_CalcCheckSum(buf);
    if (sum != buf[6]) {
        Command_SendError(APP_CMD_ERR_CHECKSUM);
        return;
    }

    cmd = buf[1];
    memcpy(&data, &buf[2], 4U);

    switch (cmd) {
        case CMD_START:
            APP_ChassisControl_Start();
            Command_SendAck(cmd);
            break;

        case CMD_STOP:
            APP_ChassisControl_Stop();
            Command_SendAck(cmd);
            break;

        case CMD_SET_SPEED:
            APP_ChassisControl_SetBaseSpeedCmps(data);
            Command_SendAck(cmd);
            break;

        case CMD_SET_KP:
            command_set_outer_kp(data);
            Command_SendAck(cmd);
            break;

        case CMD_SET_KI:
            command_set_outer_ki(data);
            Command_SendAck(cmd);
            break;

        case CMD_SET_KD:
            command_set_outer_kd(data);
            Command_SendAck(cmd);
            break;

        case CMD_SET_LEFT_RATIO:
        {
            DVE_ChassisState_t chassis_state;

            DVE_Chassis_GetState(&chassis_state);
            DVE_Chassis_SetPwmRatio(data, chassis_state.right_pwm_ratio);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_RIGHT_RATIO:
        {
            DVE_ChassisState_t chassis_state;

            DVE_Chassis_GetState(&chassis_state);
            DVE_Chassis_SetPwmRatio(chassis_state.left_pwm_ratio, data);
            Command_SendAck(cmd);
            break;
        }

        case CMD_QUERY_STATUS:
            Command_SendStatus();
            Command_SendAck(cmd);
            break;

        case CMD_SET_LEFT_SPEED:
        {
            DVE_ChassisState_t chassis_state;

            DVE_Chassis_GetState(&chassis_state);
            DVE_Chassis_SetSpeedTargetCmps(data, chassis_state.right_target_cm_s);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_RIGHT_SPEED:
        {
            DVE_ChassisState_t chassis_state;

            DVE_Chassis_GetState(&chassis_state);
            DVE_Chassis_SetSpeedTargetCmps(chassis_state.left_target_cm_s, data);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_LEFT_KP:
        {
            PID_t left_pid;

            DVE_Chassis_GetSpeedPid(&left_pid, 0);
            DVE_Chassis_SetLeftSpeedPid(data, left_pid.Ki, left_pid.Kd);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_LEFT_KI:
        {
            PID_t left_pid;

            DVE_Chassis_GetSpeedPid(&left_pid, 0);
            DVE_Chassis_SetLeftSpeedPid(left_pid.Kp, data, left_pid.Kd);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_LEFT_KD:
        {
            PID_t left_pid;

            DVE_Chassis_GetSpeedPid(&left_pid, 0);
            DVE_Chassis_SetLeftSpeedPid(left_pid.Kp, left_pid.Ki, data);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_RIGHT_KP:
        {
            PID_t right_pid;

            DVE_Chassis_GetSpeedPid(0, &right_pid);
            DVE_Chassis_SetRightSpeedPid(data, right_pid.Ki, right_pid.Kd);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_RIGHT_KI:
        {
            PID_t right_pid;

            DVE_Chassis_GetSpeedPid(0, &right_pid);
            DVE_Chassis_SetRightSpeedPid(right_pid.Kp, data, right_pid.Kd);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_RIGHT_KD:
        {
            PID_t right_pid;

            DVE_Chassis_GetSpeedPid(0, &right_pid);
            DVE_Chassis_SetRightSpeedPid(right_pid.Kp, right_pid.Ki, data);
            Command_SendAck(cmd);
            break;
        }

        default:
            Command_SendError(APP_CMD_ERR_UNKNOWN);
            break;
    }
}

static void Command_FeedByte(uint8_t data)
{
    if (!s_cmd_receiving) {
        if (data != APP_CMD_HEAD) {
            return;
        }

        s_cmd_receiving = true;
        s_cmd_rx_index = 0U;
    }

    s_cmd_rx_buf[s_cmd_rx_index++] = data;

    if (s_cmd_rx_index >= APP_CMD_PACKET_LEN) {
        Command_Parse(s_cmd_rx_buf, APP_CMD_PACKET_LEN);
        s_cmd_rx_index = 0U;
        s_cmd_receiving = false;
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
    uint32_t abs_value;

    if (value < 0) {
        *dst++ = '-';
        abs_value = (uint32_t)(-value);
    } else {
        abs_value = (uint32_t)value;
    }

    do {
        temp[index++] = (char)('0' + (abs_value % 10U));
        abs_value /= 10U;
    } while (abs_value > 0U);

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

    scaled = (int32_t)((value >= 0.0f) ? (value * 100.0f + 0.5f) : (value * 100.0f - 0.5f));
    integer = scaled / 100;
    decimal = scaled % 100;

    if (decimal < 0) {
        decimal = -decimal;
    }

    dst = append_i32(dst, integer);
    *dst++ = '.';
    *dst++ = (char)('0' + (decimal / 10));
    *dst++ = (char)('0' + (decimal % 10));
    *dst = '\0';

    return dst;
}

static void send_wave_status(void)
{
    Command_PackAndSend(CMD_STATUS_ERROR, command_get_tune_error());
}

static void send_text_status(void)
{
    DVE_ChassisState_t chassis_state;
    APP_ChassisControlState_t control_state;
    PID_t tune_pid;
    float tune_error;
    char *p = s_text_tx_buf;
    
    DVE_Chassis_GetState(&chassis_state);
    APP_ChassisControl_GetState(&control_state);
    command_get_tune_pid(&tune_pid); /* Kp、Ki、Kd一次读取 */
    tune_error = command_get_tune_error();

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
    p = append_str(p, "\r\nbase=");
    p = append_f2(p, control_state.base_speed_cm_s);
    p = append_str(p, ",kp=");
    p = append_f2(p, tune_pid.Kp);
    p = append_str(p, ",ki=");
    p = append_f2(p, tune_pid.Ki);
    p = append_str(p, ",kd=");
    p = append_f2(p, tune_pid.Kd);
    p = append_str(p, ",tune_err=");
    p = append_f2(p, tune_error);

#if (APP_CHASSIS_CONTROL_MODE == APP_CHASSIS_CONTROL_MODE_LINE_FOLLOW)
    {
        APP_LineFollowState_t state;

        APP_LineFollow_GetState(&state);
        p = append_str(p, "\r\nline_err=");
        p = append_f2(p, state.line_error);
        p = append_str(p, ",black=");
        p = append_i32(p, (int32_t)state.black_count);
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

    p = append_str(p, "\r\nsteer=");
    p = append_f2(p, control_state.steering_correction_cm_s);
    p = append_str(p, ",lt=");
    p = append_f2(p, chassis_state.left_target_cm_s);
    p = append_str(p, ",rt=");
    p = append_f2(p, chassis_state.right_target_cm_s);
    p = append_str(p, ",la=");
    p = append_f2(p, chassis_state.left_actual_cm_s);
    p = append_str(p, ",ra=");
    p = append_f2(p, chassis_state.right_actual_cm_s);
    p = append_str(p, "\r\nlpwm=");
    p = append_f2(p, chassis_state.left_pwm_out);
    p = append_str(p, ",rpwm=");
    p = append_f2(p, chassis_state.right_pwm_out);
    p = append_str(p, ",run=");
    p = append_i32(p, control_state.is_running ? 1 : 0);
    p = append_str(p, "\r\n");

    Bluetooth_SendString(s_text_tx_buf);
}

void APP_Command_Task(void)
{
    BLE_RxData_t rx;
    uint8_t i;
    uint32_t now;

    Bluetooth_ReceiveData(&rx);

    if (rx.flag) {
        for (i = 0U; i < rx.len; i++) {
            Command_FeedByte(rx.buff[i]);
        }
    }

    now = Get_Tick();
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
