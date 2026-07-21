#include "app_command.h"
#include "app_trace.h"
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
*   鍏ㄥ眬鍙橀噺
*********************************************************************************************************
*/
static uint8_t s_cmd_rx_buf[APP_CMD_PACKET_LEN];        /* 鍗忚鎺ユ敹缂撳瓨 */
static uint8_t s_cmd_rx_index = 0U;                     /* 褰撳墠鎺ユ敹浣嶇疆 */
static bool s_cmd_receiving = false;                    /* 鍗忚鎺ユ敹鐘舵€?*/

static char s_text_tx_buf[240];                         /* 鏂囨湰鐘舵€佸洖浼犵紦瀛?*/
static uint32_t s_last_wave_ms = 0U;                    /* 涓婁竴娆℃尝褰㈠洖浼犳椂闂?*/
static uint32_t s_last_text_ms = 0U;                    /* 涓婁竴娆℃枃鏈洖浼犳椂闂?*/
static float s_cmd_left_target_cm_s = 15.0f;            /* 宸﹁疆娴嬭瘯鐩爣閫熷害 */
static float s_cmd_right_target_cm_s = 15.0f;           /* 鍙宠疆娴嬭瘯鐩爣閫熷害 */

/*
*********************************************************************************************************
*   鍑芥暟澹版槑
*********************************************************************************************************
*/
static uint8_t Command_CalcCheckSum(uint8_t *buf);
static void Command_SendAck(uint8_t cmd);
static void Command_SendError(float err_code);
static void Command_FeedByte(uint8_t data);
static void command_set_tuned_kp(float value);
static void command_set_tuned_ki(float value);
static void command_set_tuned_kd(float value);
static float command_get_tuned_kp(void);
static float command_get_tuned_ki(void);
static float command_get_tuned_kd(void);
static void send_wave_status(void);
static void send_text_status(void);
static char *append_str(char *dst, const char *src);
static char *append_i32(char *dst, int32_t value);
static char *append_f2(char *dst, float value);

/*
*********************************************************************************************************
*   鍑?鏁?鍚? Command_CalcCheckSum
*   鍔熻兘璇存槑: 璁＄畻8瀛楄妭鍗忚鏍￠獙鍜?*   褰?   鍙? buf 鈥?鍗忚鍖呯紦瀛?*   杩?鍥?鍊? 鏍￠獙鍜?*********************************************************************************************************
*/
static uint8_t Command_CalcCheckSum(uint8_t *buf)
{
    return (uint8_t)(buf[1] + buf[2] + buf[3] + buf[4] + buf[5]);
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? Command_PackAndSend
*   鍔熻兘璇存槑: 鎸夊浐瀹氬崗璁粍鍖呭苟閫氳繃钃濈墮鍙戦€?*   褰?   鍙? cmd 鈥?鍛戒护瀛楋紱data 鈥?娴偣鏁版嵁
*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
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

/*
*********************************************************************************************************
*   鍑?鏁?鍚? Command_SendStatus
*   鍔熻兘璇存槑: 鎸夊崗璁抚鍥炰紶鍏抽敭鐘舵€侊紝鍋忓樊甯х敤浜庢墜鏈哄浘鍍忔樉绀?*   褰?   鍙? 鏃?*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
void Command_SendStatus(void)
{
    DVE_ChassisState_t chassis_state;
    APP_TraceState_t trace_state;

    DVE_Chassis_GetState(&chassis_state);
    APP_Trace_GetState(&trace_state);

    Command_PackAndSend(CMD_STATUS_ERROR, trace_state.last_error);

#if (APP_TRACE_CONTROL_MODE == APP_TRACE_MODE_SPEED_LOOP)
    Command_PackAndSend(CMD_STATUS_TURN, trace_state.last_turn_speed);
    Command_PackAndSend(CMD_STATUS_BASE, trace_state.base_speed_cm_s);
#else
    Command_PackAndSend(CMD_STATUS_TURN, trace_state.last_turn_pwm);
    Command_PackAndSend(CMD_STATUS_BASE, trace_state.base_pwm);
#endif

    Command_PackAndSend(CMD_STATUS_KP, command_get_tuned_kp());
    Command_PackAndSend(CMD_STATUS_KD, command_get_tuned_kd());
    Command_PackAndSend(CMD_STATUS_RUN, trace_state.is_running ? 1.0f : 0.0f);
    Command_PackAndSend(CMD_STATUS_LEFT_SPEED, chassis_state.left_actual_cm_s);
    Command_PackAndSend(CMD_STATUS_RIGHT_SPEED, chassis_state.right_actual_cm_s);
    Command_PackAndSend(CMD_STATUS_LEFT_PWM, chassis_state.left_pwm_out);
    Command_PackAndSend(CMD_STATUS_RIGHT_PWM, chassis_state.right_pwm_out);
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? command_set_tuned_kp
*   鍔熻兘璇存槑: 璁剧疆褰撳墠瀹忛€夋嫨璋冨弬鐜殑Kp
*   褰?   鍙? value 鈥?Kp鍙傛暟
*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
static void command_set_tuned_kp(float value)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_SPEED)
    PID_t left_pid;
    PID_t right_pid;

    DVE_Chassis_GetSpeedPid(&left_pid, &right_pid);
    DVE_Chassis_SetLeftSpeedPid(value, left_pid.Ki, left_pid.Kd);
    DVE_Chassis_SetRightSpeedPid(value, right_pid.Ki, right_pid.Kd);
#else
    APP_TraceState_t trace_state;

    APP_Trace_GetState(&trace_state);
    APP_Trace_SetPid(value, trace_state.trace_pid.Ki, trace_state.trace_pid.Kd);
#endif
}

static void command_set_tuned_ki(float value)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_SPEED)
    PID_t left_pid;
    PID_t right_pid;

    DVE_Chassis_GetSpeedPid(&left_pid, &right_pid);
    DVE_Chassis_SetLeftSpeedPid(left_pid.Kp, value, left_pid.Kd);
    DVE_Chassis_SetRightSpeedPid(right_pid.Kp, value, right_pid.Kd);
#else
    APP_TraceState_t trace_state;

    APP_Trace_GetState(&trace_state);
    APP_Trace_SetPid(trace_state.trace_pid.Kp, value, trace_state.trace_pid.Kd);
#endif
}

static void command_set_tuned_kd(float value)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_SPEED)
    PID_t left_pid;
    PID_t right_pid;

    DVE_Chassis_GetSpeedPid(&left_pid, &right_pid);
    DVE_Chassis_SetLeftSpeedPid(left_pid.Kp, left_pid.Ki, value);
    DVE_Chassis_SetRightSpeedPid(right_pid.Kp, right_pid.Ki, value);
#else
    APP_TraceState_t trace_state;

    APP_Trace_GetState(&trace_state);
    APP_Trace_SetPid(trace_state.trace_pid.Kp, trace_state.trace_pid.Ki, value);
#endif
}

static float command_get_tuned_kp(void)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_SPEED)
    PID_t right_pid;

    DVE_Chassis_GetSpeedPid(0, &right_pid);
    return right_pid.Kp;
#else
    APP_TraceState_t trace_state;

    APP_Trace_GetState(&trace_state);
    return trace_state.trace_pid.Kp;
#endif
}

static float command_get_tuned_ki(void)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_SPEED)
    PID_t right_pid;

    DVE_Chassis_GetSpeedPid(0, &right_pid);
    return right_pid.Ki;
#else
    APP_TraceState_t trace_state;

    APP_Trace_GetState(&trace_state);
    return trace_state.trace_pid.Ki;
#endif
}

static float command_get_tuned_kd(void)
{
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_SPEED)
    PID_t right_pid;

    DVE_Chassis_GetSpeedPid(0, &right_pid);
    return right_pid.Kd;
#else
    APP_TraceState_t trace_state;

    APP_Trace_GetState(&trace_state);
    return trace_state.trace_pid.Kd;
#endif
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? Command_Parse
*   鍔熻兘璇存槑: 瑙ｆ瀽8瀛楄妭浜岃繘鍒跺崗璁寘骞舵墽琛屽懡浠?*   褰?   鍙? buf 鈥?鏁版嵁缂撳瓨锛沴en 鈥?鏁版嵁闀垮害
*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
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
            APP_Trace_Start();
            Command_SendAck(cmd);
            break;

        case CMD_STOP:
            APP_Trace_Stop();
            Command_SendAck(cmd);
            break;

        case CMD_SET_SPEED:
            s_cmd_left_target_cm_s = data;
            s_cmd_right_target_cm_s = data;
            APP_Trace_SetBasePwm(data);
            Command_SendAck(cmd);
            break;

        case CMD_SET_KP:
            command_set_tuned_kp(data);
            Command_SendAck(cmd);
            break;

        case CMD_SET_KI:
            command_set_tuned_ki(data);
            Command_SendAck(cmd);
            break;

        case CMD_SET_KD:
            command_set_tuned_kd(data);
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
            Command_SendAck(cmd);
            break;

        case CMD_SET_LEFT_SPEED:
            s_cmd_left_target_cm_s = data;
            APP_Trace_SetTestSpeedCmps(s_cmd_left_target_cm_s, s_cmd_right_target_cm_s);
            Command_SendAck(cmd);
            break;

        case CMD_SET_RIGHT_SPEED:
            s_cmd_right_target_cm_s = data;
            APP_Trace_SetTestSpeedCmps(s_cmd_left_target_cm_s, s_cmd_right_target_cm_s);
            Command_SendAck(cmd);
            break;

        case CMD_SET_LEFT_KP:
        {
            PID_t left_pid;

            DVE_Chassis_GetSpeedPid(&left_pid, 0);
            DVE_Chassis_SetLeftSpeedPid(data,
                                        left_pid.Ki,
                                        left_pid.Kd);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_LEFT_KI:
        {
            PID_t left_pid;

            DVE_Chassis_GetSpeedPid(&left_pid, 0);
            DVE_Chassis_SetLeftSpeedPid(left_pid.Kp,
                                        data,
                                        left_pid.Kd);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_LEFT_KD:
        {
            PID_t left_pid;

            DVE_Chassis_GetSpeedPid(&left_pid, 0);
            DVE_Chassis_SetLeftSpeedPid(left_pid.Kp,
                                        left_pid.Ki,
                                        data);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_RIGHT_KP:
        {
            PID_t right_pid;

            DVE_Chassis_GetSpeedPid(0, &right_pid);
            DVE_Chassis_SetRightSpeedPid(data,
                                         right_pid.Ki,
                                         right_pid.Kd);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_RIGHT_KI:
        {
            PID_t right_pid;

            DVE_Chassis_GetSpeedPid(0, &right_pid);
            DVE_Chassis_SetRightSpeedPid(right_pid.Kp,
                                         data,
                                         right_pid.Kd);
            Command_SendAck(cmd);
            break;
        }

        case CMD_SET_RIGHT_KD:
        {
            PID_t right_pid;

            DVE_Chassis_GetSpeedPid(0, &right_pid);
            DVE_Chassis_SetRightSpeedPid(right_pid.Kp,
                                         right_pid.Ki,
                                         data);
            Command_SendAck(cmd);
            break;
        }

        default:
            Command_SendError(APP_CMD_ERR_UNKNOWN);
            break;
    }
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? Command_FeedByte
*   鍔熻兘璇存槑: 鍠傚叆鍗忚瀛楄妭锛屾寜A5鍖呭ご鍜屽浐瀹?瀛楄妭闀垮害缁勫寘
*   褰?   鍙? data 鈥?鏂版敹鍒扮殑瀛楄妭
*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
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

/*
*********************************************************************************************************
*   鍑?鏁?鍚? send_text_status
*   鍔熻兘璇存槑: 鍙戦€佷竴琛屾枃鏈姸鎬侊紝渚涙墜鏈虹纭璋冨弬鏄惁鐢熸晥
*   褰?   鍙? 鏃?*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
/*
*********************************************************************************************************
*   鍑?鏁?鍚? send_wave_status
*   鍔熻兘璇存槑: 鍙戦€佷簩杩涘埗娉㈠舰鏁版嵁锛屼緵鎵嬫満绔浘鍍忔樉绀?*   褰?   鍙? 鏃?*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
static void send_wave_status(void)
{
    DVE_ChassisState_t chassis_state;
    APP_TraceState_t trace_state;
    float wave_value;

#if (APP_TRACE_ENABLE_IR_OUTER_LOOP == 0U)
    DVE_Chassis_GetState(&chassis_state);
    wave_value = chassis_state.right_actual_cm_s - chassis_state.right_target_cm_s;
#else
    APP_Trace_GetState(&trace_state);
    wave_value = trace_state.last_error;
#endif

    Command_PackAndSend(CMD_STATUS_ERROR, wave_value);
}

static void send_text_status(void)
{
    DVE_ChassisState_t chassis_state;
    APP_TraceState_t trace_state;
    char *p = s_text_tx_buf;

    DVE_Chassis_GetState(&chassis_state);
    APP_Trace_GetState(&trace_state);

    p = append_str(p, "STAT,mode=");
#if (APP_TRACE_CONTROL_MODE == APP_TRACE_MODE_SPEED_LOOP)
#if (APP_TRACE_ENABLE_IR_OUTER_LOOP == 0U)
    p = append_str(p, "speed_test");
#else
    p = append_str(p, "speed");
#endif
#else
    p = append_str(p, "pwm");
#endif

    p = append_str(p, ",tune=");
#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_SPEED)
    p = append_str(p, "right_speed");
#else
    p = append_str(p, "trace");
#endif

    p = append_str(p, ",base=");
#if (APP_TRACE_CONTROL_MODE == APP_TRACE_MODE_SPEED_LOOP)
    p = append_f2(p, trace_state.base_speed_cm_s);
#else
    p = append_f2(p, trace_state.base_pwm);
#endif
    p = append_str(p, "\r\n");
    p = append_str(p, "kp=");
    p = append_f2(p, command_get_tuned_kp());
    p = append_str(p, ",ki=");
    p = append_f2(p, command_get_tuned_ki());
    p = append_str(p, ",kd=");
    p = append_f2(p, command_get_tuned_kd());
    
    p = append_str(p, ",err=");
    p = append_f2(p, trace_state.last_error);
    p = append_str(p, "\r\n");

    p = append_str(p, "lt=");
    p = append_f2(p, chassis_state.left_target_cm_s);
    p = append_str(p, ",rt=");
    p = append_f2(p, chassis_state.right_target_cm_s);
    p = append_str(p, ",la=");
    p = append_f2(p, chassis_state.left_actual_cm_s);
    p = append_str(p, ",ra=");
    p = append_f2(p, chassis_state.right_actual_cm_s);
    p = append_str(p, "\r\n");
    p = append_str(p, "lpwm=");
    p = append_f2(p, chassis_state.left_pwm_out);
    p = append_str(p, ",rpwm=");
    p = append_f2(p, chassis_state.right_pwm_out);
    p = append_str(p, ",run=");
    p = append_i32(p, trace_state.is_running ? 1 : 0);
    p = append_str(p, "\r\n");
    
    Bluetooth_SendString(s_text_tx_buf);
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? APP_Command_Task
*   鍔熻兘璇存槑: 钃濈墮鍛戒护浠诲姟锛屼富寰幆涓弽澶嶈皟鐢?*   褰?   鍙? 鏃?*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
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





