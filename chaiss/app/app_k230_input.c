#include "app_k230_input.h"
#include "bsp_k230_uart.h"
#include <stdbool.h>
#include <string.h>

#if (BSP_K230_UART_FRAME_LENGTH != APP_K230_PACKET_LEN)
#error "K230 UART frame length must match K230 packet length"
#endif

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static APP_K230InputState_t s_k230_state;       /* K230输入统计和最近数据状态 */
static APP_K230BallFrame_t s_latest_frame;      /* 最新有效钢球位置帧 */
static bool s_latest_frame_ready;               /* true表示最新帧未被上层消费 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static uint8_t calc_checksum(const uint8_t *buf);
static bool value_in_range(float value, float min, float max);
static void parse_frame(const BSP_K230UartFrame_t *bsp_frame);

/*
*********************************************************************************************************
*   函 数 名: calc_checksum
*   功能说明: 计算A5/5A固定帧校验和
*   形    参: buf - 数据帧
*   返 回 值: 校验和
*********************************************************************************************************
*/
static uint8_t calc_checksum(const uint8_t *buf)
{
    return (uint8_t)(buf[1] + buf[2] + buf[3] + buf[4] + buf[5]);
}

/*
*********************************************************************************************************
*   函 数 名: value_in_range
*   功能说明: 检查浮点数是否有效并位于范围内
*   形    参: value - 输入值；min - 下限；max - 上限
*   返 回 值: true有效，false无效
*********************************************************************************************************
*/
static bool value_in_range(float value, float min, float max)
{
    return ((value == value) && (value >= min) && (value <= max)); // value==value用于排除NaN
}

/*
*********************************************************************************************************
*   函 数 名: parse_frame
*   功能说明: 解析K230视觉帧并写入最新钢球位置
*   形    参: bsp_frame - BSP层完整收帧邮箱，携带真实收帧时间
*   返 回 值: 无
*********************************************************************************************************
*/
static void parse_frame(const BSP_K230UartFrame_t *bsp_frame)
{
    const uint8_t *buf;
    uint8_t cmd;
    float data;

    if (bsp_frame == 0) {
        return;
    }

    s_k230_state.rx_frame_count++;
    buf = bsp_frame->data;

    if ((bsp_frame->length != APP_K230_PACKET_LEN) ||
        (buf[0] != APP_K230_FRAME_HEAD) ||
        (buf[7] != APP_K230_FRAME_TAIL)) {
        s_k230_state.format_error_count++;
        return;
    }

    if (calc_checksum(buf) != buf[6]) {
        s_k230_state.checksum_error_count++;
        return;
    }

    cmd = buf[1];
    memcpy(&data, &buf[2], 4U);

    switch (cmd) {
        case APP_K230_CMD_BALL_X_CM:
            if (!value_in_range(data,
                                -APP_K230_POSITION_LIMIT_CM,
                                APP_K230_POSITION_LIMIT_CM)) {
                s_k230_state.format_error_count++;
                return;
            }

            s_latest_frame.x_cm = data;
            s_latest_frame.tick_ms = bsp_frame->rx_complete_tick_ms;
            s_latest_frame_ready = true; // 只保留最新视觉帧，旧帧不排队
            s_k230_state.last_x_cm = data;
            s_k230_state.data_valid = 1U;
            s_k230_state.valid_position_count++;
            break;

        default:
            s_k230_state.unknown_cmd_count++;
            break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_K230Input_Init
*   功能说明: 初始化K230输入应用层
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_K230Input_Init(void)
{
    BSP_K230Uart_Init();

    memset(&s_k230_state, 0, sizeof(s_k230_state));
    s_latest_frame.x_cm = 0.0f;
    s_latest_frame.tick_ms = 0U;
    s_latest_frame_ready = false;
}

/*
*********************************************************************************************************
*   函 数 名: APP_K230Input_Task
*   功能说明: 处理K230串口接收帧
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_K230Input_Task(void)
{
    BSP_K230UartFrame_t frame;

    if (!BSP_K230Uart_GetFrame(&frame)) {
        return;
    }

    parse_frame(&frame);
}

/*
*********************************************************************************************************
*   函 数 名: APP_K230Input_GetLatestFrame
*   功能说明: 读取并清除最新K230钢球位置帧
*   形    参: frame - 输出钢球位置帧
*   返 回 值: true存在新帧，false无新帧
*********************************************************************************************************
*/
bool APP_K230Input_GetLatestFrame(APP_K230BallFrame_t *frame)
{
    if ((frame == 0) || !s_latest_frame_ready) {
        return false;
    }

    *frame = s_latest_frame;
    s_latest_frame_ready = false;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: APP_K230Input_GetState
*   功能说明: 获取K230输入状态
*   形    参: state - 状态结构体
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_K230Input_GetState(APP_K230InputState_t *state)
{
    if (state == 0) {
        return;
    }

    *state = s_k230_state;
    state->uart_rx_frame_count = BSP_K230Uart_GetReceivedFrameCount();
    state->uart_overwrite_count = BSP_K230Uart_GetOverwriteFrameCount();
}
