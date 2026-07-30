#ifndef APP_K230_INPUT_H_
#define APP_K230_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_K230_FRAME_HEAD                 (0xA5U)     /* K230数据帧头 */
#define APP_K230_FRAME_TAIL                 (0x5AU)     /* K230数据帧尾 */
#define APP_K230_PACKET_LEN                 (8U)        /* K230固定帧长度，单位byte */
#define APP_K230_CMD_BALL_X_CM              (0x20U)     /* 钢球X位置命令，DATA=float，单位cm */
#define APP_K230_POSITION_LIMIT_CM          (50.0f)     /* K230位置输入合理范围，单位cm */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    float x_cm;                             /* 钢球X位置，单位cm，右侧为正 */
    uint32_t tick_ms;                       /* UART完整接收该帧时刻，单位ms */
} APP_K230BallFrame_t;

typedef struct
{
    uint32_t rx_frame_count;                /* APP层处理完整帧计数，单位frame */
    uint32_t valid_position_count;          /* 有效位置帧计数，单位frame */
    uint32_t checksum_error_count;          /* 校验错误计数，单位frame */
    uint32_t format_error_count;            /* 格式错误计数，单位frame */
    uint32_t unknown_cmd_count;             /* 未知命令计数，单位frame */
    uint32_t uart_rx_frame_count;           /* UART层完整收帧计数，单位frame */
    uint32_t uart_overwrite_count;          /* UART邮箱覆盖计数，单位frame */
    float last_x_cm;                        /* 最近一次有效X位置，单位cm */
    uint8_t data_valid;                     /* 1表示收到过有效位置帧 */
} APP_K230InputState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_K230Input_Init(void);
void APP_K230Input_Task(void);
bool APP_K230Input_GetLatestFrame(APP_K230BallFrame_t *frame);
void APP_K230Input_GetState(APP_K230InputState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* APP_K230_INPUT_H_ */
