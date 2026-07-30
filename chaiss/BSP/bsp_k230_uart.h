#ifndef BSP_K230_UART_H_
#define BSP_K230_UART_H_

#include "ti_msp_dl_config.h"
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
#define BSP_K230_UART_FRAME_LENGTH      (8U)       /* K230固定接收帧长度，单位byte */
#define BSP_K230_UART_RX_BUF_SIZE       BSP_K230_UART_FRAME_LENGTH

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    uint8_t data[BSP_K230_UART_FRAME_LENGTH];      /* 最新一帧原始接收数据，单位byte */
    uint8_t length;                                /* 当前完整帧长度，单位byte */
    uint32_t rx_complete_tick_ms;                  /* UART完整收帧时刻，单位ms */
    bool frame_ready;                              /* true表示存在未消费完整帧 */
} BSP_K230UartFrame_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void BSP_K230Uart_Init(void);
bool BSP_K230Uart_GetFrame(BSP_K230UartFrame_t *frame);
uint32_t BSP_K230Uart_GetReceivedFrameCount(void);
uint32_t BSP_K230Uart_GetOverwriteFrameCount(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_K230_UART_H_ */
