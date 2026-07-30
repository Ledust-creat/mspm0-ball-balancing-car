#include "bsp_k230_uart.h"
#include "interrupt.h"
#include <string.h>

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static volatile uint8_t s_rx_buffer[BSP_K230_UART_FRAME_LENGTH];  /* ISR组帧缓存，单位byte */
static volatile uint8_t s_rx_length = 0U;                         /* ISR当前组帧长度，单位byte */
static volatile BSP_K230UartFrame_t s_rx_mailbox;                 /* 最新完整帧邮箱 */
static volatile uint32_t s_received_frame_count = 0U;             /* 成功接收完整帧总数 */
static volatile uint32_t s_overwrite_frame_count = 0U;            /* 未消费旧帧被新帧覆盖次数 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static void k230_uart_clear_rx(void);

/*
*********************************************************************************************************
*   函 数 名: k230_uart_clear_rx
*   功能说明: 清除K230串口接收组帧状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void k230_uart_clear_rx(void)
{
    s_rx_length = 0U;
}

/*
*********************************************************************************************************
*   函 数 名: BSP_K230Uart_Init
*   功能说明: 初始化K230串口接收中断，UART硬件初始化由SysConfig完成
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_K230Uart_Init(void)
{
    k230_uart_clear_rx();
    s_rx_mailbox.length = 0U;
    s_rx_mailbox.rx_complete_tick_ms = 0U;
    s_rx_mailbox.frame_ready = false;
    s_received_frame_count = 0U;
    s_overwrite_frame_count = 0U;

    NVIC_ClearPendingIRQ(UART_HTS25L_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_HTS25L_INST_INT_IRQN);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_K230Uart_GetFrame
*   功能说明: 读取并清除K230 UART完整收帧邮箱
*   形    参: frame - 输出完整接收帧
*   返 回 值: true存在新帧，false无新帧或参数无效
*********************************************************************************************************
*/
bool BSP_K230Uart_GetFrame(BSP_K230UartFrame_t *frame)
{
    if (frame == 0) {
        return false;
    }

    __disable_irq();
    if (!s_rx_mailbox.frame_ready) {
        __enable_irq();
        return false;
    }

    memcpy(frame->data,
           (const void *)s_rx_mailbox.data,
           BSP_K230_UART_FRAME_LENGTH);
    frame->length = s_rx_mailbox.length;
    frame->rx_complete_tick_ms = s_rx_mailbox.rx_complete_tick_ms;
    frame->frame_ready = s_rx_mailbox.frame_ready;
    s_rx_mailbox.frame_ready = false;
    __enable_irq();

    return true;
}

uint32_t BSP_K230Uart_GetReceivedFrameCount(void)
{
    return s_received_frame_count;
}

uint32_t BSP_K230Uart_GetOverwriteFrameCount(void)
{
    return s_overwrite_frame_count;
}

/*
*********************************************************************************************************
*   函 数 名: UART_HTS25L_INST_IRQHandler
*   功能说明: UART0中断服务函数，接收K230固定8字节视觉帧
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void UART_HTS25L_INST_IRQHandler(void)
{
    uint8_t data;

    switch (DL_UART_getPendingInterrupt(UART_HTS25L_INST)) {
        case DL_UART_IIDX_RX:
            data = DL_UART_Main_receiveData(UART_HTS25L_INST);

            if ((s_rx_length == 0U) && (data != 0xA5U)) {
                break; // 帧头同步，避免从半包开始接收
            }

            s_rx_buffer[s_rx_length++] = data;
            if (s_rx_length >= BSP_K230_UART_FRAME_LENGTH) {
                if (s_rx_mailbox.frame_ready) {
                    s_overwrite_frame_count++;
                }

                memcpy((void *)s_rx_mailbox.data,
                       (const void *)s_rx_buffer,
                       BSP_K230_UART_FRAME_LENGTH);
                s_rx_mailbox.length = BSP_K230_UART_FRAME_LENGTH;
                s_rx_mailbox.rx_complete_tick_ms = Get_Tick();
                s_rx_mailbox.frame_ready = true;
                s_received_frame_count++;
                k230_uart_clear_rx();
            }
            break;

        default:
            break;
    }
}
