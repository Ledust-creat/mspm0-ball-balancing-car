#include "bsp_bluetooth.h"
#include <stdbool.h>
#include <string.h>

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static volatile uint8_t s_rx_buf[BLUETOOTH_RX_BUF_SIZE];       /* 中断接收缓存 */
static volatile uint8_t s_rx_len = 0U;                         /* 当前接收长度 */
static volatile uint8_t s_rx_flag = 0U;                        /* 一帧接收完成标志 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static void bluetooth_send_byte(uint8_t data);
static void bluetooth_clear_rx(void);

/*
*********************************************************************************************************
*   函 数 名: bluetooth_send_byte
*   功能说明: 蓝牙串口发送单字节
*   形    参: data — 待发送字节
*   返 回 值: 无
*********************************************************************************************************
*/
static void bluetooth_send_byte(uint8_t data)
{
    while (DL_UART_isBusy(UART_BLUETOOTH_INST) == true) {
    }

    DL_UART_Main_transmitData(UART_BLUETOOTH_INST, data);
}

/*
*********************************************************************************************************
*   函 数 名: bluetooth_clear_rx
*   功能说明: 清除蓝牙接收状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void bluetooth_clear_rx(void)
{
    s_rx_len = 0U;
    s_rx_flag = 0U;
}

/*
*********************************************************************************************************
*   函 数 名: Bluetooth_Init
*   功能说明: 初始化蓝牙串口中断
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void Bluetooth_Init(void)
{
    bluetooth_clear_rx();

    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
}

/*
*********************************************************************************************************
*   函 数 名: Bluetooth_SendData
*   功能说明: 蓝牙串口发送指定长度数据
*   形    参: data — 数据缓存；len — 数据长度
*   返 回 值: 无
*********************************************************************************************************
*/
void Bluetooth_SendData(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U)) {
        return;
    }

    while (len > 0U) {
        bluetooth_send_byte(*data);
        data++;
        len--;
    }
}

/*
*********************************************************************************************************
*   函 数 名: Bluetooth_SendString
*   功能说明: 蓝牙串口发送字符串
*   形    参: str — 待发送字符串
*   返 回 值: 无
*********************************************************************************************************
*/
void Bluetooth_SendString(const char *str)
{
    if (str == 0) {
        return;
    }

    while (*str != '\0') {
        bluetooth_send_byte((uint8_t)*str);
        str++;
    }
}

/*
*********************************************************************************************************
*   函 数 名: Bluetooth_ReceiveData
*   功能说明: 读取蓝牙接收完成的一帧数据
*   形    参: rx — 接收数据结构体
*   返 回 值: 无
*********************************************************************************************************
*/
void Bluetooth_ReceiveData(BLE_RxData_t *rx)
{
    uint8_t len;

    if (rx == 0) {
        return;
    }

    rx->flag = 0U;
    rx->len = 0U;

    __disable_irq();
    if (s_rx_flag != 0U) {
        len = s_rx_len;
        if (len > BLUETOOTH_RX_BUF_SIZE) {
            len = BLUETOOTH_RX_BUF_SIZE;
        }

        memcpy(rx->buff, (const void *)s_rx_buf, len);
        rx->len = len;
        rx->flag = 1U;

        bluetooth_clear_rx();
    }
    __enable_irq();
}

/*
*********************************************************************************************************
*   函 数 名: UART_BLUETOOTH_INST_IRQHandler
*   功能说明: 蓝牙串口中断服务函数，接收固定长度命令帧
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void UART_BLUETOOTH_INST_IRQHandler(void)
{
    uint8_t data;

    switch (DL_UART_getPendingInterrupt(UART_BLUETOOTH_INST)) {
        case DL_UART_IIDX_RX:
            data = DL_UART_Main_receiveData(UART_BLUETOOTH_INST);

            if (s_rx_flag != 0U) {
                break;
            }

            s_rx_buf[s_rx_len++] = data;
            if (s_rx_len >= BLUETOOTH_RX_BUF_SIZE) {
                s_rx_flag = 1U;
            }
            break;

        default:
            break;
    }
}
