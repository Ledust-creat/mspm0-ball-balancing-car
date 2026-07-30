#include "bsp_servo_bus.h"
#include "ti_msp_dl_config.h"

/*
*********************************************************************************************************
*   函 数 名: BSP_ServoBus_Init
*   功能说明: 初始化舵机总线BSP，UART硬件由SysConfig统一初始化
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_ServoBus_Init(void)
{
    // UART0已由SYSCFG_DL_init()完成初始化。
}

/*
*********************************************************************************************************
*   函 数 名: BSP_ServoBus_Send
*   功能说明: 通过UART0发送舵机协议数据
*   形    参: data - 发送缓冲区；len - 数据长度
*   返 回 值: true发送完成，false参数错误或等待超时
*********************************************************************************************************
*/
bool BSP_ServoBus_Send(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint32_t timeout;

    if ((data == 0) || (len == 0U)) {
        return false;
    }

    for (i = 0U; i < len; i++) {
        timeout = BSP_SERVO_BUS_TX_TIMEOUT_LOOPS;
        while (DL_UART_Main_isBusy(UART_HTS25L_INST)) {
            if (timeout-- == 0U) {
                return false;
            }
        }

        DL_UART_Main_transmitData(UART_HTS25L_INST, data[i]);
    }

    timeout = BSP_SERVO_BUS_TX_TIMEOUT_LOOPS;
    while (DL_UART_Main_isBusy(UART_HTS25L_INST)) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}
