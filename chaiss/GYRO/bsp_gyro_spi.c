#include "bsp_gyro_spi.h"

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static uint8_t s_spi_error = BSP_GYRO_SPI_ERROR_NONE;     /* SPI错误状态 */

/*
*********************************************************************************************************
*   函 数 名: BSP_GyroSPI_Init
*   功能说明: 初始化陀螺仪SPI BSP状态，SPI和GPIO由SysConfig完成
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_GyroSPI_Init(void)
{
    s_spi_error = BSP_GYRO_SPI_ERROR_NONE;

    // SPI_ICM20602 和 ICM20602 片选引脚已经在 SYSCFG_DL_init() 中初始化
    BSP_GyroSPI_CSHigh();
}

/*
*********************************************************************************************************
*   函 数 名: BSP_GyroSPI_CSLow
*   功能说明: 拉低片选，开始SPI访问
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_GyroSPI_CSLow(void)
{
    DL_GPIO_clearPins(BSP_GYRO_SPI_CS_PORT, BSP_GYRO_SPI_CS_PIN);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_GyroSPI_CSHigh
*   功能说明: 等待SPI空闲后拉高片选，结束SPI访问
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_GyroSPI_CSHigh(void)
{
    uint32_t timeout = BSP_GYRO_SPI_TIMEOUT_COUNT;

    while (DL_SPI_isBusy(BSP_GYRO_SPI_INST)) {
        if (timeout == 0UL) {
            s_spi_error = BSP_GYRO_SPI_ERROR_BUSY_TIMEOUT;
            break;
        }
        timeout--;
    }

    DL_GPIO_setPins(BSP_GYRO_SPI_CS_PORT, BSP_GYRO_SPI_CS_PIN);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_GyroSPI_TransferByte
*   功能说明: SPI全双工传输1字节
*   形    参: tx_data - 待发送数据
*   返 回 值: 同时接收到的数据
*********************************************************************************************************
*/
uint8_t BSP_GyroSPI_TransferByte(uint8_t tx_data)
{
    uint8_t rx_data;
    uint32_t timeout;

    while (DL_SPI_receiveDataCheck8(BSP_GYRO_SPI_INST, &rx_data)) {
    }

    DL_SPI_transmitDataBlocking8(BSP_GYRO_SPI_INST, tx_data);

    timeout = BSP_GYRO_SPI_TIMEOUT_COUNT;
    while (!DL_SPI_receiveDataCheck8(BSP_GYRO_SPI_INST, &rx_data)) {
        if (timeout == 0UL) {
            s_spi_error = BSP_GYRO_SPI_ERROR_RX_TIMEOUT;
            return 0xFFU;
        }
        timeout--;
    }

    return rx_data;
}

/*
*********************************************************************************************************
*   函 数 名: BSP_GyroSPI_GetError
*   功能说明: 获取SPI错误状态
*   形    参: 无
*   返 回 值: SPI错误状态
*********************************************************************************************************
*/
uint8_t BSP_GyroSPI_GetError(void)
{
    return s_spi_error;
}

/*
*********************************************************************************************************
*   函 数 名: BSP_GyroSPI_ClearError
*   功能说明: 清除SPI错误状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_GyroSPI_ClearError(void)
{
    s_spi_error = BSP_GYRO_SPI_ERROR_NONE;
}
