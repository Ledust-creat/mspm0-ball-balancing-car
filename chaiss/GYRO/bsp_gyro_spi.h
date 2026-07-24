#ifndef BSP_GYRO_SPI_H_
#define BSP_GYRO_SPI_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define BSP_GYRO_SPI_ERROR_NONE              (0x00U)     /* SPI无错误 */
#define BSP_GYRO_SPI_ERROR_RX_TIMEOUT        (0x01U)     /* SPI接收超时 */
#define BSP_GYRO_SPI_ERROR_BUSY_TIMEOUT      (0x02U)     /* SPI忙超时 */

#define BSP_GYRO_SPI_INST                    SPI_ICM20602_INST      /* SysConfig生成的SPI实例 */
#define BSP_GYRO_SPI_TIMEOUT_COUNT           (100000UL)             /* 阻塞等待超时计数 */

#define BSP_GYRO_SPI_CS_PORT                 ICM20602_PORT          /* SysConfig生成的片选端口 */
#define BSP_GYRO_SPI_CS_PIN                  ICM20602_PIN_1_PIN     /* SysConfig生成的片选引脚 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void BSP_GyroSPI_Init(void);
void BSP_GyroSPI_CSHigh(void);
void BSP_GyroSPI_CSLow(void);
uint8_t BSP_GyroSPI_TransferByte(uint8_t tx_data);
uint8_t BSP_GyroSPI_GetError(void);
void BSP_GyroSPI_ClearError(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_GYRO_SPI_H_ */
