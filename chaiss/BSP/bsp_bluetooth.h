#ifndef BSP_BLUETOOTH_H_
#define BSP_BLUETOOTH_H_

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
#define BLUETOOTH_RX_BUF_SIZE      (8U)      /* 蓝牙固定接收帧长度 */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    uint8_t buff[BLUETOOTH_RX_BUF_SIZE];     /* 接收数据缓存 */
    uint8_t len;                             /* 接收数据长度 */
    uint8_t flag;                            /* 接收完成标志 */
} BLE_RxData_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void Bluetooth_Init(void);
void Bluetooth_SendData(const uint8_t *data, uint16_t len);
void Bluetooth_SendString(const char *str);
void Bluetooth_ReceiveData(BLE_RxData_t *rx);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BLUETOOTH_H_ */
