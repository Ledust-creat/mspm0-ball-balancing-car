#ifndef BSP_SERVO_BUS_H_
#define BSP_SERVO_BUS_H_

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
#define BSP_SERVO_BUS_TX_TIMEOUT_LOOPS     (80000U)   /* 单字节发送等待上限 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void BSP_ServoBus_Init(void);
bool BSP_ServoBus_Send(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_BUS_H_ */
