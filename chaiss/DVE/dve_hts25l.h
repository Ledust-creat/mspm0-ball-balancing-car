#ifndef DVE_HTS25L_H_
#define DVE_HTS25L_H_

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
#define DVE_HTS25L_DEFAULT_ID              (0xFEU)    /* 单舵机总线使用广播ID */
#define DVE_HTS25L_POS_MIN                 (270U)     /* 机械安全最小位置 */
#define DVE_HTS25L_POS_CENTER              (500U)     /* 舵机中位 */
#define DVE_HTS25L_POS_MAX                 (650U)     /* 机械安全最大位置 */
#define DVE_HTS25L_MOVE_TIME_MS            (30U)      /* 闭环命令默认运动时间 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void DVE_HTS25L_Init(void);
bool DVE_HTS25L_SetPosition(uint16_t pos, uint16_t time_ms);
uint16_t DVE_HTS25L_GetCommandPosition(void);

#ifdef __cplusplus
}
#endif

#endif /* DVE_HTS25L_H_ */
