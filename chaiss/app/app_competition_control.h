#ifndef APP_COMPETITION_CONTROL_H_
#define APP_COMPETITION_CONTROL_H_

#include "app_competition_menu.h"
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
#define APP_COMPETITION_FAULT_MODE_NOT_IMPLEMENTED     (1U)   /* 当前菜单模式尚未实现 */
#define APP_COMPETITION_FAULT_MOBILE_BALL              (2U)   /* 移动控球视觉或控制故障 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_CompetitionControl_Init(void);
void APP_CompetitionControl_Start(APP_CompetitionMode_t mode);
void APP_CompetitionControl_Task(uint32_t now_ms);
void APP_CompetitionControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMPETITION_CONTROL_H_ */
