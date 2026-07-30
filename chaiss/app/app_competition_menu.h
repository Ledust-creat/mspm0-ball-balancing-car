#ifndef APP_COMPETITION_MENU_H_
#define APP_COMPETITION_MENU_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef enum
{
    APP_COMP_MODE_TRACK_LAP = 0,          /* 循迹一圈并停车 */
    APP_COMP_MODE_STATIC_RECIPROCATE,     /* 静态O点、+5cm、-5cm控球 */
    APP_COMP_MODE_AB_CENTER,              /* AB段行驶，钢球保持中心 */
    APP_COMP_MODE_LAP_CENTER,             /* 整圈行驶，钢球保持中心 */
    APP_COMP_MODE_LAP_TARGET,             /* 整圈行驶，钢球保持指定位置 */

    APP_COMP_MODE_COUNT                   /* 菜单模式数量 */
} APP_CompetitionMode_t;

typedef enum
{
    APP_COMP_UI_SELECT = 0,               /* 小题选择页 */
    APP_COMP_UI_COUNTDOWN,                /* 3秒倒计时页 */
    APP_COMP_UI_RUNNING,                  /* 运行页 */
    APP_COMP_UI_FINISHED,                 /* 外部通知完成页 */
    APP_COMP_UI_FAULT                     /* 外部通知故障页 */
} APP_CompetitionUiState_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_CompetitionMenu_Init(void);
void APP_CompetitionMenu_Task(void);
void APP_CompetitionMenu_UpdateTask(void);
void APP_CompetitionMenu_DisplayTask(void);

bool APP_CompetitionMenu_ConsumeStartRequest(APP_CompetitionMode_t *mode);
bool APP_CompetitionMenu_ConsumeStopRequest(void);

void APP_CompetitionMenu_NotifyFinished(void);
void APP_CompetitionMenu_NotifyFault(uint8_t fault_code);

APP_CompetitionMode_t APP_CompetitionMenu_GetSelectedMode(void);
APP_CompetitionUiState_t APP_CompetitionMenu_GetUiState(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMPETITION_MENU_H_ */
