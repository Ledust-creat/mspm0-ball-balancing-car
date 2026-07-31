#include "app_scheduler.h"
#include "app_ball_control.h"
#include "app_chassis_control.h"
#include "app_command.h"
#include "app_competition_control.h"
#include "app_competition_menu.h"
#include "app_gyro_control.h"
#include "app_k230_input.h"
#include "interrupt.h"

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static APP_CompetitionMode_t s_current_mode;     /* 当前菜单选择模式，后续比赛状态机接管 */

/*
*********************************************************************************************************
*   函 数 名: APP_Scheduler_Init
*   功能说明: 初始化统一任务调度器状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_Scheduler_Init(void)
{
    s_current_mode = APP_COMP_MODE_TRACK_LAP;
}

/*
*********************************************************************************************************
*   函 数 名: APP_Scheduler_RunOnce
*   功能说明: 按优先级执行一次主循环任务调度
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_Scheduler_RunOnce(void)
{
    uint32_t now_ms;
    APP_K230BallFrame_t frame;

    now_ms = Get_Tick();

    APP_K230Input_Task();                 // 先消费UART0接收邮箱，发布最新K230帧
    APP_GyroControl_SensorTask10ms(now_ms); // IMU按10ms独立采样并更新缓存

    if (APP_K230Input_GetLatestFrame(&frame)) {
        APP_BallControl_UpdateFromVision(&frame); // 新视觉帧到达后执行一次控球
    }

    APP_BallControl_SafetyTask();         // 无新帧时只做视觉超时保护
    APP_ChassisControl_Task();            // 内部仍只消费20ms硬件控制标志
    APP_CompetitionMenu_UpdateTask();     // 只处理按键和UI状态，不访问OLED

    if (APP_CompetitionMenu_ConsumeStartRequest(&s_current_mode)) {
        APP_CompetitionControl_Start(s_current_mode);
    }

    if (APP_CompetitionMenu_ConsumeStopRequest()) {
        APP_CompetitionControl_Stop();
    }

    APP_CompetitionControl_Task(now_ms);

    APP_Command_Task();                   // 蓝牙显式命令和低频状态回传
    APP_CompetitionMenu_DisplayTask();    // OLED最低优先级执行
}
