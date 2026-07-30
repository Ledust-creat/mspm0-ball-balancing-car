#include "ti_msp_dl_config.h"
#include "app_ball_control.h"
#include "app_chassis_control.h"
#include "app_command.h"
#include "app_competition_menu.h"
#include "app_gyro_control.h"
#include "app_line_follow.h"
#include "bsp_bluetooth.h"
#include "bsp_key.h"
#include "dve_chassis.h"
#include "OLED.h"

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static APP_CompetitionMode_t s_current_comp_mode = APP_COMP_MODE_TRACK_LAP; /* 当前菜单选择模式 */

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);

    OLED_Init();
    BSP_Key_Init();

    Bluetooth_Init();
    DVE_Chassis_Init();
    APP_LineFollow_Init();
    APP_GyroControl_Init();
    APP_ChassisControl_Init();
    APP_BallControl_Init();
    APP_CompetitionMenu_Init();
	
    while (1) {
        APP_Command_Task();
        APP_ChassisControl_Task();
        APP_BallControl_Task();
        APP_CompetitionMenu_Task();

        if (APP_CompetitionMenu_ConsumeStartRequest(&s_current_comp_mode)) {
            /*
             * 当前只保存模式，后续由比赛控制APP接管具体小题启动。
             */
        }

        if (APP_CompetitionMenu_ConsumeStopRequest()) {
            APP_ChassisControl_Stop();
            APP_BallControl_SetEnable(0U);
        }
    }
}
