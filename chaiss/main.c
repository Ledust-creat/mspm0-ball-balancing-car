#include "ti_msp_dl_config.h"
#include "app_ball_control.h"
#include "app_chassis_control.h"
#include "app_command.h"
#include "app_competition_control.h"
#include "app_competition_menu.h"
#include "app_gyro_control.h"
#include "app_k230_input.h"
#include "app_scheduler.h"

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);
	
    APP_Command_Init();
    APP_K230Input_Init();
    APP_ChassisControl_Init();
    APP_GyroControl_Init();
    APP_BallControl_Init();
    APP_CompetitionMenu_Init();
    APP_CompetitionControl_Init();
    APP_Scheduler_Init();
                      // 立即启动纯速度环输出
    while (1) {
        APP_Scheduler_RunOnce();
    }
}
