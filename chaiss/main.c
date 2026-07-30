#include "ti_msp_dl_config.h"
#include "app_ball_control.h"
#include "app_chassis_control.h"
#include "app_command.h"
#include "app_competition_menu.h"
#include "app_gyro_control.h"
#include "app_k230_input.h"
#include "app_line_follow.h"
#include "app_scheduler.h"
#include "bsp_bluetooth.h"
#include "bsp_key.h"
#include "bsp_k230_uart.h"
#include "dve_chassis.h"
#include "OLED.h"

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);

    OLED_Init();
    BSP_Key_Init();

    Bluetooth_Init();
    BSP_K230Uart_Init();
    DVE_Chassis_Init();
    APP_LineFollow_Init();
    APP_GyroControl_Init();
    APP_ChassisControl_Init();
    APP_BallControl_Init();
    APP_K230Input_Init();
    APP_CompetitionMenu_Init();
    APP_Scheduler_Init();

    while (1) {
        APP_Scheduler_RunOnce();
    }
}
