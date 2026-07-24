#include "ti_msp_dl_config.h"
#include "dve_chassis.h"
#include "app_line_follow.h"
#include "app_chassis_control.h"
#include "app_gyro_control.h"
#include "app_command.h"
#include "bsp_bluetooth.h"
#include "bsp_chassis_motor.h"
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);
	
    Bluetooth_Init();	
    DVE_Chassis_Init();
    APP_LineFollow_Init();
    APP_GyroControl_Init();
    APP_ChassisControl_Init();
    
    while (1) {
        APP_Command_Task();
        APP_ChassisControl_Task();

    }
}
