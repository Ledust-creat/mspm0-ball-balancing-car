#include "ti_msp_dl_config.h"
#include "dve_chassis.h"
#include "app_trace.h"
#include "app_command.h"
#include "bsp_bluetooth.h"

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);

    DVE_Chassis_Init();
	APP_Trace_Init();
    Bluetooth_Init();
    
    while (1) {
        APP_Command_Task();
        APP_Trace_Task();
    }
}
