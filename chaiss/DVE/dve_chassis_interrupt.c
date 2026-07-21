#include "ti_msp_dl_config.h"
#include "dve_chassis.h"

/*
*********************************************************************************************************
*   函 数 名: TIMER_Chassis_Motor_Encoder_Read_INST_IRQHandler
*   功能说明: 底盘控制周期定时器中断服务函数
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void TIMER_Chassis_Motor_Encoder_Read_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_Chassis_Motor_Encoder_Read_INST)) {
        case DL_TIMER_IIDX_ZERO:
            DVE_Chassis_Timer20msIRQ();
            break;

        default:
            break;
    }
}