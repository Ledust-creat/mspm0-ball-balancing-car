#include "bsp_encoder.h"

/*
*********************************************************************************************************
*   函 数 名: encoder_count_x2
*   功能说明: 根据A相双边沿中断和B相电平完成二倍频计数
*   形    参: port — GPIO端口；phase_a_pin — A相引脚；phase_b_pin — B相引脚；count — 计数变量
*   返 回 值: 无
*********************************************************************************************************
*/
static void encoder_count_x2(GPIO_Regs *phase_a_port,
                             uint32_t phase_a_pin,
                             GPIO_Regs *phase_b_port,
                             uint32_t phase_b_pin,
                             volatile int32_t *count)
{
    uint32_t phase_a;
    uint32_t phase_b;

    phase_a = DL_GPIO_readPins(phase_a_port, phase_a_pin);
    phase_b = DL_GPIO_readPins(phase_b_port, phase_b_pin);

    if ((phase_a != 0U) == (phase_b != 0U)) {
        (*count)--;
    } else {
        (*count)++;
    }
}

/*
*********************************************************************************************************
*   函 数 名: BSP_Encoder_GPIOGroupIRQHandler
*   功能说明: 编码器GPIO组中断回调，由GROUP1_IRQHandler统一转发
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_Encoder_GPIOGroupIRQHandler(void)
{
    uint32_t motor_a_status;
    uint32_t motor_b_status;

#if (BSP_ENCODER_DECODE_MODE == BSP_ENCODER_DECODE_X2)
    motor_a_status = DL_GPIO_getEnabledInterruptStatus(BSP_ENCODER_A_PHASE_A_PORT,
                                                       BSP_ENCODER_A_INT_PINS);
    motor_b_status = DL_GPIO_getEnabledInterruptStatus(BSP_ENCODER_B_PHASE_A_PORT,
                                                       BSP_ENCODER_B_INT_PINS);
	
    if ((motor_a_status & BSP_ENCODER_A_INT_PINS) != 0U) {
        encoder_count_x2(BSP_ENCODER_A_PHASE_A_PORT,
                         BSP_ENCODER_A_PHASE_A_PIN,
                         BSP_ENCODER_A_PHASE_B_PORT,
                         BSP_ENCODER_A_PHASE_B_PIN,
                         &g_MotorA_Encoder_Count);
        DL_GPIO_clearInterruptStatus(BSP_ENCODER_A_PHASE_A_PORT, BSP_ENCODER_A_INT_PINS);
    }

    if ((motor_b_status & BSP_ENCODER_B_INT_PINS) != 0U) {
        encoder_count_x2(BSP_ENCODER_B_PHASE_A_PORT,
                         BSP_ENCODER_B_PHASE_A_PIN,
                         BSP_ENCODER_B_PHASE_B_PORT,
                         BSP_ENCODER_B_PHASE_B_PIN,
                         &g_MotorB_Encoder_Count);
        DL_GPIO_clearInterruptStatus(BSP_ENCODER_B_PHASE_A_PORT, BSP_ENCODER_B_INT_PINS);
    }
#else
#error "Unsupported BSP_ENCODER_DECODE_MODE"
#endif
}
