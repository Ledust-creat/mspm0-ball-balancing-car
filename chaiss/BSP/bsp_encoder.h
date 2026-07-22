#ifndef BSP_ENCODER_H_
#define BSP_ENCODER_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define BSP_ENCODER_DECODE_X2              (2U)        /* 二倍频：A相双边沿中断，B相判断方向 */
#define BSP_ENCODER_DECODE_MODE            BSP_ENCODER_DECODE_X2

#ifdef EncoderA_PORT
#define BSP_ENCODER_A_PORT                 EncoderA_PORT
#define BSP_ENCODER_A_PHASE_A_PIN          EncoderA_E1A_PIN
#define BSP_ENCODER_A_PHASE_A_IOMUX        EncoderA_E1A_IOMUX
#define BSP_ENCODER_A_PHASE_B_PIN          EncoderA_E1B_PIN
#define BSP_ENCODER_A_PHASE_B_IOMUX        EncoderA_E1B_IOMUX
#else
#define BSP_ENCODER_A_PORT                 GPIOB
#define BSP_ENCODER_A_PHASE_A_PIN          DL_GPIO_PIN_6
#define BSP_ENCODER_A_PHASE_A_IOMUX        IOMUX_PINCM23
#define BSP_ENCODER_A_PHASE_B_PIN          DL_GPIO_PIN_7
#define BSP_ENCODER_A_PHASE_B_IOMUX        IOMUX_PINCM24
#endif

#ifdef EncoderB_PORT
#define BSP_ENCODER_B_PORT                 EncoderB_PORT
#define BSP_ENCODER_B_PHASE_A_PIN          EncoderB_E2A_PIN
#define BSP_ENCODER_B_PHASE_A_IOMUX        EncoderB_E2A_IOMUX
#define BSP_ENCODER_B_PHASE_B_PIN          EncoderB_E2B_PIN
#define BSP_ENCODER_B_PHASE_B_IOMUX        EncoderB_E2B_IOMUX
#else
#define BSP_ENCODER_B_PORT                 GPIOB
#define BSP_ENCODER_B_PHASE_A_PIN          DL_GPIO_PIN_8
#define BSP_ENCODER_B_PHASE_A_IOMUX        IOMUX_PINCM25
#define BSP_ENCODER_B_PHASE_B_PIN          DL_GPIO_PIN_9
#define BSP_ENCODER_B_PHASE_B_IOMUX        IOMUX_PINCM26
#endif

#define BSP_ENCODER_A_PHASE_A_EDGE         DL_GPIO_PIN_6_EDGE_RISE_FALL
#define BSP_ENCODER_B_PHASE_A_EDGE         DL_GPIO_PIN_8_EDGE_RISE_FALL

#ifdef GPIO_MULTIPLE_GPIOB_INT_IRQN
#define BSP_ENCODER_GPIO_INT_IRQN          GPIO_MULTIPLE_GPIOB_INT_IRQN
#else
#define BSP_ENCODER_GPIO_INT_IRQN          GPIOB_INT_IRQn
#endif

#define BSP_ENCODER_A_SIGN                 (-1)        /* 电机A编码器方向修正，可为1或-1 */
#define BSP_ENCODER_B_SIGN                 (1)         /* 电机B编码器方向修正，可为1或-1 */

#define BSP_ENCODER_A_INT_PINS             (BSP_ENCODER_A_PHASE_A_PIN)
#define BSP_ENCODER_B_INT_PINS             (BSP_ENCODER_B_PHASE_A_PIN)
#define BSP_ENCODER_ALL_INT_PINS           (BSP_ENCODER_A_INT_PINS | BSP_ENCODER_B_INT_PINS)

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    int32_t sample_count;                  /* 本测速周期内二倍频有符号计数 */
    uint8_t direction;                     /* 0正转，1反转 */
    float speed_cm_s;                      /* 轮速，单位cm/s */
} EncoderData_t;

typedef struct
{
    EncoderData_t motorA;                  /* 电机A编码器数据 */
    EncoderData_t motorB;                  /* 电机B编码器数据 */
} Encoder_t;

/*
*********************************************************************************************************
*   全局变量声明
*********************************************************************************************************
*/
extern volatile int32_t g_MotorA_Encoder_Count;
extern volatile int32_t g_MotorB_Encoder_Count;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void Encoder_Init(void);
void Encoder_CalculateSpeed(Encoder_t *_encoder);
void BSP_Encoder_GPIOGroupIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H_ */
