/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2000
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for Chassis_Motor_PWM */
#define Chassis_Motor_PWM_INST                                             TIMA1
#define Chassis_Motor_PWM_INST_IRQHandler                        TIMA1_IRQHandler
#define Chassis_Motor_PWM_INST_INT_IRQN                         (TIMA1_INT_IRQn)
#define Chassis_Motor_PWM_INST_CLK_FREQ                                 80000000
/* GPIO defines for channel 0 */
#define GPIO_Chassis_Motor_PWM_C0_PORT                                     GPIOA
#define GPIO_Chassis_Motor_PWM_C0_PIN                             DL_GPIO_PIN_17
#define GPIO_Chassis_Motor_PWM_C0_IOMUX                          (IOMUX_PINCM39)
#define GPIO_Chassis_Motor_PWM_C0_IOMUX_FUNC             IOMUX_PINCM39_PF_TIMA1_CCP0
#define GPIO_Chassis_Motor_PWM_C0_IDX                        DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_Chassis_Motor_PWM_C1_PORT                                     GPIOB
#define GPIO_Chassis_Motor_PWM_C1_PIN                             DL_GPIO_PIN_18
#define GPIO_Chassis_Motor_PWM_C1_IOMUX                          (IOMUX_PINCM44)
#define GPIO_Chassis_Motor_PWM_C1_IOMUX_FUNC             IOMUX_PINCM44_PF_TIMA1_CCP1
#define GPIO_Chassis_Motor_PWM_C1_IDX                        DL_TIMER_CC_1_INDEX



/* Defines for TIMER_Chassis_Motor_Encoder_Read */
#define TIMER_Chassis_Motor_Encoder_Read_INST                                 (TIMG0)
#define TIMER_Chassis_Motor_Encoder_Read_INST_IRQHandler                        TIMG0_IRQHandler
#define TIMER_Chassis_Motor_Encoder_Read_INST_INT_IRQN                        (TIMG0_INT_IRQn)
#define TIMER_Chassis_Motor_Encoder_Read_INST_LOAD_VALUE                                 (3124U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                400000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                        DL_GPIO_PIN_28
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM3)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                        DL_GPIO_PIN_31
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM6)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM6_PF_I2C0_SCL


/* Defines for UART_BLUETOOTH */
#define UART_BLUETOOTH_INST                                                UART2
#define UART_BLUETOOTH_INST_FREQUENCY                                   40000000
#define UART_BLUETOOTH_INST_IRQHandler                          UART2_IRQHandler
#define UART_BLUETOOTH_INST_INT_IRQN                              UART2_INT_IRQn
#define GPIO_UART_BLUETOOTH_RX_PORT                                        GPIOA
#define GPIO_UART_BLUETOOTH_TX_PORT                                        GPIOA
#define GPIO_UART_BLUETOOTH_RX_PIN                                DL_GPIO_PIN_24
#define GPIO_UART_BLUETOOTH_TX_PIN                                DL_GPIO_PIN_23
#define GPIO_UART_BLUETOOTH_IOMUX_RX                             (IOMUX_PINCM54)
#define GPIO_UART_BLUETOOTH_IOMUX_TX                             (IOMUX_PINCM53)
#define GPIO_UART_BLUETOOTH_IOMUX_RX_FUNC               IOMUX_PINCM54_PF_UART2_RX
#define GPIO_UART_BLUETOOTH_IOMUX_TX_FUNC               IOMUX_PINCM53_PF_UART2_TX
#define UART_BLUETOOTH_BAUD_RATE                                        (115200)
#define UART_BLUETOOTH_IBRD_40_MHZ_115200_BAUD                              (21)
#define UART_BLUETOOTH_FBRD_40_MHZ_115200_BAUD                              (45)
/* Defines for UART_HTS25L */
#define UART_HTS25L_INST                                                   UART0
#define UART_HTS25L_INST_FREQUENCY                                      40000000
#define UART_HTS25L_INST_IRQHandler                             UART0_IRQHandler
#define UART_HTS25L_INST_INT_IRQN                                 UART0_INT_IRQn
#define GPIO_UART_HTS25L_RX_PORT                                           GPIOA
#define GPIO_UART_HTS25L_TX_PORT                                           GPIOA
#define GPIO_UART_HTS25L_RX_PIN                                   DL_GPIO_PIN_11
#define GPIO_UART_HTS25L_TX_PIN                                   DL_GPIO_PIN_10
#define GPIO_UART_HTS25L_IOMUX_RX                                (IOMUX_PINCM22)
#define GPIO_UART_HTS25L_IOMUX_TX                                (IOMUX_PINCM21)
#define GPIO_UART_HTS25L_IOMUX_RX_FUNC                 IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_HTS25L_IOMUX_TX_FUNC                 IOMUX_PINCM21_PF_UART0_TX
#define UART_HTS25L_BAUD_RATE                                           (115200)
#define UART_HTS25L_IBRD_40_MHZ_115200_BAUD                                 (21)
#define UART_HTS25L_FBRD_40_MHZ_115200_BAUD                                 (45)




/* Defines for SPI_ICM20602 */
#define SPI_ICM20602_INST                                                  SPI1
#define SPI_ICM20602_INST_IRQHandler                            SPI1_IRQHandler
#define SPI_ICM20602_INST_INT_IRQN                                SPI1_INT_IRQn
#define GPIO_SPI_ICM20602_PICO_PORT                                       GPIOB
#define GPIO_SPI_ICM20602_PICO_PIN                                DL_GPIO_PIN_8
#define GPIO_SPI_ICM20602_IOMUX_PICO                            (IOMUX_PINCM25)
#define GPIO_SPI_ICM20602_IOMUX_PICO_FUNC            IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_ICM20602_POCI_PORT                                       GPIOB
#define GPIO_SPI_ICM20602_POCI_PIN                                DL_GPIO_PIN_7
#define GPIO_SPI_ICM20602_IOMUX_POCI                            (IOMUX_PINCM24)
#define GPIO_SPI_ICM20602_IOMUX_POCI_FUNC            IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI_ICM20602 */
#define GPIO_SPI_ICM20602_SCLK_PORT                                       GPIOB
#define GPIO_SPI_ICM20602_SCLK_PIN                                DL_GPIO_PIN_9
#define GPIO_SPI_ICM20602_IOMUX_SCLK                            (IOMUX_PINCM26)
#define GPIO_SPI_ICM20602_IOMUX_SCLK_FUNC            IOMUX_PINCM26_PF_SPI1_SCLK



/* Port definition for Pin Group led */
#define led_PORT                                                         (GPIOB)

/* Defines for PIN_0: GPIOB.27 with pinCMx 58 on package pin 29 */
#define led_PIN_0_PIN                                           (DL_GPIO_PIN_27)
#define led_PIN_0_IOMUX                                          (IOMUX_PINCM58)
/* Port definition for Pin Group ICM20602 */
#define ICM20602_PORT                                                    (GPIOB)

/* Defines for PIN_1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define ICM20602_PIN_1_PIN                                       (DL_GPIO_PIN_6)
#define ICM20602_PIN_1_IOMUX                                     (IOMUX_PINCM23)
/* Port definition for Pin Group Chassis_MotorA */
#define Chassis_MotorA_PORT                                              (GPIOB)

/* Defines for AIN1: GPIOB.19 with pinCMx 45 on package pin 16 */
#define Chassis_MotorA_AIN1_PIN                                 (DL_GPIO_PIN_19)
#define Chassis_MotorA_AIN1_IOMUX                                (IOMUX_PINCM45)
/* Defines for AIN2: GPIOB.20 with pinCMx 48 on package pin 19 */
#define Chassis_MotorA_AIN2_PIN                                 (DL_GPIO_PIN_20)
#define Chassis_MotorA_AIN2_IOMUX                                (IOMUX_PINCM48)
/* Defines for BIN1: GPIOA.2 with pinCMx 7 on package pin 42 */
#define Chassis_MotorB_BIN1_PORT                                         (GPIOA)
#define Chassis_MotorB_BIN1_PIN                                  (DL_GPIO_PIN_2)
#define Chassis_MotorB_BIN1_IOMUX                                 (IOMUX_PINCM7)
/* Defines for BIN2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define Chassis_MotorB_BIN2_PORT                                         (GPIOB)
#define Chassis_MotorB_BIN2_PIN                                 (DL_GPIO_PIN_24)
#define Chassis_MotorB_BIN2_IOMUX                                (IOMUX_PINCM52)
/* Defines for D1: GPIOA.30 with pinCMx 5 on package pin 37 */
#define Digital_D1_PORT                                                  (GPIOA)
#define Digital_D1_PIN                                          (DL_GPIO_PIN_30)
#define Digital_D1_IOMUX                                          (IOMUX_PINCM5)
/* Defines for D2: GPIOB.21 with pinCMx 49 on package pin 20 */
#define Digital_D2_PORT                                                  (GPIOB)
#define Digital_D2_PIN                                          (DL_GPIO_PIN_21)
#define Digital_D2_IOMUX                                         (IOMUX_PINCM49)
/* Defines for D3: GPIOB.17 with pinCMx 43 on package pin 14 */
#define Digital_D3_PORT                                                  (GPIOB)
#define Digital_D3_PIN                                          (DL_GPIO_PIN_17)
#define Digital_D3_IOMUX                                         (IOMUX_PINCM43)
/* Defines for D4: GPIOA.16 with pinCMx 38 on package pin 9 */
#define Digital_D4_PORT                                                  (GPIOA)
#define Digital_D4_PIN                                          (DL_GPIO_PIN_16)
#define Digital_D4_IOMUX                                         (IOMUX_PINCM38)
/* Defines for D5: GPIOA.14 with pinCMx 36 on package pin 7 */
#define Digital_D5_PORT                                                  (GPIOA)
#define Digital_D5_PIN                                          (DL_GPIO_PIN_14)
#define Digital_D5_IOMUX                                         (IOMUX_PINCM36)
/* Defines for D6: GPIOB.25 with pinCMx 56 on package pin 27 */
#define Digital_D6_PORT                                                  (GPIOB)
#define Digital_D6_PIN                                          (DL_GPIO_PIN_25)
#define Digital_D6_IOMUX                                         (IOMUX_PINCM56)
/* Defines for D7: GPIOA.25 with pinCMx 55 on package pin 26 */
#define Digital_D7_PORT                                                  (GPIOA)
#define Digital_D7_PIN                                          (DL_GPIO_PIN_25)
#define Digital_D7_IOMUX                                         (IOMUX_PINCM55)
/* Defines for D8: GPIOA.27 with pinCMx 60 on package pin 31 */
#define Digital_D8_PORT                                                  (GPIOA)
#define Digital_D8_PIN                                          (DL_GPIO_PIN_27)
#define Digital_D8_IOMUX                                         (IOMUX_PINCM60)
/* Defines for E1A: GPIOB.26 with pinCMx 57 on package pin 28 */
#define EncoderA_E1A_PORT                                                (GPIOB)
// pins affected by this interrupt request:["E1A"]
#define EncoderA_INT_IRQN                                       (GPIOB_INT_IRQn)
#define EncoderA_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define EncoderA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO26)
#define EncoderA_E1A_PIN                                        (DL_GPIO_PIN_26)
#define EncoderA_E1A_IOMUX                                       (IOMUX_PINCM57)
/* Defines for E1B: GPIOA.12 with pinCMx 34 on package pin 5 */
#define EncoderA_E1B_PORT                                                (GPIOA)
#define EncoderA_E1B_PIN                                        (DL_GPIO_PIN_12)
#define EncoderA_E1B_IOMUX                                       (IOMUX_PINCM34)
/* Defines for E2A: GPIOA.13 with pinCMx 35 on package pin 6 */
#define EncoderB_E2A_PORT                                                (GPIOA)
// pins affected by this interrupt request:["E2A"]
#define EncoderB_INT_IRQN                                       (GPIOA_INT_IRQn)
#define EncoderB_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define EncoderB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO13)
#define EncoderB_E2A_PIN                                        (DL_GPIO_PIN_13)
#define EncoderB_E2A_IOMUX                                       (IOMUX_PINCM35)
/* Defines for E2B: GPIOB.23 with pinCMx 51 on package pin 22 */
#define EncoderB_E2B_PORT                                                (GPIOB)
#define EncoderB_E2B_PIN                                        (DL_GPIO_PIN_23)
#define EncoderB_E2B_IOMUX                                       (IOMUX_PINCM51)
/* Defines for SELECT: GPIOA.9 with pinCMx 20 on package pin 55 */
#define KEY_SELECT_PORT                                                  (GPIOA)
#define KEY_SELECT_PIN                                           (DL_GPIO_PIN_9)
#define KEY_SELECT_IOMUX                                         (IOMUX_PINCM20)
/* Defines for CONFIRM: GPIOA.8 with pinCMx 19 on package pin 54 */
#define KEY_CONFIRM_PORT                                                 (GPIOA)
#define KEY_CONFIRM_PIN                                          (DL_GPIO_PIN_8)
#define KEY_CONFIRM_IOMUX                                        (IOMUX_PINCM19)
/* Defines for ADD: GPIOB.5 with pinCMx 18 on package pin 53 */
#define KEY_ADD_PORT                                                     (GPIOB)
#define KEY_ADD_PIN                                              (DL_GPIO_PIN_5)
#define KEY_ADD_IOMUX                                            (IOMUX_PINCM18)
/* Defines for SUB: GPIOB.4 with pinCMx 17 on package pin 52 */
#define KEY_SUB_PORT                                                     (GPIOB)
#define KEY_SUB_PIN                                              (DL_GPIO_PIN_4)
#define KEY_SUB_IOMUX                                            (IOMUX_PINCM17)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_Chassis_Motor_PWM_init(void);
void SYSCFG_DL_TIMER_Chassis_Motor_Encoder_Read_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_UART_BLUETOOTH_init(void);
void SYSCFG_DL_UART_HTS25L_init(void);
void SYSCFG_DL_SPI_ICM20602_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
