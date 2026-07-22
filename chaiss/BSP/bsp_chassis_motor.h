#ifndef BSP_CHASSIS_MOTOR_H_
#define BSP_CHASSIS_MOTOR_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>


/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define BSP_CHASSIS_MOTOR_PWM_MAX      (1500)      /* PWM最大比较值2500，需与SysConfig定时器周期匹配 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void BSP_ChassisMotor_Init(void);

void BSP_ChassisMotor_SetLeftPwm(int16_t pwm);
void BSP_ChassisMotor_SetRightPwm(int16_t pwm);

void BSP_ChassisMotor_StopLeft(void);
void BSP_ChassisMotor_StopRight(void);
void BSP_ChassisMotor_StopAll(void);


#endif /* BSP_CHASSIS_MOTOR_H_ */