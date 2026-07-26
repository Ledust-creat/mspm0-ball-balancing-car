#include "bsp_chassis_motor.h"

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static int16_t pwm_limit(int16_t pwm);
static int16_t abs_i16(int16_t value);

static void left_motor_forward(void);
static void left_motor_backward(void);
static void left_motor_coast(void);

static void right_motor_forward(void);
static void right_motor_backward(void);
static void right_motor_coast(void);

/*
*********************************************************************************************************
*   函 数 名: pwm_limit
*   功能说明: 限制PWM输出范围
*   形    参: pwm — 输入PWM，带符号
*   返 回 值: 限幅后的PWM
*********************************************************************************************************
*/
static int16_t pwm_limit(int16_t pwm)
{
    if (pwm > BSP_CHASSIS_MOTOR_PWM_MAX) {
        pwm = BSP_CHASSIS_MOTOR_PWM_MAX;
    }

    if (pwm < -BSP_CHASSIS_MOTOR_PWM_MAX) {
        pwm = -BSP_CHASSIS_MOTOR_PWM_MAX;
    }

    return pwm;
}

/*
*********************************************************************************************************
*   函 数 名: abs_i16
*   功能说明: 计算int16_t绝对值
*   形    参: value — 输入值
*   返 回 值: 绝对值
*********************************************************************************************************
*/
static int16_t abs_i16(int16_t value)
{
    return (value >= 0) ? value : -value;
}

/*
*********************************************************************************************************
*   函 数 名: left_motor_forward
*   功能说明: 左电机正转方向设置
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void left_motor_forward(void)
{
    DL_GPIO_clearPins(Chassis_MotorA_PORT, Chassis_MotorA_AIN1_PIN);
    DL_GPIO_setPins(Chassis_MotorA_PORT, Chassis_MotorA_AIN2_PIN);
}

/*
*********************************************************************************************************
*   函 数 名: left_motor_backward
*   功能说明: 左电机反转方向设置
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void left_motor_backward(void)
{
    DL_GPIO_setPins(Chassis_MotorA_PORT, Chassis_MotorA_AIN1_PIN);
    DL_GPIO_clearPins(Chassis_MotorA_PORT, Chassis_MotorA_AIN2_PIN);
}

/*
*********************************************************************************************************
*   函 数 名: left_motor_coast
*   功能说明: 左电机停止输出
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void left_motor_coast(void)
{
    DL_GPIO_clearPins(Chassis_MotorA_PORT, Chassis_MotorA_AIN1_PIN);
    DL_GPIO_clearPins(Chassis_MotorA_PORT, Chassis_MotorA_AIN2_PIN);
}

/*
*********************************************************************************************************
*   函 数 名: right_motor_forward
*   功能说明: 右电机正转方向设置
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void right_motor_forward(void)
{
    DL_GPIO_clearPins(Chassis_MotorB_BIN1_PORT, Chassis_MotorB_BIN1_PIN);
    DL_GPIO_setPins(Chassis_MotorB_BIN2_PORT, Chassis_MotorB_BIN2_PIN);
}

/*
*********************************************************************************************************
*   函 数 名: right_motor_backward
*   功能说明: 右电机反转方向设置
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void right_motor_backward(void)
{
    DL_GPIO_setPins(Chassis_MotorB_BIN1_PORT, Chassis_MotorB_BIN1_PIN);
    DL_GPIO_clearPins(Chassis_MotorB_BIN2_PORT, Chassis_MotorB_BIN2_PIN);
}

/*
*********************************************************************************************************
*   函 数 名: right_motor_coast
*   功能说明: 右电机停止输出
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void right_motor_coast(void)
{
    DL_GPIO_clearPins(Chassis_MotorB_BIN1_PORT, Chassis_MotorB_BIN1_PIN);
    DL_GPIO_clearPins(Chassis_MotorB_BIN2_PORT, Chassis_MotorB_BIN2_PIN);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_ChassisMotor_Init
*   功能说明: 初始化底盘电机输出状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_ChassisMotor_Init(void)
{
    BSP_ChassisMotor_SetLeftPwm(0);
    BSP_ChassisMotor_SetRightPwm(0);

}

/*
*********************************************************************************************************
*   函 数 名: BSP_ChassisMotor_SetLeftPwm
*   功能说明: 设置左电机PWM，正数前进，负数后退
*   形    参: pwm — PWM输出值，范围-BSP_CHASSIS_MOTOR_PWM_MAX~+BSP_CHASSIS_MOTOR_PWM_MAX
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_ChassisMotor_SetLeftPwm(int16_t pwm)
{
    int16_t pwm_abs;

    pwm = pwm_limit(pwm);
    
    if (pwm == 0) {
        DL_TimerG_setCaptureCompareValue(Chassis_Motor_PWM_INST, 0, GPIO_Chassis_Motor_PWM_C1_IDX);
        right_motor_coast();
        return;
    }

    if (pwm > 0) {
        right_motor_forward();
    } else {
        right_motor_backward();
    }

    pwm_abs = abs_i16(pwm);
    DL_TimerG_setCaptureCompareValue(Chassis_Motor_PWM_INST, (uint32_t)pwm_abs, GPIO_Chassis_Motor_PWM_C1_IDX);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_ChassisMotor_SetRightPwm
*   功能说明: 设置右电机PWM，正数前进，负数后退
*   形    参: pwm — PWM输出值，范围-BSP_CHASSIS_MOTOR_PWM_MAX~+BSP_CHASSIS_MOTOR_PWM_MAX
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_ChassisMotor_SetRightPwm(int16_t pwm)
{
    int16_t pwm_abs;

    pwm = pwm_limit(pwm);

    if (pwm == 0) {
        DL_TimerG_setCaptureCompareValue(Chassis_Motor_PWM_INST, 0, GPIO_Chassis_Motor_PWM_C0_IDX);
        left_motor_coast();
        return;
    }

    if (pwm > 0) {
        left_motor_forward();
    } else {
        left_motor_backward();
    }

    pwm_abs = abs_i16(pwm);
    DL_TimerG_setCaptureCompareValue(Chassis_Motor_PWM_INST, (uint32_t)pwm_abs, GPIO_Chassis_Motor_PWM_C0_IDX);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_ChassisMotor_StopLeft
*   功能说明: 停止左电机
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_ChassisMotor_StopLeft(void)
{
    BSP_ChassisMotor_SetLeftPwm(0);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_ChassisMotor_StopRight
*   功能说明: 停止右电机
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_ChassisMotor_StopRight(void)
{
    BSP_ChassisMotor_SetRightPwm(0);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_ChassisMotor_StopAll
*   功能说明: 停止左右电机
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_ChassisMotor_StopAll(void)
{
    BSP_ChassisMotor_SetLeftPwm(0);
    BSP_ChassisMotor_SetRightPwm(0);
}
