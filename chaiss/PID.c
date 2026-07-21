#include "PID.h"

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float limit_float(float value, float min, float max);

/*
*********************************************************************************************************
*   函 数 名: limit_float
*   功能说明: 浮点限幅
*   形    参: value — 输入值；min — 最小值；max — 最大值
*   返 回 值: 限幅后的值
*********************************************************************************************************
*/
static float limit_float(float value, float min, float max)
{
    if (value > max) {
        value = max;
    }

    if (value < min) {
        value = min;
    }

    return value;
}

/*
*********************************************************************************************************
*   函 数 名: PID_Init
*   功能说明: 初始化PID结构体
*   形    参: pid — PID结构体；kp,ki,kd — PID参数；max_iout — 积分限幅；max_out — 输出限幅
*   返 回 值: 无
*********************************************************************************************************
*/
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_iout, float max_out)
{
    if (pid == 0) {
        return;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->max_iout = max_iout;
    pid->max_out = max_out;

    PID_Reset(pid);
}

/*
*********************************************************************************************************
*   函 数 名: PID_Reset
*   功能说明: 清除PID历史状态
*   形    参: pid — PID结构体
*   返 回 值: 无
*********************************************************************************************************
*/
void PID_Reset(PID_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->out = 0.0f;

    for (uint8_t i = 0; i < PID_HIST_LEN; i++) {
        pid->error[i] = 0.0f;
        pid->Dbuf[i] = 0.0f;
    }
}

/*
*********************************************************************************************************
*   函 数 名: PID_SetParam
*   功能说明: 设置PID参数
*   形    参: pid — PID结构体；kp,ki,kd — PID参数；max_iout — 积分限幅；max_out — 输出限幅
*   返 回 值: 无
*********************************************************************************************************
*/
void PID_SetParam(PID_t *pid, float kp, float ki, float kd, float max_iout, float max_out)
{
    if (pid == 0) {
        return;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    
    pid->max_iout = max_iout;
    pid->max_out = max_out;
}

/*
*********************************************************************************************************
*   函 数 名: PID_Calc
*   功能说明: 位置式PID计算
*   形    参: pid — PID结构体；set — 目标值；fdb — 反馈值
*   返 回 值: PID输出值
*********************************************************************************************************
*/
float PID_Calc(PID_t *pid, float set, float fdb)
{
    if (pid == 0) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = set - fdb;

    pid->Pout = pid->Kp * pid->error[0];

    pid->Iout += pid->Ki * pid->error[0];
    pid->Iout = limit_float(pid->Iout, -pid->max_iout, pid->max_iout);

    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = pid->error[0] - pid->error[1];

    pid->Dout = pid->Kd * pid->Dbuf[0];

    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = limit_float(pid->out, -pid->max_out, pid->max_out);

    return pid->out;
}