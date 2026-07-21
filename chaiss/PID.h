#ifndef PID_H_
#define PID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define PID_HIST_LEN        (3U)      /* PID误差历史长度 */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    float Kp;                         /* 比例参数 */
    float Ki;                         /* 积分参数 */
    float Kd;                         /* 微分参数 */

    float max_iout;                   /* 积分限幅 */
    float max_out;                    /* 总输出限幅 */

    float Pout;                       /* 比例输出 */
    float Iout;                       /* 积分输出 */
    float Dout;                       /* 微分输出 */
    float out;                        /* 总输出 */

    float error[PID_HIST_LEN];        /* 误差缓存，0最新 */
    float Dbuf[PID_HIST_LEN];         /* 微分缓存，0最新 */
} PID_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_iout, float max_out);
void PID_Reset(PID_t *pid);
void PID_SetParam(PID_t *pid, float kp, float ki, float kd, float max_iout, float max_out);
float PID_Calc(PID_t *pid, float set, float fdb);

#ifdef __cplusplus
}
#endif

#endif /* PID_H_ */
