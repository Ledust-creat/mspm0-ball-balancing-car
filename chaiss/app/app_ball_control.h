#ifndef APP_BALL_CONTROL_H_
#define APP_BALL_CONTROL_H_

#include "PID.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_BALL_CONTROL_PERIOD_MS          (33U)       /* 钢球闭环周期 */
#define APP_BALL_VISION_TIMEOUT_MS          (500U)      /* 视觉数据超时时间 */
#define APP_BALL_TARGET_LIMIT_CM            (10.0f)     /* 目标位置限幅 */
#define APP_BALL_ANGLE_LIMIT_DEG            (6.0f)      /* 横梁角度限幅 */
#define APP_BALL_SERVO_DIR                  (1.0f)      /* 舵机方向，反向时改为-1 */
#define APP_BALL_SERVO_POS_PER_DEG          (12.0f)     /* 横梁每变化1度对应的舵机协议位置增量 */
#define APP_BALL_MEASUREMENT_LIMIT_CM       (50.0f)     /* 外部位置输入合理范围 */
#define APP_BALL_SPEED_LPF_ALPHA            (0.30f)     /* 速度低通新数据权重 */
#define APP_BALL_DEFAULT_KP                 (0.80f)
#define APP_BALL_DEFAULT_KI                 (0.00f)
#define APP_BALL_DEFAULT_KD                 (0.15f)
#define APP_BALL_INTEGRAL_LIMIT_DEG         (3.0f)      /* 积分输出限幅 */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef enum
{
    APP_BALL_STATE_STOP = 0,
    APP_BALL_STATE_OPEN_LOOP_TEST,
    APP_BALL_STATE_CENTER_HOLD,
    APP_BALL_STATE_TARGET_HOLD,
    APP_BALL_STATE_FAULT
} APP_BallControlState_t;

typedef struct
{
    float center_x_px;                    /* 图像中心X坐标 */
    float cm_per_px;                      /* 像素到厘米比例 */
    float valid_y_min_px;                 /* 有效Y范围下限 */
    float valid_y_max_px;                 /* 有效Y范围上限 */
    uint8_t is_calibrated;                /* 1表示投影参数有效 */
} APP_BallVisionCalibration_t;

typedef struct
{
    PID_t ball_pid;                       /* 钢球位置PID参数及输出 */
    APP_BallControlState_t state;         /* 当前控制状态 */
    float target_cm;                      /* 目标位置 */
    float measured_cm;                    /* 实测位置 */
    float error_cm;                       /* 位置误差 */
    float ball_speed_cm_s;                /* 低通后的钢球速度 */
    float angle_cmd_deg;                  /* 横梁角度命令 */
    uint16_t servo_position;              /* 舵机位置命令 */
    uint8_t enabled;                      /* 闭环使能状态 */
    uint8_t measurement_valid;            /* 位置数据有效状态 */
    uint8_t saturated;                    /* 输出限幅标志 */
} APP_BallControl_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_BallControl_Init(void);
void APP_BallControl_Task(void);
void APP_BallControl_SetEnable(uint8_t enable);
void APP_BallControl_SetTargetCm(float target_cm);
void APP_BallControl_SetMeasuredPositionCm(float x_cm, uint32_t tick_ms);
void APP_BallControl_SetVisionPoint(float x_px, float y_px, uint32_t tick_ms);
void APP_BallControl_SetVisionCalibration(const APP_BallVisionCalibration_t *calibration);
void APP_BallControl_SetPid(float kp, float ki, float kd);
void APP_BallControl_SetServoTestPosition(uint16_t pos);
const APP_BallControl_t *APP_BallControl_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALL_CONTROL_H_ */
