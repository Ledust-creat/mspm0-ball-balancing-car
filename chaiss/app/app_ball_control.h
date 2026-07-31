#ifndef APP_BALL_CONTROL_H_
#define APP_BALL_CONTROL_H_

#include "PID.h"
#include "app_k230_input.h"
#include "dve_hts25l.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_BALL_VISION_TIMEOUT_MS          (1000U)     /* K230无有效新帧故障超时，单位ms */
#define APP_BALL_TARGET_LIMIT_CM            (15.0f)     /* 目标位置限幅，单位cm */
#define APP_BALL_ANGLE_LIMIT_DEG            (15.0f)      /* 横梁角度输出总限幅，单位deg */
#define APP_BALL_SERVO_DIR                  (-1.0f)     /* 舵机方向修正，反向时改为-1 */
#define APP_BALL_SERVO_POS_PER_DEG          (10.0f)      /* 横梁每变化1deg对应的舵机位置增量 */
#define APP_BALL_MEASUREMENT_LIMIT_CM       (16.0f)     /* 外部位置输入合理范围，单位cm */
#define APP_BALL_SPEED_WINDOW_FRAME         (10U)       /* 速度估计位置窗口帧数 */
#define APP_BALL_SPEED_FILTER_ALPHA         (0.25f)     /* 窗口速度一阶低通新数据权重 */
#define APP_BALL_DEFAULT_KP                 (0.25f)     /* 钢球位置环默认Kp */
#define APP_BALL_DEFAULT_KI                 (0.01f)     /* 钢球位置环默认Ki */
#define APP_BALL_DEFAULT_KD                 (0.8f)     /* 钢球位置环默认Kd */
#define APP_BALL_INTEGRAL_LIMIT_DEG         (6.0f)      /* 位置环积分输出限幅，单位deg */
#define APP_BALL_STATIC_ERROR_CM             (0.5f)      /* 超过该误差才允许静摩擦补偿 */
#define APP_BALL_STATIC_SPEED_CM_S           (0.3f)      /* 低于该速度认为钢球基本静止 */
#define APP_BALL_STATIC_MIN_ANGLE_DEG        (0.0f)      /* 克服机构和钢球静摩擦的最低倾角 */
#define APP_BALL_SERVO_LEVEL_DEFAULT        (470U)      /* 上电默认水平零点，仅保存在RAM */
#define APP_BALL_SERVO_LEVEL_MIN            (DVE_HTS25L_POS_MIN + 60U)  /* 零点允许下限，预留负偏移空间 */
#define APP_BALL_SERVO_LEVEL_MAX            (DVE_HTS25L_POS_MAX - 100U) /* 零点允许上限，预留正偏移空间 */

#define APP_BALL_ACCEL_FF_ENABLE            (0U)        /* 加速度前馈总开关，初期保持关闭 */
#define APP_BALL_PLAN_ACCEL_FF_ENABLE       (0U)        /* 规划加速度前馈开关 */
#define APP_BALL_IMU_ACCEL_FF_ENABLE        (0U)        /* IMU残差修正开关 */
#define APP_BALL_ACCEL_RESIDUAL_GAIN        (0.30f)     /* IMU相对规划加速度的残差增益 */
#define APP_BALL_TUBE_FORWARD_GAIN          (0.0f)      /* 管轴对车体前向加速度投影系数 */
#define APP_BALL_TUBE_LATERAL_GAIN          (1.0f)      /* 管轴对车体横向加速度投影系数 */
#define APP_BALL_ACCEL_FF_DEG_PER_G         (0.0f)      /* 管轴加速度到前馈角度系数 */
#define APP_BALL_ACCEL_FF_LIMIT_DEG         (2.0f)      /* 加速度前馈单独限幅 */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef enum
{
    APP_BALL_STATE_STOP = 0,                /* 控球停止，舵机回水平位置 */
    APP_BALL_STATE_OPEN_LOOP_TEST,          /* 舵机开环位置测试 */
    APP_BALL_STATE_CENTER_HOLD,             /* 钢球保持在管中心 */
    APP_BALL_STATE_TARGET_HOLD,             /* 钢球保持在指定目标位置 */
    APP_BALL_STATE_FAULT                    /* 视觉或设备故障，进入安全状态 */
} APP_BallControlState_t;

typedef enum
{
    APP_BALL_MODE_DISABLE = 0,              /* 失能控球，舵机回中位 */
    APP_BALL_MODE_MANUAL_TILT,              /* 固定角度开环倾斜，不执行位置PID */
    APP_BALL_MODE_POSITION_HOLD,            /* 位置闭环保持指定目标 */
    APP_BALL_MODE_FAULT                     /* 故障保护模式 */
} APP_BallControlMode_t;

typedef enum
{
    APP_BALL_FAULT_NONE = 0,                /* 无故障 */
    APP_BALL_FAULT_VISION_TIMEOUT           /* K230视觉数据超时 */
} APP_BallControlFault_t;

typedef struct
{
    float center_x_px;                      /* 图像中心X坐标，单位px */
    float cm_per_px;                        /* 像素到厘米比例，单位cm/px */
    float valid_y_min_px;                   /* 有效Y范围下限，单位px */
    float valid_y_max_px;                   /* 有效Y范围上限，单位px */
    uint8_t is_calibrated;                  /* 1表示投影参数有效 */
} APP_BallVisionCalibration_t;

typedef struct
{
    float position_kp;                      /* 球位置比例增益 */
    float position_ki;                      /* 球位置积分增益 */
    float position_kd;                      /* 球位置速度反馈增益 */
    float target_position_cm;               /* 默认目标位置，单位cm */
    float accel_forward_ff_gain;            /* 预留车体前向加速度前馈系数，单位deg/g */
    uint16_t servo_position_min;            /* 舵机软件最小允许位置，范围0~1000 */
    uint16_t servo_position_max;            /* 舵机软件最大允许位置，范围0~1000 */
} APP_BallControlConfig_t;

typedef struct
{
    APP_BallControlState_t state;           /* 当前控球状态 */
    APP_BallControlFault_t fault;           /* 当前控球故障 */
    float target_position_cm;               /* 当前目标位置，单位cm */
    float measured_position_cm;             /* 当前视觉位置，单位cm */
    float error_cm;                         /* 位置误差，单位cm */
    float estimated_velocity_cm_s;          /* 估算球速，单位cm/s */
    float estimated_acceleration_cm_s2;     /* 估算球加速度，单位cm/s^2 */
    float angle_cmd_deg;                    /* 横梁角度命令，单位deg */
    float velocity_feedback_deg;            /* 移动控球附加速度反馈角，单位deg */
    float vehicle_accel_feedforward_deg;    /* 车辆加速度前馈角，单位deg */
    float accel_feedforward_deg;            /* IMU加速度前馈角度，单位deg */
    uint16_t servo_command;                 /* 当前舵机位置命令，范围0~1000 */
    uint32_t last_vision_tick_ms;           /* 最近有效视觉帧收帧时刻，单位ms */
    bool vision_valid;                      /* true表示当前视觉数据有效 */
    bool enabled;                           /* true表示闭环使能 */
    bool imu_accel_valid;                   /* true表示本次使用的IMU加速度缓存有效 */
    bool saturated;                         /* true表示角度或舵机输出发生限幅 */
} APP_BallControlStatus_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_BallControl_Init(void);
void APP_BallControl_Task(void);
void APP_BallControl_UpdateFromVision(const APP_K230BallFrame_t *frame);
void APP_BallControl_SafetyTask(void);
void APP_BallControl_SetEnable(uint8_t enable);
void APP_BallControl_SetMode(APP_BallControlMode_t mode);
void APP_BallControl_SetManualTiltDeg(float angle_deg, uint16_t move_time_ms);
void APP_BallControl_StartHold(float target_cm);
void APP_BallControl_Reset(void);
void APP_BallControl_SetTargetCm(float target_cm);
void APP_BallControl_SetMeasuredPositionCm(float x_cm, uint32_t tick_ms);
void APP_BallControl_SetVisionPoint(float x_px, float y_px, uint32_t tick_ms);
void APP_BallControl_SetVisionCalibration(const APP_BallVisionCalibration_t *calibration);
void APP_BallControl_SetPid(float kp, float ki, float kd);
void APP_BallControl_GetPid(PID_t *pid);
void APP_BallControl_SetMotionCompensation(float velocity_gain,
                                           float acceleration_gain);
void APP_BallControl_SetMotionEstimator(uint8_t window_frame, float filter_alpha);
void APP_BallControl_SetServoTestPosition(uint16_t pos);
void APP_BallControl_SetServoLevelPosition(uint16_t position);
uint16_t APP_BallControl_GetServoLevelPosition(void);
uint16_t APP_BallControl_PositionFromLevelOffset(int16_t offset);
void APP_BallControl_SetServoControlOffset(int16_t offset);
bool APP_BallControl_IsFault(void);
APP_BallControlFault_t APP_BallControl_GetFault(void);
void APP_BallControl_GetStatus(APP_BallControlStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALL_CONTROL_H_ */
