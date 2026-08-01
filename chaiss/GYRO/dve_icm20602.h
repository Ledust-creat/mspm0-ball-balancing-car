#ifndef DVE_ICM20602_H_
#define DVE_ICM20602_H_

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
#define DVE_ICM20602_WHO_AM_I_VALUE             (0x12U)     /* WHO_AM_I正常值 */

#define DVE_ICM20602_ROLL_ANGLE_SIGN            (1.0f)      /* roll方向修正 */
#define DVE_ICM20602_PITCH_ANGLE_SIGN           (1.0f)      /* pitch方向修正 */
#define DVE_ICM20602_YAW_GYRO_SIGN              (1.0f)      /* Z轴角速度方向修正 */
#define DVE_ICM20602_YAW_ANGLE_SIGN             (1.0f)      /* yaw角方向修正 */

#define DVE_ICM20602_REVERSE                    (1U)        /* 原姿态轴交换兼容宏 */

#if (DVE_ICM20602_REVERSE != 0U)
#define DVE_ICM20602_MAP_ACCEL_X(data)          ((data).accel_y_g)
#define DVE_ICM20602_MAP_ACCEL_Y(data)          ((data).accel_x_g)
#define DVE_ICM20602_MAP_ACCEL_Z(data)          ((data).accel_z_g)
#define DVE_ICM20602_MAP_GYRO_X(data)           ((data).gyro_y_dps)
#define DVE_ICM20602_MAP_GYRO_Y(data)           ((data).gyro_x_dps)
#define DVE_ICM20602_MAP_GYRO_Z(data)           ((data).gyro_z_dps)
#else
#define DVE_ICM20602_MAP_ACCEL_X(data)          ((data).accel_x_g)
#define DVE_ICM20602_MAP_ACCEL_Y(data)          ((data).accel_y_g)
#define DVE_ICM20602_MAP_ACCEL_Z(data)          ((data).accel_z_g)
#define DVE_ICM20602_MAP_GYRO_X(data)           ((data).gyro_x_dps)
#define DVE_ICM20602_MAP_GYRO_Y(data)           ((data).gyro_y_dps)
#define DVE_ICM20602_MAP_GYRO_Z(data)           ((data).gyro_z_dps)
#endif

#define DVE_ICM20602_ROLL_INSTALL_OFFSET_DEG    (-1.6f)     /* roll安装角补偿 */
#define DVE_ICM20602_PITCH_INSTALL_OFFSET_DEG   (0.0f)      /* pitch安装角补偿 */
#define DVE_ICM20602_YAW_INSTALL_OFFSET_DEG     (0.0f)      /* yaw安装角补偿 */

#define DVE_ICM20602_YAW_DEADBAND_DPS           (0.5f)      /* 静止小角速度死区 */
#define DVE_ICM20602_YAW_SCALE                  (1.0f)      /* yaw积分比例补偿 */

#define DVE_ICM20602_USE_MAHONY                 (1U)        /* 1使用无磁力计Mahony */
#define DVE_ICM20602_MAHONY_KP                  (2.0f)      /* Mahony比例增益 */
#define DVE_ICM20602_MAHONY_KI                  (0.05f)     /* Mahony积分增益 */

#define DVE_ICM20602_ENABLE_ATTITUDE_SOLVE      (0U)        /* 1运行姿态解算，0仅保留源码 */
#define DVE_ICM20602_ENABLE_ACCEL_XY_CALIB      (1U)        /* 1启动时标定X/Y静态加速度零偏 */
#define DVE_ICM20602_ACCEL_XY_LPF_ALPHA         (0.20f)     /* X/Y动态加速度低通新数据权重 */
#define DVE_ICM20602_ACCEL_X_SIGN               (1.0f)      /* 车体X加速度方向修正 */
#define DVE_ICM20602_ACCEL_Y_SIGN               (-1.0f)     /* 车体Y加速度方向修正，向右加速为正 */
#define DVE_ICM20602_ENABLE_FRAME_TRANSFORM     (1U)        /* 1启用安装方向与固定偏角修正 */

#define DVE_ICM20602_INSTALL_X_FORWARD          (0U)        /* 传感器X轴指向车头 */
#define DVE_ICM20602_INSTALL_X_BACKWARD         (1U)        /* 传感器X轴指向车尾 */
#define DVE_ICM20602_INSTALL_Y_FORWARD          (2U)        /* 传感器Y轴指向车头 */
#define DVE_ICM20602_INSTALL_Y_BACKWARD         (3U)        /* 传感器Y轴指向车尾 */
#define DVE_ICM20602_INSTALL_DIRECTION          DVE_ICM20602_INSTALL_X_FORWARD
#define DVE_ICM20602_MOUNT_YAW_DEG              (0.0f)      /* 水平安装固定偏角，逆时针为正 */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    float accel_x_g;                            /* 车体X轴动态加速度，已扣静态零偏并低通 */
    float accel_y_g;                            /* 车体Y轴动态加速度，已扣静态零偏并低通 */
    float roll_deg;                             /* 当前roll角 */
    float pitch_deg;                            /* 当前pitch角 */
    float yaw_deg;                              /* 当前yaw角 */
    bool is_ready;                              /* ICM20602就绪标志 */
} DVE_ICM20602State_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
bool DVE_ICM20602_Init(void);
void DVE_ICM20602_Update(float dt_s);
void DVE_ICM20602_SetZero(void);
void DVE_ICM20602_GetState(DVE_ICM20602State_t *state);
bool DVE_ICM20602_ReadAccelXY(float *accel_x_g, float *accel_y_g);
uint8_t DVE_ICM20602_ReadWhoAmI(void);

#ifdef __cplusplus
}
#endif

#endif /* DVE_ICM20602_H_ */
