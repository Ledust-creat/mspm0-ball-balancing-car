#include "dve_icm20602.h"
#include "bsp_gyro_spi.h"
#include "delay.h"
#include <math.h>
#include <string.h>

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define ICM20602_SPI_READ_BIT                  (0x80U)
#define ICM20602_SPI_WRITE_MASK                (0x7FU)

#define ICM20602_REG_SMPLRT_DIV                (0x19U)
#define ICM20602_REG_CONFIG                    (0x1AU)
#define ICM20602_REG_GYRO_CONFIG               (0x1BU)
#define ICM20602_REG_ACCEL_CONFIG              (0x1CU)
#define ICM20602_REG_ACCEL_CONFIG2             (0x1DU)
#define ICM20602_REG_INT_PIN_CFG               (0x37U)
#define ICM20602_REG_ACCEL_XOUT_H              (0x3BU)
#define ICM20602_REG_USER_CTRL                 (0x6AU)
#define ICM20602_REG_PWR_MGMT_1                (0x6BU)
#define ICM20602_REG_PWR_MGMT_2                (0x6CU)
#define ICM20602_REG_WHO_AM_I                  (0x75U)

#define ICM20602_ACCEL_FS_16G                  (0x18U)
#define ICM20602_GYRO_FS_2000DPS               (0x18U)
#define ICM20602_ACCEL_SENS_16G                (2048.0f)
#define ICM20602_GYRO_SENS_2000DPS             (16.4f)
#define ICM20602_GYRO_CALIB_TIMES              (250U)
#define ICM20602_GYRO_CALIB_PERIOD_MS          (5U)
#define ICM20602_RAD_TO_DEG                    (57.2957795f)
#define ICM20602_DEG_TO_RAD                    (0.0174532925f)
#define ICM20602_NORM_MIN                      (1.0e-12f)
#define ICM20602_INTEGRAL_LIMIT                (0.5f)
#define ICM20602_POWER_ON_DELAY_MS             (200U)
#define ICM20602_WHO_AM_I_RETRY_TIMES          (5U)
#define ICM20602_WHO_AM_I_RETRY_PERIOD_MS      (20U)
#define ICM20602_ACCEL_XY_CALIB_TIMES          (100U)
#define ICM20602_ACCEL_XY_CALIB_PERIOD_MS      (2U)

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    int16_t accel_x;                           /* X轴加速度原始值 */
    int16_t accel_y;                           /* Y轴加速度原始值 */
    int16_t accel_z;                           /* Z轴加速度原始值 */
    int16_t gyro_x;                            /* X轴角速度原始值 */
    int16_t gyro_y;                            /* Y轴角速度原始值 */
    int16_t gyro_z;                            /* Z轴角速度原始值 */
} ICM20602Raw_t;

typedef struct
{
    float accel_x_g;                           /* X轴加速度，单位g */
    float accel_y_g;                           /* Y轴加速度，单位g */
    float accel_z_g;                           /* Z轴加速度，单位g */
    float gyro_x_dps;                          /* X轴角速度，单位deg/s */
    float gyro_y_dps;                          /* Y轴角速度，单位deg/s */
    float gyro_z_dps;                          /* Z轴角速度，单位deg/s */
} ICM20602Data_t;

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static DVE_ICM20602State_t s_icm_state;         /* ICM20602设备状态 */
static float s_gyro_x_offset_dps = 0.0f;        /* X轴角速度零偏 */
static float s_gyro_y_offset_dps = 0.0f;        /* Y轴角速度零偏 */
static float s_gyro_z_offset_dps = 0.0f;        /* Z轴角速度零偏 */
static float s_relative_yaw_deg = 0.0f;         /* 相对yaw角，校准后从0开始 */
static float s_q0 = 1.0f;                       /* 姿态四元数q0 */
static float s_q1 = 0.0f;                       /* 姿态四元数q1 */
static float s_q2 = 0.0f;                       /* 姿态四元数q2 */
static float s_q3 = 0.0f;                       /* 姿态四元数q3 */
static float s_integral_x = 0.0f;               /* Mahony积分项X */
static float s_integral_y = 0.0f;               /* Mahony积分项Y */
static float s_integral_z = 0.0f;               /* Mahony积分项Z */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static float s_accel_x_offset_g = 0.0f;         /* 传感器X轴静态加速度零偏 */
static float s_accel_y_offset_g = 0.0f;         /* 传感器Y轴静态加速度零偏 */
static float s_accel_x_filter_g = 0.0f;         /* 车体X轴动态加速度低通结果 */
static float s_accel_y_filter_g = 0.0f;         /* 车体Y轴动态加速度低通结果 */
static float s_mount_yaw_sin = 0.0f;            /* 固定安装偏角sin值 */
static float s_mount_yaw_cos = 1.0f;            /* 固定安装偏角cos值 */

static void icm20602_write_reg(uint8_t reg, uint8_t data);
static uint8_t icm20602_read_reg(uint8_t reg);
static void icm20602_read_bytes(uint8_t reg, uint8_t *buffer, uint16_t len);
static void icm20602_read_raw(ICM20602Raw_t *raw);
static void icm20602_read_accel_xy_raw(int16_t *raw_x, int16_t *raw_y);
static void icm20602_read_data(ICM20602Data_t *data);
static ICM20602Data_t icm20602_axis_map(ICM20602Data_t data);
static void icm20602_map_install_direction(float sensor_x_g, float sensor_y_g,
                                           float *mapped_x_g, float *mapped_y_g);
static void icm20602_rotate_to_car_frame(float mapped_x_g, float mapped_y_g,
                                         float *car_x_g, float *car_y_g);
static void calibrate_accel_xy(void);
static int16_t make_i16(uint8_t high, uint8_t low);
static float wrap_180(float angle);
static float apply_deadband(float gyro_dps);
static float inv_sqrt(float x);
static float limit_float(float value, float min, float max);
static void calc_accel_attitude(const ICM20602Data_t *data, float *roll_deg, float *pitch_deg);
static void mahony_update_nomagnetic(float gx_dps, float gy_dps, float gz_dps,
                                     float ax_g, float ay_g, float az_g,
                                     float dt_s);
static void calc_quaternion_attitude(float *roll_deg, float *pitch_deg, float *yaw_deg);
static void calibrate_gyro(void);
static void reset_attitude_quaternion(void);
static uint8_t read_who_am_i_retry(void);

static int16_t make_i16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static float wrap_180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static float apply_deadband(float gyro_dps)
{
    if ((gyro_dps > -DVE_ICM20602_YAW_DEADBAND_DPS) &&
        (gyro_dps < DVE_ICM20602_YAW_DEADBAND_DPS)) {
        return 0.0f;
    }

    return gyro_dps;
}



static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    uint32_t i;

    if (x <= ICM20602_NORM_MIN) {
        return 0.0f;
    }

    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759dfU - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));

    return y;
}

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

static void calc_accel_attitude(const ICM20602Data_t *data, float *roll_deg, float *pitch_deg)
{
    float roll_raw;
    float pitch_raw;
    float ay_az_norm;

    if ((data == 0) || (roll_deg == 0) || (pitch_deg == 0)) {
        return;
    }

    // roll/pitch先用加速度计静态解算，适合静止或低动态验证安装方向
    ay_az_norm = sqrtf((data->accel_y_g * data->accel_y_g) + (data->accel_z_g * data->accel_z_g));
    ay_az_norm = sqrtf((data->accel_y_g * data->accel_y_g) + (data->accel_z_g * data->accel_z_g));
    roll_raw = atan2f(data->accel_y_g, data->accel_z_g) * ICM20602_RAD_TO_DEG;
    pitch_raw = atan2f(-data->accel_x_g, ay_az_norm) * ICM20602_RAD_TO_DEG;

    *roll_deg = (roll_raw *
                 DVE_ICM20602_ROLL_ANGLE_SIGN ) +
                DVE_ICM20602_ROLL_INSTALL_OFFSET_DEG;
    *pitch_deg = (pitch_raw *
                  DVE_ICM20602_PITCH_ANGLE_SIGN ) +
                 DVE_ICM20602_PITCH_INSTALL_OFFSET_DEG;
}

static void mahony_update_nomagnetic(float gx_dps, float gy_dps, float gz_dps,
                                     float ax_g, float ay_g, float az_g,
                                     float dt_s)
{
    float gx = gx_dps * ICM20602_DEG_TO_RAD;
    float gy = gy_dps * ICM20602_DEG_TO_RAD;
    float gz = gz_dps * ICM20602_DEG_TO_RAD;
    float q0 = s_q0;
    float q1 = s_q1;
    float q2 = s_q2;
    float q3 = s_q3;
    float half_vx;
    float half_vy;
    float half_vz;
    float half_ex;
    float half_ey;
    float half_ez;
    float qa;
    float qb;
    float qc;
    float norm;
    float norm_sq;

    norm_sq = (ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g);
    if (norm_sq > ICM20602_NORM_MIN) {
        norm = inv_sqrt(norm_sq);
        ax_g *= norm;
        ay_g *= norm;
        az_g *= norm;

        // 用加速度方向修正roll/pitch，平地转向时yaw仍主要依赖Z轴角速度
        half_vx = q1 * q3 - q0 * q2;
        half_vy = q0 * q1 + q2 * q3;
        half_vz = q0 * q0 - 0.5f + q3 * q3;

        half_ex = ay_g * half_vz - az_g * half_vy;
        half_ey = az_g * half_vx - ax_g * half_vz;
        half_ez = ax_g * half_vy - ay_g * half_vx;

        s_integral_x += 2.0f * DVE_ICM20602_MAHONY_KI * half_ex * dt_s;
        s_integral_y += 2.0f * DVE_ICM20602_MAHONY_KI * half_ey * dt_s;
        s_integral_z += 2.0f * DVE_ICM20602_MAHONY_KI * half_ez * dt_s;

        s_integral_x = limit_float(s_integral_x, -ICM20602_INTEGRAL_LIMIT, ICM20602_INTEGRAL_LIMIT);
        s_integral_y = limit_float(s_integral_y, -ICM20602_INTEGRAL_LIMIT, ICM20602_INTEGRAL_LIMIT);
        s_integral_z = limit_float(s_integral_z, -ICM20602_INTEGRAL_LIMIT, ICM20602_INTEGRAL_LIMIT);

        gx += s_integral_x + 2.0f * DVE_ICM20602_MAHONY_KP * half_ex;
        gy += s_integral_y + 2.0f * DVE_ICM20602_MAHONY_KP * half_ey;
        gz += s_integral_z + 2.0f * DVE_ICM20602_MAHONY_KP * half_ez;
    }

    gx *= 0.5f * dt_s;
    gy *= 0.5f * dt_s;
    gz *= 0.5f * dt_s;

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 +=  qa * gx + qc * gz - q3 * gy;
    q2 +=  qa * gy - qb * gz + q3 * gx;
    q3 +=  qa * gz + qb * gy - qc * gx;

    norm_sq = (q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3);
    if (norm_sq < ICM20602_NORM_MIN) {
        return;
    }

    norm = inv_sqrt(norm_sq);
    s_q0 = q0 * norm;
    s_q1 = q1 * norm;
    s_q2 = q2 * norm;
    s_q3 = q3 * norm;
}

static void calc_quaternion_attitude(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float roll_raw;
    float pitch_raw;
    float yaw_raw;
    float pitch_sin;

    if ((roll_deg == 0) || (pitch_deg == 0) || (yaw_deg == 0)) {
        return;
    }

    roll_raw = atan2f(2.0f * (s_q0 * s_q1 + s_q2 * s_q3),
                      1.0f - 2.0f * (s_q1 * s_q1 + s_q2 * s_q2)) * ICM20602_RAD_TO_DEG;
    pitch_sin = 2.0f * (s_q0 * s_q2 - s_q3 * s_q1);
    pitch_sin = limit_float(pitch_sin, -1.0f, 1.0f);
    pitch_raw = asinf(pitch_sin) * ICM20602_RAD_TO_DEG;
    yaw_raw = atan2f(2.0f * (s_q1 * s_q2 + s_q0 * s_q3),
                     1.0f - 2.0f * (s_q2 * s_q2 + s_q3 * s_q3)) * ICM20602_RAD_TO_DEG;

    *roll_deg = (roll_raw *
                 DVE_ICM20602_ROLL_ANGLE_SIGN ) +
                DVE_ICM20602_ROLL_INSTALL_OFFSET_DEG;
    *pitch_deg = (pitch_raw *
                  DVE_ICM20602_PITCH_ANGLE_SIGN ) +
                 DVE_ICM20602_PITCH_INSTALL_OFFSET_DEG;
    *yaw_deg = wrap_180((yaw_raw * DVE_ICM20602_YAW_ANGLE_SIGN) - DVE_ICM20602_YAW_INSTALL_OFFSET_DEG);
}

static void reset_attitude_quaternion(void)
{
    s_q0 = 1.0f;
    s_q1 = 0.0f;
    s_q2 = 0.0f;
    s_q3 = 0.0f;
    s_integral_x = 0.0f;
    s_integral_y = 0.0f;
    s_integral_z = 0.0f;
}

static uint8_t read_who_am_i_retry(void)
{
    uint8_t i;
    uint8_t who = 0U;

    for (i = 0U; i < ICM20602_WHO_AM_I_RETRY_TIMES; i++) {
        who = DVE_ICM20602_ReadWhoAmI();
        if (who == DVE_ICM20602_WHO_AM_I_VALUE) {
            break;
        }
        delay_ms(ICM20602_WHO_AM_I_RETRY_PERIOD_MS);
    }

    return who;
}

static void icm20602_write_reg(uint8_t reg, uint8_t data)
{
    BSP_GyroSPI_CSLow();
    (void)BSP_GyroSPI_TransferByte(reg & ICM20602_SPI_WRITE_MASK); // 写寄存器地址bit7清零
    (void)BSP_GyroSPI_TransferByte(data);
    BSP_GyroSPI_CSHigh();
}

static uint8_t icm20602_read_reg(uint8_t reg)
{
    uint8_t data;

    BSP_GyroSPI_CSLow();
    (void)BSP_GyroSPI_TransferByte(reg | ICM20602_SPI_READ_BIT); // 读寄存器地址bit7置位
    data = BSP_GyroSPI_TransferByte(0xFFU);
    BSP_GyroSPI_CSHigh();

    return data;
}

static void icm20602_read_bytes(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    uint16_t i;

    if ((buffer == 0) || (len == 0U)) {
        return;
    }

    BSP_GyroSPI_CSLow();
    (void)BSP_GyroSPI_TransferByte(reg | ICM20602_SPI_READ_BIT);
    for (i = 0; i < len; i++) {
        buffer[i] = BSP_GyroSPI_TransferByte(0xFFU); // 连续发送dummy时钟读出数据
    }
    BSP_GyroSPI_CSHigh();
}

static void icm20602_read_raw(ICM20602Raw_t *raw)
{
    uint8_t buffer[14];

    if (raw == 0) {
        return;
    }

    icm20602_read_bytes(ICM20602_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));

    raw->accel_x = make_i16(buffer[0], buffer[1]);
    raw->accel_y = make_i16(buffer[2], buffer[3]);
    raw->accel_z = make_i16(buffer[4], buffer[5]);
    raw->gyro_x  = make_i16(buffer[8], buffer[9]);
    raw->gyro_y  = make_i16(buffer[10], buffer[11]);
    raw->gyro_z  = make_i16(buffer[12], buffer[13]);
}

/*
*********************************************************************************************************
*   函 数 名: icm20602_read_accel_xy_raw
*   功能说明: 从ACCEL_XOUT_H开始只读取X/Y轴4字节原始加速度
*   形    参: raw_x - X轴原始值；raw_y - Y轴原始值
*   返 回 值: 无
*********************************************************************************************************
*/
static void icm20602_read_accel_xy_raw(int16_t *raw_x, int16_t *raw_y)
{
    uint8_t buffer[4];

    if ((raw_x == 0) || (raw_y == 0)) {
        return;
    }

    icm20602_read_bytes(ICM20602_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));

    *raw_x = make_i16(buffer[0], buffer[1]);
    *raw_y = make_i16(buffer[2], buffer[3]);
}

/*
*********************************************************************************************************
*   函 数 名: icm20602_map_install_direction
*   功能说明: 根据ICM20602粗安装方向完成轴交换和符号修正
*   形    参: sensor_x_g,sensor_y_g - 传感器坐标加速度；mapped_x_g,mapped_y_g - 粗映射结果
*   返 回 值: 无
*********************************************************************************************************
*/
static void icm20602_map_install_direction(float sensor_x_g, float sensor_y_g,
                                           float *mapped_x_g, float *mapped_y_g)
{
    if ((mapped_x_g == 0) || (mapped_y_g == 0)) {
        return;
    }

#if (DVE_ICM20602_INSTALL_DIRECTION == DVE_ICM20602_INSTALL_X_FORWARD)
    *mapped_x_g = sensor_x_g;
    *mapped_y_g = sensor_y_g;
#elif (DVE_ICM20602_INSTALL_DIRECTION == DVE_ICM20602_INSTALL_X_BACKWARD)
    *mapped_x_g = -sensor_x_g;
    *mapped_y_g = -sensor_y_g;
#elif (DVE_ICM20602_INSTALL_DIRECTION == DVE_ICM20602_INSTALL_Y_FORWARD)
    *mapped_x_g = sensor_y_g;
    *mapped_y_g = -sensor_x_g;
#elif (DVE_ICM20602_INSTALL_DIRECTION == DVE_ICM20602_INSTALL_Y_BACKWARD)
    *mapped_x_g = -sensor_y_g;
    *mapped_y_g = sensor_x_g;
#else
#error "Unsupported DVE_ICM20602_INSTALL_DIRECTION"
#endif
}

/*
*********************************************************************************************************
*   函 数 名: icm20602_rotate_to_car_frame
*   功能说明: 将粗映射后的加速度旋转到车体坐标系
*   形    参: mapped_x_g,mapped_y_g - 粗映射结果；car_x_g,car_y_g - 车体坐标结果
*   返 回 值: 无
*********************************************************************************************************
*/
static void icm20602_rotate_to_car_frame(float mapped_x_g, float mapped_y_g,
                                         float *car_x_g, float *car_y_g)
{
    if ((car_x_g == 0) || (car_y_g == 0)) {
        return;
    }

    *car_x_g = (mapped_x_g * s_mount_yaw_cos) - (mapped_y_g * s_mount_yaw_sin);
    *car_y_g = (mapped_x_g * s_mount_yaw_sin) + (mapped_y_g * s_mount_yaw_cos);
}

/*
*********************************************************************************************************
*   函 数 名: calibrate_accel_xy
*   功能说明: 标定传感器X/Y轴静态加速度零偏
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void calibrate_accel_xy(void)
{
    uint16_t i;
    int16_t raw_x;
    int16_t raw_y;
    float sum_x = 0.0f;
    float sum_y = 0.0f;

    delay_ms(100U);

    for (i = 0U; i < ICM20602_ACCEL_XY_CALIB_TIMES; i++) {
        icm20602_read_accel_xy_raw(&raw_x, &raw_y);
        sum_x += (float)raw_x / ICM20602_ACCEL_SENS_16G;
        sum_y += (float)raw_y / ICM20602_ACCEL_SENS_16G;
        delay_ms(ICM20602_ACCEL_XY_CALIB_PERIOD_MS);
    }

    s_accel_x_offset_g = sum_x / (float)ICM20602_ACCEL_XY_CALIB_TIMES;
    s_accel_y_offset_g = sum_y / (float)ICM20602_ACCEL_XY_CALIB_TIMES;
    s_accel_x_filter_g = 0.0f;
    s_accel_y_filter_g = 0.0f;
    s_icm_state.accel_x_g = 0.0f;
    s_icm_state.accel_y_g = 0.0f;
}

static void calibrate_gyro(void)
{
    uint16_t i;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    ICM20602Data_t data;

    delay_ms(200); // 上电后先静置，避免刚初始化的漂移进入零偏
    for (i = 0U; i < ICM20602_GYRO_CALIB_TIMES; i++) {
        icm20602_read_data(&data);
        data = icm20602_axis_map(data);
        sum_x += data.gyro_x_dps;
        sum_y += data.gyro_y_dps;
        sum_z += data.gyro_z_dps;
        delay_ms(ICM20602_GYRO_CALIB_PERIOD_MS);
    }

    s_gyro_x_offset_dps = sum_x / (float)ICM20602_GYRO_CALIB_TIMES;
    s_gyro_y_offset_dps = sum_y / (float)ICM20602_GYRO_CALIB_TIMES;
    s_gyro_z_offset_dps = sum_z / (float)ICM20602_GYRO_CALIB_TIMES;
    s_relative_yaw_deg = 0.0f;
    s_icm_state.yaw_deg = 0.0f;
    reset_attitude_quaternion();
}

/*
*********************************************************************************************************
*   函 数 名: DVE_ICM20602_Init
*   功能说明: 初始化ICM20602并标定Z轴陀螺仪零偏
*   形    参: 无
*   返 回 值: true初始化成功，false初始化失败
*********************************************************************************************************
*/
bool DVE_ICM20602_Init(void)
{
    uint8_t who;
    float mount_yaw_rad;

    BSP_GyroSPI_Init();

    delay_ms(ICM20602_POWER_ON_DELAY_MS); // MCU独立复位后等待ICM20602电源和SPI接口稳定

    icm20602_write_reg(ICM20602_REG_PWR_MGMT_1, 0x80U);
    delay_ms(100);
    icm20602_write_reg(ICM20602_REG_PWR_MGMT_1, 0x01U); // 使用陀螺仪PLL时钟
    delay_ms(10);
    icm20602_write_reg(ICM20602_REG_PWR_MGMT_2, 0x00U);
    icm20602_write_reg(ICM20602_REG_USER_CTRL, 0x10U); // 禁用I2C，仅使用SPI
    icm20602_write_reg(ICM20602_REG_SMPLRT_DIV, 0x00U);
    icm20602_write_reg(ICM20602_REG_CONFIG, 0x03U);
    icm20602_write_reg(ICM20602_REG_ACCEL_CONFIG2, 0x03U);
    icm20602_write_reg(ICM20602_REG_ACCEL_CONFIG, ICM20602_ACCEL_FS_16G);
    icm20602_write_reg(ICM20602_REG_GYRO_CONFIG, ICM20602_GYRO_FS_2000DPS);
    icm20602_write_reg(ICM20602_REG_INT_PIN_CFG, 0x00U);

    who = read_who_am_i_retry();
    s_icm_state.is_ready = (who == DVE_ICM20602_WHO_AM_I_VALUE);
    if (s_icm_state.is_ready) {
        mount_yaw_rad = DVE_ICM20602_MOUNT_YAW_DEG * ICM20602_DEG_TO_RAD;
        s_mount_yaw_sin = sinf(mount_yaw_rad);
        s_mount_yaw_cos = cosf(mount_yaw_rad);
        calibrate_gyro();
#if (DVE_ICM20602_ENABLE_ACCEL_XY_CALIB != 0U)
        calibrate_accel_xy();
#else
        s_accel_x_offset_g = 0.0f;
        s_accel_y_offset_g = 0.0f;
        s_accel_x_filter_g = 0.0f;
        s_accel_y_filter_g = 0.0f;
        s_icm_state.accel_x_g = 0.0f;
        s_icm_state.accel_y_g = 0.0f;
#endif
    }

    return s_icm_state.is_ready;
}

/*
*********************************************************************************************************
*   函 数 名: icm20602_read_data
*   功能说明: 读取ICM20602并转换为物理量
*   形    参: data - 物理量数据输出
*   返 回 值: 无
*********************************************************************************************************
*/
static void icm20602_read_data(ICM20602Data_t *data)
{
    ICM20602Raw_t raw;

    if (data == 0) {
        return;
    }

    icm20602_read_raw(&raw);

    data->accel_x_g  = ((float)raw.accel_x) / ICM20602_ACCEL_SENS_16G;
    data->accel_y_g  = ((float)raw.accel_y) / ICM20602_ACCEL_SENS_16G;
    data->accel_z_g  = ((float)raw.accel_z) / ICM20602_ACCEL_SENS_16G;
    data->gyro_x_dps = ((float)raw.gyro_x) / ICM20602_GYRO_SENS_2000DPS;
    data->gyro_y_dps = ((float)raw.gyro_y) / ICM20602_GYRO_SENS_2000DPS;
    data->gyro_z_dps = ((float)raw.gyro_z) / ICM20602_GYRO_SENS_2000DPS;
}

static ICM20602Data_t icm20602_axis_map(ICM20602Data_t data)
{
    ICM20602Data_t mapped_data;

    mapped_data.accel_x_g = DVE_ICM20602_MAP_ACCEL_X(data);
    mapped_data.accel_y_g = DVE_ICM20602_MAP_ACCEL_Y(data);
    mapped_data.accel_z_g = DVE_ICM20602_MAP_ACCEL_Z(data);
    mapped_data.gyro_x_dps = DVE_ICM20602_MAP_GYRO_X(data);
    mapped_data.gyro_y_dps = DVE_ICM20602_MAP_GYRO_Y(data);
    mapped_data.gyro_z_dps = DVE_ICM20602_MAP_GYRO_Z(data);

    return mapped_data;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_ICM20602_Update
*   功能说明: 更新ICM20602姿态角和传感器状态
*   形    参: dt_s - 更新周期，单位s
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_ICM20602_Update(float dt_s)
{
#if (DVE_ICM20602_ENABLE_ATTITUDE_SOLVE != 0U)
    ICM20602Data_t data;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mahony_yaw = 0.0f;

    if (!s_icm_state.is_ready) {
        return;
    }

    icm20602_read_data(&data);
    data = icm20602_axis_map(data);
    
    gyro_x = data.gyro_x_dps - s_gyro_x_offset_dps;
    gyro_y = data.gyro_y_dps - s_gyro_y_offset_dps;
    gyro_z = (data.gyro_z_dps - s_gyro_z_offset_dps) * DVE_ICM20602_YAW_GYRO_SIGN;

#if (DVE_ICM20602_USE_MAHONY != 0U)
    mahony_update_nomagnetic(gyro_x,
                             gyro_y,
                             gyro_z * DVE_ICM20602_YAW_SCALE,
                             data.accel_x_g,
                             data.accel_y_g,
                             data.accel_z_g,
                             dt_s);
    calc_quaternion_attitude(&s_icm_state.roll_deg, &s_icm_state.pitch_deg, &mahony_yaw);
#else
    calc_accel_attitude(&data, &s_icm_state.roll_deg, &s_icm_state.pitch_deg);
#endif

    // 与原独立ICM20602工程一致：无磁力计时yaw使用扣零偏后的Z轴角速度相对积分
    (void)mahony_yaw;
    s_relative_yaw_deg += apply_deadband(gyro_z) * dt_s * DVE_ICM20602_YAW_SCALE;
    s_relative_yaw_deg = wrap_180(s_relative_yaw_deg);
    s_icm_state.yaw_deg = wrap_180((s_relative_yaw_deg * DVE_ICM20602_YAW_ANGLE_SIGN) +
                                   DVE_ICM20602_YAW_INSTALL_OFFSET_DEG);
#else
    (void)dt_s;
#endif
}

/*
*********************************************************************************************************
*   函 数 名: DVE_ICM20602_SetZero
*   功能说明: 清零相对yaw角
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_ICM20602_SetZero(void)
{
    s_relative_yaw_deg = 0.0f;
    s_icm_state.yaw_deg = 0.0f;
#if (DVE_ICM20602_ENABLE_ATTITUDE_SOLVE != 0U)
    reset_attitude_quaternion();
#endif
}

/*
*********************************************************************************************************
*   函 数 名: DVE_ICM20602_GetState
*   功能说明: 获取ICM20602状态
*   形    参: state - 状态输出结构体
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_ICM20602_GetState(DVE_ICM20602State_t *state)
{
    if (state == 0) {
        return;
    }

    *state = s_icm_state;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_ICM20602_ReadAccelXY
*   功能说明: 仅读取并输出车体X/Y轴动态加速度
*   形    参: accel_x_g - X轴加速度输出；accel_y_g - Y轴加速度输出
*   返 回 值: true读取成功，false设备未就绪或参数无效
*********************************************************************************************************
*/
bool DVE_ICM20602_ReadAccelXY(float *accel_x_g, float *accel_y_g)
{
    int16_t raw_x;
    int16_t raw_y;
    float sensor_x_g;
    float sensor_y_g;
    float mapped_x_g;
    float mapped_y_g;
    float car_x_g;
    float car_y_g;

    if ((!s_icm_state.is_ready) || (accel_x_g == 0) || (accel_y_g == 0)) {
        return false;
    }

    icm20602_read_accel_xy_raw(&raw_x, &raw_y);

    sensor_x_g = ((float)raw_x / ICM20602_ACCEL_SENS_16G) - s_accel_x_offset_g;
    sensor_y_g = ((float)raw_y / ICM20602_ACCEL_SENS_16G) - s_accel_y_offset_g;

#if (DVE_ICM20602_ENABLE_FRAME_TRANSFORM != 0U)
    icm20602_map_install_direction(sensor_x_g, sensor_y_g, &mapped_x_g, &mapped_y_g);
    icm20602_rotate_to_car_frame(mapped_x_g, mapped_y_g, &car_x_g, &car_y_g);
#else
    car_x_g = sensor_x_g;
    car_y_g = sensor_y_g;
#endif

    car_x_g *= DVE_ICM20602_ACCEL_X_SIGN;
    car_y_g *= DVE_ICM20602_ACCEL_Y_SIGN;

    s_accel_x_filter_g += DVE_ICM20602_ACCEL_XY_LPF_ALPHA *
                          (car_x_g - s_accel_x_filter_g);
    s_accel_y_filter_g += DVE_ICM20602_ACCEL_XY_LPF_ALPHA *
                          (car_y_g - s_accel_y_filter_g);

    s_icm_state.accel_x_g = s_accel_x_filter_g;
    s_icm_state.accel_y_g = s_accel_y_filter_g;

    *accel_x_g = s_icm_state.accel_x_g;
    *accel_y_g = s_icm_state.accel_y_g;

    return true;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_ICM20602_ReadWhoAmI
*   功能说明: 读取WHO_AM_I寄存器
*   形    参: 无
*   返 回 值: WHO_AM_I寄存器值
*********************************************************************************************************
*/
uint8_t DVE_ICM20602_ReadWhoAmI(void)
{
    return icm20602_read_reg(ICM20602_REG_WHO_AM_I);
}
