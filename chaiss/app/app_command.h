#ifndef APP_COMMAND_H_
#define APP_COMMAND_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_CMD_HEAD                         (0xA5U)
#define APP_CMD_TAIL                         (0x5AU)
#define APP_CMD_PACKET_LEN                   (8U)
#define APP_CMD_WAVE_PERIOD_MS               (100U)
#define APP_CMD_TEXT_PERIOD_MS               (3000U)

#define APP_CMD_ENABLE_ACK_FRAME             (0U)
#define APP_CMD_ENABLE_ERR_FRAME             (0U)
#define APP_CMD_ENABLE_WAVE_FRAME            (0U)
#define APP_CMD_ENABLE_TEXT_STATUS           (1U)

#define APP_CMD_TUNE_TARGET_LINE             (0U)
#define APP_CMD_TUNE_TARGET_GYRO             (1U)
#define APP_CMD_TUNE_TARGET_LEFT_SPEED       (2U)
#define APP_CMD_TUNE_TARGET_RIGHT_SPEED      (3U)
#define APP_CMD_TUNE_TARGET_BALL             (4U)

#ifndef APP_CMD_TUNE_TARGET
#define APP_CMD_TUNE_TARGET                  APP_CMD_TUNE_TARGET_BALL
#endif

#if (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LINE)
#define APP_CMD_TUNE_TARGET_NAME             "line"
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_GYRO)
#define APP_CMD_TUNE_TARGET_NAME             "gyro"
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_LEFT_SPEED)
#define APP_CMD_TUNE_TARGET_NAME             "left_speed"
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_RIGHT_SPEED)
#define APP_CMD_TUNE_TARGET_NAME             "right_speed"
#elif (APP_CMD_TUNE_TARGET == APP_CMD_TUNE_TARGET_BALL)
#define APP_CMD_TUNE_TARGET_NAME             "ball"
#else
#error "Unsupported APP_CMD_TUNE_TARGET"
#endif

#define CMD_START                            (0x01U)     /* 启动底盘控制 */
#define CMD_STOP                             (0x02U)     /* 停止底盘控制 */
#define CMD_SET_SPEED                        (0x03U)     /* 设置底盘基础速度cm/s */
#define CMD_SET_KP                           (0x04U)     /* 设置当前调参对象Kp */
#define CMD_SET_KI                           (0x05U)     /* 设置当前调参对象Ki */
#define CMD_SET_KD                           (0x06U)     /* 设置当前调参对象Kd */
#define CMD_SET_LEFT_RATIO                   (0x07U)     /* 设置左轮PWM补偿比例 */
#define CMD_SET_RIGHT_RATIO                  (0x08U)     /* 设置右轮PWM补偿比例 */
#define CMD_QUERY_STATUS                     (0x09U)     /* 立即查询状态 */
#define CMD_SET_LEFT_SPEED                   (0x0AU)     /* 设置左轮目标速度cm/s */
#define CMD_SET_RIGHT_SPEED                  (0x0BU)     /* 设置右轮目标速度cm/s */
#define CMD_SET_LEFT_KP                      (0x0CU)     /* 设置左轮速度环Kp */
#define CMD_SET_LEFT_KI                      (0x0DU)     /* 设置左轮速度环Ki */
#define CMD_SET_LEFT_KD                      (0x0EU)     /* 设置左轮速度环Kd */
#define CMD_SET_RIGHT_KP                     (0x0FU)     /* 设置右轮速度环Kp */
#define CMD_SET_RIGHT_KI                     (0x10U)     /* 设置右轮速度环Ki */
#define CMD_SET_RIGHT_KD                     (0x11U)     /* 设置右轮速度环Kd */
#define CMD_BALL_START                       (0x12U)     /* 启动钢球闭环，DATA不做要求 */
#define CMD_SET_BALL_TARGET                  (0x13U)     /* 设置钢球目标位置cm */
#define CMD_SET_BALL_POSITION                (0x14U)     /* 写入钢球实测/模拟位置cm */
#define CMD_SET_SERVO_POSITION               (0x15U)     /* 设置舵机开环位置430~570 */
#define CMD_BALL_STOP                        (0x16U)     /* 停止钢球闭环，DATA不做要求 */

#define CMD_ACK                              (0x80U)     /* 命令执行成功应答 */
#define CMD_STATUS_ERROR                     (0x82U)     /* 当前调参对象误差 */
#define CMD_STATUS_TURN                      (0x83U)     /* 转向修正或钢球角度命令 */
#define CMD_STATUS_BASE                      (0x84U)     /* 底盘基础速度 */
#define CMD_STATUS_KP                        (0x85U)     /* 当前调参对象Kp */
#define CMD_STATUS_KD                        (0x86U)     /* 当前调参对象Kd */
#define CMD_STATUS_RUN                       (0x87U)     /* 底盘运行状态 */
#define CMD_STATUS_LEFT_SPEED                (0x88U)     /* 左轮实际速度 */
#define CMD_STATUS_RIGHT_SPEED               (0x89U)     /* 右轮实际速度 */
#define CMD_STATUS_LEFT_PWM                  (0x8AU)     /* 左轮PWM输出 */
#define CMD_STATUS_RIGHT_PWM                 (0x8BU)     /* 右轮PWM输出 */
#define CMD_ERR                              (0xE0U)     /* 命令错误应答 */

#define APP_CMD_ERR_FORMAT                   (1.0f)
#define APP_CMD_ERR_CHECKSUM                 (2.0f)
#define APP_CMD_ERR_UNKNOWN                  (3.0f)
#define APP_CMD_ERR_PARAM                    (4.0f)

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_Command_Task(void);
void APP_Command_Init(void);
void Command_Parse(uint8_t *buf, uint8_t len);
void Command_PackAndSend(uint8_t cmd, float data);
void Command_SendStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMAND_H_ */
