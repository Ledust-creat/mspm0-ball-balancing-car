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
#define APP_CMD_HEAD                    (0xA5U)     /* 协议包头 */
#define APP_CMD_TAIL                    (0x5AU)     /* 协议包尾 */
#define APP_CMD_PACKET_LEN              (8U)        /* 固定协议长度 */
#define APP_CMD_WAVE_PERIOD_MS          (100U)      /* 二进制波形回传周期 */
#define APP_CMD_TEXT_PERIOD_MS          (3000U)     /* 文本状态回传周期 */

#define APP_CMD_ENABLE_ACK_FRAME        (0U)        /* 1发送ACK二进制帧，0不发送 */
#define APP_CMD_ENABLE_ERR_FRAME        (0U)        /* 1发送ERR二进制帧，0不发送 */
#define APP_CMD_ENABLE_WAVE_FRAME       (1U)        /* 1发送波形二进制帧，0不发送 */
#define APP_CMD_ENABLE_TEXT_STATUS      (1U)        /* 1发送文本状态，0不发送 */

#define CMD_START                       (0x01U)     /* 启动运行 */
#define CMD_STOP                        (0x02U)     /* 停止运行 */
#define CMD_SET_SPEED                   (0x03U)     /* 设置循迹基础速度 */
#define CMD_SET_KP                      (0x04U)     /* 设置循迹外环Kp */
#define CMD_SET_KI                      (0x05U)     /* 设置循迹外环Ki */
#define CMD_SET_KD                      (0x06U)     /* 设置循迹外环Kd */
#define CMD_SET_LEFT_RATIO              (0x07U)     /* 设置左轮PWM补偿 */
#define CMD_SET_RIGHT_RATIO             (0x08U)     /* 设置右轮PWM补偿 */
#define CMD_QUERY_STATUS                (0x09U)     /* 查询状态 */
#define CMD_SET_LEFT_SPEED              (0x0AU)     /* 设置左轮目标速度，调试命令 */
#define CMD_SET_RIGHT_SPEED             (0x0BU)     /* 设置右轮目标速度，调试命令 */
#define CMD_SET_LEFT_KP                 (0x0CU)     /* 设置左轮速度环Kp */
#define CMD_SET_LEFT_KI                 (0x0DU)     /* 设置左轮速度环Ki */
#define CMD_SET_LEFT_KD                 (0x0EU)     /* 设置左轮速度环Kd */
#define CMD_SET_RIGHT_KP                (0x0FU)     /* 设置右轮速度环Kp */
#define CMD_SET_RIGHT_KI                (0x10U)     /* 设置右轮速度环Ki */
#define CMD_SET_RIGHT_KD                (0x11U)     /* 设置右轮速度环Kd */

#define CMD_ACK                         (0x80U)     /* 通用应答，data=原命令字 */
#define CMD_STATUS_ERROR                (0x82U)     /* 回传循迹误差，兼容图像显示 */
#define CMD_STATUS_TURN                 (0x83U)     /* 回传转向修正速度 */
#define CMD_STATUS_BASE                 (0x84U)     /* 回传基础速度 */
#define CMD_STATUS_KP                   (0x85U)     /* 回传循迹Kp */
#define CMD_STATUS_KD                   (0x86U)     /* 回传循迹Kd */
#define CMD_STATUS_RUN                  (0x87U)     /* 回传运行状态 */
#define CMD_STATUS_LEFT_SPEED           (0x88U)     /* 回传左轮实际速度 */
#define CMD_STATUS_RIGHT_SPEED          (0x89U)     /* 回传右轮实际速度 */
#define CMD_STATUS_LEFT_PWM             (0x8AU)     /* 回传左轮PWM输出 */
#define CMD_STATUS_RIGHT_PWM            (0x8BU)     /* 回传右轮PWM输出 */
#define CMD_ERR                         (0xE0U)     /* 错误应答 */

#define APP_CMD_ERR_FORMAT              (1.0f)      /* 包格式错误 */
#define APP_CMD_ERR_CHECKSUM            (2.0f)      /* 校验错误 */
#define APP_CMD_ERR_UNKNOWN             (3.0f)      /* 未知命令 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_Command_Task(void);
void Command_Parse(uint8_t *buf, uint8_t len);
void Command_PackAndSend(uint8_t cmd, float data);
void Command_SendStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMAND_H_ */
