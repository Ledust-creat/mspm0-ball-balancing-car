#include "dve_hts25l.h"
#include "bsp_servo_bus.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define HTS25L_FRAME_HEADER                 (0x55U)
#define HTS25L_CMD_MOVE_TIME_WRITE          (1U)
#define HTS25L_CMD_MODE_WRITE               (29U)
#define HTS25L_CMD_LOAD_WRITE               (31U)
#define HTS25L_MOVE_FRAME_LEN               (10U)
#define HTS25L_MODE_FRAME_LEN               (10U)
#define HTS25L_LOAD_FRAME_LEN               (7U)
#define HTS25L_PARAM_LEN_MOVE               (7U)
#define HTS25L_PARAM_LEN_MODE               (7U)
#define HTS25L_PARAM_LEN_LOAD               (4U)
#define HTS25L_SERVO_MODE                   (0U)
#define HTS25L_LOAD_ENABLE                  (1U)

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static uint16_t s_command_position = DVE_HTS25L_POS_CENTER;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static uint16_t limit_position(uint16_t pos);
static uint8_t calc_checksum(const uint8_t *data, uint8_t len);
static bool set_servo_mode(void);
static bool set_servo_load(void);

/*
*********************************************************************************************************
*   函 数 名: limit_position
*   功能说明: 将舵机位置限制在机械安全范围
*   形    参: pos - 输入位置
*   返 回 值: 限幅后位置
*********************************************************************************************************
*/
static uint16_t limit_position(uint16_t pos)
{
    if (pos < DVE_HTS25L_POS_MIN) {
        return DVE_HTS25L_POS_MIN;
    }

    if (pos > DVE_HTS25L_POS_MAX) {
        return DVE_HTS25L_POS_MAX;
    }

    return pos;
}

/*
*********************************************************************************************************
*   函 数 名: calc_checksum
*   功能说明: 计算HTS-25L协议反码校验和
*   形    参: data - 从ID开始的数据；len - 数据长度
*   返 回 值: 校验和
*********************************************************************************************************
*/
static uint8_t calc_checksum(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint16_t sum = 0U;

    for (i = 0U; i < len; i++) {
        sum += data[i];
    }

    return (uint8_t)(~sum);
}

/*
*********************************************************************************************************
*   函 数 名: set_servo_mode
*   功能说明: 将舵机设置为位置控制模式
*   形    参: 无
*   返 回 值: true发送成功，false发送失败
*********************************************************************************************************
*/
static bool set_servo_mode(void)
{
    uint8_t frame[HTS25L_MODE_FRAME_LEN];

    frame[0] = HTS25L_FRAME_HEADER;
    frame[1] = HTS25L_FRAME_HEADER;
    frame[2] = DVE_HTS25L_DEFAULT_ID;
    frame[3] = HTS25L_PARAM_LEN_MODE;
    frame[4] = HTS25L_CMD_MODE_WRITE;
    frame[5] = HTS25L_SERVO_MODE;
    frame[6] = 0U;
    frame[7] = 0U;
    frame[8] = 0U;
    frame[9] = calc_checksum(&frame[2], 7U);

    return BSP_ServoBus_Send(frame, HTS25L_MODE_FRAME_LEN);
}

/*
*********************************************************************************************************
*   函 数 名: set_servo_load
*   功能说明: 使能舵机扭矩输出
*   形    参: 无
*   返 回 值: true发送成功，false发送失败
*********************************************************************************************************
*/
static bool set_servo_load(void)
{
    uint8_t frame[HTS25L_LOAD_FRAME_LEN];

    frame[0] = HTS25L_FRAME_HEADER;
    frame[1] = HTS25L_FRAME_HEADER;
    frame[2] = DVE_HTS25L_DEFAULT_ID;
    frame[3] = HTS25L_PARAM_LEN_LOAD;
    frame[4] = HTS25L_CMD_LOAD_WRITE;
    frame[5] = HTS25L_LOAD_ENABLE;
    frame[6] = calc_checksum(&frame[2], 4U);

    return BSP_ServoBus_Send(frame, HTS25L_LOAD_FRAME_LEN);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_HTS25L_Init
*   功能说明: 初始化HTS-25L并回到安全中位
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_HTS25L_Init(void)
{
    BSP_ServoBus_Init();
    (void)set_servo_mode();
    (void)set_servo_load();
    (void)DVE_HTS25L_SetPosition(DVE_HTS25L_POS_CENTER, DVE_HTS25L_MOVE_TIME_MS);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_HTS25L_SetPosition
*   功能说明: 发送安全限幅后的舵机位置命令
*   形    参: pos - 目标位置；time_ms - 运动时间
*   返 回 值: true发送成功，false发送失败
*********************************************************************************************************
*/
bool DVE_HTS25L_SetPosition(uint16_t pos, uint16_t time_ms)
{
    uint8_t frame[HTS25L_MOVE_FRAME_LEN];

    pos = limit_position(pos);

    frame[0] = HTS25L_FRAME_HEADER;
    frame[1] = HTS25L_FRAME_HEADER;
    frame[2] = DVE_HTS25L_DEFAULT_ID;
    frame[3] = HTS25L_PARAM_LEN_MOVE;
    frame[4] = HTS25L_CMD_MOVE_TIME_WRITE;
    frame[5] = (uint8_t)(pos & 0xFFU);
    frame[6] = (uint8_t)(pos >> 8);
    frame[7] = (uint8_t)(time_ms & 0xFFU);
    frame[8] = (uint8_t)(time_ms >> 8);
    frame[9] = calc_checksum(&frame[2], 7U);

    if (!BSP_ServoBus_Send(frame, HTS25L_MOVE_FRAME_LEN)) {
        return false;
    }

    s_command_position = pos;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_HTS25L_GetCommandPosition
*   功能说明: 获取最近一次成功发送的舵机位置
*   形    参: 无
*   返 回 值: 舵机位置
*********************************************************************************************************
*/
uint16_t DVE_HTS25L_GetCommandPosition(void)
{
    return s_command_position;
}
