#include "dve_ir_sensor.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define DVE_IR_BLACK_LEVEL        (1U)     /* 黑线有效电平：1=高电平为黑线，0=低电平为黑线 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static uint8_t read_ir_level(GPIO_Regs *port, uint32_t pin);

/*
*********************************************************************************************************
*   函 数 名: read_ir_level
*   功能说明: 读取单个循迹传感器通道，并转换为黑线有效值
*   形    参: port — GPIO端口；pin — GPIO引脚
*   返 回 值: 1表示检测到黑线，0表示未检测到黑线
*********************************************************************************************************
*/
static uint8_t read_ir_level(GPIO_Regs *port, uint32_t pin)
{
    uint8_t level;

    level = (DL_GPIO_readPins(port, pin) == pin) ? 1U : 0U;

#if (DVE_IR_BLACK_LEVEL == 1U)
    return level;
#else
    return !level;
#endif
}

/*
*********************************************************************************************************
*   函 数 名: DVE_IRSensor_Init
*   功能说明: 初始化循迹传感器设备层
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_IRSensor_Init(void)
{
    /*
     * GPIO输入模式由SysConfig完成，这里暂时不需要额外初始化。
     */
}

/*
*********************************************************************************************************
*   函 数 名: DVE_IRSensor_Read
*   功能说明: 读取8路循迹传感器，并计算循迹偏差
*   形    参: sensor — 循迹传感器数据结构体
*   返 回 值: 无
*********************************************************************************************************
*/
void DVE_IRSensor_Read(DVE_IRSensor_t *sensor)
{
    static const int8_t weights[DVE_IR_CHANNEL_NUM] = {-7, -5, -3, -1, 1, 3, 5, 7};

    int16_t weighted_sum = 0;
    uint8_t black_count = 0;

    if (sensor == 0) {
        return;
    }

    sensor->ch[0] = read_ir_level(Digital_D1_PORT, Digital_D1_PIN);
    sensor->ch[1] = read_ir_level(Digital_D2_PORT, Digital_D2_PIN);
    sensor->ch[2] = read_ir_level(Digital_D3_PORT, Digital_D3_PIN);
    sensor->ch[3] = read_ir_level(Digital_D4_PORT, Digital_D4_PIN);
    sensor->ch[4] = read_ir_level(Digital_D5_PORT, Digital_D5_PIN);
    sensor->ch[5] = read_ir_level(Digital_D6_PORT, Digital_D6_PIN);
    sensor->ch[6] = read_ir_level(Digital_D7_PORT, Digital_D7_PIN);
    sensor->ch[7] = read_ir_level(Digital_D8_PORT, Digital_D8_PIN);

    for (uint8_t i = 0; i < DVE_IR_CHANNEL_NUM; i++) {
        if (sensor->ch[i]) {
            weighted_sum += weights[i];
            black_count++;
        }
    }

    sensor->black_count = black_count;

    if (black_count == 0U) {
        sensor->error = 0;
        sensor->error_f = 0.0f;
        sensor->is_lost = 1U;
        return;
    }

    /*
     * 第一版不除以black_count，保持偏差响应更明显。
     * 后续如果车左右摆动明显，可以改成 weighted_sum / black_count。
     */
    sensor->error = weighted_sum;
    sensor->error_f = (float)weighted_sum;
    sensor->is_lost = 0U;
}