#ifndef DVE_IR_SENSOR_H_
#define DVE_IR_SENSOR_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define DVE_IR_CHANNEL_NUM       (8U)     /* 循迹传感器通道数量 */
#define DVE_IR_ERROR_MODE_WEIGHTED_SUM      (0U)     /* 加权求和误差 */
#define DVE_IR_ERROR_MODE_WEIGHTED_AVERAGE  (1U)     /* 加权平均误差 */

#ifndef DVE_IR_ERROR_MODE
#define DVE_IR_ERROR_MODE        DVE_IR_ERROR_MODE_WEIGHTED_AVERAGE
#endif
/* 0: 正常接线 D1->D8
 * 1: 反向接线 D8->D1
 */
#define DVE_IR_REVERSE_CHANNEL   (1U)
/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    uint8_t ch[DVE_IR_CHANNEL_NUM];       /* 8路原始值，1表示检测到黑线 */
    uint8_t black_count;                  /* 检测到黑线的通道数量 */
    int16_t error;                        /* 循迹偏差，左负右正 */
    float error_f;                        /* 浮点偏差，给PID使用 */
    uint8_t is_lost;                      /* 1表示丢线 */
} DVE_IRSensor_t;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void DVE_IRSensor_Init(void);
void DVE_IRSensor_Read(DVE_IRSensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* DVE_IR_SENSOR_H_ */
