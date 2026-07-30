#ifndef BSP_KEY_H_
#define BSP_KEY_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void BSP_Key_Init(void);
bool BSP_Key_IsSelectPressed(void);
bool BSP_Key_IsConfirmPressed(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_KEY_H_ */
