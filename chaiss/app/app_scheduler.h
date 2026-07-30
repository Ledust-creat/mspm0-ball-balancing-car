#ifndef APP_SCHEDULER_H_
#define APP_SCHEDULER_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
void APP_Scheduler_Init(void);
void APP_Scheduler_RunOnce(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCHEDULER_H_ */
