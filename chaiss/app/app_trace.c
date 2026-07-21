#include "app_trace.h"
#include "dve_chassis.h"
#include "dve_ir_sensor.h"
#include "PID.h"

/*
*********************************************************************************************************
*   瀹忓畾涔?*********************************************************************************************************
*/
#define APP_TRACE_DEFAULT_BASE_PWM          (300.0f)    /* 寮€鐜惊杩瑰熀纭€PWM */
#define APP_TRACE_DEFAULT_BASE_SPEED_CM_S   (35.0f)     /* 閫熷害鐜惊杩瑰熀纭€閫熷害 */

#define APP_TRACE_DEFAULT_KP                (2.5f)      /* 寰抗澶栫幆Kp */
#define APP_TRACE_DEFAULT_KI                (0.0f)      /* 寰抗澶栫幆Ki */
#define APP_TRACE_DEFAULT_KD                (0.0f)      /* 寰抗澶栫幆Kd */

#define APP_TRACE_MAX_BASE_PWM              (1500.0f)   /* 鍩虹PWM涓婇檺 */
#define APP_TRACE_MAX_BASE_SPEED_CM_S       (50.0f)     /* 鍩虹閫熷害涓婇檺 */
#define APP_TRACE_MAX_TURN_PWM              (800.0f)    /* 杞悜PWM淇涓婇檺 */
#define APP_TRACE_MAX_TURN_SPEED_CM_S       (35.0f)     /* 杞悜閫熷害淇涓婇檺 */

/*
*********************************************************************************************************
*   鍏ㄥ眬鍙橀噺
*********************************************************************************************************
*/
static PID_t s_trace_pid;                         /* 寰抗澶栫幆PID */
static float s_base_pwm = APP_TRACE_DEFAULT_BASE_PWM;
static float s_base_speed_cm_s = APP_TRACE_DEFAULT_BASE_SPEED_CM_S;
static float s_left_test_speed_cm_s = APP_TRACE_DEFAULT_BASE_SPEED_CM_S;
static float s_right_test_speed_cm_s = APP_TRACE_DEFAULT_BASE_SPEED_CM_S;
static bool s_is_running = false;

static float s_last_error = 0.0f;
static float s_last_turn_pwm = 0.0f;
static float s_last_turn_speed = 0.0f;
static uint8_t s_last_black_count = 0U;
static bool s_last_lost = false;

/*
*********************************************************************************************************
*   鍑芥暟澹版槑
*********************************************************************************************************
*/
static float limit_float(float value, float min, float max);

/*
*********************************************************************************************************
*   鍑?鏁?鍚? limit_float
*   鍔熻兘璇存槑: 娴偣闄愬箙
*   褰?   鍙? value 鈥?杈撳叆鍊硷紱min 鈥?鏈€灏忓€硷紱max 鈥?鏈€澶у€?*   杩?鍥?鍊? 闄愬箙鍚庣殑鍊?*********************************************************************************************************
*/
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

/*
*********************************************************************************************************
*   鍑?鏁?鍚? APP_Trace_Init
*   鍔熻兘璇存槑: 鍒濆鍖栧惊杩瑰簲鐢ㄥ眰
*   褰?   鍙? 鏃?*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
void APP_Trace_Init(void)
{
    float max_turn;

    DVE_IRSensor_Init();

#if (APP_TRACE_CONTROL_MODE == APP_TRACE_MODE_SPEED_LOOP)
    max_turn = APP_TRACE_MAX_TURN_SPEED_CM_S;
#else
    max_turn = APP_TRACE_MAX_TURN_PWM;
#endif

    PID_Init(&s_trace_pid,
             APP_TRACE_DEFAULT_KP,
             APP_TRACE_DEFAULT_KI,
             APP_TRACE_DEFAULT_KD,
             0.0f,
             max_turn);

    s_base_pwm = APP_TRACE_DEFAULT_BASE_PWM;
    s_base_speed_cm_s = APP_TRACE_DEFAULT_BASE_SPEED_CM_S;
    s_left_test_speed_cm_s = APP_TRACE_DEFAULT_BASE_SPEED_CM_S;
    s_right_test_speed_cm_s = APP_TRACE_DEFAULT_BASE_SPEED_CM_S;
    s_is_running = false;
        
    DVE_Chassis_Enable(false);
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? APP_Trace_Task
*   鍔熻兘璇存槑: 寰抗浠诲姟锛屼富寰幆涓弽澶嶈皟鐢?*   褰?   鍙? 鏃?*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
void APP_Trace_Task(void)
{
    if (!DVE_Chassis_ConsumeControlFlag()) {
        return;
    }

    if (!s_is_running) {
        DVE_Chassis_Stop();
        return;
    }

#if (APP_TRACE_ENABLE_IR_OUTER_LOOP == 0U)
    s_last_error = 0.0f;
    s_last_black_count = 0U;
    s_last_lost = false;
    s_last_turn_pwm = 0.0f;
    s_last_turn_speed = 0.0f;

#if (APP_TRACE_CONTROL_MODE == APP_TRACE_MODE_SPEED_LOOP)
    DVE_Chassis_SetSpeedTargetCmps(s_left_test_speed_cm_s, s_right_test_speed_cm_s);
    DVE_Chassis_Update20ms();
#else
    DVE_Chassis_SetTracePwm(s_base_pwm, 0.0f);
#endif
    
#else
    {
        DVE_IRSensor_t sensor;
        float turn;

        DVE_IRSensor_Read(&sensor);
        s_last_error = sensor.error_f;
        s_last_black_count = sensor.black_count;
        s_last_lost = sensor.is_lost ? true : false;

        if (sensor.is_lost) {
            DVE_Chassis_Stop();
            s_last_turn_pwm = 0.0f;
            s_last_turn_speed = 0.0f;
            return;
        }

        turn = PID_Calc(&s_trace_pid, 0.0f, sensor.error_f);

#if (APP_TRACE_CONTROL_MODE == APP_TRACE_MODE_SPEED_LOOP)
        s_last_turn_speed = turn;
        s_last_turn_pwm = 0.0f;
        DVE_Chassis_SetTraceSpeedCmps(s_base_speed_cm_s, turn);
        DVE_Chassis_Update20ms();
#else
        s_last_turn_pwm = turn;
        s_last_turn_speed = 0.0f;
        DVE_Chassis_SetTracePwm(s_base_pwm, turn);
#endif
    }
#endif
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? APP_Trace_Start
*   鍔熻兘璇存槑: 鍚姩寰抗
*   褰?   鍙? 鏃?*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
void APP_Trace_Start(void)
{
    PID_Reset(&s_trace_pid);

    s_is_running = true;
    DVE_Chassis_Enable(true);
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? APP_Trace_Stop
*   鍔熻兘璇存槑: 鍋滄寰抗
*   褰?   鍙? 鏃?*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
void APP_Trace_Stop(void)
{
    s_is_running = false;
    DVE_Chassis_Enable(false);
    DVE_Chassis_Stop();
    PID_Reset(&s_trace_pid);
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? APP_Trace_SetBasePwm
*   鍔熻兘璇存槑: 璁剧疆鍩虹PWM锛涢€熷害鐜ā寮忎笅鍏煎涓鸿缃熀纭€閫熷害
*   褰?   鍙? base_pwm 鈥?鍩虹PWM鎴栧熀纭€閫熷害
*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
void APP_Trace_SetBasePwm(float base_pwm)
{
#if (APP_TRACE_CONTROL_MODE == APP_TRACE_MODE_SPEED_LOOP)
    APP_Trace_SetBaseSpeedCmps(base_pwm);
#else
    s_base_pwm = limit_float(base_pwm, 0.0f, APP_TRACE_MAX_BASE_PWM);
#endif
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? APP_Trace_SetBaseSpeedCmps
*   鍔熻兘璇存槑: 璁剧疆寰抗鍩虹閫熷害
*   褰?   鍙? base_cm_s 鈥?鍩虹閫熷害锛屽崟浣峜m/s
*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
void APP_Trace_SetBaseSpeedCmps(float base_cm_s)
{
    s_base_speed_cm_s = limit_float(base_cm_s, 0.0f, APP_TRACE_MAX_BASE_SPEED_CM_S);
    s_left_test_speed_cm_s = s_base_speed_cm_s;
    s_right_test_speed_cm_s = s_base_speed_cm_s;
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? APP_Trace_SetTestSpeedCmps
*   鍔熻兘璇存槑: 璁剧疆閫熷害鐜崟鐙皟璇曠殑宸﹀彸杞洰鏍囬€熷害
*   褰?   鍙? left_cm_s 鈥?宸﹁疆鐩爣閫熷害锛況ight_cm_s 鈥?鍙宠疆鐩爣閫熷害
*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
void APP_Trace_SetTestSpeedCmps(float left_cm_s, float right_cm_s)
{
    s_left_test_speed_cm_s = limit_float(left_cm_s,
                                         -APP_TRACE_MAX_BASE_SPEED_CM_S,
                                         APP_TRACE_MAX_BASE_SPEED_CM_S);
    s_right_test_speed_cm_s = limit_float(right_cm_s,
                                          -APP_TRACE_MAX_BASE_SPEED_CM_S,
                                          APP_TRACE_MAX_BASE_SPEED_CM_S);
}

/*
*********************************************************************************************************
*   鍑?鏁?鍚? APP_Trace_SetPid
*   鍔熻兘璇存槑: 璁剧疆寰抗澶栫幆PID鍙傛暟
*   褰?   鍙? kp, ki, kd 鈥?PID鍙傛暟
*   杩?鍥?鍊? 鏃?*********************************************************************************************************
*/
void APP_Trace_SetPid(float kp, float ki, float kd)
{
    float max_turn;

#if (APP_TRACE_CONTROL_MODE == APP_TRACE_MODE_SPEED_LOOP)
    max_turn = APP_TRACE_MAX_TURN_SPEED_CM_S;
#else
    max_turn = APP_TRACE_MAX_TURN_PWM;
#endif

    PID_SetParam(&s_trace_pid, kp, ki, kd, 0.0f, max_turn);
    PID_Reset(&s_trace_pid);
}

/*
*********************************************************************************************************
*   函 数 名: APP_Trace_GetState
*   功能说明: 获取循迹应用层状态快照
*   形    参: state — 状态输出结构体
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_Trace_GetState(APP_TraceState_t *state)
{
    if (state == 0) {
        return;
    }

    state->base_pwm = s_base_pwm;
    state->base_speed_cm_s = s_base_speed_cm_s;
    state->trace_pid = s_trace_pid;
    state->last_error = s_last_error;
    state->last_turn_pwm = s_last_turn_pwm;
    state->last_turn_speed = s_last_turn_speed;
    state->last_black_count = s_last_black_count;
    state->is_lost = s_last_lost;
    state->is_running = s_is_running;
}
