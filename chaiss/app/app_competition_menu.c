#include "app_competition_menu.h"
#include "bsp_key.h"
#include "interrupt.h"
#include "OLED.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_MENU_KEY_SCAN_MS             (10U)      /* 菜单按键扫描周期，单位ms */
#define APP_MENU_RUN_TIME_REFRESH_MS     (100U)     /* RUN页时间数字刷新周期，单位ms */
#define APP_MENU_COUNTDOWN_MS            (3000U)    /* 倒计时时长，单位ms */
#define APP_MENU_DEBOUNCE_COUNT          (3U)       /* 按键消抖连续稳定次数，单位次 */
#define APP_MENU_LINE_WIDTH              (16U)      /* OLED一行字符数，单位char */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    bool raw_last;                        /* 上一次原始按键状态，true表示按下 */
    bool stable_state;                    /* 消抖后的稳定状态，true表示按下 */
    uint8_t stable_count;                 /* 连续相同采样次数 */
    bool pressed_event;                   /* 单次按下事件，消费后清除 */
} APP_KeyFilter_t;

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/
static const char * const s_mode_name_table[APP_COMP_MODE_COUNT] = {
    "TRACK LAP",
    "STATIC O+5-5",
    "AB CENTER",
    "LAP CENTER",
    "LAP TARGET"
};

static APP_KeyFilter_t s_select_key;              /* SELECT按键消抖状态 */
static APP_KeyFilter_t s_confirm_key;             /* CONFIRM按键消抖状态 */

static APP_CompetitionMode_t s_selected_mode;     /* 当前选中的比赛小题 */
static APP_CompetitionUiState_t s_ui_state;       /* 当前OLED菜单状态 */

static bool s_start_request;                      /* 倒计时结束产生的一次性启动请求 */
static bool s_stop_request;                       /* 用户停止产生的一次性停止请求 */
static bool s_oled_dirty;                         /* 非RUN页面需要整页刷新标志 */
static bool s_oled_full_redraw;                   /* 页面切换整页重绘标志 */
static bool s_countdown_cancel_armed;             /* 倒计时取消按键使能，需先松开确认键 */
static bool s_run_page_initialized;               /* RUN固定页面是否已绘制 */

static uint8_t s_fault_code;                      /* 外部通知故障码 */
static uint8_t s_last_countdown_number;           /* 上一次显示的倒计时数字 */
static uint32_t s_countdown_start_ms;             /* 倒计时开始时刻，单位ms */
static uint32_t s_running_start_ms;               /* 当前小题运行开始时刻，单位ms */
static uint32_t s_last_key_scan_ms;               /* 最近一次按键扫描时刻，单位ms */
static uint32_t s_last_time_update_ms;            /* 最近一次RUN时间数字刷新时刻，单位ms */
static uint32_t s_last_display_time_ds;           /* 上一次OLED显示的0.1s计时值 */

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static void key_filter_update(APP_KeyFilter_t *key, bool raw_pressed);
static void menu_scan_keys(void);
static void menu_update_state(uint32_t now_ms);
static void menu_enter_select(void);
static void menu_enter_countdown(uint32_t now_ms);
static void menu_enter_running(uint32_t now_ms);
static void menu_mark_full_redraw(void);
static void menu_draw_current_page(uint32_t now_ms);
static void menu_draw_running_page_once(void);
static void menu_update_running_time_digits(uint32_t now_ms);
static void menu_show_line(uint8_t line, const char *text);
static uint8_t menu_get_countdown_number(uint32_t now_ms);
static uint32_t menu_get_run_time_ds(uint32_t now_ms);
static void menu_append_u32(char *buf, uint32_t value, uint8_t width);

/*
*********************************************************************************************************
*   函 数 名: key_filter_update
*   功能说明: 更新按键消抖状态并生成单次按下事件
*   形    参: key - 按键滤波结构体；raw_pressed - 原始按下状态
*   返 回 值: 无
*********************************************************************************************************
*/
static void key_filter_update(APP_KeyFilter_t *key, bool raw_pressed)
{
    bool old_stable;

    key->pressed_event = false;
    if (raw_pressed == key->raw_last) {
        if (key->stable_count < APP_MENU_DEBOUNCE_COUNT) {
            key->stable_count++;
        }
    } else {
        key->raw_last = raw_pressed;
        key->stable_count = 1U;
    }

    if ((key->stable_count >= APP_MENU_DEBOUNCE_COUNT) &&
        (raw_pressed != key->stable_state)) {
        old_stable = key->stable_state;
        key->stable_state = raw_pressed;
        key->pressed_event = (!old_stable && key->stable_state);
    }
}

/*
*********************************************************************************************************
*   函 数 名: menu_scan_keys
*   功能说明: 扫描两个按键并更新消抖事件
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_scan_keys(void)
{
    key_filter_update(&s_select_key, BSP_Key_IsSelectPressed());
    key_filter_update(&s_confirm_key, BSP_Key_IsConfirmPressed());
}

/*
*********************************************************************************************************
*   函 数 名: menu_enter_select
*   功能说明: 进入小题选择页面
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_enter_select(void)
{
    s_ui_state = APP_COMP_UI_SELECT;
    s_run_page_initialized = false;
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: menu_enter_countdown
*   功能说明: 进入3秒倒计时页面
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_enter_countdown(uint32_t now_ms)
{
    s_ui_state = APP_COMP_UI_COUNTDOWN;
    s_countdown_start_ms = now_ms;
    s_last_countdown_number = 0U;
    s_countdown_cancel_armed = false;
    s_run_page_initialized = false;
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: menu_enter_running
*   功能说明: 进入运行页面并产生一次启动请求
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_enter_running(uint32_t now_ms)
{
    s_ui_state = APP_COMP_UI_RUNNING;
    s_running_start_ms = now_ms;
    s_last_time_update_ms = now_ms;
    s_last_display_time_ds = UINT32_MAX;
    s_run_page_initialized = false;
    s_start_request = true;
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: menu_mark_full_redraw
*   功能说明: 标记OLED需要整页重绘
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_mark_full_redraw(void)
{
    s_oled_dirty = true;
    s_oled_full_redraw = true;
}

/*
*********************************************************************************************************
*   函 数 名: menu_update_state
*   功能说明: 更新菜单状态机和OLED脏标志
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_update_state(uint32_t now_ms)
{
    uint8_t countdown_number;

    switch (s_ui_state) {
        case APP_COMP_UI_SELECT:
            if (s_select_key.pressed_event) {
                s_selected_mode = (APP_CompetitionMode_t)((s_selected_mode + 1U) % APP_COMP_MODE_COUNT);
                menu_mark_full_redraw();
            }
            if (s_confirm_key.pressed_event) {
                menu_enter_countdown(now_ms);
            }
            break;

        case APP_COMP_UI_COUNTDOWN:
            countdown_number = menu_get_countdown_number(now_ms);
            if (countdown_number != s_last_countdown_number) {
                s_last_countdown_number = countdown_number;
                s_oled_dirty = true;
            }

            if (!s_countdown_cancel_armed) {
                if (!s_select_key.stable_state && !s_confirm_key.stable_state) {
                    s_countdown_cancel_armed = true;
                }
            } else if (s_select_key.stable_state) {
                menu_enter_select();
            } else if (s_confirm_key.stable_state) {
                s_start_request = false;
                s_stop_request = true;
                menu_enter_select();
            } else if ((uint32_t)(now_ms - s_countdown_start_ms) >= APP_MENU_COUNTDOWN_MS) {
                menu_enter_running(now_ms);
            }
            break;

        case APP_COMP_UI_RUNNING:
            if (s_confirm_key.pressed_event) {
                s_stop_request = true;
                menu_enter_select();
            }
            break;

        case APP_COMP_UI_FINISHED:
            if (s_select_key.pressed_event) {
                s_selected_mode = (APP_CompetitionMode_t)((s_selected_mode + 1U) % APP_COMP_MODE_COUNT);
                menu_enter_select();
            }
            if (s_confirm_key.pressed_event) {
                menu_enter_countdown(now_ms);
            }
            break;

        case APP_COMP_UI_FAULT:
            if (s_select_key.pressed_event || s_confirm_key.pressed_event) {
                menu_enter_select();
            }
            break;

        default:
            menu_enter_select();
            break;
    }

    s_select_key.pressed_event = false;
    s_confirm_key.pressed_event = false;
}

/*
*********************************************************************************************************
*   函 数 名: menu_show_line
*   功能说明: 显示固定宽度OLED文本行，自动补空格覆盖残留字符
*   形    参: line - OLED行号；text - 显示字符串
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_show_line(uint8_t line, const char *text)
{
    char line_buf[APP_MENU_LINE_WIDTH + 1U];
    uint8_t i;

    for (i = 0U; i < APP_MENU_LINE_WIDTH; i++) {
        if ((text != 0) && (text[i] != '\0')) {
            line_buf[i] = text[i];
        } else {
            line_buf[i] = ' ';
        }
    }
    line_buf[APP_MENU_LINE_WIDTH] = '\0';

    OLED_ShowString(line, 1U, line_buf);
}

/*
*********************************************************************************************************
*   函 数 名: menu_get_countdown_number
*   功能说明: 获取当前倒计时显示数字
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 1~3
*********************************************************************************************************
*/
static uint8_t menu_get_countdown_number(uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - s_countdown_start_ms;
    uint8_t elapsed_s = (uint8_t)(elapsed_ms / 1000U);

    if (elapsed_s >= 3U) {
        return 1U;
    }

    return (uint8_t)(3U - elapsed_s);
}

/*
*********************************************************************************************************
*   函 数 名: menu_get_run_time_ds
*   功能说明: 获取运行时间，单位0.1s
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 运行时间0.1秒计数
*********************************************************************************************************
*/
static uint32_t menu_get_run_time_ds(uint32_t now_ms)
{
    return (uint32_t)((now_ms - s_running_start_ms) / 100U);
}

/*
*********************************************************************************************************
*   函 数 名: menu_append_u32
*   功能说明: 按固定宽度追加无符号整数
*   形    参: buf - 输出缓存；value - 数值；width - 固定宽度
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_append_u32(char *buf, uint32_t value, uint8_t width)
{
    uint8_t i;

    for (i = 0U; i < width; i++) {
        buf[width - 1U - i] = (char)('0' + (value % 10U));
        value /= 10U;
    }
}

/*
*********************************************************************************************************
*   函 数 名: menu_draw_running_page_once
*   功能说明: 首次进入RUN时绘制固定文本，后续只刷新时间数字
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_draw_running_page_once(void)
{
    OLED_Clear();
    menu_show_line(1U, s_mode_name_table[s_selected_mode]);
    menu_show_line(2U, "TIME: 000.0s");
    menu_show_line(3U, "STATE: RUN");
    menu_show_line(4U, "OK: STOP");
    s_run_page_initialized = true;
}

/*
*********************************************************************************************************
*   函 数 名: menu_update_running_time_digits
*   功能说明: 只更新RUN页面TIME后的数字区域
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_update_running_time_digits(uint32_t now_ms)
{
    uint32_t elapsed_ds;
    uint32_t seconds;
    uint32_t tenths;

    if ((uint32_t)(now_ms - s_last_time_update_ms) < APP_MENU_RUN_TIME_REFRESH_MS) {
        return;
    }
    s_last_time_update_ms = now_ms;

    elapsed_ds = menu_get_run_time_ds(now_ms);
    if (elapsed_ds == s_last_display_time_ds) {
        return;
    }

    s_last_display_time_ds = elapsed_ds;
    seconds = elapsed_ds / 10U;
    tenths = elapsed_ds % 10U;

    if (seconds > 999U) {
        seconds = 999U;
    }

    OLED_ShowNum(2U, 7U, seconds, 3U);
    OLED_ShowChar(2U, 10U, '.');
    OLED_ShowNum(2U, 11U, tenths, 1U);
    OLED_ShowChar(2U, 12U, 's');
}

/*
*********************************************************************************************************
*   函 数 名: menu_draw_current_page
*   功能说明: 根据当前状态刷新OLED非RUN页面
*   形    参: now_ms - 当前系统时间，单位ms
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_draw_current_page(uint32_t now_ms)
{
    char line[APP_MENU_LINE_WIDTH + 1U] = {0};
    uint32_t run_ds;

    if (s_oled_full_redraw) {
        OLED_Clear();
        s_oled_full_redraw = false;
    }

    switch (s_ui_state) {
        case APP_COMP_UI_SELECT:
            line[0] = 'H';
            line[1] = ' ';
            line[2] = 'T';
            line[3] = 'E';
            line[4] = 'S';
            line[5] = 'T';
            line[6] = ' ';
            line[7] = (char)('1' + s_selected_mode);
            line[8] = '/';
            line[9] = (char)('0' + APP_COMP_MODE_COUNT);
            menu_show_line(1U, line);
            menu_show_line(2U, s_mode_name_table[s_selected_mode]);
            menu_show_line(3U, "SEL:NEXT");
            menu_show_line(4U, "OK:START");
            break;

        case APP_COMP_UI_COUNTDOWN:
            line[0] = 'S';
            line[1] = 'T';
            line[2] = 'A';
            line[3] = 'R';
            line[4] = 'T';
            line[5] = ' ';
            line[6] = 'I';
            line[7] = 'N';
            line[8] = ':';
            line[9] = ' ';
            line[10] = (char)('0' + menu_get_countdown_number(now_ms));
            menu_show_line(1U, s_mode_name_table[s_selected_mode]);
            menu_show_line(2U, line);
            menu_show_line(3U, "PLEASE WAIT");
            menu_show_line(4U, "SEL/OK:CANCEL");
            break;

        case APP_COMP_UI_FINISHED:
            run_ds = menu_get_run_time_ds(now_ms);
            menu_show_line(1U, "TEST FINISHED");
            line[0] = 'T';
            line[1] = 'I';
            line[2] = 'M';
            line[3] = 'E';
            line[4] = ':';
            line[5] = ' ';
            menu_append_u32(&line[6], run_ds / 10U, 3U);
            line[9] = '.';
            line[10] = (char)('0' + (run_ds % 10U));
            line[11] = 's';
            menu_show_line(2U, line);
            menu_show_line(3U, "STATE: DONE");
            menu_show_line(4U, "OK:RESTART");
            break;

        case APP_COMP_UI_FAULT:
            line[0] = 'F';
            line[1] = 'A';
            line[2] = 'U';
            line[3] = 'L';
            line[4] = 'T';
            line[5] = ':';
            line[6] = ' ';
            menu_append_u32(&line[7], s_fault_code, 3U);
            menu_show_line(1U, "TEST FAULT");
            menu_show_line(2U, line);
            menu_show_line(3U, "STATE: FAULT");
            menu_show_line(4U, "KEY:BACK");
            break;

        default:
            break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_Init
*   功能说明: 初始化比赛菜单模块
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionMenu_Init(void)
{
    s_select_key.raw_last = false;
    s_select_key.stable_state = false;
    s_select_key.stable_count = 0U;
    s_select_key.pressed_event = false;

    s_confirm_key.raw_last = false;
    s_confirm_key.stable_state = false;
    s_confirm_key.stable_count = 0U;
    s_confirm_key.pressed_event = false;

    s_selected_mode = APP_COMP_MODE_TRACK_LAP;
    s_ui_state = APP_COMP_UI_SELECT;
    s_start_request = false;
    s_stop_request = false;
    s_oled_dirty = false;
    s_oled_full_redraw = false;
    s_countdown_cancel_armed = false;
    s_run_page_initialized = false;
    s_fault_code = 0U;
    s_last_countdown_number = 0U;
    s_countdown_start_ms = 0U;
    s_running_start_ms = 0U;
    s_last_key_scan_ms = 0U;
    s_last_time_update_ms = 0U;
    s_last_display_time_ds = UINT32_MAX;
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_Task
*   功能说明: 执行按键扫描、菜单状态机和OLED刷新
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionMenu_Task(void)
{
    APP_CompetitionMenu_UpdateTask();
    APP_CompetitionMenu_DisplayTask();
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_UpdateTask
*   功能说明: 扫描按键并更新菜单状态，不访问OLED
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionMenu_UpdateTask(void)
{
    uint32_t now_ms = Get_Tick();

    if ((uint32_t)(now_ms - s_last_key_scan_ms) >= APP_MENU_KEY_SCAN_MS) {
        s_last_key_scan_ms = now_ms;
        menu_scan_keys();
    }

    menu_update_state(now_ms);
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_DisplayTask
*   功能说明: 根据页面状态低优先级刷新OLED
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionMenu_DisplayTask(void)
{
    uint32_t now_ms = Get_Tick();

    if (s_ui_state == APP_COMP_UI_RUNNING) {
        if (!s_run_page_initialized || s_oled_full_redraw) {
            menu_draw_running_page_once();
            s_oled_full_redraw = false;
            s_oled_dirty = false;
            s_last_display_time_ds = UINT32_MAX;
        }

        menu_update_running_time_digits(now_ms);
        return;
    }

    if (!s_oled_dirty) {
        return;
    }

    s_oled_dirty = false;
    menu_draw_current_page(now_ms);
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_ConsumeStartRequest
*   功能说明: 读取并清除启动请求
*   形    参: mode - 输出当前选择的小题模式
*   返 回 值: true存在启动请求，false无请求
*********************************************************************************************************
*/
bool APP_CompetitionMenu_ConsumeStartRequest(APP_CompetitionMode_t *mode)
{
    if (!s_start_request) {
        return false;
    }

    s_start_request = false;
    if (mode != 0) {
        *mode = s_selected_mode;
    }

    return true;
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_ConsumeStopRequest
*   功能说明: 读取并清除停止请求
*   形    参: 无
*   返 回 值: true存在停止请求，false无请求
*********************************************************************************************************
*/
bool APP_CompetitionMenu_ConsumeStopRequest(void)
{
    if (!s_stop_request) {
        return false;
    }

    s_stop_request = false;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_NotifyFinished
*   功能说明: 外部通知当前小题已完成
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionMenu_NotifyFinished(void)
{
    s_ui_state = APP_COMP_UI_FINISHED;
    s_run_page_initialized = false;
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_NotifyFault
*   功能说明: 外部通知当前小题故障
*   形    参: fault_code - 故障码
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionMenu_NotifyFault(uint8_t fault_code)
{
    s_fault_code = fault_code;
    s_ui_state = APP_COMP_UI_FAULT;
    s_run_page_initialized = false;
    menu_mark_full_redraw();
}

APP_CompetitionMode_t APP_CompetitionMenu_GetSelectedMode(void)
{
    return s_selected_mode;
}

APP_CompetitionUiState_t APP_CompetitionMenu_GetUiState(void)
{
    return s_ui_state;
}
