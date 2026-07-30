#include "app_competition_menu.h"
#include "bsp_key.h"
#include "interrupt.h"
#include "OLED.h"

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/
#define APP_MENU_FRAME_MS               (20U)       /* UI低优先级框架周期 */
#define APP_MENU_KEY_SCAN_MS            (10U)       /* 按键扫描周期 */
#define APP_MENU_KEY_PHASE_MS           (5U)        /* 按键扫描相位 */
#define APP_MENU_OLED_PHASE_MS          (13U)       /* OLED刷新相位 */
#define APP_MENU_RUN_REFRESH_MS         (200U)      /* 运行页面刷新周期 */
#define APP_MENU_COUNTDOWN_MS           (3000U)     /* 倒计时时长 */
#define APP_MENU_DEBOUNCE_COUNT         (3U)        /* 消抖连续稳定次数 */
#define APP_MENU_LINE_WIDTH             (16U)       /* OLED一行字符数 */

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/
typedef struct
{
    bool raw_last;                       /* 上一次原始状态 */
    bool stable_state;                   /* 消抖后的稳定状态 */
    uint8_t stable_count;                /* 连续相同计数 */
    bool pressed_event;                  /* 单次按下事件 */
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

static APP_KeyFilter_t s_select_key;
static APP_KeyFilter_t s_confirm_key;

static APP_CompetitionMode_t s_selected_mode;
static APP_CompetitionUiState_t s_ui_state;

static bool s_start_request;
static bool s_stop_request;
static bool s_oled_dirty;
static bool s_oled_full_redraw;

static uint8_t s_fault_code;
static uint64_t s_countdown_start_ms;
static uint64_t s_running_start_ms;
static uint64_t s_last_key_epoch;
static uint64_t s_last_oled_frame;
static uint64_t s_last_run_refresh_ms;

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/
static void key_filter_update(APP_KeyFilter_t *key, bool raw_pressed);
static void menu_scan_keys(void);
static void menu_update_state(uint64_t now_ms);
static void menu_enter_select(void);
static void menu_enter_countdown(uint64_t now_ms);
static void menu_enter_running(uint64_t now_ms);
static void menu_mark_full_redraw(void);
static void menu_draw_current_page(uint64_t now_ms);
static void menu_show_line(uint8_t line, const char *text);
static uint8_t menu_get_countdown_number(uint64_t now_ms);
static uint32_t menu_get_run_time_ds(uint64_t now_ms);
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

    key->pressed_event = false; // 每次采样只允许重新产生一次事件

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
        key->pressed_event = (!old_stable && key->stable_state); // 只在松开到按下时触发
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
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: menu_enter_countdown
*   功能说明: 进入3秒倒计时页面
*   形    参: now_ms - 当前系统时间
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_enter_countdown(uint64_t now_ms)
{
    s_ui_state = APP_COMP_UI_COUNTDOWN;
    s_countdown_start_ms = now_ms;
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: menu_enter_running
*   功能说明: 进入运行页面并产生一次启动请求
*   形    参: now_ms - 当前系统时间
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_enter_running(uint64_t now_ms)
{
    s_ui_state = APP_COMP_UI_RUNNING;
    s_running_start_ms = now_ms;
    s_last_run_refresh_ms = now_ms;
    s_start_request = true;
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: menu_mark_full_redraw
*   功能说明: 标记OLED需要整页重画
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
*   形    参: now_ms - 当前系统时间
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_update_state(uint64_t now_ms)
{
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
            if (s_select_key.pressed_event) {
                menu_enter_select();
            } else if (s_confirm_key.pressed_event) {
                s_start_request = false;
                s_stop_request = true;
                menu_enter_select();
            } else if ((now_ms - s_countdown_start_ms) >= APP_MENU_COUNTDOWN_MS) {
                menu_enter_running(now_ms);
            } else {
                s_oled_dirty = true; // 倒计时数字可能跨秒变化，只覆盖固定行
            }
            break;

        case APP_COMP_UI_RUNNING:
            if (s_confirm_key.pressed_event) {
                s_stop_request = true;
                menu_enter_select();
            } else if ((now_ms - s_last_run_refresh_ms) >= APP_MENU_RUN_REFRESH_MS) {
                s_last_run_refresh_ms = now_ms;
                s_oled_dirty = true;
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

    s_select_key.pressed_event = false; // 状态机消费后立即清除事件
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
*   形    参: now_ms - 当前系统时间
*   返 回 值: 1~3
*********************************************************************************************************
*/
static uint8_t menu_get_countdown_number(uint64_t now_ms)
{
    uint64_t elapsed_ms = now_ms - s_countdown_start_ms;
    uint8_t elapsed_s = (uint8_t)(elapsed_ms / 1000U);

    if (elapsed_s >= 3U) {
        return 1U;
    }

    return (uint8_t)(3U - elapsed_s);
}

/*
*********************************************************************************************************
*   函 数 名: menu_get_run_time_ds
*   功能说明: 获取运行时间，单位0.1秒
*   形    参: now_ms - 当前系统时间
*   返 回 值: 运行时间0.1秒计数
*********************************************************************************************************
*/
static uint32_t menu_get_run_time_ds(uint64_t now_ms)
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
*   函 数 名: menu_draw_current_page
*   功能说明: 根据当前状态刷新OLED页面
*   形    参: now_ms - 当前系统时间
*   返 回 值: 无
*********************************************************************************************************
*/
static void menu_draw_current_page(uint64_t now_ms)
{
    char line[APP_MENU_LINE_WIDTH + 1U] = {0};
    uint32_t run_ds;

    if (s_oled_full_redraw) {
        OLED_Clear(); // 只在页面切换时清屏
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

        case APP_COMP_UI_RUNNING:
            run_ds = menu_get_run_time_ds(now_ms);
            menu_show_line(1U, "RUN");
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
            menu_show_line(3U, "STATE: RUN");
            menu_show_line(4U, "OK:STOP");
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
    s_fault_code = 0U;
    s_countdown_start_ms = 0U;
    s_running_start_ms = 0U;
    s_last_key_epoch = 0U;
    s_last_oled_frame = 0U;
    s_last_run_refresh_ms = 0U;
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_Task
*   功能说明: 执行按键扫描、菜单状态机和OLED相位刷新
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_CompetitionMenu_Task(void)
{
    uint64_t now_ms = Get_Tick();
    uint64_t key_epoch = now_ms / APP_MENU_KEY_SCAN_MS;
    uint64_t oled_frame = now_ms / APP_MENU_FRAME_MS;
    uint8_t phase_ms = (uint8_t)(now_ms % APP_MENU_FRAME_MS);

    if (((now_ms % APP_MENU_KEY_SCAN_MS) >= APP_MENU_KEY_PHASE_MS) &&
        (key_epoch != s_last_key_epoch)) {
        s_last_key_epoch = key_epoch;
        menu_scan_keys();
    }

    menu_update_state(now_ms);

    if (s_oled_dirty &&
        (phase_ms >= APP_MENU_OLED_PHASE_MS) &&
        (oled_frame != s_last_oled_frame)) {
        s_last_oled_frame = oled_frame;
        s_oled_dirty = false;
        menu_draw_current_page(now_ms);
    }
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
    menu_mark_full_redraw();
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_GetSelectedMode
*   功能说明: 获取当前选择的小题模式
*   形    参: 无
*   返 回 值: 当前小题模式
*********************************************************************************************************
*/
APP_CompetitionMode_t APP_CompetitionMenu_GetSelectedMode(void)
{
    return s_selected_mode;
}

/*
*********************************************************************************************************
*   函 数 名: APP_CompetitionMenu_GetUiState
*   功能说明: 获取当前菜单UI状态
*   形    参: 无
*   返 回 值: 当前UI状态
*********************************************************************************************************
*/
APP_CompetitionUiState_t APP_CompetitionMenu_GetUiState(void)
{
    return s_ui_state;
}
