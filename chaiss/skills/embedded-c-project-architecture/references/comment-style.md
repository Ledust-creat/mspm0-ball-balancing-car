# Embedded C Engineering Comment Style

## 1. File Structure

Use concise section blocks only when the section exists:

```c
/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*   类型定义
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*   全局变量
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*   函数定义
*********************************************************************************************************
*/
```

Do not add empty sections. Prefer `static` module state; label it “模块变量” if “全局变量” could imply public linkage.

## 2. Function Header

Use this format for public functions and non-obvious private functions:

```c
/*
*********************************************************************************************************
*   函 数 名: APP_ChassisControl_Update
*   功能说明: 根据任务目标和反馈更新一次底盘控制输出
*   形    参: input  - 同一控制周期的输入快照
*             output - 输出命令，由调用者提供存储空间
*             dt_s   - 本次控制周期，单位s
*   返 回 值: true表示输出有效，false表示输入失效并已生成安全输出
*********************************************************************************************************
*/
```

State units, direction, ownership or failure behavior when they are not obvious from the type. Do not write “设置变量”“调用函数” as the functional description.

Small obvious private helpers such as a three-line clamp do not need a large header unless the repository requires one consistently.

## 3. Macros, Types, And Variables

Use short end-of-line comments for meaning and units:

```c
#define CHASSIS_CONTROL_PERIOD_MS       (20U)    /* 底盘闭环周期，单位ms */
#define MOTOR_SPEED_LIMIT_CM_S          (40.0f)  /* 单轮目标速度绝对值上限 */

typedef struct
{
    float target_speed_cm_s;                     /* 目标轮速，前进为正 */
    float measured_speed_cm_s;                   /* 编码器反馈轮速，前进为正 */
    uint32_t sample_tick_ms;                      /* 反馈采样完成时间 */
    bool valid;                                  /* true表示反馈未超时且范围有效 */
} WheelControlInput_t;
```

Avoid comments that only repeat the identifier. Explain physical meaning, range, sign, lifetime, concurrency, or calibration source.

## 4. 函数内部的条件与注释

先整理逻辑，再补注释。把不同原因的拒绝条件分开；相同目的且能一眼读完的条件可以横向写。超过正常阅读宽度时换行，按含义对齐。注释用 `//` 写在非直观判断之前或相应语句旁，说明为什么拒绝、为什么转移，以及失败时的行为；不用给每个比较运算都写一句。

例如设置电机速度 PI：先看控制是否处于可配置状态，再看参数范围，最后初始化。保留现有条件及返回语义时可整理为：

```c
/*
*********************************************************************************************************
*   函 数 名: MotorControl_SetSpeedPid
*   功能说明: 仅在速度模式停机状态下配置速度 PI
*   形    参: kp - 比例系数；ki_per_s - 每秒积分系数
*   返 回 值: true配置成功；false表示状态、参数或初始化不满足要求
*********************************************************************************************************
*/
bool MotorControl_SetSpeedPid(float kp, float ki_per_s)
{
    // 限幅未就绪或控制器正在运行时，不允许修改速度环参数。
    if (!MOTOR_SPEED_LIMITS_READY ||
        s_control_mode != MOTOR_CONTROL_MODE_SPEED ||
        s_current_control_state != MOTOR_CONTROL_STATE_DISABLED) {
        return false;
    }

    // 同时拒绝负值、超限和NaN，避免把无效增益写入控制器。
    if (!(kp >= 0.0f && kp <= MOTOR_SPEED_KP_MAX) ||
        !(ki_per_s >= 0.0f && ki_per_s <= MOTOR_SPEED_KI_PER_S_MAX)) {
        return false;
    }

    s_speed_pid_ready = FocPid_Init(&s_speed_pi, kp, ki_per_s, 0.0f,
                                    MOTOR_SPEED_LOOP_PERIOD_S,
                                    -MOTOR_CURRENT_LIMIT_A, MOTOR_CURRENT_LIMIT_A);
    return s_speed_pid_ready;
}
```

此例是排版和注释示范，实际修改时沿用工程的括号、缩进及错误处理风格；不要为了套模板改变短路顺序、状态条件或 NaN 处理。单个短守卫条件可以保持一行，不需要单独成段。

其他非直观逻辑也在邻近位置说明原因：

```c
error_cm = target_cm - measured_cm; // 正误差要求小球向舵机侧移动

if ((uint32_t)(now_ms - last_frame_ms) >= VISION_TIMEOUT_MS) {
    command->speed_cm_s = 0.0f;      // 陈旧视觉数据不得继续驱动执行器
    command->valid = false;
}

integral += ki * error * dt_s;
integral = limit_float(integral, -max_iout, max_iout); // 先限制积分项，避免饱和后长时间恢复
```

不要描述显而易见的语法：

```c
count++;                 // Bad: 计数加一
speed = 0.0f;            // Bad: 将速度设置为0
```

## 5. ISR And State-Machine Comments

ISR comments must identify source, edge/condition, shared data, and deferred work. State transitions must identify the reason and next state's responsibility.

```c
if ((status & ENCODER_A_PIN) != 0U) {
    encoder_count_x2();  // ISR只累计有符号边沿，速度换算留给20ms任务
    DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_A_PIN);
}

if (position_reached && speed_stable) {
    enter_state(BALL_STATE_HOLD, now_ms); // 连续稳定确认后再交给位置保持PID
}
```

## 6. Comment Maintenance Rules

- Update comments in the same patch as behavior, units, signs, timing or state transitions.
- Delete stale comments and commented-out historical code; history belongs in Git.
- Keep formulas close to implementation and state assumptions, but avoid essay-length derivations in `.c`; place long design rationale in `docs/`.
- Use one language consistently within a module. Store source as UTF-8 and verify the editor/toolchain does not convert it to mojibake.
- Treat a misleading comment as a defect: fix it before adding more explanation around it.

## 7. 交付前逐函数验收

对本次新增或修改的每个函数，从头到尾读一次，并检查：

1. 公开函数或重要私有函数的用途、参数单位和失败返回是否能从名称、签名及函数头注释看懂；是否与实现一致。
2. 是否有跨行密集条件、混合状态与参数校验、魔法常量或非直观短路顺序；能否用有意义的分组和适度横向排版让判断顺序清楚。
3. 状态机跳转、早退原因、坐标/单位换算、重试和安全边界是否有就近的简洁 `//` 注释；简单赋值和显然的限幅不必凑注释数。
4. 宏、结构体字段和共享状态是否标明关键含义、单位、方向或并发约束；旧注释是否随行为一起更新。
5. 读者是否能不追踪多个无意义小函数就理解核心路径；若拆函数，拆出的职责和命名是否确实帮助复查。

验收针对改动范围，不要求顺手给全项目补注释；代码仍难读时继续整理，不能用大段注释掩盖混乱的控制流。
