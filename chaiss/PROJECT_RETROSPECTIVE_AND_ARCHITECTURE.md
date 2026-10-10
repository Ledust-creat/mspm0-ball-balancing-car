# MSPM0 控制项目复盘与通用工程架构

## 1. 复盘结论

这份工程已经从单电机测试逐步发展为“底盘速度环 + 循迹外环 + IMU + 视觉控球 + 舵机 + OLED 菜单 + 多小题状态机”的完整裸机系统。比赛阶段最重要的目标是快速得到可运行结果，因此当前代码中同时存在值得保留的工程骨架和需要在下一项目中提前治理的临时结构。

| 维度 | 可取之处 | 有待改善 | 下一项目做法 |
| --- | --- | --- | --- |
| 分层 | 已形成 BSP / DVE / APP，`main.c` 只初始化和运行调度器 | `APP_BallControl` 直接读取底盘与 IMU，APP 间依赖较多；`DVE` 命名不通用 | 固定 `platform -> bsp -> device -> control -> app` 单向依赖，上层通过输入/输出结构体交换数据 |
| 实时控制 | 底盘速度环由唯一 20 ms 标志驱动，中断只置标志 | 调度器没有执行时间、超周期和丢周期统计；OLED、UART 仍为阻塞发送 | 周期任务带 deadline、预算和 overrun 计数；低优先级 I/O 使用队列、分片或 DMA |
| 中断 | 编码器、Timer、UART ISR 已从业务代码拆开，ISR 基本只采集和投递 | 临界区直接 `disable/enable`，未保存进入前中断状态；串口固定 8 字节组帧抗错较弱 | ISR 只写计数器/环形缓冲/邮箱；临界区保存 PRIMASK；协议层负责超时、重同步、CRC |
| 控制算法 | 速度内环、循迹外环、控球 PID、前馈和限幅已有明确角色 | 参数宏、运行时参数、任务启动赋值曾同时存在，容易互相覆盖；控制周期并非全部固定 | 每个控制器只有一个参数源；显式传入 `dt`；固定顺序为采样、估计、参考规划、控制、限幅、执行 |
| 状态机 | 比赛控制层统一分发五种题目，各题有独立状态 | 状态机直接调用多个模块，资源所有权曾冲突；完成/故障信息最终交给菜单 | 增加资源所有权和模式管理器；状态机只发意图，控制服务独占电机/舵机输出 |
| 通信 | A5/5A 二进制命令用于控制，文本用于低频观察，思路正确 | `app_command.c` 同时负责组帧、路由、调参、波形和文本，文件过大；蓝牙任务目前靠注释关闭 | 拆成 transport、protocol、parameter、telemetry；用构建配置开关功能，不注释调用语句 |
| 安全 | 舵机位置、速度、角度、视觉超时、丢线和故障停车均有限制 | 故障分散在菜单、任务和控制器中；部分底层失败被 `(void)` 忽略 | 统一 Fault Manager，记录来源、时间、锁存策略、恢复条件和最终安全输出 |
| 可维护性 | 公开状态逐步改为结构体快照，避免大量零散 getter | 大文件明显：控球、菜单、IMU、命令均超过约 680 行；配置宏散在公开头文件 | 单模块只保留一个变化原因；配置放 `config/`，算法、协议、显示分别拆分 |
| 工程卫生 | 当前 Keil 工程可以 0 error / 0 warning 构建 | README 仍是 TI GPIO 示例；工程内保留旧 `.o/.d` 和历史模块产物；中文编码容易显示乱码 | 补真实 README、`.gitignore`、UTF-8 `.editorconfig`；CI 做 clean build；不提交 Objects/JLink 日志 |
| 测试 | 实车调试积累了大量有效现象和参数 | 缺少可重复测试记录、主机单元测试和硬件回归清单 | PID、协议、滤波、状态机做 host test；硬件建立冒烟测试和每次改动的回归矩阵 |

### 当前资源快照

以 2026-08-08 最新 Keil 链接结果为准：构建为 `0 error / 0 warning`，map 中 `Total RO = 34432 B`、`Total RW = 2480 B`；链接区为 `128 KiB Flash / 32 KiB SRAM`，静态占用约为 Flash 26.3%、SRAM 7.6%。启动文件分配 `Stack = 1024 B`、`Heap = 0 B`。因此当前不是静态内存不足，而是尚未用水位填充或静态调用图证明 1 KiB 栈足够；大数组、格式化字符串、深调用链和 ISR 嵌套仍应重点检查。Keil 的 SysConfig pre-build 同时输出了 `Couldn't find .metadata/product.json` 和批处理 `SET` 提示，虽未阻止本次编译，但下一项目应修复 SDK/SysConfig 脚本环境，避免实际配置生成失败后仍误用旧文件。

## 2. 当前工程应保留的主干

```text
main
  -> APP_Scheduler_RunOnce
       -> 输入采集：K230 邮箱、IMU 周期采样、按键
       -> 任务管理：CompetitionControl + 各小题状态机
       -> 控制：Line/Ball 外环 + Chassis 速度内环
       -> 执行：电机 PWM、舵机协议
       -> 低优先级：Telemetry、OLED
```

保留原则：`main` 保持短小；SysConfig 只做芯片级初始化；真实 ISR 名称集中在中断适配文件；20 ms 速度环只有一个消费者；每个比赛小题独立状态机；二进制控制帧和文本诊断分开；底层继续无动态内存。

当前调度顺序需要调整为“输入 -> 任务状态机/目标生成 -> 控制器 -> 执行器 -> UI/遥测”。现在 `APP_ChassisControl_Task()` 早于 `APP_CompetitionControl_Task()`，开始、停止和新目标通常会晚一个主循环或控制周期生效；安全停止最好在同一周期先改变目标，再执行控制更新。

## 3. 通用企业级目标结构

“企业级”不等于文件越多越好，而是职责、依赖、时序、资源所有权和验证证据明确。对 MSPM0 这类裸机项目，推荐结构如下：

```text
project/
  app/                 # 产品/比赛任务、模式和状态机，不直接操作寄存器
    mission_manager.c
    mission_track.c
    mission_ball.c
  control/             # 可复用控制器：参考规划、估计、PID、限幅
    chassis_controller.c
    line_controller.c
    ball_controller.c
  service/             # 调度、事件、故障、参数、遥测、协议路由
    scheduler.c
    fault_manager.c
    parameter_service.c
    telemetry_service.c
  device/              # 设备协议、校准、单位和有效性
    encoder.c
    icm20602.c
    hts25l.c
    line_sensor.c
  bsp/                 # GPIO/SPI/UART/Timer/PWM 字节或寄存器级访问
  platform/            # 启动、时基、ISR 分发、SysConfig 生成文件
  ui/                  # OLED 页面和按键事件，不控制执行器
  config/              # 板级、产品级、任务级配置和默认参数
  common/              # 无硬件依赖的 PID、滤波、环形缓冲、数学工具
  tests/               # host 单元测试、协议向量、状态机测试、HIL 脚本
  docs/                # 接口、时序、接线、标定、测试和决策记录
  main.c
```

### 依赖与所有权

| 层 | 可以依赖 | 不允许依赖 | 唯一职责 |
| --- | --- | --- | --- |
| APP/Mission | control、service 的公开接口 | BSP、寄存器、其他任务的私有状态 | 决定当前模式、目标、完成和故障策略 |
| Control | common、输入快照、device 抽象 | OLED、蓝牙、菜单、具体比赛题号 | 按固定周期把目标和反馈变成受限命令 |
| Service | common、必要的 BSP transport | 具体控制算法 | 调度、协议、参数、遥测、事件、故障 |
| Device | BSP、common | APP、任务状态机 | 协议、物理单位、校准、状态有效性 |
| BSP/Platform | TI DriverLib/SysConfig | Device、Control、APP | 引脚、外设、ISR、字节流和时间基准 |

执行器必须有唯一写入者：电机 PWM 只能由底盘执行器提交，舵机只能由钢球/舵机仲裁器提交，OLED 只能由 UI 服务写，UART TX 只能由通信服务排队。校准页、蓝牙测试和比赛任务都只能提交“请求”，不能绕过所有者直接写硬件。

### 控制周期模板

```c
void ControlCycle_Run20ms(uint32_t now_ms)
{
    InputSnapshot_t input = InputService_GetSnapshot(now_ms); // 同一周期使用一致输入
    MissionReference_t ref = MissionManager_Update(&input);   // 先决定目标和模式
    ControlCommand_t cmd = Controller_Update(&ref, &input, 0.020f);
    Safety_ApplyLimits(&cmd, &input);                          // 限幅、超时和故障覆盖
    Actuator_Commit(&cmd);                                     // 每周期只提交一次
}
```

推荐周期预算：1 ms SysTick 只计时；编码器边沿 ISR 只计数；IMU 10 ms；底盘控制 20 ms；视觉按帧事件更新缓存、控制器在固定周期读取快照；按键 10 ms；OLED 100~200 ms 且仅更新变化区域；文本遥测 200~500 ms。每个周期任务记录 `last_start/last_finish/max_exec/overrun_count/missed_count`，先测量再优化。

### 配置、参数和状态

1. 编译期配置只描述硬件能力和产品变体，例如引脚、编码器倍频、启用哪个设备；运行时 PID、速度、前馈和阈值进入参数结构体，不散落在多个头文件。
2. 每个模块采用 `Init(config) -> Start(request) -> Update(input, output) -> Stop() -> GetStatus(snapshot)`，同一参数只有一个权威存储位置。
3. 单位写入名称：`speed_cm_s`、`period_ms`、`angle_deg`；坐标系、正方向、有效范围、更新率和超时写入接口注释。
4. 状态机每个状态明确进入动作、周期动作、退出条件、超时、故障和安全输出；禁止在多个状态隐式争夺同一执行器。
5. 协议帧至少包含帧头、版本/类型、长度、序号、负载、CRC；解析器支持字节丢失后的重新同步，并统计 CRC 错误、超时和覆盖次数。

## 4. 从当前工程迁移的最小路线

| 阶段 | 修改范围 | 完成判据 |
| --- | --- | --- |
| 0. 冻结比赛版 | 提交当前源码、SysConfig、参数、接线和 map | 能 clean build、烧录、运行；有可回退 tag |
| 1. 工程卫生 | 修 README、UTF-8、`.gitignore`，清理旧 Objects；恢复明确的调试构建开关 | 仓库不再混入旧模块产物，蓝牙不是靠注释启停 |
| 2. 调度与所有权 | 调整为输入/任务/控制/输出顺序；增加执行器唯一所有者和 overrun 统计 | 同一周期每个执行器只写一次，停止事件不晚一周期 |
| 3. 参数统一 | 建立 board/product/mission 参数结构和 Parameter Service | 启动任务不再偷偷覆盖串口调好的参数 |
| 4. 拆大模块 | 先拆 command、ball、menu、ICM；算法和格式化从状态机移出 | 单文件通常小于约 400 行，每个文件只有一个变化原因 |
| 5. 自动验证 | PID/滤波/协议/状态机 host test + 板级 smoke test | 每次提交自动 clean build，关键状态转移和协议向量可重复 |
| 6. 运行证据 | 栈水位、周期最大耗时、UART 覆盖率、传感器丢帧、故障日志 | 所有实时和内存结论都有测量值，不靠感觉判断 |

下一项目不要直接大重写。先冻结可跑版本，然后优先完成“唯一参数源、唯一执行器所有者、固定控制周期、可测调度预算”四项；它们能消除这次开发中绝大多数反复回退和模块互相影响的问题。
