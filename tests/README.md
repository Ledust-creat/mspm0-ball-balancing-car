# 主机回归测试

`test_task_switch_gains.c` 链接真实控球、静态任务、移动任务及 PID 源码，以桩函数替代硬件接口，不调用真实电机或舵机。

覆盖：默认增益、静态任务临时关闭补偿、关闭期间修改增益、退出静态任务、三个移动模式恢复补偿、实际补偿角计算，以及用户主动将Kv/Ka设为0后仍保持0。

在仓库根目录，安装 GCC 后可用 PowerShell 运行：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -I chaiss -I chaiss/app -I chaiss/DVE tests/test_task_switch_gains.c chaiss/app/app_ball_control.c chaiss/app/app_task_static_ball.c chaiss/app/app_task_mobile_ball.c chaiss/PID.c -o "$env:TEMP/task_switch_gains_test.exe"
if ($LASTEXITCODE -eq 0) { & "$env:TEMP/task_switch_gains_test.exe" }
```

通过时打印 `PASS`，断言失败时进程以非0状态退出。可执行文件写入临时目录，不加入源码仓库。测试文件不用加入 Keil 工程；实车仍需按“移动任务→静态任务→移动任务”顺序验证控球效果。
