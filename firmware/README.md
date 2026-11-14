# 固件 v2（dual_scale_structure/firmware）

交接版 v1 的重写（v1 原件在 `Items/9月25交接/1/1`，未改动）+ 930 交接的数码管支持（`Items/930交接`）。
对应《算法设计-A轨》。Keil 工程：`MDK-ARM/dual_scale.uvprojx`（已编译通过，0 错误 0 警告；
hex 在 `MDK-ARM/dual_scale/dual_scale.hex`）。

## 模块

| 文件 | 职责 |
|---|---|
| `dwt_time.c/h` | DWT 循环计数器 → 64 位 µs 公共时间轴（两通道共模） |
| `qsh_frame.c/h` | 单路 RS485 帧接收（IDLE+DMA），帧完成即盖 µs 时间戳 |
| `qsh_drv.c/h` | 双通道 QSH-H 驱动：**读内码 0x17**、上电配置状态机、样本环形缓冲、TIM2 10Hz 并行轮询 |
| `dbg_uart.c/h` | USART1 **PA9** TX@115200（自持寄存器初始化，不经 CubeMX），CSV/事件/心跳输出 |
| `digit_tube.c/h` | **UART4（PA0/PA1）9600** 数码管（930 交接，`$001,<字段>#` 协议，非阻塞+心跳重发） |
| `keys.c/h` | 三按键消抖+短/长按：PD12=MARK_B、PD10=MARK_A、PD8=预留（校准入口） |

通道映射：**USART2 = B 称量（称盘）**、**USART3 = A 参考（砝码槽）**；CSV sensor 字段 0=A、1=B。
数码管显示：上电 666 → **B 通道（串口2）内码 ÷ 10**（数码管字段上限 -9999.9，内码满量程 ~30000
放不下，÷10 后 ≤3000；精确值看 `# CODE` 调试行）；B 通道断流 >3s 黑屏。
调试串口每秒输出 `# CODE A=<参考内码>(<数据年龄>ms) B=<称量内码>(<数据年龄>ms)`——双通道内码
直读（`---` 表示该通道尚无数据）。

## 与 v1 的行为差异

1. 轮询命令 `01 03 00 17 00 02`（内码），非 40001 净重值——绕过模块侧零点管理；
2. 上电自动向两模块写 40005=1（开机不置零）、40006=0（滤波弱）、40007=0（蠕动关），回显确认，每条重试 3 次；
3. 不再发送模块清零指令（按键改为事件打点，校准语义归 host 算法层）；
4. 每帧带 µs 时间戳，CSV 输出到调试串口：`t_us,sensor,code,seq,dup`；
5. 1s 心跳：`# STAT A ok=.. crc=.. fe=.. tx=.. ack=.. fail=.. drop=.. | B ... ready=..`；
6. **无 stdio**（采纳 930 交接的实测经验：ARM 标准库 printf 家族在小栈上跑飞）——
   dbg_uart 自实现 %s/%u/%d/%ld/%lu 迷你格式化；启动栈 1KB→4KB。

## RAM 录制（零接线取数，推荐 M1 使用）

板上无任何面向 PC 的串口引出线时，用已有的 SWD 调试器取数：

1. 烧录后在 µVision 连接调试（无需断点，程序自由运行）；
2. Watch 窗口添加 `g_recArm`，**双击值改为 1** → 录制开始（自动清空缓冲）；
3. 做台架动作；`g_recFull` 变 **1**（约 3.4 分钟双通道数据写满 4096 条）后自动冻结；
4. µVision 命令窗口（View → Command Window）执行：
   `SAVE "C:/data/m1.bin" s_ring, s_ring+0x18000`
5. 转 CSV：`python algo/tools/qsh_bin2csv.py C:/data/m1.bin -o C:/data/m1.csv`
6. 再次录制：把 `g_recArm` 改 0 再改 1（沿触发重新清空开始）。

注：≥3.4 分钟的场景（如 5min 双空载）分两段录（冻住后先导出再重录）；
按键事件（MARK/ZERO）只走串口输出，RAM 录制不含事件行——关键动作请在段边界做。

## 数据模式（无 PA9 引出线时的取数通道）

板上只有数码管接口（UART4 PA0/PA1）对外引出——**长按 PD8 切换数据模式**：
UART4 @9600 输出完整 CSV（数据行 + EVENT + 每秒 # CODE），数码管静默。
USB-TTL（3.3V）接数码管接口：**板子 PA0(TX) → 转接板 RXD，共地**，即可在串口工具录制；
再长按一次回到显示模式。忙时丢行计数在 `# CODE ... drop=N`（PA9 口）与 `DataUart_Dropped()` 可查。
算法管线 `load_csv` 已对非数据行容错（数码管帧混入会被跳过）。

## CubeMX 再生成须知

- 所有自定义代码都在 `USER CODE BEGIN/END` 区或独立模块文件内，`1.ioc` 未改动，可安全再生成；
- **例外**：`dbg_uart`（USART1/PA9）完全自持初始化、不在 .ioc 中——再生成不受影响，但也
  不会出现在 CubeMX 里，属预期。

## 编译

```bash
"C:\software\Keil_v5\UV4\UV4.exe" -b MDK-ARM\dual_scale.uvprojx -j0 -o build_out.log
```
