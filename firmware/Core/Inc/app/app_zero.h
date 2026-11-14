#ifndef __APP_ZERO_H
#define __APP_ZERO_H

/* ==== 清零模式（KEY1/PB14 长按，2026-10-16）：双上下文 ====
   单键按"盘上状态"自动分流，构成分解式两点标定：

   【空盘 · 零点重锚】估计器待机（空盘）时长按：
     10s/100 样本中值 → est.baseline 重锚（idle 历史同步重锚，不写
     Flash，k 不动）。动因：清零时姿态≠称量时姿态的差分倾斜
     （0.353 g/°，直流）是 0.3~1g 级误差主因，滤波滤不掉，唯一对策
     在与称量相同的姿态下重取参考。同时覆盖长期零漂。

   【带标准砝码 · 跨度校正】估计器锁定称量中、读数在
     APP_ZERO_SPAN_MASS_G ±5% 窗口内时长按：
     10s/100 样本中值 r₁ → k = M₀/(r₁ − baseline) → AppCalibCommit
     门限校验 → 内存生效 + Flash 持久（m5 跨会话 k 漂 0.05~0.2%的
     现场修正通道）。这是 prep 两点标定的第二点分解——操作者可随时
     在与使用一致的姿态下做，顺序规则：先零点后跨度（脏基线会把
     零点误差乘进 k）。注意：空盘清零在数学上永远不可能修 k
     （单点单参数，斜率不可辨识）——带已知砝码才可。

   双门拒绝优于过滤（滤波救不了帧失配，环境太吵唯一诚实的动作
   是拒绝）；两上下文都尽量水平/正浮姿态做（载荷依赖的差分倾斜
   会进任何带载标定——台架微抬 prep 实测 0.25% k 误差）。

   错误码（2.5s）：9100 色散（太晃）/ 9200 跨度窗不符（非标准砝码、
   采集中被取走、或 Δr 异常）/ 9300 样本不足或入口条件不满足 /
   7777 k 超范围（砝码与配置不符，语义同 prep）。
   采集中数码管闪烁 "---"：正在取参考/算跨度，勿放物勿取物。
   与 prep/录制互斥由 main.c 分发保证。 */

#include "app/app_scale.h"
#include <stdint.h>

#define APP_ZERO_N_MIN      100    /* 10s@10Hz */
#define APP_ZERO_SAFETY_S   12.0   /* 样本积累硬超时 */
#define APP_ZERO_SIGMA_G    1.0    /* 色散门（σ_mad 等效克，两上下文同）*/
#define APP_ZERO_OFFSET_G   5.0    /* 零点上下文：与旧基线偏移门（克）*/
#define APP_ZERO_SPAN_MASS_G 500.0 /* 跨度标准砝码（与演示砝码同源）*/
#define APP_ZERO_SPAN_FRAC  0.05   /* 跨度识别窗 ±5% */
#define APP_ZERO_RESULT_S   2.5    /* 错误码停留 */
#define APP_ZERO_REARM_N    30     /* 清零重锚后填充的 idle 样本数——须等于
                                      feed_idle/AppScaleLiveMass 的 ready 门限
                                      （idle_n<30 → ready=0 → 实时显示闪
                                      "---"）；填满 30 使 ready 立即恢复，
                                      避免"清零完成后闪烁多拖 ~3s" */

typedef enum
{
  ZO_IDLE = 0,
  ZO_COLLECT,      /* 零点采集中（显示闪烁 "---"）*/
  ZO_SPAN,         /* 跨度采集中（显示闪烁 "---"）*/
  ZO_RESULT        /* 错误码停留（成功无此态）*/
} AppZeroState;

typedef struct
{
  AppZeroState state;
  double t_state;
  double t_linger;
  int    linger_code;             /* 9100/9200/9300/7777，0=无 */

  /* 诊断（# ZERO 调试行）：无论成败都记录判定量 */
  double sigma_g;                 /* 样本 σ_mad（克等效）*/
  double offset_g;                /* 零点:新中值−旧基线；跨度:隐含质量−M₀ */
  double k_old;                   /* 跨度：校正前 k */
  int    n_used;
} AppZero;

/* 状态机驱动。ev_key1_long：IDLE=进入（按上下文自动分流，条件不
   满足立即拒绝）；运行中=中止。
   返回：0=无完成；1=刚完成零点重锚；2=刚完成跨度校正（main 发事件行）。*/
int AppZeroPoll(AppZero *z, AppScale *app, double t_now, int ev_key1_long);

/* 显示：-1=非清零态；-3=采集中（main 闪烁 "---"）；≥0=错误码。*/
int AppZeroDisplay(AppZero *z, double t_now);

#endif
