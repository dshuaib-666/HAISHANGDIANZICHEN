#ifndef __APP_PREP_H
#define __APP_PREP_H

/* ==== 预备模式（v1.1）：自检+标定合并为一次 40s 预备 ====
   动机（2026-10-11 设计讨论）：原自检 3-4 分钟（通信 60s + 空盘 15s +
   砝码 ≤120s + 运动 ≤90s + 显示/结果 12s），与两点标定物理步骤完全
   同构（通信→空盘→砝码），操作者要各走一遍。合并后单键单流程：

   流程（单键 KEY0 长按进入/中止，全程零中间按键；先热机，秒盘空置）：
     1 通信门 5s    双通道新鲜（<500ms）+ 错帧<0.5% + 环零丢弃，
                   之后转后台监控直至结果；入口空盘粗查：B 码超
                   APP_PREP_EMPTY_MAX_B → 显示 "9000" 保持
     2 空盘 10s     静态检查（|实时质量**中值** ≤±0.5g——中值判据运动
                   免疫，m7 海上实测微晃下瞬时振荡 ±0.2~0.9g 曾令旧
                   瞬时判据误杀）+ 标定 r0 采集
                   （原始码对比值；到点等样本凑齐，+12s 硬保险）→ 2888。
                   k=0（新机/Flash 回退）时静态门不可算 → 跳过，
                   prep 可自举出第一个 k（空盘由流程卡保证）
     3 砝码段 3xxx  放置/移除检测在**比值域**（尾窗 2s 均值 vs r0，
                   门限 0.03≈44g）：涌荡/晃动下 B 原始码摆动 ±数千码，
                   只有比值是共模相消的运动免疫判据（残余 ~±0.002）。
                   自动检测放置 → 稳定 2s → 采集 10s（移除自动回等待，
                   参考基准固定为 r0，无重捕污染）→ 旧 k 回差 drift_g →
                   算 k → 门限 → 生效 → Flash → 8888/6666/7777/9002
     4 结果 5s      "d1d2d3"：d1 通信 / d2 静态 / d3 标定。
   **无运动补偿检查位（2026-10-16 移除 d4）**：海上 prep 必然挑风平
   浪静时刻（否则 d2 空盘静态检查过不了），此刻按定义无运动证据；
   静水替代证据又无信息量——结构上不可得。运动补偿质量由称量路径
   持续自评（每次锁存按 ρ/rel_std 打可信度等级，数码管常亮/慢闪/
   快闪），失效在读数上当场可见，属算法质量职责而非预备自检职责。
   深检运动性能走录制分析（tau_track.py）。
   砝码质量与 APP_CALIB_MASS_G 同源（单一配置点，改需重编译）。
   与独立标定模式（KEY2）互斥；旧自检模块保留在代码树，入口由本模式
   取代（深检用调试器+录制）。 */

#include "app/app_calib.h"
#include <stdint.h>

#define APP_PREP_COMMS_S       5.0     /* 通信门时长（后台监控贯穿全程）*/
#define APP_PREP_EMPTY_S       10.0    /* 空盘静态检查 + r0 采集 */
#define APP_PREP_EMPTY_SAFETY_S 12.0   /* 样本积累异常的硬超时（10Hz×10s
                                          恰为 MIN_N=100，相位对齐差 1 拍
                                          → 99 样本：到点后等凑齐再走）*/
#define APP_PREP_PROMPT_S      60.0    /* 等待放砝码超时 */
#define APP_PREP_COLLECT_S     10.0    /* 砝码在位采集（MIN_N=100 样本门）*/
#define APP_PREP_COLLECT_SAFETY_S 14.0 /* 样本积累异常的硬超时 */
#define APP_PREP_RESULT_S      5.0     /* 结果码停留 */
#define APP_PREP_EMPTY_DEV_G   0.5     /* 空盘静态判据（|实时质量中值|：
                                          海上实测空盘瞬时比值振荡
                                          ±0.2~0.9g（m7 数据），旧"瞬时
                                          最大偏差"判据微晃即误杀
                                          （结果 100）；中值消交流振荡，
                                          检测的本来是"盘上有物"的直流）*/
#define APP_PREP_EMPTY_LIVE_N  128     /* 空盘实时质量样本环（12.8s 覆盖
                                          10s 段 + 保险期）*/
#define APP_PREP_EMPTY_MAX_B   70000   /* 入口空盘粗查：m10-1 实测新 6kg B
                                          模块空盘 53817（MAD 1.0，2026-10-31）
                                          + 跨会话漂移与遗留物余量；忘盘
                                          砝码/瓶子必拦，100g 级残留靠 d2
                                          静态门兜底。模块再清零或改容量
                                          参数后此值须复核（裸码绝对门，
                                          旧值 43000 系台架时代值，从未对
                                          过真机空盘）。
                                          2026-11-14 现场：本机空盘 B 内码
                                          60000 出头 > 旧值 57000，prep 恒判
                                          "未清盘"（9000）无法标定 → 抬到
                                          70000。新值仍低于 空盘+500g 的约
                                          84000，忘盘砝码照拦；100g 级残留
                                          由 d2 静态门兜底，不靠此门 */
#define APP_PREP_R_PLACE       0.03    /* 放置/移除门限（比值域）≈44g：
                                          运动残余 ±0.002 的 15×，最小
                                          砝码 100g(Δr≈0.068) 的 44% */
#define APP_PREP_MON_N         20      /* 放置/移除监视尾窗样本数（2s）*/
#define APP_PREP_SETTLE_S      2.0     /* 检测到放置后丢弃的稳定段 */

typedef enum
{
  PP_IDLE = 0,
  PP_COMMS,        /* 1 通信门（含入口空盘粗查）*/
  PP_EMPTY,        /* 2 空盘静态 + r0 */
  PP_PROMPT,       /* 3 等待放砝码（比值域监视）*/
  PP_SETTLE,       /* 放置后稳定段（不采集）*/
  PP_COLLECT,      /* 砝码在位采集 */
  PP_CALC,         /* 瞬时：算 k → 门限 → 生效 → Flash */
  PP_RESULT        /* 结果码 */
} AppPrepState;

typedef struct
{
  AppPrepState state;
  double t_state;              /* 进入当前状态时刻（HAL tick 秒）*/
  double t_linger;             /* 段末码停留起点（0=不停留）*/
  int    linger_code;

  /* 通信（计数器增量式，基线在进入后首个 Notify 锁定）*/
  uint32_t c_ok0, c_err0, c_drop0;
  uint8_t  comms_pending_base;
  uint8_t  comms_viol;
  uint8_t  comms_ok;

  /* 空盘 */
  double   empty_maxdev;              /* 诊断：段内瞬时|质量|最大值 */
  double   empty_live[APP_PREP_EMPTY_LIVE_N];  /* 实时质量样本（中值判定）*/
  int      empty_live_n;
  uint8_t  empty_ready_seen;
  uint8_t  empty_ok;
  uint8_t  pan_loaded;          /* 通信段入口粗查：未清盘（显示 9000）*/

  /* 标定（放置/移除检测基准固定为 r0，无重捕）*/
  double   r0, r1;
  double   a_mean0;
  double   drift_g;            /* 重标前旧 k 回差（诊断输出）*/
  uint8_t  cal_ok;
  uint8_t  flash_ok;

  uint8_t  result[3];          /* 通信/静态/标定 */
} AppPrep;

/* 通知接口（main.c 喂，与驱动解耦）：
   Comms 每轮；Live 每轮（空盘静态检查用）*/
void AppPrepNotifyComms(AppPrep *pp, uint8_t both_fresh,
                        uint32_t n_ok, uint32_t n_err, uint32_t n_drop);
void AppPrepNotifyLive(AppPrep *pp, int ready, double mass_g);

/* 状态机驱动。ev_key0_long：IDLE=进入；运行中=中止。
   返回 1 = 刚完成一次标定提交（main 发事件行）。*/
int AppPrepPoll(AppPrep *pp, AppScale *app, double t_now, int ev_key0_long);

/* 显示：≥0=显示码；-1=非预备态。-2 未用（无闪烁半拍语义）。*/
int AppPrepDisplay(AppPrep *pp, double t_now);

#endif
