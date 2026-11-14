#ifndef __APP_SCALE_H__
#define __APP_SCALE_H__

/* ==== 称量应用引擎（M-C3 上板集成）====
   增量式驱动算法层：轮询样本 → 滑窗变化沿流（源头 Hampel k=3 延迟 3 点）
   → 慢环对齐（45s 窗/2.5s 更新）→ 快环配对（τ̂ 轨迹 + A 插值 + 比值流
   Hampel k=5 延迟 5 点）→ 估计器状态机 → 读数事件。

   与 Python 批处理（atrack/pipeline.py）的等价性：
   - 延迟定点的源头/比值 Hampel ≡ 批量 hampel（边缘左钳位一致；会话末
     3/5 点除外）；AlgoTauEval 按 t 二分 ≡ keys≤t 语义；
   - 已知发散（记录于 C移植计划 §M-C3）：比值流 MAD 地板用最近 256 点
     差分滑窗估计（Python 为全流），可信度分级在回臂时就地打分
     （Python 在锁存时刻事后打）——均不影响质量读数。
   单线程非重入：主循环独占调用。 */

#include "algo/algo_stream.h"
#include "algo/algo_align.h"
#include "algo/algo_pair.h"
#include "algo/algo_est.h"
#include "algo/algo_params.h"
#include <stdint.h>

/* 原始轮询 FIFO 深度：源头 Hampel k=3 需 i+3 可见 */
#define APP_RAW_FIFO_N     8
#define APP_PEND_N         8
/* 比值流 MAD 地板滑窗（最近 N 对差分中值），每 RATIO_FLOOR_UPD 对重估 */
#define APP_RR_HIST_N      256
#define APP_RR_FLOOR_UPD   64

typedef enum
{
  APP_EV_NONE = 0,
  APP_EV_ALIGN,          /* 一个对齐窗完成（res 见 ev.align）*/
  APP_EV_READING,        /* 新读数锁存（est.readings 末条 + level）*/
  APP_EV_REFINE,         /* 锁存读数精化刷新（同上）*/
  APP_EV_REARM,          /* 移除回臂：读数定稿（显示冻结抄录）*/
  APP_EV_RESTEP          /* 盘中增减定稿：显示直接切实时值收敛（不冻结）*/
} AppEvType;

typedef struct
{
  AppEvType type;
  double t;
  AlgoAlignResult align;   /* APP_EV_ALIGN */
  double mass_g;           /* 其余事件 */
  double rel_std;
  int level;               /* 0 可信 / 1 勉强 / 2 超限 */
} AppEvent;

typedef struct
{
  /* 参数快照（标定后填充）*/
  AlgoParams p;

  /* —— 每通道：原始 FIFO + 延迟 3 点的源头 Hampel → 清洗流（滑窗）—— */
  struct
  {
    double t[APP_RAW_FIFO_N];
    double v[APP_RAW_FIFO_N];
    int n;                 /* FIFO 内样本数（≤8）*/
    int32_t last_code;     /* 上一轮询内码（变化沿检测）*/
    uint8_t has_last;
  } raw[2];
  AlgoStream clean[2];     /* A=0 B=1 清洗后的变化沿流 */

  /* —— 慢环 —— */
  AlgoTauHist hist;
  double t_lo;             /* 首个清洗样本时刻（对齐调度锚点）*/
  double next_align;
  uint8_t have_anchor;
  AlgoAlignResult last_align;
  uint8_t have_pass_align; /* _rho_at 的 calm 语义：出现过 ρ≥门限窗 */
  double last_pass_rho;

  /* —— 待配对队列（B 清洗样本，等 A 流末端越过 tb+0.15s）—— */
  double pend_t[8];
  double pend_v[8];
  int pend_n;

  /* —— 快环：比值流延迟 5 点 Hampel + 滑动 MAD 地板 —— */
  double rr[12];                  /* 延迟窗口（k=5 需 6 点在视线内）*/
  double pair_t[12];              /* 对应的 B 清洗样本时刻 */
  int rr_n;
  double rr_hist[APP_RR_HIST_N];  /* 原始比值历史（地板估计用）*/
  int rr_hist_n;
  double rr_floor;
  int n_pair_b;           /* 配对尝试数（gap_frac 分母）*/
  int n_interp_fail;

  double last_r_clean;         /* 最近定点清洗比值（实时质量显示用）*/

  /* —— 标定原始采样（app_calib 用）：直取轮询码对，不经变化沿流 ——
     静态台架上内码长时间不变 → 变化沿稀疏 → 配对流饿死（实测 110s
     采不到 100 对）；标定只需码对比值均值，10Hz 原始采样 + 截尾均值。*/
  uint8_t calib_collect;              /* 1=采集（标定步骤中置位）*/
  double  calib_r[APP_RR_HIST_N];     /* 原始比值（满 256 滚动）*/
  int     calib_r_n;
  double  calib_a_sum;                /* 同步 A 码累计（在位中心用）*/
  int     calib_a_n;
  int32_t calib_a_last;               /* 最近 A 轮询码（B 采样时配对用）*/
  double  calib_a_t;
  int32_t calib_b_last;               /* 最近 B 轮询码（标定放置检测用）*/
  double last_pair_t;                 /* 最近一次配对样本入队时刻 */
  uint8_t raw_fallback;               /* 静态回退：配对饿死 >3s 时原始码对
                                         直接走比值/估计器流水线（静态无
                                         α 泄漏，原始码对即正确测量；
                                         运动恢复→配对出样本→自动切回）*/

  /* —— 估计器 —— */
  AlgoEstimator est;

  /* —— 事件环 —— */
  AppEvent ev[8];
  int ev_head, ev_tail;
} AppScale;

void AppScaleInit(AppScale *app, const AlgoParams *p);

/* 喂一条轮询样本（t 秒；ch 0=A 参考通道 1=B 测量通道；code 内码）。
   主循环每收到一条样本调用一次（10Hz 双通道交错）。 */
void AppScaleFeed(AppScale *app, double t, int ch, int32_t code);

/* 周期调用（主循环每轮，t_now 秒）：调度到期的对齐慢环。 */
void AppScaleProcess(AppScale *app, double t_now);

/* 取事件（无则 APP_EV_NONE）。主循环消费用于显示/调试输出。 */
AppEvent AppScalePopEvent(AppScale *app);

/* —— 标定采样（M-C3 app_calib 用）—— */
/* 比值流最近 ≤256 对的 10% 截尾均值（k 两点标定用；样本 <30 返回 0）*/
double AppScaleRatioTrim(AppScale *app);
/* —— 标定原始采样接口（app_calib 用）—— */
void   AppScaleCalibReset(AppScale *app);        /* 清空采集（保留开关位）*/
double AppScaleCalibMeanR(AppScale *app);        /* 原始比值 10% 截尾均值 */
double AppScaleCalibTailMeanR(AppScale *app, int k); /* 最近 k 个比值均值（prep 放置监视）*/
int    AppScaleCalibN(AppScale *app);            /* 已采比值数 */
int32_t AppScaleCalibB(AppScale *app);           /* 最近 B 原始码 */
double AppScaleCalibMeanA(AppScale *app);        /* 同步 A 码均值（在位中心）*/

/* 实时质量估计 k×(最新清洗比值 − 会话基线)：数码管实时显示用。
   基线未建立（开机前 ~3s / 移除后重建中）返回 0 且 *ready=0。*/
double AppScaleLiveMass(AppScale *app, int *ready);

#endif
