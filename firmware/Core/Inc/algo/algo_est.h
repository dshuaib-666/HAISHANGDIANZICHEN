#ifndef __ALGO_EST_H__
#define __ALGO_EST_H__

#include "algo/algo_params.h"
#include <stdint.h>
#include <string.h>

/* ==== 估计器状态机（对拍 atrack/estimate.py + confidence.py）==== */

#define EST_IDLE_HIST_MAX  600
#ifdef ALGO_EMBEDDED
#define EST_WIN_MAX        1536   /* 称量事件窗 153s@10Hz 滚动记忆（满窗
                                     淘汰最旧，与 Python 无界 append 语义
                                     对齐——见 feed_weighing 满窗分支）*/
#else
#define EST_WIN_MAX        2048
#endif
#define EST_READINGS_MAX   16

typedef struct
{
  double t_step;
  double t_lock;
  double mass_g;
  double rel_std;
  double conv_s;
  int    outliers;
  int    n_win;
  int    level;       /* 0 可信 / 1 勉强 / 2 超限 */
} AlgoReading;

typedef struct
{
  /* 状态 */
  int state;                     /* 0=IDLE 1=WEIGHING */
  double baseline;
  double baseline_sigma;
  double idle_t[EST_IDLE_HIST_MAX];
  double idle_v[EST_IDLE_HIST_MAX];
  int idle_n;
  int idle_head;                 /* 环形写入指针 */

  AlgoReading readings[EST_READINGS_MAX];
  int n_readings;

  double step_t;
  int above_n, near_n;
  int down_n;                     /* 连续低于基线样本数（重锚持续性门）*/
  int restep_n;                   /* 盘中增减检测连续计数 */
  double r_fast[10];              /* 最近 10 个 r（≈1s 快均值，检测用）*/
  int    r_fast_n;
  uint8_t quiet;                  /* 静水标志（最近对齐窗带通能量低），
                                     由喂入方按 last_align.bp_rms 设置 */
  int locked;
  double win_t[EST_WIN_MAX];
  double win_r[EST_WIN_MAX];
  uint8_t win_o[EST_WIN_MAX];
  int win_n;

  /* 外部输入（可信度分级） */
  double last_rho;
  int last_calm;
  double gap_frac;
} AlgoEstimator;

void AlgoEstInit(AlgoEstimator *e);
void AlgoEstFeed(AlgoEstimator *e, const AlgoParams *p,
                 double t, double r, uint8_t is_outlier);

/* 截尾均值（实现在 algo_est.c——大工作缓冲静态化，禁止上栈） */
double AlgoTrimmedMean(const double *x, int n, double frac);

#endif
