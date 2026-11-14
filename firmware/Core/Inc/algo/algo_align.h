#ifndef __ALGO_ALIGN_H__
#define __ALGO_ALIGN_H__

#include "algo/algo_dsp.h"
#include "algo/algo_stream.h"
#include <stdint.h>

/* ==== 自对齐慢环（对拍 atrack/align.py）==== */

#define ALIGN_GRID_HZ     20.0
#define ALIGN_WIN_S       45.0
#define ALIGN_HEAD_DROP_S 8.0
#define ALIGN_UPDATE_S    2.5
#define BP_LO_HZ          0.05
#define BP_HI_HZ          1.0
#define PHASE_LO_HZ       0.06
#define PHASE_HI_HZ       0.60
#define FULL_SEARCH_S     0.100
#define RHO_PASS          0.998
#define CALM_EPS_RMS      0.004
#define MIN_COVER_S       30.0
#define MIN_OVERLAP_FRAC  0.80
#define STEP_GUARD_G      200.0
#define REF_SPAN_CODES    47616.0
/* 网格最大点数：对齐窗(45s−头丢弃8s)×20Hz=740 + 裕量 */
#ifdef ALGO_EMBEDDED
#define ALIGN_GRID_MAX    768
#else
#define ALIGN_GRID_MAX    1024
#endif

typedef struct
{
  double t;                 /* 窗结束时刻 */
  uint8_t ok;
  uint8_t calm;
  double tau;               /* 估计时延（秒） */
  double rho;               /* 相关系数 */
  double bp_rms;            /* A 通道带通 RMS / ref_span */
  double rms_b;             /* B 通道带通 RMS / ref_span */
  int n_pts;                /* 有效格点数 */
} AlgoAlignResult;

/* 变化沿相位圆均值差 → 粗时延（模 1/fs） */
double AlgoCoarseEdgeDelay(const double *ca, int na, const double *cb, int nb,
                           double sensor_fs);

/* 对齐窗计算（对拍 Python align_window）*/
void AlgoAlignWindow(const AlgoStream *eff_a, const AlgoStream *eff_b,
                     double t_end, AlgoAlignResult *res);

#endif
