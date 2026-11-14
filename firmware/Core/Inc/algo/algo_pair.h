#ifndef __ALGO_PAIR_H__
#define __ALGO_PAIR_H__

#include "algo/algo_stream.h"
#include "algo/algo_params.h"
#include <stdint.h>

/* ==== 快环比值配对（对拍 atrack/ratio.py）==== */

#define PAIR_MAX        8192
#define TAU_HIST_MAX    64

typedef struct
{
  double t;
  double r;
  double tau;
} AlgoPair;

typedef struct
{
  int n;
  AlgoPair p[PAIR_MAX];
  int n_interp_fail;
  int n_presence_fail;
  int n_b;
} AlgoPairResult;

/* 对齐锚点历史（时间+τ̂+质量权重）*/
typedef struct
{
  double t[TAU_HIST_MAX];
  double tau[TAU_HIST_MAX];
  double w[TAU_HIST_MAX];
  int n;
} AlgoTauHist;

void AlgoTauHistInit(AlgoTauHist *h);
void AlgoTauHistPush(AlgoTauHist *h, double t, double tau, double w);

/* τ̂ 轨迹外推（Theil-Sen 或加权 LS），返回 t_eval 时刻的 τ̂ */
double AlgoTauEval(const AlgoTauHist *h, double t_eval);

/* 快环配对（对拍 Python pair_with）*/
void AlgoPairWith(const AlgoStream *eff_a, const AlgoStream *eff_b,
                  const AlgoParams *p, const AlgoTauHist *hist,
                  AlgoPairResult *res);

#endif
