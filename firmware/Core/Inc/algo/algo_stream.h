#ifndef __ALGO_STREAM_H__
#define __ALGO_STREAM_H__

#include <stdint.h>

/* ==== 流层：变化沿有效样本流 + 带间隙保护的三次插值 ====
   对拍 Python atrack/streams.py（M-C1）。 */

/* 容量：host 对拍（CSV 全量回放 360s+）与上板（滑窗，仅保留对齐窗所需
   ~51s）分开配置。上板由 Keil 工程定义 ALGO_EMBEDDED。 */
#ifdef ALGO_EMBEDDED
#define ALGO_STREAM_MAX 512    /* 10Hz×51.2s：对齐窗 45s（格点起点 t0+8s）+ 裕量 */
#else
#define ALGO_STREAM_MAX 4096   /* 单通道有效样本容量（10Hz×409s，覆盖称量会话） */
#endif

typedef struct
{
  double t[ALGO_STREAM_MAX];
  double v[ALGO_STREAM_MAX];
  int n;
} AlgoStream;

/* 从原始轮询记录构建变化沿流（code 变化的首个轮询时刻）。
   polls 按 t 升序、每通道已分离传入。Python build_stream 语义。 */
void AlgoStreamBuild(AlgoStream *s, const double *t, const int32_t *code, int n);

/* 4 点拉格朗日三次插值 + 间隙保护（Python interp_masked, cubic=True）。
   tq 落在 [t0, tn-1] 外、或两侧样本距 > max_span → ok=0。
   单点查询版（快环逐样本用）。 */
double AlgoInterpCubic(const AlgoStream *s, double tq, double max_span,
                       uint8_t *ok);

/* 向量版（慢环窗口重采样用；out/ok 长度 m） */
void AlgoInterpCubicVec(const AlgoStream *s, const double *tq, int m,
                        double max_span, double *out, uint8_t *ok);

#endif
