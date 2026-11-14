#include "algo/algo_align.h"
#include <math.h>
#include <string.h>

/* ==== 对拍 atrack/align.py（M-C2）==== */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double AlgoCoarseEdgeDelay(const double *ca, int na, const double *cb, int nb,
                           double sensor_fs)
{
  double period, sa, ca_, sb, cb_;
  double d;

  period = 1.0 / sensor_fs;

  /* 圆均值相位 */
  sa = ca_ = sb = cb_ = 0.0;
  for (int i = 0; i < na; i++)
  {
    double ang = 2.0 * M_PI * ca[i] / period;
    sa += sin(ang); ca_ += cos(ang);
  }
  for (int i = 0; i < nb; i++)
  {
    double ang = 2.0 * M_PI * cb[i] / period;
    sb += sin(ang); cb_ += cos(ang);
  }
  double pa = atan2(sa, ca_);
  double pb = atan2(sb, cb_);
  d = (pa - pb) / (2.0 * M_PI) * period;
  if (d > period / 2) d -= period;
  if (d <= -period / 2) d += period;
  return d;
}

static double pearson(const double *x, const double *y, int n)
{
  double mx = 0, my = 0, sx = 0, sy = 0, cxy = 0;
  if (n < 3) return 0.0;
  for (int i = 0; i < n; i++) { mx += x[i]; my += y[i]; }
  mx /= n; my /= n;
  for (int i = 0; i < n; i++)
  {
    sx += (x[i] - mx) * (x[i] - mx);
    sy += (y[i] - my) * (y[i] - my);
    cxy += (x[i] - mx) * (y[i] - my);
  }
  if (sx <= 0 || sy <= 0) return 0.0;
  return cxy / sqrt(sx * sy);
}

/* 静态工作区：单线程非重入（慢环独占）。裸机无堆——全部静态化。
   缓冲生命周期：ga/gb(插值→Hampel) → fa/fb(带通) 贯穿全函数；
   wk1/wk2 复用为 Hampel 副本 → 守卫收集 → 互相关压缩 → 分数精化。
   上板放 CCM RAM（0x10000000，64KB，无 DMA 需求、CPU 零等待）——
   主 RAM 因此省出 44KB（见 MDK-ARM/dual_scale_app.sct）。 */
#ifdef ALGO_EMBEDDED
#define ALGO_CCMRAM __attribute__((section(".ccmram")))
#else
#define ALGO_CCMRAM
#endif
static ALGO_CCMRAM double s_ga[ALIGN_GRID_MAX], s_gb[ALIGN_GRID_MAX];
static ALGO_CCMRAM double s_fa[ALIGN_GRID_MAX], s_fb[ALIGN_GRID_MAX];
static ALGO_CCMRAM double s_wk1[ALIGN_GRID_MAX], s_wk2[ALIGN_GRID_MAX];
static uint8_t s_oka[ALIGN_GRID_MAX], s_okb[ALIGN_GRID_MAX];
static uint8_t s_oma[ALIGN_GRID_MAX], s_omb[ALIGN_GRID_MAX];
static uint8_t s_ok[ALIGN_GRID_MAX];
static int s_idx_ok[ALIGN_GRID_MAX];

void AlgoAlignWindow(const AlgoStream *eff_a, const AlgoStream *eff_b,
                     double t_end, AlgoAlignResult *res)
{
  /* 网格 + 插值 + Hampel + 守卫 + 带通 + 互相关 + 相位斜率。
     全部声明置顶（ARMCC 对 goto 跨越块内初始化报 #546，C99 虽合法但警告）*/
  double *ga = s_ga, *gb = s_gb, *fa = s_fa, *fb = s_fb;
  double *wk1 = s_wk1, *wk2 = s_wk2;
  uint8_t *oka = s_oka, *okb = s_okb, *oma = s_oma, *omb = s_omb, *ok = s_ok;
  int *idx_ok = s_idx_ok;
  double mean_a = 0.0, mean_b = 0.0, s2a = 0.0, s2b = 0.0;
  double best_c = -2.0, best_frac_m = 0.0, tau = 0.0;
  int m_max = 0, best_m = 0;
  double dt = 1.0 / ALIGN_GRID_HZ;
  double t0 = t_end - ALIGN_WIN_S;
  int m = (int)((t_end - t0 - ALIGN_HEAD_DROP_S) / dt);
  res->t = t_end;
  if (m < 10 || m > ALIGN_GRID_MAX)
  {
    res->ok = 0; res->calm = 0; res->rho = 0; res->tau = 0; res->n_pts = 0;
    return;
  }

  /* 逐点插值（tq 内联计算，省一个数组） */
  for (int i = 0; i < m; i++)
  {
    double tq = t0 + ALIGN_HEAD_DROP_S + i * dt;
    ga[i] = AlgoInterpCubic(eff_a, tq, 0.25, &oka[i]);
    gb[i] = AlgoInterpCubic(eff_b, tq, 0.25, &okb[i]);
  }

  /* 网格 Hampel k=10 h=3（在副本上计算——与 Python 向量化语义一致：
     所有中值都基于原始数据，替换是整体的） */
  memcpy(wk1, ga, m * sizeof(double));
  memcpy(wk2, gb, m * sizeof(double));
  for (int i = 0; i < m; i++)
  {
    uint8_t o;
    ga[i] = AlgoHampel(wk1, m, i, 10, 3.0, 0.0, &o); oma[i] = o;
    gb[i] = AlgoHampel(wk2, m, i, 10, 3.0, 0.0, &o); omb[i] = o;
  }
  int n_ok = 0;
  for (int i = 0; i < m; i++)
  {
    ok[i] = (oka[i] && !oma[i]) && (okb[i] && !omb[i]);
    if (ok[i]) n_ok++;
  }

  if (n_ok < (int)(MIN_COVER_S * ALIGN_GRID_HZ * 0.5)
      || (double)n_ok / m < MIN_OVERLAP_FRAC)
  {
    goto fail;
  }

  /* 阶跃守卫：B 前后半中值差 > step_guard_g × ref_span/1000。
     切分按时间序（前 half 个有效样本 vs 其余）——检测窗内阶跃；
     不能先全排序再切两半，那是分布宽度而非阶跃（涌浪大摆动会被误拦）。 */
  {
    int half = n_ok / 2;
    if (half < 3) goto fail;
    double *v = wk1;                    /* 复用：Hampel 副本已用完 */
    int vi = 0;
    for (int i = 0; i < m; i++) if (ok[i]) v[vi++] = gb[i];
    /* 前半排序取中值 */
    for (int i = 1; i < half; i++)
    {
      double key = v[i]; int j2 = i - 1;
      while (j2 >= 0 && v[j2] > key) { v[j2+1] = v[j2]; j2--; }
      v[j2+1] = key;
    }
    double m1 = (half % 2) ? v[half/2] : 0.5*(v[half/2-1]+v[half/2]);
    /* 后半排序取中值 */
    for (int i = half + 1; i < n_ok; i++)
    {
      double key = v[i]; int j2 = i - 1;
      while (j2 >= half && v[j2] > key) { v[j2+1] = v[j2]; j2--; }
      v[j2+1] = key;
    }
    int L2 = n_ok - half;
    double m2 = (L2 % 2) ? v[half + L2/2]
                : 0.5*(v[half + L2/2 - 1] + v[half + L2/2]);
    double guard = STEP_GUARD_G * REF_SPAN_CODES / 1000.0;
    if (fabs(m1 - m2) > guard) goto fail;
  }

  /* 带通（先取 ok 子集均值做去趋势） */
  for (int i = 0; i < m; i++) if (ok[i]) { mean_a += ga[i]; mean_b += gb[i]; }
  mean_a /= n_ok; mean_b /= n_ok;

  AlgoBiquad hp_a, lp_a, hp_b, lp_b;
  AlgoBpInit(&hp_a, &lp_a, BP_LO_HZ, BP_HI_HZ, ALIGN_GRID_HZ);
  AlgoBpInit(&hp_b, &lp_b, BP_LO_HZ, BP_HI_HZ, ALIGN_GRID_HZ);
  for (int i = 0; i < m; i++)
  {
    fa[i] = AlgoBpStep(&hp_a, &lp_a, ga[i] - mean_a);
    fb[i] = AlgoBpStep(&hp_b, &lp_b, gb[i] - mean_b);
  }

  /* bp_rms / calm 判定 */
  for (int i = 0; i < m; i++) if (ok[i]) s2a += fa[i] * fa[i];
  res->bp_rms = sqrt(s2a / n_ok) / REF_SPAN_CODES;
  for (int i = 0; i < m; i++) if (ok[i]) s2b += fb[i] * fb[i];
  res->rms_b = sqrt(s2b / n_ok) / REF_SPAN_CODES;

  if (res->bp_rms < CALM_EPS_RMS)
  {
    res->calm = 1; res->ok = 1; res->tau = 0; res->rho = 0; res->n_pts = n_ok;
    return;
  }

  /* 互相关搜索 ±100ms：压缩 ok 子集到 wk1/wk2 */
  m_max = (int)(FULL_SEARCH_S * ALIGN_GRID_HZ);
  {
    int vi = 0;
    for (int i = 0; i < m; i++) if (ok[i]) { wk1[vi] = fa[i]; wk2[vi] = fb[i]; vi++; }
  }
  for (int lag = -m_max; lag <= m_max; lag++)
  {
    int i0 = (lag >= 0) ? lag : 0;
    int i1 = (lag >= 0) ? 0 : -lag;
    int L = n_ok - ((lag < 0) ? -lag : lag);
    if (L < 3) continue;
    double c = pearson(&wk1[i0], &wk2[i1], L);
    if (c > best_c) { best_c = c; best_m = lag; }
  }

  /* 分数滞后精化（±0.6 格、步 0.02）。
     fa_inner = xa[3..n_ok-4] 前移进 wk1（写 i-3 读 i，前移安全）；
     fb_shift 复用 wk2（整数搜索的 xb 已无用）。 */
  best_frac_m = (double)best_m;   /* 初始 = 整数峰，后续保留分数 */
  {
    int inner_n = n_ok - 6;
    if (inner_n >= 10)
    {
      int vi = 0;
      for (int i = 0; i < m; i++) if (ok[i]) idx_ok[vi++] = i;
      for (int i = 3; i < n_ok - 3; i++) wk1[i - 3] = wk1[i];

      /* mm 取值与 np.arange(m*-0.6, m*+0.6001, 0.02) 逐位一致：
         乘法生成（i*0.02），不用 += 累加（累积漂移会平移候选点） */
      double mm_stop = best_m + 0.6001;
      for (int im = 0; ; im++)
      {
        double mm = best_m - 0.6 + im * 0.02;
        if (mm > mm_stop) break;
        for (int i = 3; i < n_ok - 3; i++)
        {
          int ii = idx_ok[i];
          double src = ii - mm;
          /* 线性插值 fb（在原始 m 格点上） */
          int jj = (int)src;
          if (jj < 0) jj = 0;
          if (jj > m - 2) jj = m - 2;
          double f2 = src - jj;
          if (f2 < 0) f2 = 0;
          if (f2 > 1) f2 = 1;
          wk2[i - 3] = fb[jj] + f2 * (fb[jj + 1] - fb[jj]);
        }
        double c = pearson(wk1, wk2, inner_n);
        if (c > best_c) { best_c = c; best_frac_m = mm; }
      }
    }
  }

  tau = best_frac_m * dt;
  if (tau > FULL_SEARCH_S) tau = FULL_SEARCH_S;
  if (tau < -FULL_SEARCH_S) tau = -FULL_SEARCH_S;
  /* —— 交叉谱相位斜率精化 ——
     与 Python np.fft.rfft 一致：自然长度 n=m 的 DFT bin（不补 2 的幂——
     补零会改变 bin 频率，加权拟合在另一组频率上给出不同的 tau_ps）。
     只需 [PHASE_LO, PHASE_HI] 内 ~20 个 bin，逐 bin 旋转递推直接求和：
     ~2×20×740 次 MAC，上板 2.5s 慢环下开销可忽略。 */
  {
    int n = m;
    double df = 1.0 / ((double)n * dt);          /* rfftfreq: k/(n·dt) */
    int k_lo = (int)ceil(PHASE_LO_HZ / df);
    int k_hi = (int)floor(PHASE_HI_HZ / df);
    if (k_lo < 1) k_lo = 1;
    if (k_hi > n / 2) k_hi = n / 2;

    double num = 0, den = 0;
    for (int k = k_lo; k <= k_hi; k++)
    {
      /* Hann 加权后两通道在 bin k 的 DFT（旋转因子递推，e^{-i2πk/n}） */
      double ang0 = -2.0 * M_PI * (double)k / (double)n;
      double wr = cos(ang0), wi = sin(ang0);
      double cr = 1.0, ci = 0.0;
      double ar = 0, ai = 0, br = 0, bi = 0;
      for (int j = 0; j < n; j++)
      {
        double w = 0.5 - 0.5 * cos(2.0 * M_PI * (double)j / (double)(n - 1));
        double va = fa[j] * w, vb = fb[j] * w;
        /* (cr,ci) 递推本身就是 e^{-iθ}（ci 已带负号），直接乘 */
        ar += va * cr;  ai += va * ci;
        br += vb * cr;  bi += vb * ci;
        double ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr; cr = ncr;
      }
      /* X = FA · conj(FB) = (a+bi)(c-di) = (ac+bd) + (bc-ad)i */
      double Xr = ar * br + ai * bi;
      double Xi = ai * br - ar * bi;
      double mag2 = Xr * Xr + Xi * Xi;
      if (mag2 < 1e-18) continue;
      double phi = atan2(Xi, Xr);
      double f = (double)k * df;
      double w = mag2;
      num += w * phi / (2.0 * M_PI * f);
      den += w;
    }
    if (den > 0)
    {
      double tau_ps = -num / den;
      if (fabs(tau_ps - tau) <= 0.05)
        tau = tau_ps;
    }
  }

  res->ok = 1;
  res->calm = 0;
  res->tau = tau;
  res->rho = best_c;
  res->n_pts = n_ok;
  return;

fail:
  res->ok = 0; res->calm = 0; res->rho = 0; res->tau = 0; res->n_pts = 0;
}
