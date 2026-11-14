#include "algo/algo_dsp.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ==== 对拍 atrack/dsp.py（M-C1）==== */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void biquad_coeffs(double f0, double fs, int is_hp, double q,
                          double *b0, double *b1, double *b2,
                          double *a1, double *a2)
{
  double w0, c, s, alpha, a0, bg;

  w0 = 2.0 * M_PI * f0 / fs;
  c = cos(w0);
  s = sin(w0);
  alpha = s / (2.0 * q);
  if (is_hp)
  {
    bg = (1.0 + c) / 2.0;
    *b1 = -(1.0 + c);
  }
  else
  {
    bg = (1.0 - c) / 2.0;
    *b1 = (1.0 - c);
  }
  a0 = 1.0 + alpha;
  *b0 = bg / a0;
  *b1 = *b1 / a0;
  *b2 = bg / a0;
  *a1 = (-2.0 * c) / a0;
  *a2 = (1.0 - alpha) / a0;
}

void AlgoBpInit(AlgoBiquad *hp, AlgoBiquad *lp, double lo_hz, double hi_hz,
                double fs_hz)
{
  double q = 1.0 / sqrt(2.0);   /* Butterworth，与 dsp.py 一致 */

  biquad_coeffs(lo_hz, fs_hz, 1, q, &hp->b0, &hp->b1, &hp->b2, &hp->a1, &hp->a2);
  hp->z1 = hp->z2 = hp->y1 = hp->y2 = 0.0;
  biquad_coeffs(hi_hz, fs_hz, 0, q, &lp->b0, &lp->b1, &lp->b2, &lp->a1, &lp->a2);
  lp->z1 = lp->z2 = lp->y1 = lp->y2 = 0.0;
}

double AlgoBiquadStep(AlgoBiquad *f, double x)
{
  /* 直接 II 型（与 Python biquad_filter 逐步同构，逐位对齐） */
  double y = f->b0 * x + f->b1 * f->z1 + f->b2 * f->z2
             - f->a1 * f->y1 - f->a2 * f->y2;
  f->z2 = f->z1;
  f->z1 = x;
  f->y2 = f->y1;
  f->y1 = y;
  return y;
}

double AlgoBpStep(AlgoBiquad *hp, AlgoBiquad *lp, double x)
{
  return AlgoBiquadStep(lp, AlgoBiquadStep(hp, x));
}

/* 滑动中值：把窗口拷贝到栈上小数组做插入排序取中值（k≤16，定长 33） */
#define HAMPEL_MAX_K 16

static void win_median_abs(const double *x, int n, int i, int k,
                           double *med, double *mad)
{
  double w[2 * HAMPEL_MAX_K + 1] = {0};
  double v[2 * HAMPEL_MAX_K + 1] = {0};
  int j, m, len;
  double key;

  len = 0;
  for (j = i - k; j <= i + k; j++)
  {
    int jj = j;
    if (jj < 0)
    {
      jj = 0;              /* np.pad edge 模式 */
    }
    if (jj > n - 1)
    {
      jj = n - 1;
    }
    w[len] = x[jj];
    len++;
  }
  /* 插入排序 */
  for (j = 1; j < len; j++)
  {
    key = w[j];
    m = j - 1;
    while (m >= 0 && w[m] > key)
    {
      w[m + 1] = w[m];
      m--;
    }
    w[m + 1] = key;
  }
  *med = (len % 2 == 1) ? w[len / 2] : 0.5 * (w[len / 2 - 1] + w[len / 2]);
  /* |w - med| 的中值 */
  for (j = 0; j < len; j++)
  {
    v[j] = fabs(w[j] - *med);
  }
  for (j = 1; j < len; j++)
  {
    key = v[j];
    m = j - 1;
    while (m >= 0 && v[m] > key)
    {
      v[m + 1] = v[m];
      m--;
    }
    v[m + 1] = key;
  }
  *mad = (len % 2 == 1) ? v[len / 2] : 0.5 * (v[len / 2 - 1] + v[len / 2]);
}

double AlgoHampel(const double *x, int n, int i, int k, double h,
                  double mad_floor, uint8_t *outlier)
{
  double med, mad, thr;

  *outlier = 0U;
  if (n < 2 * k + 1)
  {
    return x[i];           /* Python: 窗口不足时原样返回 */
  }
  if (k > HAMPEL_MAX_K)
  {
    k = HAMPEL_MAX_K;
  }
  win_median_abs(x, n, i, k, &med, &mad);
  mad *= 1.4826;
  if (mad < mad_floor)
  {
    mad = mad_floor;
  }
  mad += 1e-12;
  thr = h * mad;
  if (fabs(x[i] - med) > thr)
  {
    *outlier = 1U;
    return med;
  }
  return x[i];
}

/* 中值（n ≤ 513）：堆上排序。noise_from_diff 的差分数组长度可到流长 */
static double median_sorted(double *v, int n)
{
  int i, j;
  double key;

  for (i = 1; i < n; i++)
  {
    key = v[i];
    j = i - 1;
    while (j >= 0 && v[j] > key)
    {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }
  return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double AlgoNoiseFromDiff(const double *x, int n)
{
  double *d;
  double med;
  int i;

  if (n < 2)
  {
    return 0.0;
  }
  d = (double *)malloc((size_t)(n - 1) * sizeof(double));
  if (d == NULL)
  {
    return 0.0;
  }
  for (i = 0; i < n - 1; i++)
  {
    d[i] = fabs(x[i + 1] - x[i]);
  }
  med = median_sorted(d, n - 1);
  free(d);
  return 1.4826 * med / sqrt(2.0);
}

double AlgoParabolic(const double *y, int n, int i)
{
  double denom;

  if (i <= 0 || i >= n - 1)
  {
    return 0.0;
  }
  denom = y[i - 1] - 2.0 * y[i] + y[i + 1];
  if (denom == 0.0)
  {
    return 0.0;
  }
  return 0.5 * (y[i - 1] - y[i + 1]) / denom;
}
