#include "algo/algo_pair.h"
#include "algo/algo_params.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ==== 对拍 atrack/ratio.py（M-C2）==== */

void AlgoTauHistInit(AlgoTauHist *h)
{
  h->n = 0;
}

void AlgoTauHistPush(AlgoTauHist *h, double t, double tau, double w)
{
  if (h->n >= TAU_HIST_MAX)
  {
    memmove(&h->t[0], &h->t[1], (TAU_HIST_MAX - 1) * sizeof(double));
    memmove(&h->tau[0], &h->tau[1], (TAU_HIST_MAX - 1) * sizeof(double));
    memmove(&h->w[0], &h->w[1], (TAU_HIST_MAX - 1) * sizeof(double));
    h->n = TAU_HIST_MAX - 1;
  }
  h->t[h->n] = t;
  h->tau[h->n] = tau;
  h->w[h->n] = w;
  h->n++;
}

/* Theil-Sen：成对斜率中值（≤7 点 → ≤21 对） */
static double theil_sen(const double *x, const double *y, int n)
{
  double slopes[64];
  int ns = 0;
  for (int a = 0; a < n; a++)
    for (int b = a + 1; b < n; b++)
      if (x[b] != x[a])
        slopes[ns++] = (y[b] - y[a]) / (x[b] - x[a]);
  if (ns == 0) return 0.0;
  /* 中值 */
  for (int i = 1; i < ns; i++)
  {
    double key = slopes[i]; int j = i - 1;
    while (j >= 0 && slopes[j] > key) { slopes[j+1] = slopes[j]; j--; }
    slopes[j+1] = key;
  }
  return (ns % 2) ? slopes[ns/2] : 0.5*(slopes[ns/2-1]+slopes[ns/2]);
}

static double median_arr(double *v, int n)
{
  for (int i = 1; i < n; i++)
  {
    double key = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > key) { v[j+1] = v[j]; j--; }
    v[j+1] = key;
  }
  return (n % 2) ? v[n/2] : 0.5*(v[n/2-1]+v[n/2]);
}

double AlgoTauEval(const AlgoTauHist *h, double t_eval)
{
  if (h->n == 0) return 0.0;

  /* j = 最后一个 t[j] <= t_eval（Python searchsorted side='right' −1）：
     轨迹必须用"当时已存在"的锚点——上板时快环与慢环并行推进，
     回放/批量模式下也必须按 t 定位历史，不能用最终历史的末尾外推 */
  int lo = 0, hi = h->n;
  while (lo < hi)
  {
    int mid = (lo + hi) / 2;
    if (h->t[mid] <= t_eval) lo = mid + 1;
    else hi = mid;
  }
  int j = lo - 1;
  if (j < 0) return 0.0;    /* 预热段：无锚点 */

  double t_center_off = (45.0 - 8.0) / 2.0;   /* (win-head)/2 */
  double t_clip = t_eval;
  if (t_clip > h->t[j] + 30.0)
    t_clip = h->t[j] + 30.0;

  double stale = t_eval - h->t[j];
  int fresh = (stale <= 12.0 && j >= 7);

  if (fresh)
  {
    /* keys[j-7..j] 共 8 点：3 点中值预平滑（边缘 2 点取均值——与
       Python raw[max(0,i-1):i+2] 切片语义一致）+ Theil-Sen */
    int n8 = 8;
    double raw[8], sm[8], xs[8], vs[8];
    for (int i = 0; i < n8; i++)
      raw[i] = h->tau[j - n8 + 1 + i];
    for (int i = 0; i < n8; i++)
    {
      if (i == 0)
        sm[i] = 0.5 * (raw[0] + raw[1]);
      else if (i == n8 - 1)
        sm[i] = 0.5 * (raw[n8 - 2] + raw[n8 - 1]);
      else
      {
        double v[3] = {raw[i - 1], raw[i], raw[i + 1]};
        sm[i] = median_arr(v, 3);
      }
    }
    for (int i = 0; i < n8; i++)
    {
      xs[i] = h->t[j - n8 + 1 + i] - t_center_off;
      vs[i] = sm[i];
    }
    double c1 = theil_sen(xs, vs, n8);
    if (c1 > 1e-3) c1 = 1e-3;
    if (c1 < -1e-3) c1 = -1e-3;
    /* c0 = median(v - c1*x) */
    double resid[8];
    for (int i = 0; i < n8; i++) resid[i] = vs[i] - c1 * xs[i];
    double c0 = median_arr(resid, n8);
    double tau = c1 * (t_clip - t_center_off) + c0;
    if (tau > 0.100) tau = 0.100;
    if (tau < -0.100) tau = -0.100;
    return tau;
  }
  else
  {
    /* 断供：≤12 点加权 LS（keys[j-nfit+1..j]） */
    int nfit = (j + 1 < 12) ? j + 1 : 12;
    double xs[12], vs[12], ws[12];
    for (int i = 0; i < nfit; i++)
    {
      xs[i] = h->t[j - nfit + 1 + i] - t_center_off;
      vs[i] = h->tau[j - nfit + 1 + i];
      double w = h->w[j - nfit + 1 + i];
      ws[i] = (w > 0.05) ? sqrt(w) : sqrt(0.05);
    }
    /* 线性加权拟合 y = c1*x + c0 */
    double sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < nfit; i++)
    {
      double w2 = ws[i] * ws[i];
      sw += w2; sx += w2 * xs[i]; sy += w2 * vs[i];
      sxx += w2 * xs[i] * xs[i]; sxy += w2 * xs[i] * vs[i];
    }
    double det = sw * sxx - sx * sx;
    double c1, c0;
    if (fabs(det) < 1e-30 || nfit < 3)
    {
      c1 = 0; c0 = vs[nfit - 1];
    }
    else
    {
      c1 = (sw * sxy - sx * sy) / det;
      c0 = (sy - c1 * sx) / sw;
    }
    if (c1 > 1e-3) c1 = 1e-3;
    if (c1 < -1e-3) c1 = -1e-3;
    double tau = c1 * (t_clip - t_center_off) + c0;
    if (tau > 0.100) tau = 0.100;
    if (tau < -0.100) tau = -0.100;
    return tau;
  }
}

void AlgoPairWith(const AlgoStream *eff_a, const AlgoStream *eff_b,
                  const AlgoParams *p, const AlgoTauHist *hist,
                  AlgoPairResult *res)
{
  res->n = 0;
  res->n_interp_fail = 0;
  res->n_presence_fail = 0;
  res->n_b = eff_b->n;

  double lo = p->presence_lo * p->presence_center_codes;
  double hi = p->presence_hi * p->presence_center_codes;

  for (int i = 0; i < eff_b->n && res->n < PAIR_MAX; i++)
  {
    double t = eff_b->t[i];
    double tau = AlgoTauEval(hist, t);
    double a_val;
    uint8_t ok;

    a_val = AlgoInterpCubic(eff_a, t + tau, p->interp_max_span_s, &ok);
    if (!ok)
    {
      res->n_interp_fail++;
      continue;
    }
    if (a_val < lo || a_val > hi)
    {
      res->n_presence_fail++;
      continue;
    }
    res->p[res->n].t = t;
    res->p[res->n].r = (eff_b->v[i] - p->zero_codes_b)
                       / (a_val - p->zero_codes_a);
    res->p[res->n].tau = tau;
    res->n++;
  }
}
