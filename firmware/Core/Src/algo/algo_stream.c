#include "algo/algo_stream.h"
#include <string.h>

/* ==== 对拍 atrack/streams.py（M-C1）==== */

void AlgoStreamBuild(AlgoStream *s, const double *t, const int32_t *code, int n)
{
  int i;
  int32_t last = 0;

  s->n = 0;
  for (i = 0; i < n; i++)
  {
    if (s->n == 0 || code[i] != last)
    {
      if (s->n >= ALGO_STREAM_MAX)
      {
        break;             /* 容量保护：丢弃更早数据应在上层做滑动窗口 */
      }
      s->t[s->n] = t[i];
      s->v[s->n] = (double)code[i];
      s->n++;
      last = code[i];
    }
  }
}

static double lagrange4(const double *xs, const double *ys, double x)
{
  double tot = 0.0;
  int m, l;

  for (m = 0; m < 4; m++)
  {
    double term = ys[m];
    for (l = 0; l < 4; l++)
    {
      if (l != m)
      {
        term *= (x - xs[l]) / (xs[m] - xs[l]);
      }
    }
    tot += term;
  }
  return tot;
}

double AlgoInterpCubic(const AlgoStream *s, double tq, double max_span,
                       uint8_t *ok)
{
  int j, i0;
  double span, val;

  *ok = 0U;
  if (s->n == 0)
  {
    return 0.0;
  }
  /* 二分：j = 第一个 >= tq 的下标 */
  {
    int lo = 0, hi = s->n;
    while (lo < hi)
    {
      int mid = (lo + hi) / 2;
      if (s->t[mid] < tq)
      {
        lo = mid + 1;
      }
      else
      {
        hi = mid;
      }
    }
    j = lo;
  }

  /* 值：对所有查询点计算（无效点也有值）——Python interp_masked 的 val
     先于 span 判定计算，后续 Hampel/带通会扫过无效位置，0 与 Python 的
     外推值不同会污染野值掩码和滤波器状态。无效点 ok=0 由下方单独判定。 */
  if (s->n >= 4)
  {
    i0 = j - 2;
    if (i0 < 0)
    {
      i0 = 0;
    }
    if (i0 > s->n - 4)
    {
      i0 = s->n - 4;
    }
    val = lagrange4(&s->t[i0], &s->v[i0], tq);
  }
  else
  {
    /* Python fallback：np.interp 线性，范围外夹到端点值 */
    if (j <= 0)
    {
      val = s->v[0];
    }
    else if (j >= s->n)
    {
      val = s->v[s->n - 1];
    }
    else
    {
      double f = (tq - s->t[j - 1]) / (s->t[j] - s->t[j - 1]);
      val = s->v[j - 1] + f * (s->v[j] - s->v[j - 1]);
    }
  }

  /* 有效性：范围 + 两侧样本距（span=0 视为有效，与 Python 一致） */
  span = 0.0;
  if (j >= 1 && j < s->n)
  {
    span = s->t[j] - s->t[j - 1];
  }
  if (tq >= s->t[0] && tq <= s->t[s->n - 1] && span <= max_span)
  {
    *ok = 1U;
  }
  return val;
}

void AlgoInterpCubicVec(const AlgoStream *s, const double *tq, int m,
                        double max_span, double *out, uint8_t *ok)
{
  int i;

  for (i = 0; i < m; i++)
  {
    out[i] = AlgoInterpCubic(s, tq[i], max_span, &ok[i]);
  }
}
