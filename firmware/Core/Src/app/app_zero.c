#include "app/app_zero.h"
#include "app/app_calib.h"
#include <string.h>
#include <math.h>

/* ==== 清零模式实现：见 app_zero.h 头注（双上下文）==== */

/* 工作副本上限：12s 硬保险 ×10Hz=120，留裕量 */
#define ZO_WORK_MAX 128

static void enter(AppZero *z, AppZeroState s, double t_now)
{
  z->state = s;
  z->t_state = t_now;
}

/* 升序排序副本（插入排序，n≤128 主循环单线程）*/
static void sort_copy(const double *src, int n, double *dst)
{
  int i, j;
  double key;
  memcpy(dst, src, (size_t)n * sizeof(double));
  for (i = 1; i < n; i++)
  {
    key = dst[i]; j = i - 1;
    while (j >= 0 && dst[j] > key) { dst[j + 1] = dst[j]; j--; }
    dst[j + 1] = key;
  }
}

static double median_of(const double *v, int n)
{
  return (n % 2) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/* 样本统计：中值 + σ_mad（克等效，用给定 k 折算）*/
static void sample_stats(AppScale *app, int n,
                         double *med_out, double *sig_g_out, double k)
{
  static double v[ZO_WORK_MAX], d[ZO_WORK_MAX];
  double med, mad;
  int i;
  if (n > ZO_WORK_MAX) n = ZO_WORK_MAX;
  sort_copy(app->calib_r, n, v);
  med = median_of(v, n);
  for (i = 0; i < n; i++)
  {
    d[i] = fabs(app->calib_r[i] - med);
  }
  sort_copy(d, n, v);
  mad = median_of(v, n);
  *med_out = med;
  *sig_g_out = 1.4826 * mad * k;
}

static void fail(AppZero *z, double t_now, int code)
{
  z->linger_code = code;
  z->t_linger = t_now;
  enter(z, ZO_RESULT, t_now);
}

/* 跨度上下文入口：锁定称量中且读数在标准砝码 ±20%（400~600 g）窗内 */
static int span_entry_ok(AppScale *app)
{
  double m, d, lo, hi;
  if ((app->est.state != 1) || (app->est.locked == 0)
      || (app->est.n_readings <= 0))
  {
    return 0;
  }
  m = app->est.readings[app->est.n_readings - 1].mass_g;
  d = (m < 0.0) ? -m : m;
  lo = APP_ZERO_SPAN_MASS_G * (1.0 - APP_ZERO_SPAN_FRAC);
  hi = APP_ZERO_SPAN_MASS_G * (1.0 + APP_ZERO_SPAN_FRAC);
  return (d > lo && d < hi) ? 1 : 0;
}

int AppZeroPoll(AppZero *z, AppScale *app, double t_now,
                int ev_zero, int ev_span)
{
  double el = t_now - z->t_state;

  if ((ev_zero != 0) || (ev_span != 0))
  {
    if (z->state == ZO_IDLE)
    {
      if (ev_span != 0)
      {
        /* 左键：跨度校正（带 500 g 标准砝码修 k）。
           入口门：估计器锁定且读数在 ±20%（400~600 g）窗内；不在窗内
           直接拒绝，不再回退成零点（分键后语义单一）。 */
        if (span_entry_ok(app))
        {
          z->k_old = app->p.k_g_per_r;
          z->sigma_g = 0.0;
          z->offset_g = 0.0;
          z->n_used = 0;
          AppScaleCalibReset(app);
          app->calib_collect = 1U;
          enter(z, ZO_SPAN, t_now);
          return 0;
        }
        z->sigma_g = 0.0;
        z->offset_g = 0.0;
        z->n_used = 0;
        fail(z, t_now, 9300);
        return 0;
      }
      /* 中键：零点重锚（空盘）——k 已知、基线已建立、估计器空闲、无并行采集 */
      if ((app->p.k_g_per_r == 0.0)
          || (app->est.idle_n < 30)
          || (app->est.state != 0)
          || (app->calib_collect != 0U))
      {
        z->sigma_g = 0.0;
        z->offset_g = 0.0;
        z->n_used = 0;
        fail(z, t_now, 9300);
        return 0;
      }
      z->k_old = 0.0;
      z->sigma_g = 0.0;
      z->offset_g = 0.0;
      z->n_used = 0;
      AppScaleCalibReset(app);
      app->calib_collect = 1U;
      enter(z, ZO_COLLECT, t_now);
    }
    else
    {
      app->calib_collect = 0U;      /* 中止（任何采集中状态，任一键都可中止）*/
      enter(z, ZO_IDLE, t_now);
    }
    return 0;
  }

  if (z->state == ZO_COLLECT)
  {
    /* 凑齐即止 + 12s 硬保险（与 prep 空盘段同款）*/
    if (((el >= (double)APP_ZERO_N_MIN / 10.0)
         && (AppScaleCalibN(app) >= APP_ZERO_N_MIN))
        || (el >= APP_ZERO_SAFETY_S))
    {
      double med;
      app->calib_collect = 0U;
      z->n_used = AppScaleCalibN(app);
      if (z->n_used < APP_ZERO_N_MIN)
      {
        fail(z, t_now, 9300);
        return 0;
      }
      sample_stats(app, z->n_used, &med, &z->sigma_g,
                   app->p.k_g_per_r);
      z->offset_g = (med - app->est.baseline) * app->p.k_g_per_r;
      if (z->sigma_g > APP_ZERO_SIGMA_G)
      {
        fail(z, t_now, 9100);       /* 太晃：稳稳再按 */
        return 0;
      }
      if (fabs(z->offset_g) > APP_ZERO_OFFSET_G)
      {
        fail(z, t_now, 9200);       /* 盘上有物？清零≠去皮 */
        return 0;
      }
      /* 重锚定：基线=新中值，idle 历史用新值填满（否则旧姿态样本
         会把中值拖回去）。填满 APP_ZERO_REARM_N(30)=ready 门限——若
         只置 idle_n=1，清零完成后 AppScaleLiveMass 的 ready=0，实时
         显示继续闪 "---" 直到 idle 重新攒满 30（~3s），即实测的
         "清零闪烁超 10s"（跨度校正不改 idle，故无此现象）。
         step 计数清零防误触发。*/
      app->est.baseline = med;
      app->est.idle_head = 0;
      app->est.above_n = 0;
      app->est.idle_n = APP_ZERO_REARM_N;
      for (int i = 0; i < APP_ZERO_REARM_N; i++)
      {
        app->est.idle_t[i] = t_now;
        app->est.idle_v[i] = med;
      }
      enter(z, ZO_IDLE, t_now);     /* 成功：回实时显示 ≈0.0 */
      return 1;
    }
  }
  else if (z->state == ZO_SPAN)
  {
    if (((el >= (double)APP_ZERO_N_MIN / 10.0)
         && (AppScaleCalibN(app) >= APP_ZERO_N_MIN))
        || (el >= APP_ZERO_SAFETY_S))
    {
      double med, implied;
      uint8_t fok = 0U;
      int rc;
      app->calib_collect = 0U;
      z->n_used = AppScaleCalibN(app);
      if (z->n_used < APP_ZERO_N_MIN)
      {
        fail(z, t_now, 9300);
        return 0;
      }
      sample_stats(app, z->n_used, &med, &z->sigma_g, z->k_old);
      implied = (med - app->est.baseline) * z->k_old;
      z->offset_g = implied - APP_ZERO_SPAN_MASS_G;
      if (z->sigma_g > APP_ZERO_SIGMA_G)
      {
        fail(z, t_now, 9100);
        return 0;
      }
      /* 窗口复核（入口查的是锁定读数，此处查实测中值——采集中
         被取走/更换在此拦截）：隐含质量须仍在 ±20% 窗内 */
      if (fabs(z->offset_g) > APP_ZERO_SPAN_MASS_G * APP_ZERO_SPAN_FRAC)
      {
        fail(z, t_now, 9200);
        return 0;
      }
      rc = AppCalibCommit(app, APP_ZERO_SPAN_MASS_G,
                          app->est.baseline, med,
                          AppScaleCalibMeanA(app),
                          AppScaleCalibMeanA(app), &fok);
      if (rc == 1)
      {
        enter(z, ZO_IDLE, t_now);   /* 成功：锁定读数即刻按新 k 精化 */
        return 2;
      }
      else if (rc == -1)
      {
        fail(z, t_now, 7777);       /* k 超范围：砝码与配置不符 */
      }
      else
      {
        fail(z, t_now, 9200);       /* Δr 异常 */
      }
      (void)fok;
    }
  }
  else if (z->state == ZO_RESULT)
  {
    if (el >= APP_ZERO_RESULT_S)
    {
      enter(z, ZO_IDLE, t_now);
    }
  }
  return 0;
}

int AppZeroDisplay(AppZero *z, double t_now)
{
  (void)t_now;
  if ((z->state == ZO_COLLECT) || (z->state == ZO_SPAN))
  {
    return -3;                      /* main：闪烁 "---" */
  }
  if (z->state == ZO_RESULT)
  {
    return z->linger_code;
  }
  return -1;
}
