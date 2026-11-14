#include "app/app_scale.h"
#include <string.h>

/* ==== 称量应用引擎：见 app_scale.h 头注 ==== */

/* 比值 FIFO 深度：延迟 Hampel k=5 需 6 点在视线内，留裕量 */
#define APP_RR_FIFO_N  12

/* 满时保留最后 KEEP 个（一次性裁剪，均摊 memmove）*/
#define STREAM_KEEP  ((ALGO_STREAM_MAX - 64) > 0 ? (ALGO_STREAM_MAX - 64) \
                                                 : (ALGO_STREAM_MAX / 2))

static void stream_trim(AlgoStream *s)
{
  int keep;
  if (s->n < ALGO_STREAM_MAX) return;
  keep = STREAM_KEEP;
  memmove(&s->t[0], &s->t[s->n - keep], keep * sizeof(double));
  memmove(&s->v[0], &s->v[s->n - keep], keep * sizeof(double));
  s->n = keep;
}

static void push_event(AppScale *app, const AppEvent *e)
{
  int next = (app->ev_tail + 1) % 8;
  if (next == app->ev_head) return;   /* 满：丢最新。读数事件低频，
                                         主循环消费快，正常不触达 */
  app->ev[app->ev_tail] = *e;
  app->ev_tail = next;
}

static void emit_reading_ev(AppScale *app, AppEvType ty, double t)
{
  AlgoReading *rd = &app->est.readings[app->est.n_readings - 1];
  AppEvent e = { .type = ty, .t = t, .mass_g = rd->mass_g,
                 .rel_std = rd->rel_std, .level = rd->level };
  push_event(app, &e);
}

/* —— 延迟定点源头清洗：j 的 ±3 窗口齐（j+3 已到）时计算 —— */
static void clean_finalize(AppScale *app, int ch)
{
  int i = app->raw[ch].n - 4;         /* 待定点的原始索引 */
  uint8_t o;
  AlgoStream *s;
  double v;
  if (i < 0) return;
  v = AlgoHampel(app->raw[ch].v, app->raw[ch].n, i, 3, 3.0, 0.0, &o);
  s = &app->clean[ch];
  stream_trim(s);
  s->t[s->n] = app->raw[ch].t[i];
  s->v[s->n] = v;
  s->n++;
}

void AppScaleInit(AppScale *app, const AlgoParams *p)
{
  memset(app, 0, sizeof(*app));
  app->p = *p;
  AlgoTauHistInit(&app->hist);
  AlgoEstInit(&app->est);
}

/* 比值入公共流水线：FIFO + MAD 地板滑窗 + 延迟 5 点 Hampel 定点 +
   估计器喂入 + 读数事件。配对路径与静态回退路径共用（统计一致）。 */
static void feed_ratio(AppScale *app, double t, double r)
{
  /* 比值 FIFO + 对应 B 时刻（延迟 5 点 Hampel 需并行时间戳）*/
  if (app->rr_n == APP_RR_FIFO_N)
  {
    memmove(&app->rr[0], &app->rr[1], (APP_RR_FIFO_N - 1) * sizeof(double));
    memmove(&app->pair_t[0], &app->pair_t[1],
            (APP_RR_FIFO_N - 1) * sizeof(double));
    app->rr_n--;
  }
  app->rr[app->rr_n] = r;
  app->pair_t[app->rr_n] = t;
  app->rr_n++;

  /* 原始比值历史（MAD 地板滑窗估计）*/
  if (app->rr_hist_n == APP_RR_HIST_N)
    memmove(&app->rr_hist[0], &app->rr_hist[1],
            (APP_RR_HIST_N - 1) * sizeof(double));
  else
    app->rr_hist_n++;
  app->rr_hist[app->rr_hist_n - 1] = r;
  if ((app->rr_hist_n & (APP_RR_FLOOR_UPD - 1)) == 0)
    app->rr_floor = AlgoNoiseFromDiff(app->rr_hist, app->rr_hist_n);

  /* 定点 j = rr_n − 6（±5 窗口齐）*/
  {
    int j = app->rr_n - 6;
    uint8_t om;
    double rc;
    double tj;
    int prev_nr, prev_state, prev_locked;
    if (j < 0) return;
    rc = AlgoHampel(app->rr, app->rr_n, j, app->p.hampel_k,
                    app->p.hampel_h, app->rr_floor, &om);
    app->last_r_clean = rc;
    tj = app->pair_t[j];

    /* 估计器可信度输入（_rho_at 语义：最近一个 ρ≥门限窗）+
       静水标志（最近对齐窗带通能量，供盘中增减检测自适应门限；
       无窗时 bp_rms=0 → 静水）*/
    app->est.quiet = (app->last_align.bp_rms < 0.008) ? 1U : 0U;
    app->est.last_rho = app->have_pass_align ? app->last_pass_rho : 0.0;
    app->est.last_calm = app->have_pass_align ? 0 : 1;
    app->est.gap_frac = (double)app->n_interp_fail /
                        (app->n_pair_b > 0 ? app->n_pair_b : 1);

    prev_nr = app->est.n_readings;
    prev_state = app->est.state;
    prev_locked = app->est.locked;
    AlgoEstFeed(&app->est, &app->p, tj, rc, om);

    if (app->est.n_readings > prev_nr)
      emit_reading_ev(app, APP_EV_READING, tj);
    else if (app->est.state == 1 && app->est.locked && app->est.n_readings > 0)
      emit_reading_ev(app, APP_EV_REFINE, tj);   /* 精化持续到移除（滑动窗）*/
    /* 定稿事件：回臂（1→0）→ REARM（显示冻结抄录）；
       盘中增减（locked 1→0 仍在 WEIGHING）→ RESTEP（不冻结）*/
    if (prev_state == 1 && app->est.n_readings > 0
        && app->est.state == 0)
      emit_reading_ev(app, APP_EV_REARM, tj);
    else if (prev_state == 1 && prev_locked == 1 && app->est.locked == 0
             && app->est.n_readings > 0)
      emit_reading_ev(app, APP_EV_RESTEP, tj);
  }
}

void AppScaleFeed(AppScale *app, double t, int ch, int32_t code)
{
  if ((uint32_t)ch > 1U) return;

  /* —— 标定原始采样：在变化沿过滤之前直取轮询码对 —— */
  if (ch == 0)
  {
    app->calib_a_last = code;
    app->calib_a_t = t;
  }
  else if (ch == 1)
  {
    double afresh = t - app->calib_a_t;
    app->calib_b_last = code;        /* 放置检测用：无条件跟踪 */
    if (app->calib_collect && afresh < 0.25)
    {
      double r = ((double)code - app->p.zero_codes_b) /
                 ((double)app->calib_a_last - app->p.zero_codes_a);
      if (app->calib_r_n == APP_RR_HIST_N)
      {
        memmove(&app->calib_r[0], &app->calib_r[1],
                (APP_RR_HIST_N - 1) * sizeof(double));
        app->calib_r_n--;
      }
      app->calib_r[app->calib_r_n] = r;
      app->calib_r_n++;
      app->calib_a_sum += (double)app->calib_a_last;
      app->calib_a_n++;
    }
    /* —— 静态回退：配对饿死 >3s 时，原始码对直接走比值/估计器流水线 ——
       静态下无 α 泄漏，同拍原始码对就是正确测量；配对恢复则自动切回。*/
    if (afresh < 0.25 && (t - app->last_pair_t) > 3.0)
    {
      double a = (double)app->calib_a_last;
      double lo = app->p.presence_lo * app->p.presence_center_codes;
      double hi = app->p.presence_hi * app->p.presence_center_codes;
      if (a >= lo && a <= hi)
      {
        double r = ((double)code - app->p.zero_codes_b) /
                   (a - app->p.zero_codes_a);
        app->raw_fallback = 1U;
        app->n_pair_b++;
        feed_ratio(app, t, r);
      }
    }
  }

  /* 变化沿检测（Python build_stream：code 变化的首个轮询时刻）*/
  if (app->raw[ch].has_last && code == app->raw[ch].last_code)
    return;
  app->raw[ch].last_code = code;
  app->raw[ch].has_last = 1;

  /* 原始 FIFO（滚动 APP_RAW_FIFO_N 点）*/
  if (app->raw[ch].n == APP_RAW_FIFO_N)
  {
    memmove(&app->raw[ch].t[0], &app->raw[ch].t[1],
            (APP_RAW_FIFO_N - 1) * sizeof(double));
    memmove(&app->raw[ch].v[0], &app->raw[ch].v[1],
            (APP_RAW_FIFO_N - 1) * sizeof(double));
    app->raw[ch].n--;
  }
  app->raw[ch].t[app->raw[ch].n] = t;
  app->raw[ch].v[app->raw[ch].n] = (double)code;
  app->raw[ch].n++;

  int prev_clean_n = app->clean[ch].n;
  clean_finalize(app, ch);

  /* 对齐锚点：首个清洗样本（任一通道）*/
  if (!app->have_anchor && (app->clean[0].n > 0 || app->clean[1].n > 0))
  {
    double ta = (app->clean[0].n > 0) ? app->clean[0].t[0] : 1e30;
    double tb = (app->clean[1].n > 0) ? app->clean[1].t[0] : 1e30;
    app->t_lo = (ta < tb) ? ta : tb;
    app->next_align = app->t_lo + ALIGN_WIN_S;
    app->have_anchor = 1;
  }

  /* —— 快环配对：B 新清洗样本入待配队列，A 流末端越过 tb+0.15s 再配 ——
     （B 定点滞后 0.3s，A 清洗流末端同样只到 ≈tb；τ>0 时 tb+τ 超出末端，
       立即配会 interp_fail。Python 批处理用最终 A 流无此问题。） */
  if (ch == 1 && app->clean[1].n > prev_clean_n)
  {
    if (app->pend_n == APP_PEND_N)
    {                                   /* 队满：最老条目超时强配（A 仍缺
                                           数据则按缺口计）*/
      app->pend_n--;
      memmove(&app->pend_t[0], &app->pend_t[1], app->pend_n * sizeof(double));
      memmove(&app->pend_v[0], &app->pend_v[1], app->pend_n * sizeof(double));
    }
    app->pend_t[app->pend_n] = app->clean[1].t[app->clean[1].n - 1];
    app->pend_v[app->pend_n] = app->clean[1].v[app->clean[1].n - 1];
    app->pend_n++;
  }

  /* A 清洗流末端时间 */
  double a_end = (app->clean[0].n > 0) ? app->clean[0].t[app->clean[0].n - 1]
                                       : -1e30;
  while (app->pend_n > 0 && app->pend_t[0] + 0.15 <= a_end)
  {
    double tb = app->pend_t[0];
    double vb = app->pend_v[0];
    memmove(&app->pend_t[0], &app->pend_t[1],
            (app->pend_n - 1) * sizeof(double));
    memmove(&app->pend_v[0], &app->pend_v[1],
            (app->pend_n - 1) * sizeof(double));
    app->pend_n--;
    {
    double tau = AlgoTauEval(&app->hist, tb);
    uint8_t iok;
    double a_val = AlgoInterpCubic(&app->clean[0], tb + tau,
                                   app->p.interp_max_span_s, &iok);
    app->n_pair_b++;
    if (!iok)
    {
      app->n_interp_fail++;
      continue;
    }
    double lo = app->p.presence_lo * app->p.presence_center_codes;
    double hi = app->p.presence_hi * app->p.presence_center_codes;
    if (a_val < lo || a_val > hi)
    {
      app->n_interp_fail++;   /* 在位保护拒绝也计缺口（Python stats 语义）*/
      continue;
    }

    double r = (vb - app->p.zero_codes_b) / (a_val - app->p.zero_codes_a);

    /* 配对样本入队：回退解除（运动恢复，切回经验证的运动路径）*/
    app->last_pair_t = tb;
    app->raw_fallback = 0U;
    feed_ratio(app, tb, r);
    }
  }
}

void AppScaleProcess(AppScale *app, double t_now)
{
  if (!app->have_anchor) return;
  while (app->next_align <= t_now)
  {
    AlgoAlignResult ar;
    AlgoAlignWindow(&app->clean[0], &app->clean[1], app->next_align, &ar);
    app->last_align = ar;

    if (ar.ok && !ar.calm && ar.rho >= RHO_PASS)
    {
      double base = ar.rms_b / 0.06;
      double w_rms = base * base * base * base;
      double w_rho, d;
      if (w_rms < 0.02) w_rms = 0.02;
      if (w_rms > 1.0) w_rms = 1.0;
      d = 1.0 - ar.rho * ar.rho;
      if (d < 0.001) d = 0.001;
      w_rho = 0.004 / d;
      if (w_rho < 0.1) w_rho = 0.1;
      if (w_rho > 4.0) w_rho = 4.0;
      AlgoTauHistPush(&app->hist, app->next_align, ar.tau, w_rms * w_rho);
      app->have_pass_align = 1;
      app->last_pass_rho = ar.rho;
    }

    {
      AppEvent e = { .type = APP_EV_ALIGN, .t = app->next_align, .align = ar };
      push_event(app, &e);
    }
    app->next_align += ALIGN_UPDATE_S;
  }
}

double AppScaleRatioTrim(AppScale *app)
{
  if (app->rr_hist_n < 30) return 0.0;
  return AlgoTrimmedMean(app->rr_hist, app->rr_hist_n, 0.10);
}

void AppScaleCalibReset(AppScale *app)
{
  app->calib_r_n = 0;
  app->calib_a_sum = 0.0;
  app->calib_a_n = 0;
}

double AppScaleCalibMeanR(AppScale *app)
{
  if (app->calib_r_n < 30) return 0.0;
  return AlgoTrimmedMean(app->calib_r, app->calib_r_n, 0.10);
}

int AppScaleCalibN(AppScale *app)
{
  return app->calib_r_n;
}

/* 最近 k 个已采比值的算术均值（不足 k 用全部；n=0 返回 0）。
   prep 放置/移除监视用：比值域共模相消，涌荡残余 ~±0.002，
   与 B 原始码（±数千码摆动）不同，是运动免疫的判据。 */
double AppScaleCalibTailMeanR(AppScale *app, int k)
{
  int i;
  double s = 0.0;
  if (app->calib_r_n <= 0 || k <= 0) return 0.0;
  if (k > app->calib_r_n) k = app->calib_r_n;
  for (i = app->calib_r_n - k; i < app->calib_r_n; i++)
  {
    s += app->calib_r[i];
  }
  return s / (double)k;
}

int32_t AppScaleCalibB(AppScale *app)
{
  return app->calib_b_last;
}

double AppScaleCalibMeanA(AppScale *app)
{
  if (app->calib_a_n <= 0) return 0.0;
  return app->calib_a_sum / (double)app->calib_a_n;
}

double AppScaleLiveMass(AppScale *app, int *ready)
{
  if (app->est.idle_n < 30 || app->p.k_g_per_r == 0.0)
  {
    if (ready) *ready = 0;
    return 0.0;
  }
  if (ready) *ready = 1;
  return app->p.k_g_per_r * (app->last_r_clean - app->est.baseline);
}

AppEvent AppScalePopEvent(AppScale *app)
{
  AppEvent e;
  if (app->ev_head == app->ev_tail)
  {
    e.type = APP_EV_NONE;
    return e;
  }
  e = app->ev[app->ev_head];
  app->ev_head = (app->ev_head + 1) % 8;
  return e;
}
