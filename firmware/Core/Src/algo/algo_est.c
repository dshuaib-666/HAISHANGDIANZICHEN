#include "algo/algo_est.h"
#include <math.h>
#include <string.h>

/* ==== 对拍 atrack/estimate.py + confidence.py（M-C2）==== */

/* 静态工作区：单线程、非重入（主循环独占）。大数组禁止上栈——
   上板默认栈 2KB 级，EST_WIN_MAX 数组会立刻溢栈。 */
#define EST_TRIM_WORK_MAX  512   /* trimmed_mean 收集窗上限：收敛窗 3s×10Hz=30、
                                   精化窗 hold 10s×10Hz=100、标定窗 ≤51s×10Hz */
static double s_idle_v[EST_IDLE_HIST_MAX];
static double s_lock_win[EST_TRIM_WORK_MAX];
static double s_trim_v[EST_TRIM_WORK_MAX];

/* 截尾均值（algo_est.h 声明） */
double AlgoTrimmedMean(const double *x, int n, double frac)
{
  if (n <= 0) return 0.0;
  if (n > EST_TRIM_WORK_MAX) n = EST_TRIM_WORK_MAX;   /* 防御：调用方窗bounded */
  int k = (int)(n * frac);
  if (n - 2 * k < 1) k = 0;
  memcpy(s_trim_v, x, n * sizeof(double));
  for (int i = 1; i < n; i++)
  {
    double key = s_trim_v[i]; int j = i - 1;
    while (j >= 0 && s_trim_v[j] > key) { s_trim_v[j+1] = s_trim_v[j]; j--; }
    s_trim_v[j+1] = key;
  }
  double sum = 0;
  for (int i = k; i < n - k; i++) sum += s_trim_v[i];
  return sum / (n - 2 * k);
}

static double median_sorted(double *v, int n)
{
  for (int i = 1; i < n; i++)
  {
    double key = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > key) { v[j+1] = v[j]; j--; }
    v[j+1] = key;
  }
  return (n % 2) ? v[n/2] : 0.5*(v[n/2-1]+v[n/2]);
}

void AlgoEstInit(AlgoEstimator *e)
{
  memset(e, 0, sizeof(*e));
  e->baseline_sigma = 1e-9;
}

static void feed_idle(AlgoEstimator *e, const AlgoParams *p, double t, double r)
{
  /* 滑动基线（环形缓冲，容量 600） */
  if (e->idle_n < EST_IDLE_HIST_MAX)
  {
    e->idle_t[e->idle_n] = t;
    e->idle_v[e->idle_n] = r;
    e->idle_n++;
  }
  else
  {
    /* 覆盖最旧 */
    int idx = e->idle_head;
    e->idle_t[idx] = t;
    e->idle_v[idx] = r;
    e->idle_head = (e->idle_head + 1) % EST_IDLE_HIST_MAX;
  }

  if (e->idle_n < 30) return;

  memcpy(s_idle_v, e->idle_v, e->idle_n * sizeof(double));
  e->baseline = median_sorted(s_idle_v, e->idle_n);
  for (int i = 0; i < e->idle_n; i++)
    s_idle_v[i] = fabs(e->idle_v[i] - e->baseline);
  e->baseline_sigma = median_sorted(s_idle_v, e->idle_n) * 1.4826 + 1e-9;
  if (e->baseline_sigma < p->min_baseline_sigma)
    e->baseline_sigma = p->min_baseline_sigma;   /* 量化下限（静态台架）*/

  if (fabs(r - e->baseline) > p->step_n_sigma * e->baseline_sigma)
    e->above_n++;
  else
    e->above_n = 0;

  /* 向下持续性计数（重锚门）：瞬态刮蹭/撞击回弹下冲亚秒即回，真地板
     搬家持续停在新电平——时间特征是二者唯一可靠区别 */
  if (r < e->baseline - p->step_n_sigma * e->baseline_sigma)
    e->down_n++;
  else
    e->down_n = 0;

  if (e->above_n < p->step_min_n || e->baseline_sigma >= 0.05)
    return;

  double est = p->k_g_per_r * (r - e->baseline);
  /* 绝对电平门（2026-10-31 拉力事件）：真实物体必在标定空盘锚之上
     ≥min_lock。向上提螺柱 >3s 后基线在拉力电平重锚，放手 +Δ 阶跃
     相对重锚基线为正但绝对电平仍在空盘——不设此门会开事件并虚假
     锁存（pull_test S2 实测 4.50g 假读数挂死）。r_empty 仅 prep 更
     新，跨 prep 零漂 ±1g << 2g 门限；未 prep 机器 r_empty 为台架
     占位不受此门保护，属预期（prep 是投用前置步骤）。*/
  double est_abs = p->k_g_per_r * (r - p->r_empty);
  if (est < p->min_lock_mass_g || est_abs < p->min_lock_mass_g)
  {
    /* 瞬态下冲（提瓶刮蹭/放瓶撞击回弹，亚秒即回）不重锚：显示短暂
       负数后自愈，无 "---" 无基线污染窗口；真零点跳变持续停在新
       电平，满 reanchor_min_n 后照常重锚（2026-10-31 气瓶操作实测）*/
    if (e->down_n < p->reanchor_min_n)
      return;
    /* 阶跃落到空盘（或受拉重锚后回释）：重置基线 */
    e->idle_n = 1;
    e->idle_head = 0;
    e->idle_t[0] = t;
    e->idle_v[0] = r;
    e->above_n = 0;
    e->down_n = 0;
    return;
  }
  e->state = 1;
  e->step_t = t;
  e->above_n = 0;
  e->down_n = 0;
  e->near_n = 0;
  e->locked = 0;
  e->restep_n = 0;
  e->r_fast_n = 0;
  e->win_n = 0;
}

static void try_lock(AlgoEstimator *e, const AlgoParams *p, double t_now)
{
  double t0 = e->step_t + p->settle_s;
  if (t_now - p->conv_win_s > t0) t0 = t_now - p->conv_win_s;

  /* 收集窗口样本（nw ≤ conv_win_s×10Hz，静态缓冲防御上限） */
  int nw = 0;
  int n_out_tail = 0, n_span = 0;
  for (int i = 0; i < e->win_n; i++)
  {
    if (e->win_t[i] >= t0 && nw < EST_TRIM_WORK_MAX)
      s_lock_win[nw++] = e->win_r[i];
    if (e->win_t[i] >= e->step_t + 1.0)
    {
      n_span++;
      if (e->win_o[i]) n_out_tail++;
    }
  }
  if (nw < 5) return;

  double r_mean = AlgoTrimmedMean(s_lock_win, nw, p->trim_frac);
  double diff = r_mean - e->baseline;

  /* std / sqrt(n) / max(|diff|, 0.004) */
  double s2 = 0;
  for (int i = 0; i < nw; i++) s2 += (s_lock_win[i] - r_mean) * (s_lock_win[i] - r_mean);
  double rs = sqrt(s2 / nw) / sqrt((double)nw) / fmax(fabs(diff), 0.004);
  if (rs >= p->conv_rel_std) return;

  double mass = p->k_g_per_r * (r_mean - e->baseline);
  if (mass < p->min_lock_mass_g) return;

  if (e->n_readings < EST_READINGS_MAX)
  {
    AlgoReading *rd = &e->readings[e->n_readings++];
    rd->t_step = e->step_t;
    rd->t_lock = t_now;
    rd->mass_g = mass;
    rd->rel_std = rs;
    rd->conv_s = t_now - e->step_t;
    rd->outliers = n_out_tail;
    rd->n_win = (n_span > 0) ? n_span : 1;
  }
  e->locked = 1;
}

static void refine(AlgoEstimator *e, const AlgoParams *p, double t_now)
{
  if (e->n_readings == 0) return;
  /* 滑动精化窗：起点 = max(放置稳定点, 现在−hold_win_s)。早期与
     增长窗一致（全部样本平均零均值涌浪最优）；超过窗长后持续滑入
     新样本、滑出旧样本——晃动瞬态污染最终被整段移出窗口，读数
     收敛回真值（冻结窗会永久卡在偏值）。移除时刻即定稿。*/
  double t0 = e->step_t + p->settle_s;
  if (t_now - p->hold_win_s > t0)
    t0 = t_now - p->hold_win_s;
  int nw = 0;
  for (int i = 0; i < e->win_n; i++)
  {
    if (e->win_t[i] >= t0 && e->win_t[i] <= t_now && nw < EST_TRIM_WORK_MAX)
      s_lock_win[nw++] = e->win_r[i];
  }
  if (nw < 5) return;
  double r_mean = AlgoTrimmedMean(s_lock_win, nw, p->trim_frac);
  AlgoReading *rd = &e->readings[e->n_readings - 1];
  rd->mass_g = p->k_g_per_r * (r_mean - e->baseline);
  double s2 = 0;
  for (int i = 0; i < nw; i++) s2 += (s_lock_win[i] - r_mean) * (s_lock_win[i] - r_mean);
  rd->rel_std = sqrt(s2 / nw) / sqrt((double)nw)
                / fmax(fabs(r_mean - e->baseline), 0.004);
  rd->n_win = nw;
}

static void classify(AlgoEstimator *e, const AlgoParams *p, AlgoReading *rd)
{
  /* confidence.py classify() 语义 */
  int hard = 0, soft = 0;

  if (!e->last_calm && e->last_rho > 0)
  {
    if (e->last_rho < p->conf_rho_soft) hard = 1;
    else if (e->last_rho < p->conf_rho) soft = 1;
  }
  if (rd->rel_std >= p->conf_rel_std_soft) hard = 1;
  else if (rd->rel_std >= p->conf_rel_std) soft = 1;
  if (e->gap_frac > 0.10) hard = 1;

  /* 锁存时打分（pipeline 是事后打，此处就地打） */
  rd->level = hard ? 2 : (soft ? 1 : 0);
}

static void feed_weighing(AlgoEstimator *e, const AlgoParams *p,
                          double t, double r, uint8_t is_outlier)
{
  if (e->win_n < EST_WIN_MAX)
  {
    e->win_t[e->win_n] = t;
    e->win_r[e->win_n] = r;
    e->win_o[e->win_n] = is_outlier;
    e->win_n++;
  }
  else
  {
    /* 事件超窗：淘汰最旧（原实现丢最新 → >153s 后 refine 的 30s 窗
       无新样本，读数冻结、不再跟随物理变化——垫书撤除实测 3 分钟
       纹丝不动，只能取空重放开新事件解锁。淘汰最旧与 Python 无界
       append 语义对齐：精化窗永远新鲜，变化后 ~30s 收敛回真值。
       memmove ~26KB@10Hz，F407 可忽略。n_span/n_out_tail 统计改为
       近 153s 口径（仅读数元数据，不影响质量值）。*/
    memmove(&e->win_t[0], &e->win_t[1],
            (EST_WIN_MAX - 1) * sizeof(double));
    memmove(&e->win_r[0], &e->win_r[1],
            (EST_WIN_MAX - 1) * sizeof(double));
    memmove(&e->win_o[0], &e->win_o[1],
            (EST_WIN_MAX - 1) * sizeof(uint8_t));
    e->win_t[EST_WIN_MAX - 1] = t;
    e->win_r[EST_WIN_MAX - 1] = r;
    e->win_o[EST_WIN_MAX - 1] = is_outlier;
  }

  if (fabs(r - e->baseline) < p->rearm_n_sigma * e->baseline_sigma)
    e->near_n++;
  else
    e->near_n = 0;

  if (e->near_n >= p->rearm_min_n)
  {
    e->state = 0;
    e->above_n = 0;
    /* 给最后一条读数打可信度 */
    if (e->n_readings > 0)
      classify(e, p, &e->readings[e->n_readings - 1]);
    return;
  }

  /* 快均值环（≈1s）：交流振荡在均值上收敛，真实偏移完整保留 */
  if (e->r_fast_n < 10)
  {
    e->r_fast[e->r_fast_n] = r;
    e->r_fast_n++;
  }
  else
  {
    memmove(&e->r_fast[0], &e->r_fast[1], 9 * sizeof(double));
    e->r_fast[9] = r;
  }

  /* 盘中增减检测：|快均值 − 读数均值|×k 超自适应门限（静水 2g /
     涌浪 5g——瞬时值比门限会被 ±8g 振荡误触发，快均值不会）且
     连续 5 点 → 当前读数定稿、直接开新事件（保留旧空盘基线）*/
  if (e->locked && e->n_readings > 0 && p->k_g_per_r > 0.0
      && e->r_fast_n >= 10)
  {
    AlgoReading *rd = &e->readings[e->n_readings - 1];
    double mfast = 0.0;
    double rbar, dm, thr;
    int i2;
    for (i2 = 0; i2 < 10; i2++) mfast += e->r_fast[i2];
    mfast /= 10.0;
    rbar = rd->mass_g / p->k_g_per_r + e->baseline;
    dm = (mfast - rbar) * p->k_g_per_r;
    if (dm < 0.0) dm = -dm;
    thr = e->quiet ? p->restep_quiet_g : p->restep_mass_g;
    if (dm > thr) e->restep_n++;
    else e->restep_n = 0;
    if (e->restep_n >= 5)
    {
      /* 直接开启新称量事件：保留旧空盘基线（若复位 idle_hist 会让
         基线被加载电平污染，阶跃检测失效——首版实测如此）*/
      classify(e, p, rd);
      e->state = 1;              /* WEIGHING */
      e->locked = 0;
      e->step_t = t;
      e->above_n = 0;
      e->near_n = 0;
      e->restep_n = 0;
      e->r_fast_n = 0;
      e->win_n = 0;
      return;
    }
  }

  /* 防卡死：解锁停留超时且盘面接近空 → 强制回 IDLE 重建基线
     （覆盖基线被污染/漂移后 near_n 永不满足的死锁路径）*/
  if (!e->locked && t - e->step_t > p->settle_s + p->conv_min_s + 10.0
      && p->k_g_per_r > 0.0)
  {
    double de = (r - e->baseline) * p->k_g_per_r;
    if (de < 0.0) de = -de;
    if (de < 5.0)
    {
      e->state = 0;
      e->above_n = 0;
      e->near_n = 0;
      e->restep_n = 0;
      e->r_fast_n = 0;
      e->win_n = 0;
      e->idle_n = 1;
      e->idle_head = 0;
      e->idle_t[0] = t;
      e->idle_v[0] = r;
      return;
    }
  }

  if (!e->locked && t - e->step_t >= p->settle_s + p->conv_min_s)
    try_lock(e, p, t);
  else if (e->locked)
    refine(e, p, t);              /* 精化持续到移除（滑动窗，见 refine）*/
}

void AlgoEstFeed(AlgoEstimator *e, const AlgoParams *p,
                 double t, double r, uint8_t is_outlier)
{
  if (e->state == 0)
    feed_idle(e, p, t, r);
  else
    feed_weighing(e, p, t, r, is_outlier);
}
