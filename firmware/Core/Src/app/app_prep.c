#include "app/app_prep.h"
#include <string.h>
#include <math.h>

/* ==== 预备模式实现：见 app_prep.h 头注 ==== */

static void enter(AppPrep *pp, AppPrepState s, double t_now)
{
  pp->state = s;
  pp->t_state = t_now;
  /* 不清 t_linger：段末码（1888/8888/7777…）跨状态停留依赖它 */
}

/* 进入结果前统一清采集开关（所有终路径共用）*/
static void set_result(AppPrep *pp, AppScale *app, double t_now)
{
  app->calib_collect = 0U;
  pp->comms_ok = (uint8_t)(pp->comms_viol == 0U);
  pp->result[0] = pp->comms_ok;
  pp->result[1] = pp->empty_ok;
  pp->result[2] = pp->cal_ok;
  /* 无 d4（2026-10-16 移除）：海上 prep 必挑风平浪静时刻（d2 静态
     检查所迫），运动证据此刻结构性不可得，静水替代证据无信息量。
     运动补偿质量由称量路径自评（可信度闪烁），不属预备职责——
     见 app_prep.h 头注。*/
  enter(pp, PP_RESULT, t_now);
}

/* 放置/移除监视：尾窗 2s 比值均值 vs r0（比值域共模相消，运动免疫）。
   样本不足（刚 Reset 后 2s 内）返回 0=不判定，检测惰性化。 */
static int r_loaded(AppPrep *pp, AppScale *app)
{
  if (AppScaleCalibN(app) < APP_PREP_MON_N) return -1;
  return (AppScaleCalibTailMeanR(app, APP_PREP_MON_N) - pp->r0
          > APP_PREP_R_PLACE) ? 1 : 0;
}

void AppPrepNotifyComms(AppPrep *pp, uint8_t both_fresh,
                        uint32_t n_ok, uint32_t n_err, uint32_t n_drop)
{
  if (pp->state == PP_IDLE) return;
  if (pp->comms_pending_base)
  {
    pp->c_ok0 = n_ok;
    pp->c_err0 = n_err;
    pp->c_drop0 = n_drop;
    pp->comms_pending_base = 0U;
  }
  if (both_fresh == 0U) pp->comms_viol = 1U;
  {
    uint32_t d_ok = n_ok - pp->c_ok0;
    uint32_t d_err = n_err - pp->c_err0;
    uint32_t d_drop = n_drop - pp->c_drop0;
    /* 错帧率 >0.5%（且至少 3 帧）或环形缓冲丢弃 → 记罪（贯穿全程）*/
    if ((d_err > 3U) && (d_err * 200U > d_ok)) pp->comms_viol = 1U;
    if (d_drop > 0U) pp->comms_viol = 1U;
  }
}

void AppPrepNotifyLive(AppPrep *pp, int ready, double mass_g)
{
  double d;
  if (pp->state != PP_EMPTY) return;
  if (ready)
  {
    pp->empty_ready_seen = 1U;
    d = (mass_g < 0.0) ? -mass_g : mass_g;
    if (d > pp->empty_maxdev) pp->empty_maxdev = d;
    /* 样本环（滚动保留最近 APP_PREP_EMPTY_LIVE_N 个）——中值判定用 */
    if (pp->empty_live_n < APP_PREP_EMPTY_LIVE_N)
    {
      pp->empty_live[pp->empty_live_n] = mass_g;
      pp->empty_live_n++;
    }
    else
    {
      memmove(&pp->empty_live[0], &pp->empty_live[1],
              (APP_PREP_EMPTY_LIVE_N - 1) * sizeof(double));
      pp->empty_live[APP_PREP_EMPTY_LIVE_N - 1] = mass_g;
    }
  }
}

/* 空盘段实时质量中值（插入排序副本；主循环单线程独占） */
static double empty_live_median(AppPrep *pp)
{
  static double v[APP_PREP_EMPTY_LIVE_N];
  int i, j, n = pp->empty_live_n;
  double key;
  if (n <= 0) return 0.0;
  memcpy(v, pp->empty_live, (size_t)n * sizeof(double));
  for (i = 1; i < n; i++)
  {
    key = v[i]; j = i - 1;
    while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
    v[j + 1] = key;
  }
  return (n % 2) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

int AppPrepPoll(AppPrep *pp, AppScale *app, double t_now, int ev_key0_long)
{
  double el = t_now - pp->t_state;
  int committed = 0;

  if (ev_key0_long)
  {
    if (pp->state == PP_IDLE)
    {
      pp->comms_pending_base = 1U;
      pp->comms_viol = 0U;
      pp->comms_ok = 0U;
      pp->empty_maxdev = 0.0;
      pp->empty_live_n = 0;
      pp->empty_ready_seen = 0U;
      pp->empty_ok = 0U;
      pp->cal_ok = 0U;
      pp->flash_ok = 0U;
      pp->drift_g = 0.0;
      pp->pan_loaded = 0U;
      pp->t_linger = 0.0;
      pp->c_ok0 = pp->c_err0 = pp->c_drop0 = 0U;
      app->calib_collect = 0U;
      enter(pp, PP_COMMS, t_now);
    }
    else
    {
      app->calib_collect = 0U;
      pp->t_linger = 0.0;
      enter(pp, PP_IDLE, t_now);      /* 中止 */
    }
    return 0;
  }

  switch (pp->state)
  {
    case PP_COMMS:
    {
      int64_t db0 = (int64_t)AppScaleCalibB(app);
      if (db0 > APP_PREP_EMPTY_MAX_B)
      {
        pp->pan_loaded = 1U;
        pp->t_state = t_now;          /* 未清盘：保持本段，显示 9000 */
        break;
      }
      pp->pan_loaded = 0U;
      if (el >= APP_PREP_COMMS_S)
      {
        AppScaleCalibReset(app);
        app->calib_collect = 1U;      /* 空盘 r0 采集开始 */
        pp->linger_code = 1888;
        pp->t_linger = t_now;
        enter(pp, PP_EMPTY, t_now);
      }
      break;
    }

    case PP_EMPTY:
      /* 到点后等样本凑齐（10Hz×10s 与 MIN_N=100 恰好贴边，相位差 1 拍
         会差 1 个样本）；12s 硬保险：仍不足则按样本不足失败 */
      if (((el >= APP_PREP_EMPTY_S)
           && (AppScaleCalibN(app) >= APP_CALIB_MIN_N))
          || (el >= APP_PREP_EMPTY_SAFETY_S))
      {
        /* k=0（新机/Flash 回退）时静态门不可计算——|实时质量|本身需要
           旧 k。跳过静态门只保样本数门，prep 由此可自举出第一个 k
           （空盘由流程卡保证，与 KEY2 独立标定同语义）；k≠0 行为：
           中值判据（运动免疫）——旧瞬时 maxdev 判据在海上微晃下
           ±0.2~0.9g 的交流振荡即误杀（实测结果恒 100），而"盘上有物"
           是直流偏移，中值消振荡照样抓。maxdev 保留为诊断字段。*/
        uint8_t static_ok = (uint8_t)((app->p.k_g_per_r == 0.0)
                                      || ((pp->empty_ready_seen != 0U)
                                          && (fabs(empty_live_median(pp))
                                              <= APP_PREP_EMPTY_DEV_G)));
        pp->empty_ok = (uint8_t)(static_ok
                                 && (AppScaleCalibN(app)
                                     >= APP_CALIB_MIN_N));
        if (pp->empty_ok == 0U)
        {
          set_result(pp, app, t_now); /* 静态失败：无标定意义，直接结果 */
          break;
        }
        pp->r0 = AppScaleCalibMeanR(app);
        pp->a_mean0 = AppScaleCalibMeanA(app);
        pp->linger_code = 2888;
        pp->t_linger = t_now;
        enter(pp, PP_PROMPT, t_now);  /* calib_collect 保持 1（监视）*/
      }
      break;

    case PP_PROMPT:
      if (r_loaded(pp, app) == 1)
      {
        enter(pp, PP_SETTLE, t_now);
      }
      else if (el >= APP_PREP_PROMPT_S)
      {
        pp->cal_ok = 0U;              /* 超时未放砝码 */
        set_result(pp, app, t_now);
      }
      break;

    case PP_SETTLE:
      if (r_loaded(pp, app) == 0)
      {                               /* 稳定期撤走：清样本回等待
                                       （防尾窗残留导致检测抖动）*/
        AppScaleCalibReset(app);
        enter(pp, PP_PROMPT, t_now);
      }
      else if (el >= APP_PREP_SETTLE_S)
      {
        AppScaleCalibReset(app);
        app->calib_collect = 1U;
        enter(pp, PP_COLLECT, t_now);
      }
      break;

    case PP_COLLECT:
      if (r_loaded(pp, app) == 0)
      {                               /* 移除/更换：比值回空盘 → 回等待
                                       （collect 保持 1 供监视，样本
                                       已 Reset，放置检测 2s 惰性）*/
        AppScaleCalibReset(app);
        enter(pp, PP_PROMPT, t_now);
      }
      else if (((el >= APP_PREP_COLLECT_S)
                && (AppScaleCalibN(app) >= APP_CALIB_MIN_N))
               || (el >= APP_PREP_COLLECT_SAFETY_S))
      {
        enter(pp, PP_CALC, t_now);
      }
      break;

    case PP_CALC:
    {
      /* 瞬时段：算 k → 门限 → 生效 → Flash → 补等/结果（一次走完）*/
      app->calib_collect = 0U;
      if (AppScaleCalibN(app) < APP_CALIB_MIN_N)
      {
        pp->cal_ok = 0U;              /* 样本不足 */
        pp->linger_code = 9002;
        pp->t_linger = t_now;
        set_result(pp, app, t_now);
        break;
      }
      pp->r1 = AppScaleCalibMeanR(app);
      {
        double dr = pp->r1 - pp->r0;
        uint8_t fok = 0U;
        int rc;
        /* 旧 k 回差快照：重标前的读数偏移（操作者关心的"漂了多少"）；
           k=0 自举时恒 -1000（无旧 k 可言，诊断行上可识别）*/
        pp->drift_g = app->p.k_g_per_r * dr - APP_CALIB_MASS_G;
        rc = AppCalibCommit(app, APP_CALIB_MASS_G, pp->r0, pp->r1,
                            AppScaleCalibMeanA(app), pp->a_mean0, &fok);
        if (rc == 1)
        {
          pp->cal_ok = 1U;
          pp->flash_ok = fok;
          pp->linger_code = (fok != 0U) ? 8888 : 6666;
          pp->t_linger = t_now;
          committed = 1;
          set_result(pp, app, t_now);
        }
        else if (rc == -1)
        {
          pp->cal_ok = 0U;            /* k 超范围：砝码与配置不符 */
          pp->linger_code = 7777;
          pp->t_linger = t_now;
          set_result(pp, app, t_now);
        }
        else
        {
          pp->cal_ok = 0U;            /* Δr 异常 */
          pp->linger_code = 9002;
          pp->t_linger = t_now;
          set_result(pp, app, t_now);
        }
      }
      break;
    }

    case PP_RESULT:
      if (el >= APP_PREP_RESULT_S)
      {
        enter(pp, PP_IDLE, t_now);
      }
      break;

    default:
      break;
  }
  return committed;
}

int AppPrepDisplay(AppPrep *pp, double t_now)
{
  double el = t_now - pp->t_state;
  int secs;

  if ((pp->state != PP_IDLE) && (pp->t_linger > 0.0)
      && (t_now - pp->t_linger < 1.5))
  {
    return pp->linger_code;           /* 段末码停留 1.5s */
  }

  switch (pp->state)
  {
    case PP_COMMS:
      if (pp->pan_loaded != 0U) return 9000;  /* 未清盘 */
      secs = (int)el;
      if (secs > 999) secs = 999;
      return 1000 + secs;
    case PP_EMPTY:
      secs = (int)el;
      if (secs > 999) secs = 999;
      return 2000 + secs;
    case PP_PROMPT:
    case PP_SETTLE:
    case PP_COLLECT:
      secs = (int)el;
      if (secs > 999) secs = 999;
      return 3000 + secs;             /* 3xxx：放砝码提示走字 */
    case PP_RESULT:
      return (int)pp->result[0] * 100 + (int)pp->result[1] * 10
           + (int)pp->result[2];
    default:
      (void)t_now;
      return -1;
  }
}
