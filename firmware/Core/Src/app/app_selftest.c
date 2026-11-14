#include "app/app_selftest.h"

/* ==== 现场预检自检模式实现：见 app_selftest.h 头注 ==== */

static void enter(AppSelftest *st, AppStState s, double t_now)
{
  st->state = s;
  st->t_state = t_now;
  /* 注意：不清 t_linger——段末 "N888" 提示依赖它跨过状态切换存活；
     无提示的转换须自行置 0（各转换均已显式设置）*/
}

static void set_result(AppSelftest *st, double t_now)
{
  st->result[0] = st->comms_ok;
  st->result[1] = (uint8_t)(st->empty_ok && st->weight_ok);
  st->result[2] = st->motion_ok;
  st->result[3] = 1U;                       /* 显示链：能看见即确认 */
  st->state = ST_RESULT;
  st->t_result = t_now;
}

void AppSelftestNotifyComms(AppSelftest *st, uint8_t both_fresh,
                            uint32_t n_ok, uint32_t n_err, uint32_t n_drop)
{
  if (st->state != ST_COMMS) return;
  if (st->comms_pending_base)
  {
    st->c_ok0 = n_ok;
    st->c_err0 = n_err;
    st->c_drop0 = n_drop;
    st->comms_pending_base = 0U;
  }
  if (both_fresh == 0U) st->comms_fresh_viol = 1U;
  {
    uint32_t d_ok = n_ok - st->c_ok0;
    uint32_t d_err = n_err - st->c_err0;
    uint32_t d_drop = n_drop - st->c_drop0;
    /* 错帧率 >0.5%（且至少 3 帧）或环形缓冲丢弃 → 记罪 */
    if ((d_err > 3U) && (d_err * 200U > d_ok)) st->comms_fresh_viol = 1U;
    if (d_drop > 0U) st->comms_fresh_viol = 1U;
  }
}

void AppSelftestNotifyAlign(AppSelftest *st, uint8_t ok, uint8_t calm,
                            double rho)
{
  if (st->state != ST_MOTION) return;
  if (ok && !calm)
  {
    if (rho > st->rho_best) st->rho_best = rho;
    if (rho >= APP_SELFTEST_RHO_MIN) st->rho_n++;
  }
}

void AppSelftestNotifyReading(AppSelftest *st, double mass_g)
{
  if (st->state != ST_WEIGHT) return;
  if (st->weight_seen == 0U)
  {
    st->weight_seen = 1U;
    st->reading_mass = mass_g;
  }
}

void AppSelftestNotifyLive(AppSelftest *st, int ready, double mass_g)
{
  double d;
  if (st->state != ST_EMPTY) return;
  if (ready)
  {
    st->empty_ready_seen = 1U;
    d = (mass_g < 0.0) ? -mass_g : mass_g;
    if (d > st->empty_maxdev) st->empty_maxdev = d;
  }
}

int AppSelftestPoll(AppSelftest *st, double t_now, int ev_key0_long)
{
  double el = t_now - st->t_state;

  if (ev_key0_long)
  {
    if (st->state == ST_IDLE)
    {
      /* 进入：清零各段状态 */
      st->comms_fresh_viol = 0U;
      st->comms_ok = 0U;
      st->empty_maxdev = 0.0;
      st->empty_ok = 0U;
      st->empty_ready_seen = 0U;
      st->weight_seen = 0U;
      st->weight_ok = 0U;
      st->rho_n = 0;
      st->rho_best = 0.0;
      st->motion_ok = 0U;
      st->c_ok0 = st->c_err0 = st->c_drop0 = 0U;   /* 首个 Notify 前占位 */
      st->t_linger = 0.0;
      enter(st, ST_COMMS, t_now);
      st->comms_pending_base = 1U;
    }
    else
    {
      st->t_linger = 0.0;
      enter(st, ST_IDLE, t_now);   /* 中止 */
    }
    return 0;
  }

  switch (st->state)
  {
    case ST_COMMS:
      if (el >= APP_SELFTEST_COMMS_S)
      {
        st->comms_ok = (uint8_t)(st->comms_fresh_viol == 0U);
        st->linger_code = 1000 * 1 + 888;
        st->t_linger = st->comms_ok ? t_now : 0.0;
        enter(st, ST_EMPTY, t_now);
      }
      break;

    case ST_EMPTY:
      if (el >= APP_SELFTEST_EMPTY_S)
      {
        st->empty_ok = (uint8_t)((st->empty_ready_seen != 0U)
                                 && (st->empty_maxdev <= 0.5));
        st->linger_code = 2888;
        st->t_linger = st->empty_ok ? t_now : 0.0;
        enter(st, ST_WEIGHT, t_now);
      }
      break;

    case ST_WEIGHT:
      if (st->weight_seen && el >= 1.0)
      {
        double dm = st->reading_mass - APP_SELFTEST_MASS_G;
        if (dm < 0.0) dm = -dm;
        st->weight_ok = (uint8_t)(dm <= 1.0);
        st->linger_code = 2888;
        st->t_linger = st->weight_ok ? t_now : 0.0;
        enter(st, ST_MOTION, t_now);
      }
      else if (el >= APP_SELFTEST_WEIGHT_S)
      {
        st->weight_ok = 0U;    /* 超时未锁存读数 */
        st->t_linger = 0.0;
        enter(st, ST_MOTION, t_now);
      }
      break;

    case ST_MOTION:
      if (st->rho_n >= APP_SELFTEST_RHO_N)
      {
        st->motion_ok = 1U;
        st->linger_code = 3888;
        st->t_linger = t_now;
        enter(st, ST_DISPLAY, t_now);
      }
      else if (el >= APP_SELFTEST_MOTION_S)
      {
        st->motion_ok = 0U;
        st->t_linger = 0.0;
        enter(st, ST_DISPLAY, t_now);
      }
      break;

    case ST_DISPLAY:
      if (el >= 2.0)
        set_result(st, t_now);
      break;

    case ST_RESULT:
      if (el >= 10.0)
        enter(st, ST_IDLE, t_now);
      break;

    default:
      break;
  }
  return 0;
}

int AppSelftestDisplay(AppSelftest *st, double t_now)
{
  int secs;
  double el = t_now - st->t_state;

  switch (st->state)
  {
    case ST_COMMS:
    case ST_EMPTY:
    case ST_WEIGHT:
    case ST_MOTION:
      if ((st->t_linger > 0.0) && (t_now - st->t_linger < 1.5))
        return st->linger_code;      /* 段过：N888 停留 1.5s */
      secs = (int)el;
      if (secs > 999) secs = 999;
      return (int)st->state * 1000 + secs;
    case ST_DISPLAY:
      return 8888;                   /* 显示链：看见即确认 */
    case ST_RESULT:
      return (int)st->result[0] * 1000 + (int)st->result[1] * 100
           + (int)st->result[2] * 10 + (int)st->result[3];
    default:
      (void)t_now;
      return -1;
  }
}
