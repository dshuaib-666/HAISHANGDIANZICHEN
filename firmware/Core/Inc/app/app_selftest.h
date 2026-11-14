#ifndef __APP_SELFTEST_H__
#define __APP_SELFTEST_H__

/* ==== 现场预检自检模式（M-C3）：演示前 5 分钟、无 PC、数码管判读 ====
   流程（C移植计划 §5，全程单键 KEY0=PB13 长按 进入/中止）：
     1 通信健康 60s：双通道数据新鲜（<500ms）、crc+帧错 <0.5%、环零丢弃
     2 静态核查：
        2a 空盘噪声 15s——|实时质量| 全程 ≤ ±0.5g（操作员确保 B 盘空）
        2b 已知砝码 ≤120s——放上 APP_SELFTEST_MASS_G 砝码等读数锁存，
           |读数−标称| ≤ ±1g
     3 运动适应 ≤90s：操作员晃动秤体，对齐窗 ρ≥0.95 累计 ≥5 个
        （门槛有意取 0.95 而非计划的 0.985：手持实测 ρ≈0.96（2a），
          0.985 会令台架排练恒败；预检职责是抓链路失效而非卡边界，
          台上刚性平台 ρ 应 ≥0.99，自然富余通过——偏差记录于文档）
     4 显示链：显示 "8888" 2s（操作员看到即链路确认）
     结果："d1d2d3d4" 显示 10s（dN=1 过/0 败，对应通信/静态/运动/显示）
           随后回实时质量显示。全程 # SELFTEST 调试行（串口在则记）。
   显示优先级（main.c）：标定 > 自检 > 实时质量。与标定互斥。 */

#include <stdint.h>

#define APP_SELFTEST_MASS_G      500.0   /* 2b 已知核查砝码（与标定砝码一致）*/
#define APP_SELFTEST_COMMS_S     60.0
#define APP_SELFTEST_EMPTY_S     15.0
#define APP_SELFTEST_WEIGHT_S    120.0   /* 2b 最长等待（放上+锁存）*/
#define APP_SELFTEST_MOTION_S    90.0
#define APP_SELFTEST_RHO_MIN     0.95
#define APP_SELFTEST_RHO_N       5       /* 达标窗数 */

typedef enum
{
  ST_IDLE = 0,
  ST_COMMS,          /* 1 通信健康 */
  ST_EMPTY,          /* 2a 空盘噪声 */
  ST_WEIGHT,         /* 2b 已知砝码 */
  ST_MOTION,         /* 3 运动适应 */
  ST_DISPLAY,        /* 4 显示链 */
  ST_RESULT          /* 结果码 */
} AppStState;

typedef struct
{
  AppStState state;
  double t_state;              /* 进入当前状态时刻 */
  double t_clock;              /* 时钟（主循环喂 HAL tick 秒）*/
  double t_linger;             /* 段末 "N888" 停留起点（0=不停留）*/
  int    linger_code;

  /* 1 通信（main 喂绝对计数器，本模块记起点算增量）*/
  uint32_t c_ok0, c_err0, c_drop0;
  uint8_t  comms_pending_base; /* 进入 ST_COMMS 后首个 Notify 锁基线 */
  uint8_t  comms_fresh_viol;   /* 数据新鲜度/错帧率/丢环违例 */
  uint8_t  comms_ok;

  /* 2 静态 */
  double   empty_maxdev;       /* 2a 窗内 |实时质量| 最大值 */
  uint8_t  empty_ok;
  uint8_t  empty_ready_seen;   /* 基线就绪过（否则无数据判失败）*/
  double   reading_mass;       /* 2b 锁存读数 */
  uint8_t  weight_seen;
  uint8_t  weight_ok;

  /* 3 运动 */
  int      rho_n;
  double   rho_best;
  uint8_t  motion_ok;

  uint8_t  result[4];          /* 通信/静态/运动/显示 */
  double   t_result;
} AppSelftest;

/* 通知接口（main.c 喂；与驱动解耦，host 可测）*/
void AppSelftestNotifyComms(AppSelftest *st, uint8_t both_fresh,
                            uint32_t n_ok, uint32_t n_err, uint32_t n_drop);
void AppSelftestNotifyAlign(AppSelftest *st, uint8_t ok, uint8_t calm,
                            double rho);
void AppSelftestNotifyReading(AppSelftest *st, double mass_g);
/* 2a 需要 2b 之外的实时质量：每轮喂（ready=基线就绪）*/
void AppSelftestNotifyLive(AppSelftest *st, int ready, double mass_g);

/* 状态机驱动。ev_key0_long：进入（IDLE 时）/中止。返回 1=刚出结果。*/
int AppSelftestPoll(AppSelftest *st, double t_now, int ev_key0_long);

/* 显示：-1=非自检态；-2=闪烁熄灭半拍；≥0=显示码 */
int AppSelftestDisplay(AppSelftest *st, double t_now);

#endif
