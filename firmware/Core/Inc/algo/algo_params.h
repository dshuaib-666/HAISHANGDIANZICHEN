#ifndef __ALGO_PARAMS_H__
#define __ALGO_PARAMS_H__

/* ==== 冻结参数（算法设计 §10 实测重标版，2026-10）==== */

typedef struct
{
  /* 流 */
  double interp_max_span_s;
  /* 快环 */
  int    hampel_k;
  double hampel_h;
  double ref_span_codes;
  double presence_center_codes;
  double presence_lo;
  double presence_hi;
  double step_n_sigma;
  double min_baseline_sigma;   /* 基线 σ 量化下限：静态台架上比值取离散
                                   量化值，MAD 塌到 ~0 会卡死回臂判定 */
  int    step_min_n;
  int    reanchor_min_n;   /* 向下阶跃（疑似空盘地板搬家）重锚所需连续
                              低于基线样本数：提瓶刮蹭/放瓶撞击的下冲
                              亚秒即回，真零点跳变持续停在新电平——
                              时间特征是二者唯一可靠区别（2026-10-31
                              气瓶操作实测）。向上路径（放物开事件）
                              3 样本快响应不变 */
  double step_guard_g;
  double rearm_n_sigma;
  int    rearm_min_n;
  double rearm_floor_g;      /* 回臂质量下限（克）：取走重物后传感器残留
                                0.2~5g，而 3σ 判据（σ 下限 4e-5 → 3σ≈
                                0.18g）永远不满足，状态会卡在称量态
                                （零点停更、清零键被 9300 拒）。|Δr|×k
                                小于此值连续 rearm_min_n 点也回臂。
                                1g 对 6kg 量程=0.017%FS，且 <min_lock
                                的载荷本就不开事件，不会误吃真实载荷 */
  double settle_s;
  double conv_win_s;
  double conv_min_s;
  double conv_rel_std;
  double hold_win_s;
  double trim_frac;
  double min_lock_mass_g;
  double restep_mass_g;         /* 盘中增减检测门限（涌浪中）：|1s 快
                                   均值−读数均值|×k 超此值且连续 5 点 →
                                   定稿并开新事件。快均值把交流振荡收敛
                                   到真值附近，5g 已高于 ±8g 振荡在 1s
                                   均值上的残差 */
  double restep_quiet_g;        /* 同上（静水/台架）：无涌浪时残差仅
                                   量化级，2g 覆盖最小增量 */
  /* 零点修正 */
  double zero_codes_a;
  double zero_codes_b;
  /* 标定 */
  double k_g_per_r;
  double r_empty;
  /* 可信度 */
  double conf_rho;
  double conf_rho_soft;
  double conf_rel_std;
  double conf_rel_std_soft;
} AlgoParams;

/* 冻结默认值（与 Python params.py 同步）*/
static const AlgoParams algo_params_default = {
  .interp_max_span_s     = 0.250,
  .hampel_k              = 5,
  .hampel_h              = 3.0,
  .ref_span_codes        = 47616.0,
  .presence_center_codes = 90100.0,
  .presence_lo           = 0.75,
  .presence_hi           = 1.25,
  .step_n_sigma          = 5.0,
  .min_baseline_sigma    = 4e-5,   /* ≈2×(1+r)/(A码距) 量化步，运动场景
                                       σ>>此值（下限不起作用） */
  .step_min_n            = 3,
  .reanchor_min_n        = 15,    /* 1.5s @10Hz：瞬态下冲免疫 / 真跳变延迟重锚 */
  .step_guard_g          = 200.0,
  .rearm_n_sigma         = 3.0,
  .rearm_min_n           = 3,
  .rearm_floor_g         = 1.0,   /* 回臂质量下限：残留 <1g 视为已取走 */
  .settle_s              = 1.2,
  .conv_win_s            = 3.0,
  .conv_min_s            = 2.0,
  .conv_rel_std          = 5e-4,
  .hold_win_s            = 30.0,   /* 精化窗 30s：晃动瞬态污染被后续干净样本稀释（10s 时晃两下即冻结偏值）；涌浪下更长窗平均更准 */
  .trim_frac             = 0.10,
  .min_lock_mass_g       = 2.0,
  .restep_mass_g         = 5.0,
  .restep_quiet_g        = 2.0,
  .zero_codes_a          = 0.0,   /* 工厂标定写入 */
  .zero_codes_b          = 0.0,
  .k_g_per_r             = 0.0,   /* 现场标定写入 */
  .r_empty               = 0.0,
  .conf_rho              = 0.985,
  .conf_rho_soft         = 0.95,
  .conf_rel_std          = 4e-4,
  .conf_rel_std_soft     = 8e-4,
};

#endif
