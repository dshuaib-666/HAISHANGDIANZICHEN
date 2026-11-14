#ifndef __ALGO_DSP_H__
#define __ALGO_DSP_H__

#include <stdint.h>

/* ==== A 轨算法 DSP 基础层 —— 对拍 Python atrack/dsp.py（M-C1）====
   单一真源：本文件同时被 Keil 固件工程与 host 对拍 gcc 编译。
   C99 子集（块首声明、无 VLA），双编译器兼容（ARMCC v5.06 --c99 / gcc）。 */

/* RBJ 双二阶滤波器（与 dsp.py biquad 系数公式一致） */
typedef struct
{
  double b0, b1, b2, a1, a2;   /* 归一化系数（a0=1） */
  double z1, z2;              /* 输入历史 x[i-1], x[i-2] */
  double y1, y2;              /* 输出历史 y[i-1], y[i-2] */
} AlgoBiquad;

/* 计算带通 = 高通级联低通的系数（hp@lo + lp@hi）。启动瞬态由调用方丢弃头部。 */
void AlgoBpInit(AlgoBiquad *hp, AlgoBiquad *lp, double lo_hz, double hi_hz,
                double fs_hz);

/* 就地滤波一个样本（TDF-II，双精度状态——0.05Hz 高通在 20Hz 采样下
   极点贴单位圆，float32 累积误差不可接受，见 C移植计划 §2） */
double AlgoBiquadStep(AlgoBiquad *f, double x);

/* 带通级联一步：hp 先、lp 后 */
double AlgoBpStep(AlgoBiquad *hp, AlgoBiquad *lp, double x);

/* Hampel 滤波（半窗 k，阈值 h×MAD，MAD 下限 mad_floor）。
   返回过滤后样本；*outlier 置 1 表示被中值替换。
   滑动中值用简单插入排序窗口（k≤16 固定小窗，O(k²) 每点——20Hz 流无压力）。 */
double AlgoHampel(const double *x, int n, int i, int k, double h,
                  double mad_floor, uint8_t *outlier);

/* 由相邻差分中值鲁棒估计白噪声 σ（对阶跃/慢波动不敏感） */
double AlgoNoiseFromDiff(const double *x, int n);

/* 三点抛物线插值：峰值相对整数索引 i 的偏移（单位：样本） */
double AlgoParabolic(const double *y, int n, int i);

#endif
