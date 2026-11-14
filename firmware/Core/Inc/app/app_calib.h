#ifndef __APP_CALIB_H
#define __APP_CALIB_H

/* ==== 标定持久层（M-C3 起）：两点比值标定 + Flash 持久化 ====
   2026-10-16 起独立标定模式（原 KEY2/PB15 单键协议）移除——预备模式
   已完全覆盖其职责（k=0 自举、Flash 持久、drift 诊断），本文件只保留
   持久层：开机加载 + 两点提交（prep 的 CALC 段调用）。
   标定数值语义：空盘基线 r0、加砝码 r1，均取**原始轮询码对的比值**
   10Hz × 10% 截尾均值（不经变化沿流——静态台架上内码长时间不变，
   变化沿稀疏，配对流会饿死，实测 110s 采不满）。
   k = 1000/(r1−r0)，r_empty = r0，在位中心 = 同步 A 码均值。
   z_A/z_B 是裸传感器物理偏置（M1 台架跨会话稳定），现场不标，
   随记录快照存储仅作诊断。

   Flash：Sector 11（0x080E0000，128KB，代码远未占用），单记录
   erase+write，CRC32 校验，掉电在擦写间 → CRC 失败 → 回退出厂默认
   （prep 可自举重建，见 app_prep.h）。 */

#include "app/app_scale.h"
#include <stdint.h>

#define APP_CALIB_FLASH_ADDR   0x080E0000UL   /* Sector 11 起始 */
#define APP_CALIB_MAGIC        0x314C4143UL   /* "CAL1" */
/* 步骤 2 已知砝码（克）。换用 500/200/100g 等已知砝码均可——k 精度
   主要受砝码自身不确定度限制（数值上 100g 仍有 ~0.02% → 0.2g@1kg），
   推荐 ≥500g 或直接用演示对象 1kg；改此处需重编译。*/
#define APP_CALIB_MASS_G       500.0
#define APP_CALIB_MIN_N        100            /* 每步骤最少比值对数（~10s）*/
#define APP_CALIB_MIN_DR       0.30           /* Δr 合理性下限（按 1kg 标定；
                                                  实际门限随砝码质量等比缩放）。
                                                  2026-10-31 现场验证：B 容量
                                                  调回 6kg 后内码密度不变（旧 k
                                                  下 500g 实测 499.7g），Δr(500g)
                                                  ≈0.34 维持，门限不动（曾按
                                                  ÷3 假设临时改 0.12，已回退）*/
#define APP_CALIB_K_LO         1250.0         /* k 合理范围：硬件 k=s_A·R/s_B
                                                  ≈1461，±12% 覆盖灵敏度/参考
                                                  结构容差——砝码与配置质量
                                                  不符（如 1kg 冒充 500g）会
                                                  落在 ~k/2，直接拒绝。
                                                  2026-10-31 现场验证 6kg 容量
                                                  参数不改密度（曾临时改
                                                  [3300,5600]，实测后回退）*/
#define APP_CALIB_K_HI         1650.0

typedef struct
{
  uint32_t magic;
  uint32_t seq;                 /* 每次写入递增（掉电恢复诊断）*/
  double   k_g_per_r;
  double   r_empty;
  double   presence_center_codes;
  double   zero_codes_a;        /* 工厂值快照 */
  double   zero_codes_b;
  uint32_t crc;                 /* 前面所有字段的 CRC32 */
  uint32_t _pad;                /* 对齐到 8 字节倍数（56B 总长）*/
} AppCalibRec;

/* Flash 写入失败码（调试器 Watch 读；0=成功/未尝试）：
   低 4 位 = 失败步骤（1=Unlock 2=Erase 3=Program），高 8 位 = HAL 状态 */
extern uint32_t g_calib_flash_rc;

/* 开机加载：Flash 记录有效（CRC+magic）→ 写入 app->p 并返回 1；
   无效 → 保留 app->p 现值（出厂默认）并返回 0。 */
int AppCalibLoad(AppScale *app);

/* 提交一次两点标定（预备模式 CALC 段调用）：
   校验 Δr/k 门限 → 构造记录 → 内存生效 → 写 Flash。
   返回：1=已提交（*flash_ok=Flash 写结果）；0=Δr 异常；
        -1=k 超范围（砝码与配置质量不符）。 */
int AppCalibCommit(AppScale *app, double mass_g, double r0, double r1,
                   double a_mean_loaded, double a_mean_empty,
                   uint8_t *flash_ok);

#endif
