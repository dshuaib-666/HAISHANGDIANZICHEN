#include "app/app_calib.h"
#include "main.h"               /* HAL Flash */
#include <string.h>
#include <stddef.h>

/* 主循环里的 IWDG 喂狗钩子（main.c 实现）：擦 Sector 11 最长约 4 s，
   IWDG 最坏 5.58 s，擦除前补喂一次把余量拉大。 */
extern void AppWdg_Feed(void);

/* ==== 标定流程实现：见 app_calib.h 头注 ==== */

/* —— CRC32（IEEE 802.3 反射式，逐位软件实现——记录仅 52B，无需硬件 CRC）—— */
static uint32_t crc32_calc(const uint8_t *p, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;
  int b;
  for (i = 0; i < len; i++)
  {
    crc ^= p[i];
    for (b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320UL & (0u - (crc & 1u)));
  }
  return crc ^ 0xFFFFFFFFUL;
}

/* crc 字段之前的有效载荷长度 */
#define REC_PAYLOAD_BYTES  offsetof(AppCalibRec, crc)

uint32_t g_calib_flash_rc = 0U;   /* 调试器 Watch 用（见头注）*/

/* —— Flash 读写（Sector 11）—— */

static int flash_save(const AppCalibRec *rec)
{
  FLASH_EraseInitTypeDef ei;
  uint32_t err = 0;
  uint32_t i;
  HAL_StatusTypeDef st;

  g_calib_flash_rc = 0U;
  if ((sizeof(AppCalibRec) % 8U) != 0U) return 0;

  AppWdg_Feed();                              /* 擦除前的看门狗补喂 */
  st = HAL_FLASH_Unlock();
  if (st != HAL_OK)
  {
    g_calib_flash_rc = 0x100U | 1U;            /* 步骤1：Unlock */
    return 0;
  }
  ei.TypeErase = FLASH_TYPEERASE_SECTORS;
  ei.Banks = FLASH_BANK_1;
  ei.Sector = FLASH_SECTOR_11;
  ei.NbSectors = 1U;
  ei.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* VDD 2.7~3.6V */
  st = HAL_FLASHEx_Erase(&ei, &err);
  if (st != HAL_OK)
  {
    g_calib_flash_rc = ((uint32_t)st << 8) | 2U;   /* 步骤2：Erase */
    (void)HAL_FLASH_Lock();
    return 0;
  }
  /* 逐字（x32）编程：FLASH_Program_DoubleWord 的 x64 并行要求两条
     32 位存储背靠背成对（PSIZE=x64），实测真机确定性触发 PGPERR
     （HAL_ERROR）；x32 单次存储无配对窗口，与擦除的并行度一致。*/
  for (i = 0; i < sizeof(AppCalibRec) / 4U; i++)
  {
    uint32_t w;
    const uint8_t *pb = (const uint8_t *)rec + i * 4U;
    w = (uint32_t)pb[0] | ((uint32_t)pb[1] << 8)
      | ((uint32_t)pb[2] << 16) | ((uint32_t)pb[3] << 24);
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                           APP_CALIB_FLASH_ADDR + i * 4U, w);
    if (st != HAL_OK)
    {
      g_calib_flash_rc = ((uint32_t)st << 8) | 3U; /* 步骤3：Program */
      (void)HAL_FLASH_Lock();
      return 0;
    }
  }
  (void)HAL_FLASH_Lock();
  return 1;
}

static int flash_load(AppCalibRec *rec)
{
  memcpy(rec, (const void *)APP_CALIB_FLASH_ADDR, sizeof(*rec));
  if (rec->magic != APP_CALIB_MAGIC) return 0;
  if (rec->crc != crc32_calc((const uint8_t *)rec, REC_PAYLOAD_BYTES)) return 0;
  return 1;
}

int AppCalibLoad(AppScale *app)
{
  AppCalibRec rec;
  if (!flash_load(&rec)) return 0;
  app->p.k_g_per_r = rec.k_g_per_r;
  app->p.r_empty = rec.r_empty;
  app->p.presence_center_codes = rec.presence_center_codes;
  return 1;
}

/* —— 提交（2026-10-16 起独立标定模式移除，唯一调用方是预备模式）：
   校验 Δr 与 k 门限 → 构造记录 → 内存生效 → 写 Flash。
   返回：1=已提交（*flash_ok=Flash 写结果）；0=Δr 异常/样本不足；
        -1=k 超范围（砝码与配置质量不符）。*/
int AppCalibCommit(AppScale *app, double mass_g, double r0, double r1,
                   double a_mean_loaded, double a_mean_empty,
                   uint8_t *flash_ok)
{
  double dr = r1 - r0;
  double dr_min = APP_CALIB_MIN_DR * (mass_g / 1000.0);
  double k_new;
  AppCalibRec rec;
  double amean;

  if (dr <= dr_min) return 0;
  k_new = mass_g / dr;
  if (k_new < APP_CALIB_K_LO || k_new > APP_CALIB_K_HI) return -1;

  amean = (a_mean_loaded > 0.0) ? a_mean_loaded : a_mean_empty;
  memset(&rec, 0, sizeof(rec));
  rec.magic = APP_CALIB_MAGIC;
  rec.seq = 1U;
  rec.k_g_per_r = k_new;
  rec.r_empty = r0;
  rec.presence_center_codes = amean;
  rec.zero_codes_a = app->p.zero_codes_a;
  rec.zero_codes_b = app->p.zero_codes_b;
  rec.crc = crc32_calc((const uint8_t *)&rec, REC_PAYLOAD_BYTES);

  /* k 通过门限即**内存生效**（读数立即正确，与 Flash 结果解耦——
     此前写失败被 8888 掩盖，占位 k 一直用到底，实测 +0.38% 恰为
     占位值/真值之比）*/
  app->p.k_g_per_r = rec.k_g_per_r;
  app->p.r_empty = rec.r_empty;
  app->p.presence_center_codes = rec.presence_center_codes;
  *flash_ok = flash_save(&rec);
  return 1;
}

