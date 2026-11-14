/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt_time.h"
#include "qsh_drv.h"
#include "dbg_uart.h"
#include "app/app_zero.h"
#include "keys.h"
#include "digit_tube.h"
#include "beeper.h"
#include "app/app_scale.h"
#include "app/app_calib.h"
#include "app/app_prep.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static AppScale g_scale;
static AlgoParams app_params_bench;
static AppZero g_zero;
static AppPrep g_prep;
static uint8_t g_rdLevel, g_rdActive, g_prevSt;
static uint8_t g_prevZero;
static double g_rdMass;
static uint32_t g_rdHoldUntil;
static double g_sampleT;
static uint32_t g_lastDispTick;
static uint8_t g_prevRecArm = 0U;  /* g_recArm 沿检测（算法暂停/恢复）*/
static uint8_t g_warmupBeep = 1U;  /* 上电建基线阶段："---" 闪烁同步短鸣（首次 ready 后关）*/
static uint8_t g_blinkOn = 0U;     /* "---" 亮半拍沿检测（每周期只响一次，防 200ms 刷屏连响）*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* u64 → 十进制字符串（不依赖 printf 的 %llu 实现） */
static void u64str(uint64_t v, char *out)
{
  char tmp[21];
  uint8_t n = 0U;

  if (v == 0U)
  {
    out[0] = '0';
    out[1] = '\0';
    return;
  }
  while ((v > 0U) && (n < sizeof(tmp)))
  {
    tmp[n++] = (char)('0' + (v % 10U));
    v /= 10U;
  }
  while (n > 0U)
  {
    *out++ = tmp[--n];
  }
  *out = '\0';
}

static void EmitEvent(const char *name)
{
  char tb[21];
  u64str(DwtTime_NowUs(), tb);
  Dbg_Line("# EVENT %s %s", name, tb);
}

static void HeartbeatOnce(void)
{
  QshStat_t st[QSH_NCH];

  QshDrv_GetStats(st);
  Dbg_Line("# STAT A ok=%lu crc=%lu fe=%lu tx=%lu ack=%lu fail=%lu z=%lu drop=%lu | "
           "B ok=%lu crc=%lu fe=%lu tx=%lu ack=%lu fail=%lu z=%lu drop=%lu ready=%u",
           st[0].readOk, st[0].crcErr, st[0].frameErr, st[0].txBusy,
           st[0].cfgAck, st[0].cfgFail, st[0].zeroAck, st[0].ringDrop,
           st[1].readOk, st[1].crcErr, st[1].frameErr, st[1].txBusy,
           st[1].cfgAck, st[1].cfgFail, st[1].zeroAck, st[1].ringDrop,
           (unsigned int)QshDrv_Ready());
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
	

	
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  DwtTime_Init();
  Dbg_Init();
  Keys_Init();
  QshDrv_Init();            /* 收帧启动 + 模块配置状态机，就绪后 TIM2 自动轮询 */
  HAL_TIM_Base_Start_IT(&htim2);
  Beeper_Init();            /* PWM 启动后立即静音（上电不响）*/
  DigitTube_Init();         /* 上电显示 666，随后显示 B 通道内码 */
  app_params_bench = algo_params_default;
  /* 台架占位值：仅 Flash 无效时使用，首次 prep 即被覆盖。2026-10-31 现场
     验证 6kg 容量参数不改内码密度（旧 k 下 500g 读 499.7g），占位 k 仍有效。
     z_A/z_B 仅诊断快照，不参与算法。*/
  app_params_bench.k_g_per_r = 1467.9;        /* TODO(M-C3): Flash 标定 */
  app_params_bench.r_empty = 0.35222;         /* 516/(465+1000) */
  app_params_bench.zero_codes_a = 20220.0;
  app_params_bench.zero_codes_b = 14401.0;
  app_params_bench.presence_center_codes = 90100.0;
  AppScaleInit(&g_scale, &app_params_bench);
  memset(&g_zero, 0, sizeof(g_zero));
  memset(&g_prep, 0, sizeof(g_prep));
  if (AppCalibLoad(&g_scale) != 0)
  {
    Dbg_Line("# CALIB loaded from flash: k=%.1f r_empty=%.5f center=%.0f",
             g_scale.p.k_g_per_r, g_scale.p.r_empty,
             g_scale.p.presence_center_codes);
  }
  else
  {
    Dbg_Line("# CALIB flash empty/invalid -> defaults (need calibration)");
  }
  Dbg_Line("# APP_SCALE init (algo incremental, M-C3)");

  Dbg_Line("# BOOT dual_scale fw_v2 (neima 0x17, t_us CSV)");
  Dbg_Line("# FORMAT t_us,sensor,code,seq,dup  sensor:0=A_ref(USART3) 1=B_meas(USART2)");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  {
    QshSample_t smp;
    KeyEv_t ev;
    uint32_t lastBeat = HAL_GetTick();
    uint32_t lastBTick = HAL_GetTick();
    uint8_t  tubeBlack = 0U;

    while (1)
    {
      Keys_Process();
      QshDrv_Process();
      DigitTube_Process();
      Beeper_Process();

      /* 样本 → CSV + 增量算法引擎。录制模式（g_recAlgoHold=1）跳过：
         样本只入环积累供调试器导出——若弹出读到的不是传感器数据而是
         g_scale 内存垃圾，算法会收到无效时间戳导致主循环卡死。*/
      if (g_recAlgoHold == 0U)
      {
      while (QshDrv_PopSample(&smp) != 0)
      {
        char tb[21];
        double tsmp = (double)smp.tUs / 1e6;
        u64str(smp.tUs, tb);
        Dbg_Line("%s,%u,%ld,%lu,%u", tb, (unsigned int)smp.ch, smp.code,
                 (unsigned long)smp.seq, (unsigned int)smp.dup);
        g_sampleT = tsmp;
        AppScaleFeed(&g_scale, tsmp,
                     (smp.ch == QSH_CH_A_REF) ? 0 : 1, smp.code);
        AppScaleProcess(&g_scale, tsmp);
        if (smp.ch == QSH_CH_B_MEAS)
        {
          lastBTick = HAL_GetTick();
          tubeBlack = 0U;
        }
        for (;;)
        {
          AppEvent aev = AppScalePopEvent(&g_scale);
          if (aev.type == APP_EV_NONE) break;
          if (aev.type == APP_EV_ALIGN)
          {
            Dbg_Line("# ALIGN t=%u.%03u ok=%u calm=%u tau=%d(0.1ms) n=%d",
                     (unsigned)aev.align.t,
                     (unsigned)((aev.align.t - (double)(unsigned)aev.align.t) * 1000.0),
                     (unsigned)aev.align.ok, (unsigned)aev.align.calm,
                     (int)(aev.align.tau * 10000.0), aev.align.n_pts);
          }
          else
          {
            /* 读数事件：锁定后数码管显示"锁定读数"（收敛值）而非
               瞬时值——运动中瞬时值必然波动（差分倾斜/失配泄漏是
               物理残余，由收敛窗平均掉），观众要看的稳定准确值
               就是锁定读数（S6/2a 验证的量）*/
            if (aev.type == APP_EV_READING)
            {
              g_rdActive = 1U;
              g_rdMass = aev.mass_g;
              g_rdLevel = 0U;             /* 新称量开始：常亮 */
            }
            else if (aev.type == APP_EV_REFINE)
            {
              g_rdMass = aev.mass_g;      /* 精化刷新锁定值 */
            }
            else if (aev.type == APP_EV_REARM)
            {
              /* 移除：直接回实时显示（≈0.0）——冻结/闪烁会造成"先回
                 0.0 又弹回旧值闪 5s"的反直觉序列；读数早在取下前已
                 持续展示，无需再冻结抄录 */
              g_rdActive = 0U;
              g_rdLevel = 0U;
              g_rdHoldUntil = 0U;
            }
            else if (aev.type == APP_EV_RESTEP)
            {
              /* 盘中增减定稿：不冻结不闪，切实时值快速收敛到新锁定 */
              g_rdActive = 0U;
              g_rdLevel = 0U;
              g_rdHoldUntil = 0U;
            }
            Dbg_Line("# READ ev=%u t=%u.%03u mass=%d mg level=%u",
                     (unsigned)aev.type, (unsigned)aev.t,
                     (unsigned)((aev.t - (double)(unsigned)aev.t) * 1000.0),
                     (int)(aev.mass_g * 1000.0), (unsigned)aev.level);
          }
        }
      }
      }

/* B 通道断流 >3s：数码管黑屏（通信故障可见） */
      if ((tubeBlack == 0U) && ((HAL_GetTick() - lastBTick) > 3000U))
      {
        DigitTube_ShowBlackScreen();
        tubeBlack = 1U;
      }

      int k0l = 0;   /* KEY0 长按事件（预备模式用，跨下方两个块）*/


      /* 录制模式：数码管显示 B 通道原始码/10（M1 行为）*/
      if (g_recAlgoHold != 0U)
      {
        int32_t cb2;
        uint32_t agB2;
        if (QshDrv_Last(QSH_CH_B_MEAS, &cb2, &agB2) != 0)
        {
          int32_t shown = cb2 / 10;
          if (shown > 9999)       { shown = 9999; }
          else if (shown < -9999) { shown = -9999; }
          DigitTube_ShowInteger(shown);
        }
      }

      /* —— g_recArm 运行时切换（M-C4 录制模式，调试器驱动）——
         写 1 → 暂停算法，QshDrv_RecBegin 让 g_scale 内存作大环
         写 0 或录满 → QshDrv_RecEnd 恢复小环，AppScaleInit 重启算法 */
      /* 仅 g_recArm 下降沿退出录制（录满不自动退出——数据须保留
         到调试器导出完成，AppScaleInit 的 memset 会清掉录制缓冲）*/
      if (g_recArm != g_prevRecArm)
      {
        if (g_recArm != 0U && g_prevRecArm == 0U)
        {
          /* 上升沿：进入录制 */
          QshDrv_RecBegin(&g_scale, sizeof(g_scale));
          Dbg_Line("# REC begin buf=0x%08X cap=%u", g_recBufAddr, s_ringCap);
        }
        else if (g_recArm == 0U && g_prevRecArm != 0U)
        {
          /* 下降沿：退出录制（数据已由调试器导出），重启算法 */
          QshDrv_RecEnd();
          AppScaleInit(&g_scale, &app_params_bench);
          (void)AppCalibLoad(&g_scale);
          Dbg_Line("# REC end count=%lu, algo restarted", g_recCount);
        }
        g_prevRecArm = g_recArm;
      }


/* —— 按键分发（2026-10-16 重构：数据模式/独立标定/打点已删）——
         KEY0/PB13 长按 = 预备模式进入/中止（清零运行中拒绝）
         KEY1/PB14 长按 = 清零进入/中止（预备/录制中拒绝）
         KEY2/PB15 闲置 */
      {
        int k1l = 0;
        int zeroed;
        ev = Keys_Take(0U);
        if (ev == KEY_EV_LONG)
        {
          /* 组合键门槛：按住 KEY2（闲置键）不放 + 长按 KEY0——prep
             为出厂/维修级操作（重写 Flash k），双键确认防误触 */
          if ((g_zero.state == ZO_IDLE) && (Keys_IsPressed(2U) != 0))
          {
            k0l = 1;
          }
        }
        ev = Keys_Take(1U);
        if (ev == KEY_EV_LONG)
        {
          if ((g_prep.state == PP_IDLE) && (g_recArm == 0U)
              && (g_recAlgoHold == 0U))
          {
            k1l = 1;
          }
        }
        zeroed = AppZeroPoll(&g_zero, &g_scale,
                             (double)HAL_GetTick() / 1000.0, k1l);
        /* 开始沿：长按成功进入采集（ZO_IDLE→ZO_COLLECT/SPAN）——
           1 短提示"操作已开始、可松手"。必须与完成音（零点 2 短 /
           跨度 1 长）区分开，否则与零点完成同为 2 短会混淆（实测：
           误把开始音当完成，见 "---" 仍在闪）。 */
        if ((g_prevZero == (uint8_t)ZO_IDLE)
            && ((g_zero.state == ZO_COLLECT) || (g_zero.state == ZO_SPAN)))
        {
          Beeper_Beep(1U, 80U, 0U);
        }
        if (zeroed == 1)
        {
          EmitEvent("ZERO_OK");
          Beeper_Ok();                            /* 2短：零点重锚成功 */
          Dbg_Line("# ZERO ok sigma=%.3fg offset=%+.3fg n=%d",
                   g_zero.sigma_g, g_zero.offset_g, g_zero.n_used);
        }
        else if (zeroed == 2)
        {
          EmitEvent("ZERO_SPAN_OK");
          Beeper_Beep(1U, 200U, 100U);            /* 1长：跨度校正成功
                                                     （区别于零点的 2 短）*/
          Dbg_Line("# ZERO span ok k_old=%.2f k_new=%.2f "
                   "sigma=%.3fg off=%+.3fg n=%d",
                   g_zero.k_old, g_scale.p.k_g_per_r,
                   g_zero.sigma_g, g_zero.offset_g, g_zero.n_used);
        }
        if ((g_zero.state == ZO_RESULT)
            && (g_prevZero != (uint8_t)ZO_RESULT))
        {
          EmitEvent("ZERO_FAIL");
          if (g_zero.linger_code == 7777)
          {
            Beeper_Error();                       /* 1长：k 超范围（硬错误）*/
          }
          else
          {
            Beeper_Fail();                        /* 3短：9100/9200/9300 */
          }
          Dbg_Line("# ZERO fail code=%d sigma=%.3fg offset=%+.3fg n=%d",
                   g_zero.linger_code, g_zero.sigma_g, g_zero.offset_g,
                   g_zero.n_used);
        }
        g_prevZero = (uint8_t)g_zero.state;
      }

      /* —— 预备模式驱动（每轮）：通信统计喂入 + 状态机（HAL tick 时钟，
         通信死也走表）+ 标定提交/结果沿事件 —— */
      {
        double tNowSec = (double)HAL_GetTick() / 1000.0;
        QshStat_t stq[QSH_NCH];
        int32_t ccA, ccB;
        uint32_t agA, agB;
        uint8_t fresh;
        int committed;
        QshDrv_GetStats(stq);
        fresh = (uint8_t)(((QshDrv_Last(QSH_CH_A_REF, &ccA, &agA) != 0)
                           && (QshDrv_Last(QSH_CH_B_MEAS, &ccB, &agB) != 0)
                           && (agA < 500U) && (agB < 500U)) ? 1U : 0U);
        AppPrepNotifyComms(&g_prep, fresh,
                           stq[0].readOk + stq[1].readOk,
                           stq[0].crcErr + stq[0].frameErr
                           + stq[1].crcErr + stq[1].frameErr,
                           stq[0].ringDrop + stq[1].ringDrop);
        {
          /* 空盘噪声采样必须独立于显示分支（预备期间显示步骤码，
             实时质量分支不执行——曾致空盘判定恒无数据失败）*/
          int rdy2 = 0;
          double m2 = AppScaleLiveMass(&g_scale, &rdy2);
          AppPrepNotifyLive(&g_prep, rdy2, m2);
        }
        committed = AppPrepPoll(&g_prep, &g_scale, tNowSec, k0l);
        if (k0l != 0)
        {
          EmitEvent((g_prep.state == PP_COMMS) ? "PREP_ENTER" : "PREP_ABORT");
        }
        /* 段推进提示：空盘段→砝码段（该放砝码了）*/
        if ((g_prep.state == PP_PROMPT)
            && (g_prevSt == (uint8_t)PP_EMPTY))
        {
          Beeper_Tick();
        }
        if (committed != 0)
        {
          EmitEvent(g_prep.flash_ok != 0U ? "PREP_CAL_SAVED"
                                          : "PREP_CAL_FLASH_FAIL");
          Beeper_Ok();                            /* 2短：标定提交成功 */
          Dbg_Line("# PREP cal %s: k=%.2f r0=%.6f r1=%.6f drift=%+.2f g "
                   "flash_rc=%lu",
                   (g_prep.flash_ok != 0U) ? "saved" : "FLASH-FAIL(applied)",
                   g_scale.p.k_g_per_r, g_prep.r0, g_prep.r1,
                   g_prep.drift_g, (unsigned long)g_calib_flash_rc);
        }
        if ((g_prep.state == PP_RESULT) && (g_prevSt != (uint8_t)PP_RESULT))
        {
          EmitEvent("PREP_RESULT");
          if ((g_prep.result[0] != 0U) && (g_prep.result[1] != 0U)
              && (g_prep.result[2] != 0U))
          {
            Beeper_Ok();                          /* 全通过：2短 */
          }
          else
          {
            Beeper_Fail();                        /* 有失败位：3短 */
          }
          Dbg_Line("# PREP result=%u%u%u drift=%+.2f g "
                   "maxdev=%.3f n=%d",
                   g_prep.result[0], g_prep.result[1],
                   g_prep.result[2],
                   g_prep.drift_g,
                   g_prep.empty_maxdev, AppScaleCalibN(&g_scale));
        }
        g_prevSt = (uint8_t)g_prep.state;
      }

      /* —— 数码管 5Hz 刷新：预备 > 清零 > 锁定读数 > 实时 —— */
      if ((HAL_GetTick() - g_lastDispTick) >= 200U)
      {
        int disp = AppPrepDisplay(&g_prep,
                                  (double)HAL_GetTick() / 1000.0);
        g_lastDispTick = HAL_GetTick();
        if (disp < 0)
        {
          disp = AppZeroDisplay(&g_zero,
                                (double)HAL_GetTick() / 1000.0);
        }
        if (disp == -3)
        {
          /* 清零采集中：闪烁 "---"——正在取参考，勿放物/勿换姿态 */
          if (((HAL_GetTick() / 500U) & 1U) != 0U)
          {
            DigitTube_ShowDashes();
          }
          else
          {
            DigitTube_ShowBlackScreen();
          }
        }
        else if (disp >= 0)
        {
          DigitTube_ShowInteger(disp);       /* 预备步骤码/结果码、清零错误码 */
        }
        else
        {
          uint32_t nowTick = HAL_GetTick();
          if ((g_rdActive != 0U) || (nowTick < g_rdHoldUntil))
          {
            /* 锁定/冻结期：显示收敛的锁定读数（运动中稳定），
               可信度闪烁只作用于读数：常亮=可信/慢闪=勉强/快闪=超限 */
            if (g_rdLevel != 0U)
            {
              uint32_t half = (g_rdLevel == 1U) ? 500U : 250U;
              if (((nowTick / half) & 1U) != 0U)
              {
                DigitTube_ShowBlackScreen();   /* 闪烁熄灭半拍 */
              }
              else
              {
                DigitTube_SetValue((float)g_rdMass);
              }
            }
            else
            {
              DigitTube_SetValue((float)g_rdMass);
            }
          }
          else
          {
            /* 待机/放置爬升期：实时质量（克，1 位小数）。
               基线未建立（复位后 ~8s 配对回退+样本积累）或 k=0（未标定）
               时诚实显示闪烁 "---"——自信的 0.0 会诱导操作者在基线建立
               前放砝码，首个砝码被置零吸收（100g→0.0/500/−100 签名）。
               建基线期间 "---" 每次亮起同步一声 40ms 短鸣（1Hz）：操作者
               不看数码管也知道"还在自检/别放东西"，就绪后自动停止。*/
            int ready = 0;
            double m = AppScaleLiveMass(&g_scale, &ready);
            if (ready != 0)
            {
              g_warmupBeep = 0U;          /* 基线已建立：一次性关闭提示音 */
              g_blinkOn = 0U;
              DigitTube_SetValue((float)m);
            }
            else if (((nowTick / 500U) & 1U) != 0U)
            {
              /* 亮半拍上升沿：只响一次（显示 5Hz 刷新，不检测沿会连响 2~3 声）*/
              if (g_blinkOn == 0U)
              {
                g_blinkOn = 1U;
                if (g_warmupBeep != 0U)
                {
                  if (nowTick < 15000U)
                  {
                    Beeper_Tick();        /* 1 × 40ms */
                  }
                  else
                  {
                    g_warmupBeep = 0U;    /* 兜底：k=0/通信异常时最多提示 15s */
                  }
                }
              }
              DigitTube_ShowDashes();
            }
            else
            {
              g_blinkOn = 0U;
              DigitTube_ShowBlackScreen();
            }
          }
        }
      }

/* 1s 心跳（通信健康）+ 双通道内码调试行 */
      if ((HAL_GetTick() - lastBeat) >= 1000U)
      {
        int32_t ca = 0, cb = 0;
        uint32_t ageA = 0U, ageB = 0U;
        int hasA, hasB;

        lastBeat = HAL_GetTick();
        HeartbeatOnce();
        if (g_recAlgoHold != 0U)
        {
          Dbg_Line("# REC n=%lu cap=%u buf=0x%08X",
                   g_recCount, s_ringCap, g_recBufAddr);
        }
        {
          /* 算法状态一行：比值/基线/实时质量/k —— 标定 vs 读数路径
             差异定位（M-C4 台架诊断）*/
          int rdy = 0;
          double lv = AppScaleLiveMass(&g_scale, &rdy);
          Dbg_Line("# ALG t=%.3f k=%.1f r=%.6f base=%.6f live=%.1f(%u)",
                   g_sampleT,
                   g_scale.p.k_g_per_r, g_scale.last_r_clean,
                   g_scale.est.baseline, lv, (unsigned)rdy);
        }

        hasA = QshDrv_Last(QSH_CH_A_REF, &ca, &ageA);
        hasB = QshDrv_Last(QSH_CH_B_MEAS, &cb, &ageB);
        if (hasA && hasB)
        {
          Dbg_Line("# CODE A=%ld(%lums) B=%ld(%lums) drop=%lu", (long)ca,
                   (unsigned long)ageA, (long)cb, (unsigned long)ageB);
        }
        else if (hasA)
        {
          Dbg_Line("# CODE A=%ld(%lums) B=---", (long)ca, (unsigned long)ageA);
        }
        else if (hasB)
        {
          Dbg_Line("# CODE A=--- B=%ld(%lums)", (long)cb, (unsigned long)ageB);
        }
        else
        {
          Dbg_Line("# CODE A=--- B=---");
        }
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
