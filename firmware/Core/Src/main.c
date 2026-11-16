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
#include "iwdg.h"
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

/* —— 自动休眠（2026-11-15：改为“睡前打标记 + 软复位”方案）——
   连续 SLEEP_QUIET_MS 内实时质量都在 ±SLEEP_BAND_G 内，且无称量事件/
   清零/预备/录制/按键，则 2 短鸣 → 数码管亮度 0 → BKP 打标记 → 软复位。
   复位后的“睡眠启动”不启动 IWDG、直接 STOP；按键 EXTI 唤醒后整机复位，
   回到正常启动（IWDG 重新开启）。原因：IWDG 由 LSI 驱动，在 STOP 里仍然
   计数，不这么做会被每 5.58~15.4 s 复位一次，根本睡不住。 */
#ifndef SLEEP_AUTO_ENABLE
#define SLEEP_AUTO_ENABLE     1        /* 调试期置 0（或 -DSLEEP_AUTO_ENABLE=0）整体禁用 */
#endif
#define SLEEP_QUIET_MS        300000U    /* 5 分钟空载窗口 */
#define SLEEP_BAND_G          3.0
#define SLEEP_BEEP_SETTLE_MS  350U
#define SLEEP_BKP_MAGIC       0x57444753UL  /* "WDGS"：BKP0R 睡眠请求标记 */

/* —— 电池低压检测（2026-11-15）——
   PA4/ADC1_IN4：12V—100k—PA4—20k—GND，分压比 6。
   低于 11.4V 报警、高于 11.6V 清除（0.2V 迟滞）；3 短鸣 + 数码管
   "11.4U" 闪 10s → 直接休眠。低电期间禁用 KEY4 标定；10s 内长按中键
   可取消强制休眠（应急使用）。 */
#define BAT_WARN_MV      11400U          /* 报警门限 */
#define BAT_CLEAR_MV     11600U          /* 清除门限（迟滞 +0.2V）*/
#define BAT_VREF_MV      3300U
#define BAT_ADC_MAX      4095U
#define BAT_DIV          6U              /* (100k+20k)/20k */
#define BAT_N            16U             /* 中值滤波样本数 */
#define BAT_SLEEP_MS     10000U          /* 提醒后强制休眠延时 */

/* 调试期置 1：STOP 时保持 SWD（DBGMCU_CR.DBG_STOP）；量产必须为 0 */
#ifndef SLEEP_KEEP_SWD
#define SLEEP_KEEP_SWD        0
#endif

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
#if (SLEEP_AUTO_ENABLE != 0)
static uint8_t  g_quietArmed = 0U;   /* 空载计时已启动 */
static uint32_t g_quietT0 = 0U;      /* 空载计时起点（HAL tick）*/
static uint8_t  g_sleepPending = 0U; /* 已 2 短鸣，等响完进 STOP */
static uint32_t g_sleepAt = 0U;      /* 计划进 STOP 的时刻 */
#endif
static uint8_t g_bootIwdgReset = 0U; /* 本次启动是 IWDG 复位（供启动日志）*/
static uint8_t g_bkpOk = 1U;         /* 正常启动时 BKP0R 写/读自检结果 */
static uint8_t  g_key4Armed = 1U;    /* KEY4 开机 10 s 窗口内可触发标定 */
static uint8_t  g_key4Held = 0U;     /* KEY4 按住计时中 */
static uint32_t g_key4T0 = 0U;       /* KEY4 按下时刻 */
static uint8_t  g_calReq = 0U;       /* 已连响三下，等建基线完成后进标定 */
static uint8_t  g_calBeep = 0U;      /* 待补响“连响三下”（避开忙时丢弃）*/
static uint32_t g_batMv = 0U;        /* 最近一次电池电压（mV）*/
static uint32_t g_batChkTick = 0U;   /* 上次检测时刻；0 = 还没测过 */
static uint32_t g_lowBatT0 = 0U;     /* 低电报警起始时刻 */
static uint8_t  g_lowBat = 0U;       /* 当前处于低电提醒 */
static uint8_t  g_lowBatCancel = 0U; /* 用户已取消强制休眠 */
static uint8_t  g_lowBatMuted = 0U;  /* 本次开机不再重复报警 */
static uint8_t  g_batLowEver = 0U;   /* 本次开机出现过低电（禁 KEY4 标定）*/
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

/* —— 独立看门狗（2026-11-15）——
   参数由 CubeMX 的 `MX_IWDG_Init()` 决定（当前 .ioc：预分频 128 + RLR 4095）：
     典型 16.38 s @32 kHz；最快 11.15 s（LSI 47 kHz）~ 最慢 30.84 s（17 kHz）。
   Flash 擦除最大按 4 s，余量充足；代价是卡死要 11~31 s 才复位。
   喂狗统一写 KR=0xAAAA，与预分频无关；改预分频只需改 .ioc，不用动这里。
   option byte 必须是软件启动（IWDG_SW=1），否则复位关不掉它（睡眠方案依赖这点）。 */
void AppWdg_Feed(void)
{
  IWDG->KR = 0xAAAAU;                 /* 重装计数 */
}

/* —— 睡眠启动判定（2026-11-15）——
   只有“软件复位(SFTRST) + BKP0R 魔术值”同时成立才算睡眠请求。
   加复位原因判断是为了防 VBAT 保持 BKP 时，整机断电再上电被误判成
   睡眠启动（设备会像死机一样躺着）。 */
static int BootIsSleepRequest(void)
{
  return (((RCC->CSR & RCC_CSR_SFTRSTF) != 0U)
          && (RTC->BKP0R == SLEEP_BKP_MAGIC)) ? 1 : 0;
}

/* —— 睡眠启动：不启动 IWDG，不初始化 UART/数码管/驱动 ——
   只做四件事：蜂鸣器拉低、按键 EXTI、485 四脚高阻、进 STOP。
   醒来即整机复位，回到正常启动。 */
static void SleepBoot(void)
{
  GPIO_InitTypeDef g = {0};

  /* 1) 蜂鸣器先拉低：复位后 PA6 是浮空输入，先钉住防毛刺 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  g.Pin   = GPIO_PIN_6;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_Init(GPIOA, &g);

  /* 2) 按键 + EXTI（MX_GPIO_Init 只配 PB13/14/15 与 EXTI15_10）*/
  MX_GPIO_Init();

  /* 3) 485 四脚模拟高阻（防向断电模块倒灌）*/
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  g.Pin  = GPIO_PIN_2 | GPIO_PIN_3;     /* USART2 TX/RX */
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin  = GPIO_PIN_10 | GPIO_PIN_11;   /* USART3 TX/RX */
  HAL_GPIO_Init(GPIOB, &g);

#if (SLEEP_KEEP_SWD != 0)
  DBGMCU->CR |= DBGMCU_CR_DBG_STOP;     /* 调试用；量产必须为 0 */
#endif

  /* 4) 清 pending、停 SysTick、进 STOP；按键唤醒 → 整机复位 */
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  NVIC_SystemReset();
  for (;;)
  {
  }
}

/* —— 拓展板 KEY4（PA15）：输入、内部上拉、按下低电平 ——
   PA15 复位后是 JTDI（AF0）；F4 上把它配成普通输入即可当 GPIO 用，
   SWD 仍在 PA13/PA14，不影响下载/调试。 */
static void Key4_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  g.Pin  = GPIO_PIN_15;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);
}

/* —— 电池电压：16 次采集中值，返回 mV；ADC 失败返回 0xFFFFFFFF ——
   分压 12V—100k—PA4—20k—GND（比 6），ADC 12bit、VDDA=3.3V。
   采样时间已改 480 周期（adc.c），匹配 16.7k 源阻抗。 */
static uint32_t BatVolt_ReadMv(void)
{
  uint16_t v[BAT_N];
  uint8_t i, j;
  uint16_t t;
  uint32_t code;

  for (i = 0U; i < BAT_N; i++)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return 0xFFFFFFFFU;
    if (HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return 0xFFFFFFFFU;
    }
    v[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
  }
  /* 插入排序取中值（16 个元素，开销量可忽略）*/
  for (i = 1U; i < BAT_N; i++)
  {
    t = v[i];
    j = i;
    while ((j > 0U) && (v[j - 1U] > t))
    {
      v[j] = v[j - 1U];
      j--;
    }
    v[j] = t;
  }
  code = ((uint32_t)v[BAT_N / 2U - 1U] + (uint32_t)v[BAT_N / 2U]) / 2U;
  return (uint32_t)(((uint64_t)code * (uint64_t)BAT_VREF_MV * BAT_DIV)
                    / BAT_ADC_MAX);
}

/* —— 低电显示文本：11.4V → "11.4U"（U = 请充电，模块字库需实测）—— */
static void Bat_ShowVoltage(void)
{
  char b[8];
  uint32_t whole, frac;

  if (g_batMv == 0xFFFFFFFFU) return;
  whole = g_batMv / 1000U;
  frac  = (g_batMv % 1000U) / 100U;
  if (whole >= 10U)
  {
    b[0] = (char)('0' + (whole / 10U));
    b[1] = (char)('0' + (whole % 10U));
    b[2] = '.';
    b[3] = (char)('0' + frac);
    b[4] = 'U';
    b[5] = '\0';
  }
  else
  {
    b[0] = (char)('0' + whole);
    b[1] = '.';
    b[2] = (char)('0' + frac);
    b[3] = 'U';
    b[4] = '\0';
  }
  DigitTube_SetText(b);
}

#if (SLEEP_AUTO_ENABLE != 0)
/* —— 自动休眠：前提判定（2026-11-14）——
   读数就绪且在 ±SLEEP_BAND_G、无称量事件/清零/预备/录制、无按键、双通道
   数据新鲜（<500 ms）。任一条件不满足都要重新计满 SLEEP_QUIET_MS。 */
static int SleepConditionOk(void)
{
  int32_t ca = 0, cb = 0;
  uint32_t ageA = 0U, ageB = 0U;
  int ready = 0;
  double m;

  if ((QshDrv_Last(QSH_CH_A_REF, &ca, &ageA) == 0) || (ageA > 500U)) return 0;
  if ((QshDrv_Last(QSH_CH_B_MEAS, &cb, &ageB) == 0) || (ageB > 500U)) return 0;

  m = AppScaleLiveMass(&g_scale, &ready);
  if (ready == 0) return 0;
  if ((m < -SLEEP_BAND_G) || (m > SLEEP_BAND_G)) return 0;
  /* 只看 state：est.locked 是"本事件曾锁定"标志，REARM 时不会被清
     （algo_est 的 near_n 分支只清 state），拿它当"当前是否在称量"用
     会导致第一次称量之后自动休眠永远不触发。 */
  if (g_scale.est.state != 0) return 0;
  if (g_scale.calib_collect != 0U) return 0;
  if (g_zero.state != ZO_IDLE) return 0;
  if (g_prep.state != PP_IDLE) return 0;
  if (g_recArm != 0U) return 0;
  if ((Keys_IsPressed(0U) != 0) || (Keys_IsPressed(1U) != 0)
      || (Keys_IsPressed(2U) != 0)) return 0;
  return 1;
}

/* —— 睡前序列（2026-11-15：不再直接进 STOP，改为打标记 + 软复位）——
   2 短鸣由调用方先发；这里负责：停轮询 → 蜂鸣器静音 → 数码管亮度 0 并等
   这一帧发完 → BKP0R 打标记 → 软复位。复位后的 SleepBoot 不启动 IWDG、
   直接 STOP；按键唤醒再整机复位回正常启动。复位本身会把 UART 引脚复位成
   浮空，所以这里不需要 DeInit UART / 置 analog。 */
static void RequestSleepReset(void)
{
  uint8_t i;

  /* 1) 停 10Hz 轮询，避免复位前还在发帧 */
  (void)HAL_TIM_Base_Stop_IT(&htim2);

  /* 2) 蜂鸣器静音 */
  Beeper_Init();

  /* 3) 数码管亮度 0：复位不清屏，必须先把这一帧真正发完 */
  DigitTube_SetBrightness(0U);
  for (i = 0U; i < 60U; i++)
  {
    DigitTube_Process();
    if (HAL_UART_GetState(&huart4) == HAL_UART_STATE_READY)
    {
      break;
    }
    HAL_Delay(1U);
  }
  HAL_Delay(10U);

  /* 4) 打 BKP 标记后软复位（备份寄存器跨系统复位保留；VBAT 掉电才清）*/
  HAL_PWR_EnableBkUpAccess();
  RTC->BKP0R = SLEEP_BKP_MAGIC;
  NVIC_SystemReset();
  for (;;)
  {
  }
}
#endif /* SLEEP_AUTO_ENABLE */

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

  /* —— 睡眠启动判定（必须在清 reset flags 之前读 CSR）——
     BKP0R 属 RTC 备份域：F407 没有 RTCAPBEN 位（APB1ENR bit28 是 PWREN），
     RTC APB 接口常开，只需要 PWR 时钟；写 BKP 前还要 DBP=1。 */
  __HAL_RCC_PWR_CLK_ENABLE();
  g_bootIwdgReset = ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U) ? 1U : 0U;
  if (BootIsSleepRequest() != 0)
  {
    /* 先清标记：这样睡眠期间任何复位都会回正常启动，不会“躺死” */
    HAL_PWR_EnableBkUpAccess();
    RTC->BKP0R = 0U;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    SleepBoot();                      /* 不返回 */
  }
  /* 正常启动：先做一次 BKP 写/读自检（确认备份域可访问），再清标记 */
  HAL_PWR_EnableBkUpAccess();
  RTC->BKP0R = 0x1234ABCDUL;
  g_bkpOk = (RTC->BKP0R == 0x1234ABCDUL) ? 1U : 0U;
  RTC->BKP0R = 0U;
  __HAL_RCC_CLEAR_RESET_FLAGS();

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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  /* IWDG 参数以 CubeMX/.ioc 为准（预分频 128 + 4095），这里只加调试冻结 */
  __HAL_DBGMCU_FREEZE_IWDG();       /* 调试断点时不复位；量产无影响 */
  DwtTime_Init();
  Dbg_Init();
  if (g_bootIwdgReset != 0U)
  {
    Dbg_Line("# RESET iwdg (watchdog)");
  }
  if (g_bkpOk == 0U)
  {
    Dbg_Line("# WARN RTC BKP write/read failed");
  }
  Keys_Init();
  Key4_Init();              /* 拓展板 KEY4（PA15，开机标定触发键）*/
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

      /* —— 电池低压：开机 + 每 10 s 复检（11.4V 报警 / 11.6V 清除）—— */
      {
        uint32_t nowB = HAL_GetTick();
        if ((g_batChkTick == 0U)
            || ((uint32_t)(nowB - g_batChkTick) >= 10000U))
        {
          uint32_t mv = BatVolt_ReadMv();
          uint8_t first = (g_batChkTick == 0U) ? 1U : 0U;
          g_batChkTick = nowB;
          if (mv != 0xFFFFFFFFU)
          {
            g_batMv = mv;
            if (first != 0U)
            {
              Dbg_Line("# BAT %lu mV", (unsigned long)mv);
            }
            if ((g_lowBat == 0U) && (g_lowBatMuted == 0U)
                && (mv < BAT_WARN_MV))
            {
              g_lowBat = 1U;
              g_lowBatT0 = nowB;
              g_batLowEver = 1U;
              g_warmupBeep = 0U;        /* 停建基线提示音，突出低电三连响 */
              Beeper_Beep(3U, 80U, 80U);
              Dbg_Line("# BAT low %lu mV -> charge, sleep in 10s",
                       (unsigned long)mv);
            }
            else if ((g_lowBat != 0U) && (mv > BAT_CLEAR_MV))
            {
              g_lowBat = 0U;
              Dbg_Line("# BAT ok %lu mV", (unsigned long)mv);
            }
          }
        }
        if ((g_lowBat != 0U) && (g_lowBatCancel == 0U)
            && ((uint32_t)(nowB - g_lowBatT0) >= BAT_SLEEP_MS))
        {
#if (SLEEP_AUTO_ENABLE != 0)
          Dbg_Line("# BAT sleep");
          RequestSleepReset();          /* 不返回 */
#else
          g_lowBat = 0U;
#endif
        }
      }

      /* —— 拓展板 KEY4（PA15）：开机 10 s 内持续按住 2 s 触发标定 ——
         触发后连响三下；等建基线完成（--- 消失）再进入预备标定。
         窗口一旦错过（>10 s）本次开机不再触发。 */
      if (g_batLowEver != 0U)
      {
        g_key4Armed = 0U;               /* 低电：禁止标定（弱电写 Flash 危险）*/
      }
      if (g_key4Armed != 0U)
      {
        uint32_t now4 = HAL_GetTick();
        if (now4 < 10000U)
        {
          if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
          {
            if (g_key4Held == 0U)
            {
              g_key4Held = 1U;
              g_key4T0 = now4;
            }
            else if ((uint32_t)(now4 - g_key4T0) >= 2000U)
            {
              g_key4Armed = 0U;
              g_calReq = 1U;
              g_calBeep = 1U;
              g_warmupBeep = 0U;      /* 停建基线提示音，保证三声完整 */
              Dbg_Line("# CAL trigger (KEY4)");
            }
          }
          else
          {
            g_key4Held = 0U;
          }
        }
        else
        {
          g_key4Armed = 0U;           /* 10 s 窗口结束，本次开机不再触发 */
        }
      }
      if ((g_calBeep != 0U) && (Beeper_Busy() == 0U))
      {
        g_calBeep = 0U;
        Beeper_Beep(3U, 80U, 80U);    /* 连响三下：标定已触发 */
      }

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


/* —— 按键分发（2026-11-15 新映射）——
         KEY0/PB13 长按 = 无功能（保留为唤醒源）
         KEY1/PB14 长按 = 清零（空盘零点重锚）进入/中止
         KEY2/PB15 长按 = 修正 k（带 500 g 标准砝码的跨度校正）进入/中止
         KEY4/PA15     = 开机 10 s 内按住 2 s 触发标定（见上方 KEY4 检测）
   中键只做清零、不再自动分流到跨度校正——避免“想清零却把 k 改了”。 */
      {
        int evZero = 0;
        int evSpan = 0;
        int zeroed;
        (void)Keys_Take(0U);              /* KEY0 长按无功能，消费掉事件 */
        ev = Keys_Take(1U);
        if (ev == KEY_EV_LONG)
        {
          if ((g_lowBat != 0U) && (g_lowBatCancel == 0U))
          {
            /* 低电提醒期间长按中键：取消强制休眠，允许应急使用 */
            g_lowBatCancel = 1U;
            g_lowBatMuted = 1U;
            Dbg_Line("# BAT cancel (use anyway)");
          }
          else if ((g_prep.state == PP_IDLE) && (g_recArm == 0U)
                   && (g_recAlgoHold == 0U))
          {
            /* 中键：只做清零（空盘零点重锚），不再自动分流到跨度校正 */
            evZero = 1;
          }
        }
        ev = Keys_Take(2U);
        if (ev == KEY_EV_LONG)
        {
          /* 左键：修正 k（500 g 跨度校正）*/
          if ((g_prep.state == PP_IDLE) && (g_recArm == 0U)
              && (g_recAlgoHold == 0U))
          {
            evSpan = 1;
          }
        }
        zeroed = AppZeroPoll(&g_zero, &g_scale,
                             (double)HAL_GetTick() / 1000.0, evZero, evSpan);
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
        {
          /* KEY4 触发后：等建基线完成（--- 消失、ready=1）再进预备标定 */
          int calStart = 0;
          if (g_calReq != 0U)
          {
            int rdy4 = 0;
            (void)AppScaleLiveMass(&g_scale, &rdy4);
            if (rdy4 != 0)
            {
              g_calReq = 0U;
              calStart = 1;
              Dbg_Line("# CAL start (baseline ready)");
            }
          }
          committed = AppPrepPoll(&g_prep, &g_scale, tNowSec, calStart);
          if (calStart != 0)
          {
            EmitEvent((g_prep.state == PP_COMMS) ? "PREP_ENTER" : "PREP_ABORT");
          }
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
      /* 低电提醒：最高显示优先级，闪 "11.4U"（U=请充电）*/
      if ((g_lowBat != 0U) && (g_lowBatCancel == 0U))
      {
        if ((HAL_GetTick() - g_lastDispTick) >= 200U)
        {
          g_lastDispTick = HAL_GetTick();
          if (((HAL_GetTick() / 500U) & 1U) != 0U)
          {
            Bat_ShowVoltage();
          }
          else
          {
            DigitTube_ShowBlackScreen();
          }
        }
      }
      else if ((HAL_GetTick() - g_lastDispTick) >= 200U)
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

      /* —— 自动休眠：连续 SLEEP_QUIET_MS 空载（±3 g）→ 2 短鸣 → STOP —— */
#if (SLEEP_AUTO_ENABLE != 0)
      if (g_sleepPending == 0U)
      {
        if (SleepConditionOk() != 0)
        {
          uint32_t nowQ = HAL_GetTick();
          if (g_quietArmed == 0U)
          {
            g_quietArmed = 1U;
            g_quietT0 = nowQ;
          }
          else if ((uint32_t)(nowQ - g_quietT0) >= SLEEP_QUIET_MS)
          {
            Beeper_Ok();                    /* 滴滴两下 */
            g_sleepPending = 1U;
            g_sleepAt = nowQ + SLEEP_BEEP_SETTLE_MS;
            Dbg_Line("# SLEEP arm");
          }
        }
        else
        {
          g_quietArmed = 0U;                /* 中途被触碰/放物/操作：重新计时 */
        }
      }
      else if ((int32_t)(HAL_GetTick() - g_sleepAt) >= 0)
      {
        if (SleepConditionOk() != 0)
        {
          Dbg_Line("# SLEEP enter");
          RequestSleepReset();              /* 打标记 + 软复位，不返回 */
        }
        else
        {
          g_sleepPending = 0U;              /* 提示音期间被触碰：取消本次休眠 */
          g_quietArmed = 0U;
        }
      }
#endif /* SLEEP_AUTO_ENABLE */

      AppWdg_Feed();                        /* 主循环唯一常规喂狗点 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
