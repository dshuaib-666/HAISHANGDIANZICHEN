#include "qsh_drv.h"
#include "qsh_frame.h"
#include "dwt_time.h"
#include "usart.h"
#include <string.h>

/* —— 通道绑定 —— */
#define CH_UART(ch)  (((ch) == QSH_CH_B_MEAS) ? &huart2 : &huart3)

/* —— 命令（CRC 运行时计算）——
   读内码：01 03 00 17 00 02（协议地址 0x17-0x18，32 位）
   配置写：01 06 <reg> <val>（开机不置零 / 滤波弱 / 蠕动关）
   注：本模块的模块级变量一律非 static——便于调试器直接观测。 */
uint8_t s_cmdRead[QSH_NCH][8];
uint8_t s_cmdCfg[QSH_NCH][3][8];
uint8_t s_cmdZero[QSH_NCH][8];   /* 01 06 00 04 00 01（模块清零） */
static const uint16_t CFG_REG[3] = {0x0005U, 0x0006U, 0x0007U};
static const uint16_t CFG_VAL[3] = {0x0001U, 0x0000U, 0x0000U};

QshFrame_t s_frame[QSH_NCH];

/* —— 样本环形缓冲（仅主循环上下文访问）—— */
static QshSample_t s_ringSmall[QSH_RING_N];   /* 正常模式小环 */
QshSample_t * volatile s_ring = s_ringSmall;  /* volatile：录制切换后必须立即生效 */             /* 当前环形缓冲指针 */
volatile uint16_t s_ringCap = QSH_RING_N;               /* 当前容量 */
uint16_t s_head, s_tail;

/* —— 解析上下文 —— */
int32_t  s_lastCode[QSH_NCH];
uint8_t  s_hasLast[QSH_NCH];
uint32_t s_seq[QSH_NCH];
uint32_t s_lastTick[QSH_NCH];

/* —— 启动配置状态机（主循环上下文）—— */
typedef enum { BST_SEND = 0, BST_WAIT, BST_DONE } BootSt_t;
BootSt_t s_boot;
uint8_t  s_bootCh, s_bootIdx, s_bootTry;
uint32_t s_bootTick;
volatile uint8_t s_bootAck;    /* Parse 置位（同上下文） */
volatile uint8_t s_ready;

volatile QshStat_t s_stat[QSH_NCH];

/* —— RAM 录制（零接线取数，见 qsh_drv.h 注释）—— */
volatile uint8_t g_recArm = 0U;
volatile uint8_t g_recFull = 0U;
volatile uint32_t g_recBufAddr = 0U;
volatile uint32_t g_recCount = 0U;
uint8_t g_recAlgoHold = 0U;
static uint8_t s_recPrevArm = 0U;

/* —— 调试器寄存器探针（Watch 驱动的单次 MODBUS 读/写，零接线）——
   用法：设 g_probeCh / g_probeReg /（读）g_probeCnt 或（写）g_probeVal，
   最后写 g_probeOp=1(读)/2(写06)/3(写16,32位双寄存器) 启动。
   完成后固件置 g_probeDone 并清 g_probeOp：
     0xA5 成功（读数据在 g_probeData[0..cnt-1]，寄存器原样大端转 uint16）
     0xAE 模块异常（码在 g_probeExc）   0x99 超时   0x98 参数非法
   op=3：g_probeVal=高字、g_probeData[0]=低字（模块 32 位高字在前），
         用于一次写满量程/砝码值这类 32 位参数。
   探针期间自动暂停 100ms 内码轮询；写超时 1.5s（EEPROM 慢）。 */
volatile uint8_t  g_probeOp = 0U;
volatile uint8_t  g_probeCh = 0U;
volatile uint16_t g_probeReg = 0x000DU;
volatile uint16_t g_probeCnt = 1U;      /* 1..10 个寄存器 */
volatile uint16_t g_probeVal = 0U;
volatile uint16_t g_probeData[10];
volatile uint8_t  g_probeDone = 0U;
volatile uint8_t  g_probeRx[16];       /* 探针期间收到的原始帧（诊断用） */
volatile uint8_t  g_probeRxLen = 0U;
volatile uint8_t  g_probeExc = 0U;     /* MODBUS 异常码（0x83/0x86 响应的第 3 字节） */
volatile uint8_t  g_probeLate = 0U;    /* 1=超时后收到了迟到回显（模块处理慢） */

static uint8_t  s_probeCmd[16];
static uint8_t  s_probeLen = 8U;
static uint8_t  s_probeEcho10[8];      /* 0x10 写的预期回显 */
static uint8_t  s_probeBusy = 0U;       /* 抑制 TIM2 轮询 */
static volatile uint8_t s_probeAck = 0U;/* 0 无 1 读响应 2 写回显 */
static uint8_t  s_probeCntSav = 0U;
typedef enum { PR_IDLE = 0, PR_SETTLE, PR_WAIT } ProbeSt_t;
static ProbeSt_t s_probeSt = PR_IDLE;
static uint32_t  s_probeTick;

/* ------------------------------------------------------------------ */
static uint16_t crc16(const uint8_t *d, uint16_t n)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t b;

  for (i = 0U; i < n; i++)
  {
    crc ^= d[i];
    for (b = 0U; b < 8U; b++)
    {
      crc = ((crc & 1U) != 0U) ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                               : (uint16_t)(crc >> 1U);
    }
  }
  return crc;
}

static void putCmd(uint8_t *buf, uint8_t fcn, uint16_t reg, uint16_t val)
{
  uint16_t crc;
  buf[0] = 0x01U;
  buf[1] = fcn;
  buf[2] = (uint8_t)(reg >> 8);
  buf[3] = (uint8_t)(reg & 0xFFU);
  buf[4] = (uint8_t)(val >> 8);
  buf[5] = (uint8_t)(val & 0xFFU);
  crc = crc16(buf, 6U);
  buf[6] = (uint8_t)(crc & 0xFFU);
  buf[7] = (uint8_t)(crc >> 8);
}

static void sendCmd(uint8_t ch, const uint8_t *cmd, uint8_t len)
{
  UART_HandleTypeDef *h = CH_UART(ch);

  if (h->gState == HAL_UART_STATE_READY)
  {
    if (HAL_UART_Transmit_DMA(h, (uint8_t *)cmd, len) != HAL_OK)
    {
      s_stat[ch].txBusy++;
    }
  }
  else
  {
    s_stat[ch].txBusy++;
  }
}

/* ------------------------------------------------------------------ */
void QshDrv_Init(void)
{
  uint8_t ch;
  uint8_t i;

  for (ch = 0U; ch < QSH_NCH; ch++)
  {
    putCmd(s_cmdRead[ch], 0x03U, 0x0017U, 0x0002U);
    for (i = 0U; i < 3U; i++)
    {
      putCmd(s_cmdCfg[ch][i], 0x06U, CFG_REG[i], CFG_VAL[i]);
    }
    putCmd(s_cmdZero[ch], 0x06U, 0x0004U, 0x0001U);
    s_lastCode[ch] = 0;
    s_hasLast[ch] = 0U;
    s_seq[ch] = 0U;
    QshFrame_Init(&s_frame[ch], CH_UART(ch));
  }
  memset((void *)s_stat, 0, sizeof(s_stat));
  s_head = 0U;
  s_tail = 0U;
  s_boot = BST_SEND;
  s_bootCh = 0U;
  s_bootIdx = 0U;
  s_bootTry = 0U;
  s_bootAck = 0U;
  s_ready = 0U;

  for (ch = 0U; ch < QSH_NCH; ch++)
  {
    QshFrame_Start(&s_frame[ch]);
  }
}

uint8_t QshDrv_Ready(void)
{
  return s_ready;
}

void QshDrv_SendZero(uint8_t ch)
{
  if ((s_ready != 0U) && (ch < QSH_NCH))
  {
    sendCmd(ch, s_cmdZero[ch], 8U);   /* 配置完成后才可发送 */
  }
}

void QshDrv_IdleIRQ(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    QshFrame_IdleIRQ(&s_frame[QSH_CH_B_MEAS]);
  }
  else if (huart == &huart3)
  {
    QshFrame_IdleIRQ(&s_frame[QSH_CH_A_REF]);
  }
}

/* TIM2 100ms 节拍：并行轮询两通道（s_ready 后才启动；探针期间暂停）。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim != NULL) && (htim->Instance == TIM2) && (s_ready != 0U) &&
      (s_probeBusy == 0U))
  {
    sendCmd(QSH_CH_A_REF, s_cmdRead[QSH_CH_A_REF], 8U);
    sendCmd(QSH_CH_B_MEAS, s_cmdRead[QSH_CH_B_MEAS], 8U);
  }
}

static void Parse(uint8_t ch, const uint8_t *d, uint16_t len, uint64_t tUs)
{
  uint16_t crc;
  int32_t v;
  QshSample_t s;

  /* 探针期间：无论什么帧都留一份原始快照，异常帧（fcn|0x80）解码异常码 */
  if (s_probeBusy != 0U)
  {
    uint16_t i;
    for (i = 0U; (i < len) && (i < 16U); i++)
    {
      g_probeRx[i] = d[i];
    }
    g_probeRxLen = (uint8_t)((len < 16U) ? len : 16U);
    if ((s_probeAck == 0U) && (d[0] == 0x01U) &&
        ((d[1] == 0x83U) || (d[1] == 0x86U)))
    {
      g_probeExc = d[2];
      s_probeAck = 3U;            /* 模块异常响应 */
      return;
    }
  }

  if ((len == 8U) && (d[0] == 0x01U) && (d[1] == 0x06U))
  {
    if (memcmp(d, s_cmdZero[ch], 8U) == 0)
    {
      s_stat[ch].zeroAck++;       /* 清零回显 */
      return;
    }
    /* 配置回显：仅启动配置阶段比对（s_ready 后 s_bootCh 越界） */
    if ((s_ready == 0U) && (memcmp(d, s_cmdCfg[s_bootCh][s_bootIdx], 8U) == 0))
    {
      s_bootAck = 1U;
      return;
    }
    if (memcmp(d, s_probeCmd, 8U) == 0)
    {
      if (s_probeBusy != 0U)
      {
        s_probeAck = 2U;          /* 探针 06 写回显 */
      }
      else
      {
        g_probeLate = 1U;         /* 超时后才到的回显：模块处理慢但写已生效 */
      }
    }
    return;
  }

  /* 0x10 双寄存器写的回显：01 10 <reg> <0002> CRC（不回数据） */
  if ((len == 8U) && (d[0] == 0x01U) && (d[1] == 0x10U) &&
      (memcmp(d, s_probeEcho10, 8U) == 0))
  {
    if (s_probeBusy != 0U)
    {
      s_probeAck = 2U;
    }
    else
    {
      g_probeLate = 1U;
    }
    return;
  }

  /* 探针读响应：01 03 2N <数据> CRC，字节数与请求一致才接收 */
  if ((s_probeBusy != 0U) && (s_probeAck == 0U) && (d[0] == 0x01U) && (d[1] == 0x03U) &&
      (d[2] == (uint8_t)(2U * s_probeCntSav)) &&
      (len == (uint16_t)(5U + 2U * s_probeCntSav)))
  {
    uint16_t i;
    crc = crc16(d, (uint16_t)(len - 2U));
    if ((d[len - 2U] == (uint8_t)(crc & 0xFFU)) && (d[len - 1U] == (uint8_t)(crc >> 8)))
    {
      for (i = 0U; i < s_probeCntSav; i++)
      {
        g_probeData[i] = (uint16_t)(((uint16_t)d[3U + 2U * i] << 8) |
                                    d[4U + 2U * i]);
      }
      s_probeAck = 1U;
    }
    return;
  }

  if ((len != 9U) || (d[0] != 0x01U) || (d[1] != 0x03U) || (d[2] != 0x04U))
  {
    s_stat[ch].frameErr++;
    return;
  }

  crc = crc16(d, 7U);
  if ((d[7] != (uint8_t)(crc & 0xFFU)) || (d[8] != (uint8_t)(crc >> 8)))
  {
    s_stat[ch].crcErr++;
    return;
  }

  v = ((int32_t)d[3] << 24) | ((int32_t)d[4] << 16) |
      ((int32_t)d[5] << 8) | (int32_t)d[6];

  s.ch = ch;
  s.code = v;
  s.tUs = tUs;
  s.dup = (uint8_t)((s_hasLast[ch] != 0U) && (v == s_lastCode[ch]));
  s.seq = s_seq[ch]++;
  s_lastCode[ch] = v;
  s_lastTick[ch] = HAL_GetTick();
  s_hasLast[ch] = 1U;

  if ((g_recArm != 0U) && (g_recFull != 0U))
  {
    s_stat[ch].readOk++;       /* 录满冻结：继续轮询但不再入环，缓冲稳定可导出 */
    return;
  }

  s_ring[s_head] = s;
  s_head = (uint16_t)((s_head + 1U) % s_ringCap);
  if (g_recArm != 0U)
  {
    g_recCount++;               /* 录制模式：独立计数（s_head 环绕归零不可用）*/
  }
  if (s_head == s_tail)
  {
    if (g_recArm != 0U)
    {
      g_recFull = 1U;          /* 录制模式：恰好写满 → 冻结 */
    }
    else
    {
      s_tail = (uint16_t)((s_tail + 1U) % s_ringCap);
      s_stat[ch].ringDrop++;
    }
  }
  s_stat[ch].readOk++;
}

/* 探针状态机：SETTLE 期停轮询等在途响应落地 → SEND → WAIT。
   主循环上下文，与 Parse 同线程，无并发问题。 */
static void ProbeStep(void)
{
  if (s_probeSt == PR_IDLE)
  {
    if ((g_probeOp != 0U) && (s_ready != 0U))
    {
      uint16_t cnt;
      uint16_t crc;
      if ((g_probeCh >= QSH_NCH) ||
          (g_probeOp > 3U) ||
          ((g_probeOp == 1U) && ((g_probeCnt < 1U) || (g_probeCnt > 10U))))
      {
        g_probeDone = 0x98U;      /* 参数非法 */
        g_probeOp = 0U;
        return;
      }
      memset(s_probeEcho10, 0xFFU, 8U);   /* 非 0x10 探针不会误匹配 */
      if (g_probeOp == 1U)
      {
        cnt = g_probeCnt;
        putCmd(s_probeCmd, 0x03U, g_probeReg, cnt);
        s_probeLen = 8U;
      }
      else if (g_probeOp == 2U)
      {
        putCmd(s_probeCmd, 0x06U, g_probeReg, g_probeVal);
        s_probeLen = 8U;
      }
      else
      {
        /* op=3：0x10 双寄存器写（32 位参数一次写入）
           g_probeVal=高字、g_probeData[0]=低字（默认高字在前） */
        s_probeCmd[0] = 0x01U;  s_probeCmd[1] = 0x10U;
        s_probeCmd[2] = (uint8_t)(g_probeReg >> 8);
        s_probeCmd[3] = (uint8_t)(g_probeReg & 0xFFU);
        s_probeCmd[4] = 0x00U;  s_probeCmd[5] = 0x02U;   /* 2 个寄存器 */
        s_probeCmd[6] = 0x04U;                           /* 4 字节数据  */
        s_probeCmd[7] = (uint8_t)(g_probeVal >> 8);
        s_probeCmd[8] = (uint8_t)(g_probeVal & 0xFFU);
        s_probeCmd[9] = (uint8_t)(g_probeData[0] >> 8);
        s_probeCmd[10] = (uint8_t)(g_probeData[0] & 0xFFU);
        crc = crc16(s_probeCmd, 11U);
        s_probeCmd[11] = (uint8_t)(crc & 0xFFU);
        s_probeCmd[12] = (uint8_t)(crc >> 8);
        s_probeLen = 13U;
        /* 预期回显：01 10 <reg> 00 02 CRC */
        s_probeEcho10[0] = 0x01U; s_probeEcho10[1] = 0x10U;
        s_probeEcho10[2] = s_probeCmd[2]; s_probeEcho10[3] = s_probeCmd[3];
        s_probeEcho10[4] = 0x00U; s_probeEcho10[5] = 0x02U;
        crc = crc16(s_probeEcho10, 6U);
        s_probeEcho10[6] = (uint8_t)(crc & 0xFFU);
        s_probeEcho10[7] = (uint8_t)(crc >> 8);
      }
      s_probeCntSav = (g_probeOp == 1U) ? (uint8_t)g_probeCnt : 0U;
      s_probeAck = 0U;
      s_probeBusy = 1U;
      g_probeDone = 0U;
      g_probeExc = 0U;
      g_probeRxLen = 0U;
      g_probeLate = 0U;
      s_probeTick = HAL_GetTick();
      s_probeSt = PR_SETTLE;
    }
    return;
  }

  if (s_probeSt == PR_SETTLE)
  {
    if ((HAL_GetTick() - s_probeTick) > 150U)   /* 跨过一个轮询周期 */
    {
      sendCmd(g_probeCh, s_probeCmd, s_probeLen);
      s_probeTick = HAL_GetTick();
      s_probeSt = PR_WAIT;
    }
    return;
  }

  /* PR_WAIT：读 400ms；写放宽到 1.5s（配置写可能涉及 EEPROM） */
  if (s_probeAck == 3U)
  {
    g_probeDone = 0xAEU;          /* 模块回异常（码在 g_probeExc） */
  }
  else if (s_probeAck != 0U)
  {
    g_probeDone = 0xA5U;
  }
  else if ((HAL_GetTick() - s_probeTick) >
           ((s_probeCmd[1] == 0x03U) ? 400U : 1500U))
  {
    g_probeDone = 0x99U;          /* 无响应（g_probeLate=1 说明回显迟到） */
  }
  else
  {
    return;
  }
  s_probeBusy = 0U;
  s_probeSt = PR_IDLE;
  g_probeOp = 0U;
}

static void BootStep(void)
{
  if (s_ready != 0U)
  {
    return;
  }

  switch (s_boot)
  {
    case BST_SEND:
      s_bootAck = 0U;
      sendCmd(s_bootCh, s_cmdCfg[s_bootCh][s_bootIdx], 8U);
      s_bootTick = HAL_GetTick();
      s_boot = BST_WAIT;
      break;

    case BST_WAIT:
      if (s_bootAck != 0U)
      {
        s_stat[s_bootCh].cfgAck++;
        s_bootTry = 0U;
        s_boot = BST_SEND;
        s_bootIdx++;
        if (s_bootIdx >= 3U)
        {
          s_bootIdx = 0U;
          s_bootCh++;
          if (s_bootCh >= QSH_NCH)
          {
            s_ready = 1U;      /* 配置完成，TIM2 开始轮询 */
            return;
          }
        }
      }
      else if ((HAL_GetTick() - s_bootTick) > 250U)
      {
        s_bootTry++;
        if (s_bootTry >= 3U)
        {
          s_stat[s_bootCh].cfgFail++;
          s_bootTry = 0U;
          s_boot = BST_SEND;   /* 放弃该条，继续下一条 */
          s_bootIdx++;
          if (s_bootIdx >= 3U)
          {
            s_bootIdx = 0U;
            s_bootCh++;
            if (s_bootCh >= QSH_NCH)
            {
              s_ready = 1U;
              return;
            }
          }
        }
        else
        {
          s_boot = BST_SEND;   /* 重发 */
        }
      }
      break;

    default:
      s_ready = 1U;
      break;
  }
}

void QshDrv_Process(void)
{
  uint8_t ch;
  uint8_t buf[QSH_FRAME_MAX];
  uint16_t len;
  uint64_t tUs;

  /* RAM 录制：g_recArm 上升沿（调试器写 1）→ 清空缓冲重新开始 */
  if ((g_recArm != 0U) && (s_recPrevArm == 0U))
  {
    s_head = 0U;
    s_tail = 0U;
    g_recFull = 0U;
  }
  s_recPrevArm = g_recArm;

  for (ch = 0U; ch < QSH_NCH; ch++)
  {
    if (QshFrame_Take(&s_frame[ch], buf, &len, &tUs) != 0)
    {
      Parse(ch, buf, len, tUs);
    }
  }
  BootStep();
  ProbeStep();
}

int QshDrv_Last(uint8_t ch, int32_t *code, uint32_t *ageMs)
{
  if ((ch >= QSH_NCH) || (s_hasLast[ch] == 0U) || (code == NULL))
  {
    return 0;
  }
  *code = s_lastCode[ch];
  *ageMs = HAL_GetTick() - s_lastTick[ch];
  return 1;
}

int QshDrv_PopSample(QshSample_t *out)
{
  if (s_tail == s_head)
  {
    return 0;
  }
  *out = s_ring[s_tail];
  s_tail = (uint16_t)((s_tail + 1U) % s_ringCap);
  return 1;
}

void QshDrv_GetStats(QshStat_t out[QSH_NCH])
{
  uint8_t ch;
  for (ch = 0U; ch < QSH_NCH; ch++)
  {
    out[ch].readOk = s_stat[ch].readOk;
    out[ch].crcErr = s_stat[ch].crcErr;
    out[ch].frameErr = s_stat[ch].frameErr;
    out[ch].txBusy = s_stat[ch].txBusy;
    out[ch].cfgAck = s_stat[ch].cfgAck;
    out[ch].cfgFail = s_stat[ch].cfgFail;
    out[ch].zeroAck = s_stat[ch].zeroAck;
    out[ch].ringDrop = s_stat[ch].ringDrop;
  }
}


/* —— 录制模式 API（main 调用）—— */
void QshDrv_RecBegin(void *buf, uint32_t bytes)
{
  s_ring = (QshSample_t *)buf;
  s_ringCap = (uint16_t)(bytes / sizeof(QshSample_t));
  s_head = 0U;
  s_tail = 0U;
  g_recFull = 0U;
  g_recCount = 0U;
  g_recBufAddr = (uint32_t)buf;
  g_recAlgoHold = 1U;
}

void QshDrv_RecEnd(void)
{
  s_ring = s_ringSmall;
  s_ringCap = QSH_RING_N;
  s_head = 0U;
  s_tail = 0U;
  g_recAlgoHold = 0U;
}
