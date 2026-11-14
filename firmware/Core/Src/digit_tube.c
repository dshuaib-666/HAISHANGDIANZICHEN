#include "digit_tube.h"

/* USER CODE BEGIN Includes */
#include "usart.h"
#include <string.h>
/* USER CODE END Includes */

/* 本文件刻意不使用 stdio.h / snprintf。
 *
 * 本工程未启用 MicroLIB（useUlib=0），使用的是 ARM 标准 C 库。栈 4KB
 * （Stack_Size=0x1000——曾被 CubeMX 再生成回退到 0x400 引发 HardFault，
 * 已回写 .ioc 防再犯），标准库 printf 家族仍有风险，十进制格式化全部
 * 自实现，只依赖 string.h 的 memcpy/strcpy/strcat/strlen（不需要堆，
 * 也不需要 semihosting）。 */

/* 与 hardware 项目一致的节流参数：
   20Hz 刷新节拍，9600bps 下每帧约 10ms，留足余量；
   数值未变化时按心跳周期重发，防止数码管上电较慢或偶发丢帧后不显示。 */
#define DIGIT_TUBE_MIN_PERIOD_MS 50U
#define DIGIT_TUBE_KEEPALIVE_MS  300U

/* 亮度/配置帧与显示帧之间强制留出的空闲间隔：部分数码管模块的帧解析
   靠字符间空闲复位，紧贴显示帧发配置帧会被并进前一帧而丢弃——实测
   亮度不生效的常见原因。显示刷新 5Hz，留 30ms 空档不影响刷新。 */
#define DIGIT_TUBE_TX_GAP_MS        30U

/* 亮度指令帧（模块确认的格式）：
     显示数据： "$001,<数据>#"    帧尾 '#'
     亮度设置： "$001,00<n>%"     帧尾 '%'（n = 0~4，4 最亮 / 0 最暗）
   上电默认下发 4 档。 */
#define DIGIT_TUBE_BRIGHT_PREFIX    "$001,00"
#define DIGIT_TUBE_BRIGHT_TERM      '%'

/* 亮度指令重发周期：帧丢失（上电抢跑/干扰）后 2 s 内自动补上 */
#define DIGIT_TUBE_BRIGHT_RESEND_MS 2000U

/* 未发送前默认显示 0，避免首帧字段为空。 */
static char digit_tube_payload[DIGIT_TUBE_PAYLOAD_MAX] = "0";
static volatile uint8_t digit_tube_pending = 1U;
static uint8_t digit_tube_first = 1U;
static uint32_t digit_tube_last_tick = 0U;
static char digit_tube_frame[DIGIT_TUBE_PAYLOAD_MAX + 8U];

/* 亮度状态：上电先下发默认 4 档 */
static uint8_t  digit_tube_bright = DIGIT_TUBE_BRIGHT_DEFAULT;
static uint8_t  digit_tube_bright_pending = 1U;
static uint32_t digit_tube_bright_tick = 0U;

/* 无符号十进制转字符串，text 至少需要 11 字节（10 位数字 + 结束符）。 */
static void DigitTube_UIntToStr(uint32_t value, char *text)
{
  char digits[10];
  uint8_t count = 0U;

  do
  {
    digits[count] = (char)('0' + (value % 10U));
    count++;
    value /= 10U;
  } while (value != 0U);

  while (count > 0U)
  {
    count--;
    *text = digits[count];
    text++;
  }
  *text = '\0';
}

/* 有符号十进制转字符串，text 至少需要 12 字节。 */
static void DigitTube_IntToStr(int32_t value, char *text)
{
  uint32_t magnitude;
  uint8_t index = 0U;

  if (value < 0)
  {
    text[index] = '-';
    index++;
    /* 取绝对值：这样写可避免 INT32_MIN 取负溢出。 */
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)value;
  }

  DigitTube_UIntToStr(magnitude, &text[index]);
}

static void DigitTube_SetPayload(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  (void)strncpy(digit_tube_payload, text, DIGIT_TUBE_PAYLOAD_MAX - 1U);
  digit_tube_payload[DIGIT_TUBE_PAYLOAD_MAX - 1U] = '\0';
  digit_tube_pending = 1U;
}

void DigitTube_Init(void)
{
  digit_tube_last_tick = 0U;
  digit_tube_first = 1U;
  digit_tube_bright = DIGIT_TUBE_BRIGHT_DEFAULT;
  digit_tube_bright_pending = 1U;   /* 上电先下发默认亮度 4 档 */
  digit_tube_bright_tick = 0U;
  DigitTube_ShowInteger(666);

  /* 上电后尽快发出第一帧（亮度帧优先），不等待刷新节拍。 */
  DigitTube_Process();
}

/* 组帧并发送亮度命令 "$001,00<n>%"；返回 1 = 已交给 UART（异步发送中）。 */
static uint8_t DigitTube_SendBrightness(void)
{
  uint8_t n = (uint8_t)strlen(DIGIT_TUBE_BRIGHT_PREFIX);

  if (((uint16_t)n + 3U) > sizeof(digit_tube_frame))
  {
    return 0U;
  }

  (void)memcpy(digit_tube_frame, DIGIT_TUBE_BRIGHT_PREFIX, n);
  digit_tube_frame[n] = (char)('0' + digit_tube_bright);
  digit_tube_frame[n + 1U] = DIGIT_TUBE_BRIGHT_TERM;
  digit_tube_frame[n + 2U] = '\0';

  /* 与显示帧共用静态缓冲：HAL_UART_Transmit_IT 异步发送，必须先完成再改。 */
  if (HAL_UART_Transmit_IT(&huart4, (uint8_t *)digit_tube_frame,
                           (uint16_t)(n + 2U)) != HAL_OK)
  {
    return 0U;
  }

  digit_tube_bright_pending = 0U;
  digit_tube_bright_tick = HAL_GetTick();
  return 1U;
}

void DigitTube_Process(void)
{
  uint32_t now;
  char payload[DIGIT_TUBE_PAYLOAD_MAX];

  now = HAL_GetTick();

  /* 亮度命令优先：上电/改档后立即发，另按 2 s 周期重发防丢帧。
     发完让出总线，显示帧下一轮再发（pending 一直保留）。 */
  if ((digit_tube_bright_pending != 0U) ||
      ((uint32_t)(now - digit_tube_bright_tick) >= DIGIT_TUBE_BRIGHT_RESEND_MS))
  {
    if (HAL_UART_GetState(&huart4) != HAL_UART_STATE_READY)
    {
      return;
    }
    if ((uint32_t)(now - digit_tube_last_tick) < DIGIT_TUBE_TX_GAP_MS)
    {
      return;                       /* 与显示帧留空档，避免被并帧 */
    }
    if (DigitTube_SendBrightness() != 0U)
    {
      digit_tube_last_tick = now;
    }
    return;
  }

  /* 显示帧之间也统一留空档：配置帧（亮度）之后的第一帧显示不能紧贴着发。 */
  if ((uint32_t)(now - digit_tube_last_tick) < DIGIT_TUBE_TX_GAP_MS)
  {
    return;
  }

  /* 无新内容时按心跳周期重发；有新内容时按刷新节拍限速（首帧不限速）。 */
  if (digit_tube_pending == 0U)
  {
    if ((uint32_t)(now - digit_tube_last_tick) < DIGIT_TUBE_KEEPALIVE_MS)
    {
      return;
    }
  }
  else if ((digit_tube_first == 0U) &&
           ((uint32_t)(now - digit_tube_last_tick) < DIGIT_TUBE_MIN_PERIOD_MS))
  {
    return;
  }

  /* 上一帧仍在发送中则保留 pending，下一轮再发。 */
  if (HAL_UART_GetState(&huart4) != HAL_UART_STATE_READY)
  {
    return;
  }

  (void)memcpy(payload, digit_tube_payload, sizeof(payload));

  /* 组帧："$001," + 字段 + "#" */
  (void)strcpy(digit_tube_frame, "$001,");
  (void)strcat(digit_tube_frame, payload);
  (void)strcat(digit_tube_frame, "#");

  /* frame 是静态缓冲：HAL_UART_Transmit_IT 是异步发送，发送完成前必须保持有效。 */
  if (HAL_UART_Transmit_IT(&huart4, (uint8_t *)digit_tube_frame,
                           (uint16_t)strlen(digit_tube_frame)) == HAL_OK)
  {
    digit_tube_last_tick = now;
    digit_tube_first = 0U;
    digit_tube_pending = 0U;
  }
}

void DigitTube_ShowInteger(int32_t value)
{
  char text[DIGIT_TUBE_PAYLOAD_MAX];

  DigitTube_IntToStr(value, text);
  DigitTube_SetPayload(text);
}

void DigitTube_SetValue(float gram)
{
  char text[DIGIT_TUBE_PAYLOAD_MAX];
  float value = gram;
  int32_t raw_v100;
  uint32_t abs_v10;
  uint32_t ipart;
  uint32_t frac1;
  uint8_t index = 0U;

  /* 显示范围限制在 6 位数码管可表达的 -9999.9 .. 9999.9 */
  if (value > 9999.9f)
  {
    value = 9999.9f;
  }
  else if (value < -9999.9f)
  {
    value = -9999.9f;
  }

  /* 0.01g 单位四舍五入后取绝对值，再降到 0.1g 显示精度。 */
  raw_v100 = (int32_t)(value * 100.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
  abs_v10 = (uint32_t)(((raw_v100 < 0) ? -raw_v100 : raw_v100) + 5) / 10U;
  ipart = abs_v10 / 10U;
  frac1 = abs_v10 % 10U;
  if (ipart > 9999U)
  {
    ipart = 9999U;
    frac1 = 9U;
  }

  /* 含 |值| < 0.05g 的情况：显示 "0.0" 而不是 "-0.0"。 */
  if ((raw_v100 < 0) && (abs_v10 > 0U))
  {
    text[index] = '-';
    index++;
  }

  DigitTube_UIntToStr(ipart, &text[index]);
  index = (uint8_t)strlen(text);
  text[index] = '.';
  index++;
  DigitTube_UIntToStr(frac1, &text[index]);

  DigitTube_SetPayload(text);
}

void DigitTube_ShowBlackScreen(void)
{
  DigitTube_SetPayload("");
}

void DigitTube_ShowDashes(void)
{
  DigitTube_SetPayload("---");
}

void DigitTube_SetBrightness(uint8_t level)
{
  if (level > DIGIT_TUBE_BRIGHT_MAX)
  {
    level = DIGIT_TUBE_BRIGHT_MAX;
  }
  digit_tube_bright = level;
  digit_tube_bright_pending = 1U;   /* 同档位重复调用也重发（确认/唤醒场景） */
}
