#include "beeper.h"
#include "tim.h"

/* ==== 蜂鸣器实现：见 beeper.h 头注 ==== */

static uint16_t s_level = BEEPER_LEVEL;   /* 音量 CCR（调试器可改）*/
static uint8_t  s_left;                   /* 剩余声数 */
static uint16_t s_on_ms, s_off_ms;        /* 本组音型参数 */
static uint32_t s_t_edge;                 /* 当前相位起点 */
static uint8_t  s_on;                     /* 1=发声相位 */

static void out(uint8_t on)
{
  s_on = on;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                        (on != 0U) ? s_level : 0U);
}

void Beeper_Init(void)
{
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  out(0U);                                /* 上电静音（CCR=0 引脚恒高）*/
}

void Beeper_Beep(uint8_t n, uint16_t on_ms, uint16_t off_ms)
{
  if (n == 0U || s_left != 0U) return;    /* 忙时丢弃：事件沿不堆积 */
  s_left = n;
  s_on_ms = on_ms;
  s_off_ms = off_ms;
  s_t_edge = HAL_GetTick();
  out(1U);
}

void Beeper_Tick(void)  { Beeper_Beep(1U, 40U, 40U); }
void Beeper_Ok(void)    { Beeper_Beep(2U, 80U, 80U); }
void Beeper_Fail(void)  { Beeper_Beep(3U, 80U, 80U); }
void Beeper_Error(void) { Beeper_Beep(1U, 400U, 100U); }

uint8_t Beeper_Busy(void)
{
  return (s_left != 0U) ? 1U : 0U;
}

void Beeper_SetLevel(uint16_t ccr)
{
  if (ccr > BEEPER_PERIOD / 2U) ccr = BEEPER_PERIOD / 2U;
  s_level = ccr;
  if (s_on != 0U) out(1U);                /* 发声中改音量即刻生效 */
}

void Beeper_Process(void)
{
  uint32_t now;

  if (s_left == 0U) return;               /* 静音态零开销 */
  now = HAL_GetTick();
  if (s_on != 0U)
  {
    if ((now - s_t_edge) >= s_on_ms)
    {
      s_left--;
      if (s_left == 0U)
      {
        out(0U);
        return;
      }
      s_t_edge = now;
      out(0U);
    }
  }
  else
  {
    if ((now - s_t_edge) >= s_off_ms)
    {
      s_t_edge = now;
      out(1U);
    }
  }
}
