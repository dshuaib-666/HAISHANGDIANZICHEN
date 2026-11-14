#include "dwt_time.h"

#define CYCLES_PER_US  (SystemCoreClock / 1000000U)

static volatile uint32_t s_lastCyc;
static volatile uint64_t s_accCyc;      /* 64 位周期累计 */
static volatile uint8_t  s_ready;

void DwtTime_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0U;
  s_lastCyc = 0U;
  s_accCyc = 0U;
  s_ready = 1U;
}

uint64_t DwtTime_NowUs(void)
{
  uint64_t acc;
  uint32_t primask;

  if (s_ready == 0U)
  {
    return 0U;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  {
    uint32_t now = DWT->CYCCNT;
    s_accCyc += (uint32_t)(now - s_lastCyc);   /* 无符号回绕安全 */
    s_lastCyc = now;
    acc = s_accCyc;
  }
  __set_PRIMASK(primask);
  return acc / CYCLES_PER_US;
}
