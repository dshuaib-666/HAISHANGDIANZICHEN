#include "data_uart.h"
#include "dbg_uart.h"
#include "usart.h"
#include <string.h>
#include <stdarg.h>

#define DATA_LINE_MAX 256U

static char s_buf[DATA_LINE_MAX];
static volatile uint8_t s_pending;
static volatile uint32_t s_dropped;

void DataUart_Line(const char *fmt, ...)
{
  va_list ap;

  if (s_pending != 0U)
  {
    s_dropped++;          /* 上一行还没发完：丢行计数（诊断用） */
    return;
  }
  va_start(ap, fmt);
  Dbg_FormatInto(s_buf, sizeof(s_buf), fmt, ap);
  va_end(ap);
  s_buf[strlen(s_buf)] = '\n';
  s_pending = 1U;
}

void DataUart_Flush(void)
{
  if (s_pending == 0U)
  {
    return;
  }
  /* 数码管帧（TX_IT）未完成时 gState 非 READY——数据模式下数码管静默，
     实际无竞争；显示模式下本函数不被调用 */
  if (HAL_UART_GetState(&huart4) != HAL_UART_STATE_READY)
  {
    return;
  }
  if (HAL_UART_Transmit_IT(&huart4, (uint8_t *)s_buf,
                           (uint16_t)(strlen(s_buf) + 1U)) == HAL_OK)
  {
    s_pending = 0U;
  }
}

uint32_t DataUart_Dropped(void)
{
  return s_dropped;
}
