#include "dbg_uart.h"
#include <string.h>
#include <stdarg.h>

/* USART1: APB2 = 84MHz，PA9 = AF7 TX。 */
#define DBG_BRR_115200  (84000000UL / 115200UL)
#define DBG_LINE_MAX    256

/* 不使用 stdio 的 printf/vsnprintf：本工程未启用 MicroLIB，ARM 标准库
   printf 家族在小栈上实测会跑飞（见 digit_tube.c 同款注意事项）。
   这里自实现仅支持 %s %u %d %ld %lu 的迷你格式化。 */

static void putStr(char **p, char *end, const char *s)
{
  while ((*s != '\0') && (*p < end))
  {
    *(*p)++ = *s++;
  }
}

static void putU32(char **p, char *end, uint32_t v)
{
  char tmp[10];
  uint8_t n = 0U;

  do
  {
    tmp[n++] = (char)('0' + (v % 10U));
    v /= 10U;
  } while ((v != 0U) && (n < sizeof(tmp)));
  while ((n > 0U) && (*p < end))
  {
    *(*p)++ = tmp[--n];
  }
}

static void putI32(char **p, char *end, int32_t v)
{
  uint32_t m;

  if (v < 0)
  {
    if (*p < end)
    {
      *(*p)++ = '-';
    }
    m = (uint32_t)(-(v + 1)) + 1U;
  }
  else
  {
    m = (uint32_t)v;
  }
  putU32(p, end, m);
}

void Dbg_FormatInto(char *buf, unsigned int cap, const char *fmt, va_list ap)
{
  char *p = buf;
  char *end = buf + cap - 1;

  while ((*fmt != '\0') && (p < end))
  {
    if (*fmt != '%')
    {
      *p++ = *fmt++;
      continue;
    }
    fmt++;
    if (*fmt == '%')
    {
      *p++ = *fmt++;
      continue;
    }
    if (*fmt == 'l')
    {
      fmt++;
      if (*fmt == 'u')
      {
        putU32(&p, end, va_arg(ap, uint32_t));
        fmt++;
      }
      else if (*fmt == 'd')
      {
        putI32(&p, end, va_arg(ap, int32_t));
        fmt++;
      }
    }
    else if (*fmt == 'u')
    {
      putU32(&p, end, va_arg(ap, unsigned int));
      fmt++;
    }
    else if (*fmt == 'd')
    {
      putI32(&p, end, va_arg(ap, int));
      fmt++;
    }
    else if (*fmt == 's')
    {
      putStr(&p, end, va_arg(ap, const char *));
      fmt++;
    }
    else
    {
      /* 未知/不支持specifier：原样输出并跳过 */
      if (p < end)
      {
        *p++ = '?';
      }
      if (*fmt != '\0')
      {
        fmt++;
      }
    }
  }
  *p = '\0';
}

void Dbg_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* PA9: 复用推挽 AF7，高速（PA9 属 AFR[1]：引脚 8~15） */
  GPIOA->MODER   = (GPIOA->MODER & ~(3UL << (9U * 2U)))  | (2UL << (9U * 2U));
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3UL << (9U * 2U))) | (3UL << (9U * 2U));
  GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~(0xFUL << ((9U - 8U) * 4U))) |
                   (7UL << ((9U - 8U) * 4U));

  USART1->BRR = DBG_BRR_115200;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE;   /* 仅发送 */
}

void Dbg_Raw(const char *s)
{
  while (*s != '\0')
  {
    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
    }
    USART1->DR = (uint8_t)(*s++);
  }
  while ((USART1->SR & USART_SR_TC) == 0U)
  {
  }
}

void Dbg_Line(const char *fmt, ...)
{
  char buf[DBG_LINE_MAX];
  va_list ap;
  size_t n;

  va_start(ap, fmt);
  Dbg_FormatInto(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  n = strlen(buf);
  buf[n] = '\n';
  Dbg_Raw(buf);
}
