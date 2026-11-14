#ifndef __DBG_UART_H__
#define __DBG_UART_H__

#include "main.h"
#include <stdarg.h>

/* 调试/数据导出串口：USART1 PA9 TX-only @115200。
   完全由本模块自持（直接寄存器初始化，不经 CubeMX /ioc 管理）——
   重新生成 CubeMX 工程不影响本模块。
   用途：M1 数据 CSV 输出（t_us,sensor,code,seq,dup）与状态日志。 */

void Dbg_Init(void);
void Dbg_Line(const char *fmt, ...);   /* 阻塞输出一行（自动补 \n，≤255 字符） */
void Dbg_Raw(const char *s);           /* 原样输出 */

/* 迷你格式化（%s/%u/%d/%ld/%lu）——供其它模块复用 */
void Dbg_FormatInto(char *buf, unsigned int cap, const char *fmt, va_list ap);

#endif
