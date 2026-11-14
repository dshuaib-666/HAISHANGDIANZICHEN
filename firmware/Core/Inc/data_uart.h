#ifndef __DATA_UART_H__
#define __DATA_UART_H__

#include "main.h"

/* 数据导出通道：复用数码管的 UART4（PA0/PA1，9600）。
   板上无 PA9 引出线时的 M1 数据采集路径——长按 KEY1/PB14 进入数据
   模式后，USB-TTL 接数码管接口即可录制 CSV。非阻塞（TX_IT + 单行排队）。 */

void     DataUart_Line(const char *fmt, ...);  /* 排队一行（≤255 字符，忙时丢弃计数） */
void     DataUart_Flush(void);                  /* 主循环尽快调用 */
uint32_t DataUart_Dropped(void);                /* 因忙丢弃的行数（诊断） */

#endif
