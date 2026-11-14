#ifndef __DWT_TIME_H__
#define __DWT_TIME_H__

#include "main.h"

/* DWT 循环计数器驱动的 64 位微秒时钟。
   两通道共用同一时间轴（共模），是比值法对齐的前提。 */

void     DwtTime_Init(void);
uint64_t DwtTime_NowUs(void);   /* IRQ 安全（内部关中断保护累计器） */

#endif
