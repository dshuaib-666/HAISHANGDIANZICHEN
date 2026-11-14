#ifndef __DIGIT_TUBE_H__
#define __DIGIT_TUBE_H__

#include "main.h"

/* UART4 (9600-8-N-1) 数码管显示驱动。
   协议移植自 hardware 项目：整帧为 "$001,<字段>#"，例如 "$001,12.3#"；
   黑屏为 "$001,#"。字段最长 "-9999.9"。

   注意：以下接口均在主循环（非中断）上下文中调用。 */

/* 显示字段缓冲（不含 "$001," 前缀与 "#" 后缀），含结束符。 */
#define DIGIT_TUBE_PAYLOAD_MAX 16U

/* 初始化：上电开机显示 666，并由主循环 DigitTube_Process() 负责发出。 */
void DigitTube_Init(void);

/* 主循环周期性调用：非阻塞发送待显示内容，并按心跳周期重发。 */
void DigitTube_Process(void);

/* 显示一个整数，例如 666 -> "$001,666#"。 */
void DigitTube_ShowInteger(int32_t value);

/* 显示重量（克），保留一位小数，例如 12.34 -> "$001,12.3#"。 */
void DigitTube_SetValue(float gram);

/* 发送黑屏命令 "$001,#"。 */
void DigitTube_ShowBlackScreen(void);

/* 显示 "---"（未就绪占位）。闪烁节奏由调用方控制，本驱动只发内容。 */
void DigitTube_ShowDashes(void);

/* —— 亮度控制（模块确认的亮度帧： "$001,00<n>%"）——
   n = 0~4，4 最亮 / 0 最暗；帧尾用 '%' 与显示帧的 '#' 区分。
   上电由 DigitTube_Init() 自动下发默认 4 档；亮度帧与显示帧之间
   驱动内强制留 30ms 空档（紧贴发送会被模块并帧丢弃）。 */
#define DIGIT_TUBE_BRIGHT_DEFAULT 4U
#define DIGIT_TUBE_BRIGHT_MAX     4U

/* 设置亮度档位（0~4，超范围钳位）并排队下发；同档位重复调用也会重发。 */
void DigitTube_SetBrightness(uint8_t level);

#endif
