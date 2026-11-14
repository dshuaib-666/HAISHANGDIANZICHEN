#ifndef __KEYS_H__
#define __KEYS_H__

#include "main.h"

/* 三按键（内部上拉，按下为低，EXTI 下降沿 + 电平轮询）：
   KEY0 = PB13（丝印"测量清零"）：长按 = 预备模式（自检+标定合并）
          进入/中止——**需组合键**：按住 KEY2 不放 + 长按 KEY0
          （出厂/维修级操作，双键确认防误触，清零运行中拒绝）；短按空
   KEY1 = PB14（丝印"参考清零"）：长按 = 清零（双上下文，见 app_zero.h）
          进入/中止——空盘=零点重锚 / 带 500g 标准砝码锁定中=跨度校正
          （预备/录制中拒绝）；短按空
   KEY2 = PB15（丝印"预留/单位"）：组合键修饰（按住不放 + 长按 KEY0
          = 进入预备模式）；单独短按/长按无功能
   短按 <800ms，长按 ≥800ms。2026-10-16 自 PD12/PD10/PD8 平移。 */

#define KEY_N          3U
#define KEY_LONG_MS    800U
#define KEY_DEBOUNCE_MS 15U

typedef enum
{
  KEY_EV_NONE = 0,
  KEY_EV_SHORT,
  KEY_EV_LONG
} KeyEv_t;

void    Keys_Init(void);
void    Keys_Process(void);                 /* 主循环尽快调用（内部用 tick 计时） */
KeyEv_t Keys_Take(uint8_t idx);             /* 取出并清除事件 */
int     Keys_IsPressed(uint8_t idx);        /* 当前电平：1=按下（低）——组合键用 */

#endif
