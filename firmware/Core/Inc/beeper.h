#ifndef __BEEPER_H
#define __BEEPER_H

/* ==== 蜂鸣器（2026-10-22 合入，硬件组 firmware_10_22 变体正式化）====
   硬件：无源蜂鸣器，TIM3_CH1/PA6，2kHz PWM（谐振频率，勿改），
   高有效（tim.c: PWM1 + TIM_OCPOLARITY_HIGH）——CCR=249（≈50% 占空比）
   最大音量，CCR=0 引脚恒低=静音。旧注释"低有效/CCR=0 引脚恒高"与
   tim.c 的实际配置不符，按实际行为已更正。

   音量：无源蜂鸣器响度 ∝ 驱动交流幅度 2·d·(1−d)·VDD（d=占空比），
   CCR 即音量旋钮（0~249，249=0dB，200≈−0.4dB，150≈−1.5dB，75≈−6dB，
   40≈−10dB；<40 音色发沙不建议）。BEEPER_LEVEL 默认 200（≈0dB）：
   2026-11-14 现场反馈"正常叫声偏小"，由 75（−5.8dB）提高，驱动交流
   幅度约 ×1.9（+5.5dB）。调试器 Watch 改 s_level 试听不可靠（static
   非 volatile，-O3 可能被优化），要调音量请改此宏重编译。

   语义：声音只确认"操作已被收到/完成/拒绝"，不替代数码管；
   运行期常态静音（称量/读数刷新/录制模式不响）；上电静音。
   驱动：主循环 tick 轮询（与 keys/digit_tube 同风格，零新中断——
   本工程曾因新增外设配置踩过栈回退坑，对中断路径保持谨慎）。
   音型约定：1短=段推进提示；2短=成功；3短=软失败；1长=硬错误。 */

#include "main.h"
#include <stdint.h>

#define BEEPER_PERIOD   499U    /* PWM ARR（与 tim.c 一致，勿改）*/
#define BEEPER_LEVEL    200U    /* 默认音量 CCR：0~249。200≈−0.4dB，已在
                                   50% 占空比最大音量附近；再高（>249）反而
                                   变小，等于近似直流 */
#if (BEEPER_LEVEL > (BEEPER_PERIOD / 2U))
#error "BEEPER_LEVEL exceeds the 50% duty limit (BEEPER_PERIOD/2 = 249)"
#endif

/* 上电初始化：PWM 启动后立即 CCR=0（静音——变体的开机自检信标
   BuzzerTest_Process 已删，上电不再响）*/
void Beeper_Init(void);

/* 入队 n 声（忙时新请求被丢弃——事件沿只发一次，不排队堆积）*/
void Beeper_Beep(uint8_t n, uint16_t on_ms, uint16_t off_ms);

/* 预定义音型（main.c 事件沿直接调用）*/
void Beeper_Tick(void);     /* 1×40ms  极短（段推进/按键确认）*/
void Beeper_Ok(void);       /* 2×80ms  成功（清零/跨度/prep 提交）*/
void Beeper_Fail(void);     /* 3×80ms  软失败（9100/9200/9300）*/
void Beeper_Error(void);    /* 1×400ms 硬错误（7777）*/

/* 音量（CCR 0~249，>249 钳到 249=PERIOD/2；调试器可改 s_level 等效）*/
void Beeper_SetLevel(uint16_t ccr);

/* 主循环每轮调用（HAL_GetTick 推进，静音态零开销）*/
void Beeper_Process(void);

#endif
