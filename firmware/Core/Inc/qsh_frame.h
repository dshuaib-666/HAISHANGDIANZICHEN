#ifndef __QSH_FRAME_H__
#define __QSH_FRAME_H__

#include "main.h"

/* 单路 RS485 的 MODBUS-RTU 帧接收层：IDLE 中断 + 接收 DMA 断帧。
   帧完成时刻由 DwtTime 盖戳——这是算法对齐的时间基准。 */

#define QSH_FRAME_MAX 32u    /* 内码响应 9 字节；寄存器探针最多 10 寄存器 = 25 字节 */

typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t  rxBuf[QSH_FRAME_MAX];
  uint8_t  frame[QSH_FRAME_MAX];
  volatile uint16_t frameLen;
  volatile uint8_t  framePending;
  volatile uint64_t frameTUs;
  volatile uint32_t overrunCnt;
  volatile uint32_t frameCnt;
} QshFrame_t;

void QshFrame_Init(QshFrame_t *f, UART_HandleTypeDef *huart);
void QshFrame_Start(QshFrame_t *f);
void QshFrame_IdleIRQ(QshFrame_t *f);   /* 在对应 USART IRQ 内调用 */

/* 主循环取帧：返回 1 且填充 out/len/tUs；无帧返回 0 */
int  QshFrame_Take(QshFrame_t *f, uint8_t *out, uint16_t *len, uint64_t *tUs);

#endif
