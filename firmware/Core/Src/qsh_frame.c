#include "qsh_frame.h"
#include "dwt_time.h"
#include <string.h>

void QshFrame_Init(QshFrame_t *f, UART_HandleTypeDef *huart)
{
  f->huart = huart;
  f->frameLen = 0U;
  f->framePending = 0U;
  f->frameTUs = 0U;
  f->overrunCnt = 0U;
  f->frameCnt = 0U;
}

void QshFrame_Start(QshFrame_t *f)
{
  memset((void *)f->rxBuf, 0, sizeof(f->rxBuf));
  memset((void *)f->frame, 0, sizeof(f->frame));
  f->framePending = 0U;

  if (HAL_UART_Receive_DMA(f->huart, f->rxBuf, QSH_FRAME_MAX) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_UART_ENABLE_IT(f->huart, UART_IT_IDLE);
}

void QshFrame_IdleIRQ(QshFrame_t *f)
{
  uint16_t len;

  if ((__HAL_UART_GET_FLAG(f->huart, UART_FLAG_IDLE) == RESET) ||
      (f->huart->hdmarx == NULL))
  {
    return;
  }

  __HAL_UART_CLEAR_IDLEFLAG(f->huart);
  HAL_UART_DMAStop(f->huart);
  len = (uint16_t)(QSH_FRAME_MAX - __HAL_DMA_GET_COUNTER(f->huart->hdmarx));

  if ((len > 0U) && (len <= QSH_FRAME_MAX))
  {
    if (f->framePending != 0U)
    {
      f->overrunCnt++;
    }
    else
    {
      memcpy(f->frame, f->rxBuf, len);
      f->frameLen = len;
      f->frameTUs = DwtTime_NowUs();
      f->framePending = 1U;
      f->frameCnt++;
    }
  }

  memset((void *)f->rxBuf, 0, sizeof(f->rxBuf));
  (void)HAL_UART_Receive_DMA(f->huart, f->rxBuf, QSH_FRAME_MAX);
}

int QshFrame_Take(QshFrame_t *f, uint8_t *out, uint16_t *len, uint64_t *tUs)
{
  if (f->framePending == 0U)
  {
    return 0;
  }
  memcpy(out, (void *)f->frame, f->frameLen);
  *len = f->frameLen;
  *tUs = f->frameTUs;
  __disable_irq();
  f->framePending = 0U;
  __enable_irq();
  return 1;
}
