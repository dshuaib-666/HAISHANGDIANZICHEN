#include "keys.h"
#include <string.h>

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} KeyPin_t;

static const KeyPin_t s_pin[KEY_N] =
{
  {GPIOB, GPIO_PIN_13},   /* KEY0: PB13 → MARK_B */
  {GPIOB, GPIO_PIN_14},   /* KEY1: PB14 → MARK_A */
  {GPIOB, GPIO_PIN_15}    /* KEY2: PB15 → 预留   */
};

typedef enum { KS_IDLE = 0, KS_DEBOUNCE, KS_PRESSED } KeySt_t;

typedef struct
{
  KeySt_t state;
  uint32_t t0;
  uint8_t  trig;       /* EXTI 触发请求 */
  uint8_t  firedLong;
  volatile KeyEv_t ev;
} Key_t;

static Key_t s_key[KEY_N];
static uint8_t s_waitRelease;   /* 上电/唤醒复位时按键仍按着：先等全部松开再处理 */

void Keys_Init(void)
{
  memset((void *)s_key, 0, sizeof(s_key));
  /* 按键唤醒 → 整机复位时，唤醒的那个键往往还按着。若直接跑状态机，
     会把它当成一次新按下，800ms 后误触发长按（清零/预备）。这里先
     冻结按键处理，直到三键全部松开。 */
  s_waitRelease = ((Keys_IsPressed(0U) != 0) || (Keys_IsPressed(1U) != 0)
                   || (Keys_IsPressed(2U) != 0)) ? 1U : 0U;
}

void Keys_Process(void)
{
  uint8_t i;
  uint32_t now = HAL_GetTick();
  Key_t *k;

  if (s_waitRelease != 0U)
  {
    if ((Keys_IsPressed(0U) == 0) && (Keys_IsPressed(1U) == 0)
        && (Keys_IsPressed(2U) == 0))
    {
      s_waitRelease = 0U;
    }
    return;
  }

  for (i = 0U; i < KEY_N; i++)
  {
    k = &s_key[i];
    switch (k->state)
    {
      case KS_IDLE:
        if (k->trig != 0U)
        {
          k->trig = 0U;
          k->t0 = now;
          k->state = KS_DEBOUNCE;
        }
        break;

      case KS_DEBOUNCE:
        if ((now - k->t0) >= KEY_DEBOUNCE_MS)
        {
          if (HAL_GPIO_ReadPin(s_pin[i].port, s_pin[i].pin) == GPIO_PIN_RESET)
          {
            k->state = KS_PRESSED;
            k->t0 = now;            /* 按下计时起点 */
            k->firedLong = 0U;
          }
          else
          {
            k->state = KS_IDLE;     /* 抖动 */
          }
        }
        break;

      case KS_PRESSED:
        if ((k->firedLong == 0U) && ((now - k->t0) >= KEY_LONG_MS))
        {
          k->firedLong = 1U;
          k->ev = KEY_EV_LONG;
        }
        if (HAL_GPIO_ReadPin(s_pin[i].port, s_pin[i].pin) != GPIO_PIN_RESET)
        {
          if (k->firedLong == 0U)
          {
            k->ev = KEY_EV_SHORT;
          }
          k->state = KS_IDLE;
        }
        break;

      default:
        k->state = KS_IDLE;
        break;
    }
  }
}

KeyEv_t Keys_Take(uint8_t idx)
{
  KeyEv_t e;

  if (idx >= KEY_N)
  {
    return KEY_EV_NONE;
  }
  __disable_irq();
  e = s_key[idx].ev;
  s_key[idx].ev = KEY_EV_NONE;
  __enable_irq();
  return e;
}

int Keys_IsPressed(uint8_t idx)
{
  if (idx >= KEY_N)
  {
    return 0;
  }
  return (HAL_GPIO_ReadPin(s_pin[idx].port, s_pin[idx].pin)
          == GPIO_PIN_RESET) ? 1 : 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case GPIO_PIN_13: s_key[0].trig = 1U; break;
    case GPIO_PIN_14: s_key[1].trig = 1U; break;
    case GPIO_PIN_15: s_key[2].trig = 1U; break;
    default: break;
  }
}
