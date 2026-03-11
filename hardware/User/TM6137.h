#ifndef __TM6137_H
#define __TM6137_H
#include "main.h"

//#define TM1637_CLK_PIN  SEG_CLK_Pin
//#define TM1637_DIO_PIN  SEG_DATA_Pin

#define TM1637_CLK(x)   x>0? HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin,GPIO_PIN_RESET)
#define TM1637_DIO(x)   x>0? HAL_GPIO_WritePin(SEG_DATA_GPIO_Port, SEG_DATA_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(SEG_DATA_GPIO_Port, SEG_DATA_Pin,GPIO_PIN_RESET)


void TM1637_DisplayChar(unsigned char ch,unsigned char p);

void TM1637_Set_Bright(uint8_t state,uint8_t bright);

void TM1637_Start(void);

void TM1637_Stop(void);





#endif

