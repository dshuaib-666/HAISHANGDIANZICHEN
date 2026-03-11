#ifndef __PUBLIC_H
#define __PUBLIC_H
#include "Config.h"
#include "main.h"

uint32_t Software_CRC32(const uint8_t *data, uint32_t length);

u16 GetCrc16(u8 * Data,u16 DateLength);


void Delay_Init(void);

void Delay_us(uint32_t us);


//void delay_us(u32 nus);


#endif

