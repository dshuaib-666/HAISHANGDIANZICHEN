#ifndef __PROCESSDATA_H
#define __PROCESSDATA_H

#include "Config.h"
#include "main.h"
#include <string.h>  // 添加string.h用于memset

// ========== 64字节数据包结构定义 ==========
#pragma pack(1)  // 确保结构体紧密排列，无字节对齐
typedef struct {
    // ADC数据 (4字节)
    u32 adcValue;               
    
    // 9个惯导数据 (36字节)
    float acceleration[3];       // 三轴加速度 (12字节)
    float angularVelocity[3];    // 三轴角速度 (12字节) 
    float eulerAngle[3];         // 欧拉角 (12字节)
    
    // CRC校验 (2字节)
    u16 crc16;                  
    
    // 填充字节 (22字节) - 确保总大小为64字节 (4+36+2+22=64)
    u8 padding[22];             
} DataPacket_64Bytes;
#pragma pack()

// 在C99标准中验证结构体大小 - 用编译时检查替代_Static_assert
#define VERIFY_PACKET_SIZE() typedef char packet_size_check[(sizeof(DataPacket_64Bytes) == 64) ? 1 : -1]
VERIFY_PACKET_SIZE();

// ========== 扇区缓冲区定义 ==========
#define PACKETS_PER_SECTOR 8    // 每个扇区8个数据包
typedef struct {
    DataPacket_64Bytes packets[PACKETS_PER_SECTOR];  // 8个64字节数据包 = 512字节扇区
    u8 currentIndex;            // 当前写入位置 (0-7)
    u8 isFull;                  // 缓冲区是否已满
} SectorBuffer;

// ========== 函数声明 ==========
extern void OnGetDebugUartData(u8* Data,u16 Len);

extern void OnGetCommUartData(u8* Data,u16 Len);

extern int ReadAdc(void);

void TestQuaternionConversion(void);

// PushInsData函数声明
extern void PushInsData(double acceleration[3], double angularVelocity[3], double quaternion[4]);

// CopyInsFifoToAdcUserInsFifo函数声明
extern void CopyInsFifoToAdcUserInsFifo(void);

// ========== 新增函数声明 ==========
//extern void OnAdcDataReady(u32 adcValue);
//extern void InitSectorBuffer(void);
extern void InitInsData(void);

// 砝码扇区分配函数声明 - 只暴露批量计算函数
//extern int CalculateAllWeightSectorAllocations(void);

// 按钮检测函数声明
extern void CheckButtonStatus(void);

// 数码管显示函数声明
extern void Display_SendToDigitTube(float value);
extern void Display_SendBlackScreen(void);

// 数码管闪烁控制函数声明
//extern void Display_UpdateBlinkStatus(void);

// 初始化砝码选择函数声明
//extern void InitWeightSelection(void);

// 强制更新显示函数声明（调试用）
//extern void Display_ForceUpdate(void);

#endif

