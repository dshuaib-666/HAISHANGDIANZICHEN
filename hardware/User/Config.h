#ifndef __CONFIG_H
#define __CONFIG_H

//#include  "cmsis_os.h"
#include <stdio.h>
#include <stdint.h>  // 添加标准整型定义
#define InsFifoSize 20


#define true 1
#define false 0

//#define MY_Debug             //如果定义那么就进行打印

// ========== 深度学习模型开关 ==========
// 默认不启用，避免影响业务运行/资源占用；需要部署模型时再打开。
#define ENABLE_SCALE_MODEL

// 调试：打印真实推理调用频率（约每秒一次；仅在USB连接并配置成功时可见）
#define SCALE_MODEL_DEBUG_FREQ

typedef unsigned char  BYTE;


typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;


// ========== 前置声明 ==========
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;  // UART句柄前置声明


#define MaxUartReceiveBufferSize 4096

#define MaxFilterBufferSize 20

//#define MaxUartReceiveBufferShortSize 1024


#define Task_Priority_DebugUart 2
#define Task_Priority_CommunicationUart 3



#define Cmd_FormatDisk 0x01
#define Cmd_ReadCluster 0x02

#define Cmd_ReadSector 0x03

#define Cmd_AddTestSector 0x04

#define Cmd_TestDeformat 0x05

#define Cmd_GetCurrentSectorPointer 0x06

#define Cmd_GetCurrentIns 0x07

#define Cmd_TestDataPacket 0x08

#define Cmd_StartAdc 0x09

#define Cmd_EndAdc 0x0A

#define Cmd_SetRecordTimesInSeconds 0x0D

#define Cmd_GetRecordTimesInSeconds 0x0E

#define Cmd_SetWeightCellArray 0x0F

#define Cmd_GetWeightCellArray   0x10

#define Cmd_GetWeightCellsAllInfo   0x11

#define Cmd_ResetAllWeightCells     0x12

#define Cmd_SetCurrentWeightIndex   0x13

#define Cmd_GetCurrentWeightIndex   0x14

#define Cmd_GetWeightCellsAllInfoHex   0x15

#define Cmd_ForceUpdateDisplay         0x16



//#define Debug_Ins 1
//#define Adc_Hx711 1





struct Msg_Struct
{
    u32 Msg_Type;
    u32 Msg_SubType;
    u32 Msg_Refer;
    u32 Msg_Refer2;
	u32 Msg_Refer3;
    u32 MsgLength;
    u32 MsgOffset;
    u8* MsgBodyPointer;
};



typedef struct UartReceiveData_ShortStruct
	{
       u8  DataBuffer[MaxUartReceiveBufferSize];
             
       u16 ReceiveLength;			   
    }UartData_ShortStruct;


typedef struct 
{
    // 说明：
    // 当前FIFO保存 10 路惯导相关数据（加速度3 + 角速度3 + 四元素4）
    // 之前这里写成 9，会导致 PushInsData/CopyInsFifoToAdcUserInsFifo 越界写入，可能破坏内存。
    double InsFifo[InsFifoSize][10];
    u8 InsFifo_Pointer;
    /* data */
} InsFifoStruct;

extern double InsAvgData[9];
extern UartData_ShortStruct Uart1_ReceiveData;
extern UartData_ShortStruct Uart2_ReceiveData;

//extern SemaphoreHandle_t  xSemaphore_Uart2;

// ADC控制标志
extern u8 AdcEnabled;

// 当前正在记录的砝码索引 (0-15, 255表示未选择)
extern u8 CurrentWeightIndex;

extern u32 RecordStartSector;
extern u32 RecordEndSector;




extern InsFifoStruct InsFifo;

extern InsFifoStruct AdcUserInsFifo;

// ========== 惯导和ADC相关全局变量 ==========
extern double Acceleration[3];         // 三轴加速度
extern double AngularVelocity[3];      // 三轴角速度
extern double RealaAngle[3];           // 欧拉角
extern uint32_t AdcValue;              // ADC采样值
extern UART_HandleTypeDef* UartDebugHandle2;  // UART调试句柄

extern double WeightFilterBuffer[MaxUartReceiveBufferSize];

extern u16 WeightFilterBufferPointer;

extern u16 FilterSkipCounter;

extern double FilteredrCurrentWeightValue;

//extern xQueueHandle xQueue_FromUart1;



//extern xQueueHandle xQueue_FromUart2;



//void OnUart1GetData(void);
//void OnUart2GetData(void);


// ADC相关宏定义
#define AdcMakeSectorNumberInOneSecond 1.25f  // 每秒生成的扇区数量（浮点数）



#endif

