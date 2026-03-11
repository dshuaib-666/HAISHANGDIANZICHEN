#ifndef __SDMANAGER_H
#define __SDMANAGER_H

#include "Config.h"
#include "stm32f4xx_hal.h"

// 记录数据有效性标志定义
#define RecordDataValidFlag 0x01
#define RecordDataInvalidFlag 0x00

#define ClusterSectorNumber 8
#define SectorSize 512
#define ClusterSize  4096

#define DataStartClusterIndex 2

#define FatFormatFlagByte 0xAA

#define FatUsedFlagByte 0x00

#define DataStartSectorIndex 0x00000008


extern SD_HandleTypeDef*  SdHandle;

extern UART_HandleTypeDef* UartDebugHandle;



extern void Sd_Format(void);

extern void Sd_ReadCluster(u16 ClusterIndex);

extern void Sd_ReadSector(u32 SectorIndex);

extern void Sd_Ini(void);

extern void Sd_AddSectorData(u8* Data);


extern void Sd_DeFormat(void);

// 设置RecordCell数组中指定索引的Weight值，并设置RecordDataValid为无效
extern void Sd_SetRecordCellWeight(u8 index, int16_t weight);

// 获取RecordCell数组中指定索引的Weight值
extern int16_t Sd_GetRecordCellWeight(u8 index);

// 获取RecordCell数组中指定索引的RecordDataValid值
extern u8 Sd_GetRecordCellValid(u8 index);

// 设置RecordCell数组中指定索引的RecordDataValid值
extern void Sd_SetRecordCellValid(u8 index, u8 valid);

// 设置记录时间间隔（秒）
extern void Sd_SetRecordTimesInSeconds(u32 recordTimes);

// 获取记录时间间隔（秒）
extern u32 Sd_GetRecordTimesInSeconds(void);

// 扇区分配相关函数
extern u32 Sd_GetRecordStartSector(u8 index);
extern void Sd_SetRecordStartSector(u8 index, u32 startSector);
extern u32 Sd_GetMaxSectorNumber(u8 index);
extern void Sd_SetMaxSectorNumber(u8 index, u32 maxSectorNumber);
extern u32 Sd_GetRealRecordSectorNumber(u8 index);
extern void Sd_SetRealRecordSectorNumber(u8 index, u32 realRecordSectorNumber);

extern u32 CurrentSectorPointer;

// 输出所有RecordCell的十六进制数据，类似Cmd_ReadSector格式
extern void Sd_GetWeightCellsAllInfoHex(void);

#endif 


