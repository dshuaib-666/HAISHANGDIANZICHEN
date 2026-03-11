#include "SdManager.h"
#include "stm32f4xx_hal.h"
//#include "cmsis_os.h"
#include "string.h"



#define FomatedFlagForSram 0xAA55AA55
#define RecordCellSize 16
//#define AdcMakeSectorNumberInOnSecond 1.25

typedef struct {
    
   volatile  int16_t Weight;  // 2字节
   volatile  uint8_t RecordDataValid;  // 1字节
   volatile  uint8_t Pad; // 1字节
   volatile  uint32_t RecordStartSector;  // 4字节 - 记录开始扇区
   volatile  uint32_t MaxSectorNumber;  // 4字节 - 该砝码分配的最大扇区数量
   volatile  uint32_t RealRecordSectorNumber;  // 4字节 - 实际记录的扇区数量

} RecordCellStruct;



typedef struct {
    //uint8_t  flag;     // 1字节
   volatile  uint32_t CurrentSectorPointer;  // 4字节
   volatile  uint32_t FormatFlag; // 4字节
   volatile  uint32_t RecordTimesInSeconds;  // 4字节
   volatile  RecordCellStruct RecordCell[RecordCellSize]; // 16*20字节 (每个RecordCellStruct现在是20字节)

} SdHHeadData;

#define BackupSramBase_SdHead  ((volatile SdHHeadData*)BKPSRAM_BASE)





SD_HandleTypeDef*  SdHandle;

UART_HandleTypeDef* UartDebugHandle;

u32 CurrentSectorPointer;


DMA_HandleTypeDef hdma_sdio;

//u8 BufferForRead[ClusterSize] ;

static u8 BufferForRead[SectorSize] __attribute__((aligned(4))); // 4字节对齐
static u8 BufferForWrite[SectorSize] __attribute__((aligned(4))); // 4字节对齐



//---Ini backup ram

void IniBackupSram()
{

	__HAL_RCC_PWR_CLK_ENABLE(); 		 // 使能电源时钟
    HAL_PWR_EnableBkUpAccess();		   // 解除备份域写保护
	__HAL_RCC_BKPSRAM_CLK_ENABLE();	   // 使能后备SRAM时钟
	HAL_PWREx_EnableBkUpReg();		   // 启用备份稳压器（关键！）
	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_BRR)); // 等待稳压器就绪



}



void Sd_Format(void) {

	BackupSramBase_SdHead->CurrentSectorPointer=0x00;
	//BackupSramBase_SdHead->MaxSectorNumber=0x10;
	BackupSramBase_SdHead->FormatFlag=FomatedFlagForSram;

	CurrentSectorPointer=BackupSramBase_SdHead->CurrentSectorPointer;


	BackupSramBase_SdHead->RecordTimesInSeconds=3600;

	for(int i=0;i<=RecordCellSize-1;i++)
	{
		BackupSramBase_SdHead->RecordCell[i].Weight=0;
		BackupSramBase_SdHead->RecordCell[i].RecordDataValid=RecordDataInvalidFlag;
		BackupSramBase_SdHead->RecordCell[i].Pad=0;
		BackupSramBase_SdHead->RecordCell[i].RecordStartSector=0;
		BackupSramBase_SdHead->RecordCell[i].MaxSectorNumber=0;
		BackupSramBase_SdHead->RecordCell[i].RealRecordSectorNumber=0;
	}
	
	/*HAL_StatusTypeDef status;
		

	for(int i=0;i<=SectorSize-1;i++)

	BufferForWrite[i]=FatUsedFlagByte;

	CurrentSectorPointer=DataStartSectorIndex;

	//printf("%08X\r\n",CurrentSectorPointer);
	//memcpy(BufferForWrite,(u8*)CurrentSectorPointer,4);

	BufferForWrite[0]=CurrentSectorPointer&0xFF;

	BufferForWrite[1]=(CurrentSectorPointer>>8)&0xFF;

	BufferForWrite[2]=(CurrentSectorPointer>>16)&0xFF;

	BufferForWrite[3]=(CurrentSectorPointer>>24)&0xFF;

	BufferForWrite[4]=FatFormatFlagByte;


	//for(int i=0;i<=16-1;i++)
	
	//{
		status = HAL_SD_WriteBlocks(SdHandle, BufferForWrite, 0, 1, HAL_MAX_DELAY);
		if (status != HAL_OK) 
			{
				printf("format fat0 write error  \r\n");
			}*/

		//osDelay(10);
	//}

/*	for(int i=0;i<=16-1;i++)
		{
		//
		if (HAL_SD_ReadBlocks(SdHandle, BufferForRead, 0, 1, HAL_MAX_DELAY) != HAL_OK) 
			{
        		printf("read cluster error\r\n");
        		printf("SD Status: %d\r\n", HAL_SD_GetCardState(SdHandle));
    		} 
		else 
			{
        		//printf("read success\r\n");
         		HAL_UART_Transmit(UartDebugHandle,BufferForRead,SectorSize,HAL_MAX_DELAY);
        	}
		}
	
 */
}


void Sd_ReadCluster(u16 ClusterIndex){
    u16 SectorIndex = ClusterIndex * 8;
   // printf("read data\r\n");
    for(int i=0;i<=8-1;i++)
    {
		// 使用轮询模式读取
    	if (HAL_SD_ReadBlocks(SdHandle, BufferForRead, SectorIndex+i, 1, HAL_MAX_DELAY) != HAL_OK) 
			{
     		   	printf("read cluster error\r\n");
        		printf("SD Status: %d\r\n", HAL_SD_GetCardState(SdHandle));
    		} 
		else{
        		//printf("read success\r\n");
         		HAL_UART_Transmit(UartDebugHandle,BufferForRead,SectorSize,HAL_MAX_DELAY);
    		}

	}
}

void Sd_ReadSector(u32 SectorIndex)
{
	// 读取扇区数据并以十六进制格式输出
	if (HAL_SD_ReadBlocks(SdHandle, BufferForRead, SectorIndex, 1, HAL_MAX_DELAY) == HAL_OK) 
	{
		//printf("Sector %lu data (512 bytes):\r\n", SectorIndex);
		
		// 将512字节的扇区数据转换为十六进制字符串输出，空格分隔，大写字母
		for(int i = 0; i < SectorSize; i++) {
			printf("%02X", BufferForRead[i]);
			
			// 每个字节后添加空格，但最后一个字节不添加
			if(i < SectorSize - 1) {
				printf(" ");
			}
			
			// 每16个字节换一行，便于阅读
			/*if((i + 1) % 16 == 0) {
				printf("\r\n");
			}*/
		}
		
		// 如果最后一行不满16个字节，添加换行
		/*if(SectorSize % 16 != 0) {
			printf("\r\n");
		}*/
		
		//printf("Sector %lu read completed\r\n", SectorIndex);
	}
	else
	{
		printf("Error: Failed to read sector %lu\r\n", SectorIndex);
	}
}


void Sd_Ini(void)
{

	IniBackupSram();
	if(BackupSramBase_SdHead->FormatFlag!=FomatedFlagForSram)
	{
		Sd_Format();
	 	printf("Ini format sd card\r\n");

	}
	else
	{

	  printf(" Sd card have been format\r\n");

	}
	//--Get CurrentSectorPointer
	CurrentSectorPointer=BackupSramBase_SdHead->CurrentSectorPointer;



	/*u8 FormatFlag=0;

	if (HAL_SD_ReadBlocks(SdHandle, BufferForRead, 0, 1, HAL_MAX_DELAY) != HAL_OK) 
			{
				printf("read cluster error\r\n");
				printf("SD Status: %d\r\n", HAL_SD_GetCardState(SdHandle));
			} 
		else{
					//printf("read success\r\n");
				//HAL_UART_Transmit(UartDebugHandle,BufferForRead,SectorSize,HAL_MAX_DELAY);

				CurrentSectorPointer=
				(BufferForRead[3]<<24)|(BufferForRead[2]<<16)|(BufferForRead[1]<<8)|BufferForRead[0];

				FormatFlag=BufferForRead[4];

				if(FormatFlag!=FatFormatFlagByte)
					{
					  Sd_Format();
					  printf("Ini format sd card\r\n");
				}
				else
				{
					
				    printf(" Sd card have been format\r\n");

				}

				printf("Current sector index is %d\r\n",CurrentSectorPointer);
		}*/




}


void Sd_AddSectorData(u8* Data)
{

	HAL_StatusTypeDef status;


	for(int i=0;i<=SectorSize-1;i++)
		BufferForWrite[i]=Data[i];

	
	//status = HAL_SD_WriteBlocks(SdHandle, BufferForWrite, CurrentSectorPointer, 1, HAL_MAX_DELAY);
	status =HAL_SD_WriteBlocks_DMA(SdHandle, BufferForWrite, CurrentSectorPointer, 1);


		if (status != HAL_OK) 
			{
				printf("format fat0 write error  \r\n");
			}	

//	osDelay(5);
	CurrentSectorPointer++;

	BackupSramBase_SdHead->CurrentSectorPointer=
		CurrentSectorPointer;

	printf("write data to sector %d\r\n",(CurrentSectorPointer-1));

	/*
	for(int i=0;i<=SectorSize-1;i++)
	
		BufferForWrite[i]=FatUsedFlagByte;
	
		BufferForWrite[0]=CurrentSectorPointer&0xFF;
		
		BufferForWrite[1]=(CurrentSectorPointer>>8)&0xFF;
	
		BufferForWrite[2]=(CurrentSectorPointer>>16)&0xFF;
	
		BufferForWrite[3]=(CurrentSectorPointer>>24)&0xFF;
	
		BufferForWrite[4]=FatFormatFlagByte;
		
		status = HAL_SD_WriteBlocks(SdHandle, BufferForWrite, 0, 1, HAL_MAX_DELAY);
		if (status != HAL_OK) 
			{
					printf("format fat0 write error  \r\n");
			}
		printf("Sector Add Current Sector=%d\r\n",CurrentSectorPointer);*/


}



void Sd_DeFormat(void)
{
	BackupSramBase_SdHead->FormatFlag=0;
	printf("Sd card deformat\r\n");
}

// 设置RecordCell数组中指定索引的Weight值，并设置RecordDataValid为无效
void Sd_SetRecordCellWeight(u8 index, int16_t weight)
{
    if(index < RecordCellSize) {
        BackupSramBase_SdHead->RecordCell[index].Weight = weight;
        BackupSramBase_SdHead->RecordCell[index].RecordDataValid = RecordDataInvalidFlag;
        BackupSramBase_SdHead->RecordCell[index].Pad = 0;
        BackupSramBase_SdHead->RecordCell[index].RecordStartSector = 0;
        BackupSramBase_SdHead->RecordCell[index].MaxSectorNumber = 0;
        BackupSramBase_SdHead->RecordCell[index].RealRecordSectorNumber = 0;
    }
}

// 获取RecordCell数组中指定索引的Weight值
int16_t Sd_GetRecordCellWeight(u8 index)
{
    if(index < RecordCellSize) {
        return BackupSramBase_SdHead->RecordCell[index].Weight;
    }
    return 0;
}

// 获取RecordCell数组中指定索引的RecordDataValid值
u8 Sd_GetRecordCellValid(u8 index)
{
    if(index < RecordCellSize) {
        return BackupSramBase_SdHead->RecordCell[index].RecordDataValid;
    }
    return RecordDataInvalidFlag;
}

// 设置RecordCell数组中指定索引的RecordDataValid值
void Sd_SetRecordCellValid(u8 index, u8 valid)
{
    if(index < RecordCellSize) {
        BackupSramBase_SdHead->RecordCell[index].RecordDataValid = valid;
    }
}

// 设置记录时间间隔（秒）
void Sd_SetRecordTimesInSeconds(u32 recordTimes)
{
    BackupSramBase_SdHead->RecordTimesInSeconds = recordTimes;
}

// 获取记录时间间隔（秒）
u32 Sd_GetRecordTimesInSeconds(void)
{
    return BackupSramBase_SdHead->RecordTimesInSeconds;
}

// 获取指定索引的RecordStartSector值
u32 Sd_GetRecordStartSector(u8 index)
{
    if(index < RecordCellSize) {
        return BackupSramBase_SdHead->RecordCell[index].RecordStartSector;
    }
    return 0;
}

// 设置指定索引的RecordStartSector值
void Sd_SetRecordStartSector(u8 index, u32 startSector)
{
    if(index < RecordCellSize) {
        BackupSramBase_SdHead->RecordCell[index].RecordStartSector = startSector;
    }
}

// 获取指定索引的MaxSectorNumber值
u32 Sd_GetMaxSectorNumber(u8 index)
{
    if(index < RecordCellSize) {
        return BackupSramBase_SdHead->RecordCell[index].MaxSectorNumber;
    }
    return 0;
}

// 设置指定索引的MaxSectorNumber值
void Sd_SetMaxSectorNumber(u8 index, u32 maxSectorNumber)
{
    if(index < RecordCellSize) {
        BackupSramBase_SdHead->RecordCell[index].MaxSectorNumber = maxSectorNumber;
    }
}

// 获取指定索引的RealRecordSectorNumber值
u32 Sd_GetRealRecordSectorNumber(u8 index)
{
    if(index < RecordCellSize) {
        return BackupSramBase_SdHead->RecordCell[index].RealRecordSectorNumber;
    }
    return 0;
}

// 设置指定索引的RealRecordSectorNumber值
void Sd_SetRealRecordSectorNumber(u8 index, u32 realRecordSectorNumber)
{
    if(index < RecordCellSize) {
        BackupSramBase_SdHead->RecordCell[index].RealRecordSectorNumber = realRecordSectorNumber;
    }
}

// 输出所有RecordCell的十六进制数据，类似Cmd_ReadSector格式
void Sd_GetWeightCellsAllInfoHex(void)
{
    // 计算所有RecordCell的总字节数
    u32 totalBytes = RecordCellSize * sizeof(RecordCellStruct);
    
    // 将RecordCell数组作为字节数组处理
    u8* dataPtr = (u8*)BackupSramBase_SdHead->RecordCell;
    
    // 以十六进制格式输出所有数据，空格分隔，大写字母，类似Cmd_ReadSector格式
    for(u32 i = 0; i < totalBytes; i++) {
        printf("%02X", dataPtr[i]);
        
        // 每个字节后添加空格，但最后一个字节不添加
        if(i < totalBytes - 1) {
            printf(" ");
        }
    }
}



