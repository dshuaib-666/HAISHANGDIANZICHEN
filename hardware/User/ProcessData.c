#include "ProcessData.h"
#include "SdManager.h"
#include "Public.h"
#include "main.h"
#include "Config.h"
#include <string.h>  // 添加string.h用于memset函数
#include <math.h>  // 添加数学库支持ceil函数
#include <stdio.h>  // 添加stdio.h用于snprintf函数
#include <stddef.h>  // 添加stddef.h用于offsetof宏
#include "Computer.h"
#include <math.h>
#include <stdio.h>
#include "MYRTC.h"
//static u8 Buffer[SectorSize];
//static u8 flag[]="test\r\n"; 

#define Acceleration_DataPointer 7
#define AngularVelocity_DataPointer Acceleration_DataPointer+14
#define RealaAngle_DataPointer AngularVelocity_DataPointer+14

#define M_PI		3.14159265358979323846

u8 CK1,CK2;

u8 DelayCounter=0;

double Acceleration[3]={0.0, 0.0, 0.0};//三轴加速度
double AngularVelocity[3]={0.0, 0.0, 0.0};//三轴角速度
double RealaAngle[3]={0.0, 0.0, 0.0};//欧拉角
double FOUR_element[4]={0.0,0.0,0.0,0.0};


uint32_t AdcValue=0;

// INS数据时间戳和计数器
//static u32 insDataUpdateCount = 0;      // INS数据更新计数器
//static u32 insDataTimestamp = 0;        // INS数据时间戳（毫秒）
//static u32 lastAdcInsUpdateCount = 0;   // 上次ADC采样时的INS更新计数器

UART_HandleTypeDef* UartDebugHandle2;

// ========== 64字节数据包管理变量 ==========
//static SectorBuffer sectorBuffer = {0};  // 扇区缓冲区

// ========== 64字节数据包管理函数 ==========

/**
 * @brief 初始化惯导数据为0
 */
void InitInsData(void) {
    for(int i = 0; i < 3; i++) {
        Acceleration[i] = 0.0;
        AngularVelocity[i] = 0.0;
        RealaAngle[i] = 0.0;
    }

	for(int i=0;i<4;i++)
	{FOUR_element[i]=0;}
	
    printf("INS data initialized to zero\r\n");
}
int decodeIns_count=0;
void DecodeIns(u8 *Data, u16 Len) //--解码传感器
{


  s32 Value;
  u16 Pointer;
//---获取三轴加速度
	Pointer=Acceleration_DataPointer;
	for(int i=0;i<=3-1;i++) 
	{
		Value=(Data[Pointer+3]<<24)|(Data[Pointer+2]<<16)|(Data[Pointer+1]<<8)|(Data[Pointer]);
		Acceleration[i]=(double)(Value*0.000001);
		Pointer+=4;
		//printf("i=%d,%08X\r\n",i,Value);
	}

//---获取三轴加速度
	Pointer=AngularVelocity_DataPointer;
	for(int i=0;i<=3-1;i++)
	{
		Value=(Data[Pointer+3]<<24)|(Data[Pointer+2]<<16)|(Data[Pointer+1]<<8)|(Data[Pointer]);
		
		AngularVelocity[i]=(double)(Value*0.000001);
		Pointer+=4;
	}

//---获取欧拉角
//---获取四元素（四元数） 
Pointer = RealaAngle_DataPointer; 
// 现在这个宏表示四元素起始位置
 for(int i = 0; i < 4; i++) 
 { 
	Value = (Data[Pointer+3]<<24)|(Data[Pointer+2]<<16)|(Data[Pointer+1]<<8)|(Data[Pointer]);
    FOUR_element[i] = (double)(Value * 0.000001); Pointer += 4; }
	

//---添加数据
PushInsData(Acceleration,AngularVelocity,FOUR_element);
decodeIns_count++;
// PushInsData(Acceleration, AngularVelocity, FOUR_element);



// 更新INS数据时间戳和计数器
//insDataUpdateCount++;
//insDataTimestamp = HAL_GetTick();  // 获取当前系统时间戳（毫秒）

#ifdef Debug_Ins

  printf("Xa=%f,Ya=%f Za=%f\r\n",Acceleration[0],Acceleration[1],Acceleration[2]);

  //HAL_UART_Transmit_DMA(UartDebugHandle2, Data, Len );
  printf("Xav=%f,Yav=%f Zaav=%f\r\n",
	AngularVelocity[0],AngularVelocity[1],AngularVelocity[2]);

  printf("Ax=%f,Ay=%f Az=%f\r\n",
		RealaAngle[0],RealaAngle[1],RealaAngle[2]);

  // 额外调试信息：显示解析后的全局变量值
  printf("Global INS updated: Acc=[%.6f,%.6f,%.6f], Gyro=[%.6f,%.6f,%.6f], Angle=[%.6f,%.6f,%.6f]\r\n",
         Acceleration[0], Acceleration[1], Acceleration[2],
         AngularVelocity[0], AngularVelocity[1], AngularVelocity[2],
         RealaAngle[0], RealaAngle[1], RealaAngle[2]);

 #endif


}



int ReadAdc(void)//参考HX711芯片手册
{
    uint32_t Count = 0;
    if(HAL_GPIO_ReadPin(ADC_DATA_GPIO_Port,ADC_DATA_Pin)==GPIO_PIN_SET)
    {
        return -1;//数据没有准备好，退出去干其他事
    }
    for (uint8_t i=0; i<24; i++)
    {
        HAL_GPIO_WritePin(ADC_CLK_GPIO_Port,ADC_CLK_Pin,GPIO_PIN_SET);

		Delay_us(100);

		HAL_GPIO_WritePin(ADC_CLK_GPIO_Port,ADC_CLK_Pin,GPIO_PIN_RESET);

		Delay_us(100);
        Count=Count<<1;//变量左移一位，右侧补零
        if(HAL_GPIO_ReadPin(ADC_DATA_GPIO_Port,ADC_DATA_Pin)==GPIO_PIN_SET) Count++;

		Delay_us(100);
    }
    for(uint8_t i=0; i<1; i++)//启动下一次转换的信号
    {
        HAL_GPIO_WritePin(ADC_CLK_GPIO_Port,ADC_CLK_Pin,GPIO_PIN_SET);

		Delay_us(100);

		HAL_GPIO_WritePin(ADC_CLK_GPIO_Port,ADC_CLK_Pin,GPIO_PIN_RESET);

		Delay_us(100);
    }

	   // 检查是否为异常值
	if(Count == 0x7FFFFF || Count==0xbFFFFF|| Count==0x3FFFFF||Count==0x1FFFFF) {  // 23位最大值
		#ifdef MY_Debug
		printf("ADC abnormal value: %lu\n", Count);
	   #endif
		//
		return -1;  // 返回错误ss
    }
	  TIM_GET_TASK_MS(MYADC);
    //Count=Count^0x800000;
	#ifdef Adc_Hx711
	printf("adc=%d\r\n",Count/10);
	#endif
		AdcValue=Count;
    return(Count);

}





void GetCrcSpical(u8* Buffer,u16 N)
{
	CK1 = 0;
	CK2 = 0;
	for(int i=0;i<N;i++)
	{
	CK1 = CK1 + Buffer[i];
	CK2 = CK2 + CK1;
	}


}



void OnGetCommUartData(u8* Data,u16 Len)
{
	//printf("process comm uart\r\n");


	GetCrcSpical( &Data[2],Len-4);

	/*for(int i=0;i<=Len-1;i++)
		{

		printf("%02X ",Data[i]);
	}*/

	if((CK1==Data[Len-2])&&(CK2==Data[Len-1]))
	{
		//printf("  Crc right\r\n");
		//HAL_UART_Transmit_DMA(UartDebugHandle, Data,Len);

		//ReadAdcHx711();

	#ifdef Debug_Ins

		//------方便显示
	if(DelayCounter<50)
		{
			DelayCounter++;
			return;
		}

	DelayCounter=0;
///////////////
		
		for(int i=0;i<=Len-1;i++)
			printf("%02X ",Data[i]);
		
		printf("\r\n");

	#endif


		
		DecodeIns(Data,Len);
		
	}

	else
	{
		printf("  Crc wrong\r\n");

	}
//	printf("\r\n");
	//printf("Crc16=%02X %02X\r\n",CK1,CK2);


	

}



/**
 * @brief 将四元素转换为欧拉角 (单位: 弧度)
 * @param q 四元素数组 [w, x, y, z]
 * @param euler 输出欧拉角数组 [roll, pitch, yaw]
 */
void QuaternionToEuler(double q[4], double euler[3])
{
    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];

    // 计算 Roll (绕X轴)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler[0] = atan2(sinr_cosp, cosr_cosp);

    // 计算 Pitch (绕Y轴)
    double sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        euler[1] = copysign(M_PI / 2, sinp); // 超范围时取 ±90°
    else
        euler[1] = asin(sinp);

    // 计算 Yaw (绕Z轴)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler[2] = atan2(siny_cosp, cosy_cosp);
}

void TestQuaternionConversion(void)
{
    //double q[4] = {0.7071, 0.0, 0.7071, 0.0}; // 示例四元数 (90°绕Y轴)
    double euler[3];

    QuaternionToEuler(FOUR_element, euler);

    printf("Quaternion: w=%f, x=%f, y=%f, z=%f\r\n", FOUR_element[0], FOUR_element[1], FOUR_element[2], FOUR_element[3]);
    printf("Euler Angles (rad): Roll=%f, Pitch=%f, Yaw=%f\r\n", euler[0], euler[1], euler[2]);
    printf("Euler Angles (deg): Roll=%f, Pitch=%f, Yaw=%f\r\n",
           euler[0] * 180.0 / M_PI,
           euler[1] * 180.0 / M_PI,
           euler[2] * 180.0 / M_PI);
}


void OnGetDebugUartData(u8* Data,u16 Len)
{
 //printf("process debug uart\r\n");

 //printf("----------------\n");
 //printf("free heap=%u\n", xPortGetFreeHeapSize());

// u8 Cmd=Data[0];
 //u32 Refer0=0;
 /*
 if(Cmd==Cmd_FormatDisk)
 	{
 	printf("format\r\n");
 		Sd_Format();
 }

  if(Cmd==Cmd_ReadCluster)
	   {
	   //	 printf("read data\r\n");
		  u16 ClusterIndex=(Data[2]<<8)|Data[1];
   
		  //for(int i=0;i<=8-1;i++)
		   Sd_ReadCluster(ClusterIndex);
	   }
   



  if(Cmd==Cmd_ReadSector)
	  {
	  //	printf("read data\r\n");
	   	 u32 SectorIndex=(Data[4]<<24)|(Data[3]<<16)|(Data[2]<<8)|Data[1];

		 //for(int i=0;i<=8-1;i++)
		  Sd_ReadSector(SectorIndex);
      }


  if(Cmd==Cmd_AddTestSector)
	{
	//  u8* Buffer= (u8*)pvPortMalloc( SectorSize);

	 
	  for(int i=0;i<=SectorSize-1;i++)
	  	Buffer[i]=i;

	  
	Sd_AddSectorData(Buffer);

	  
//	 vPortFree(Buffer);	

    }

  if(Cmd==Cmd_TestDeformat)
	{

	 Sd_DeFormat();
    }


  if(Cmd==Cmd_GetCurrentSectorPointer)
	{

	 printf("CurrentSectorPointer=%d\r\n",CurrentSectorPointer);
    }

  
 if(Cmd==Cmd_GetCurrentIns)
	 {
 
	    printf("Xa=%f,Ya=%f Za=%f\r\n",Acceleration[0],Acceleration[1],Acceleration[2]);

  		//HAL_UART_Transmit_DMA(UartDebugHandle2, Data, Len );
 		 printf("Xav=%f,Yav=%f Zaav=%f\r\n",
			AngularVelocity[0],AngularVelocity[1],AngularVelocity[2]);

  		printf("Ax=%f,Ay=%f Az=%f\r\n",
			RealaAngle[0],RealaAngle[1],RealaAngle[2]);
 }

 if(Cmd==Cmd_TestDataPacket)
	{
		printf("Testing data packet system...\r\n");
		
		// 模拟8次ADC数据，填满一个扇区
		for(int i = 0; i < 8; i++) {
			u32 testAdcValue = 500000 + i * 100;  // 模拟ADC值
		//	OnAdcDataReady(testAdcValue);
			HAL_Delay(10);  // 短暂延时模拟采样间隔
		}
		
		printf("Data packet test completed\r\n");
	}

 if(Cmd==Cmd_StartAdc)
	{
		// 检查是否已选择砝码
		if (CurrentWeightIndex >= 16) {
			printf("Error: Cannot start ADC - No weight selected (use Cmd_SetCurrentWeightIndex first)\r\n");
			return;
		}
		
		// 检查砝码配置
		int16_t weight = Sd_GetRecordCellWeight(CurrentWeightIndex);
		u32 maxSectors = Sd_GetMaxSectorNumber(CurrentWeightIndex);
		u32 startSector = Sd_GetRecordStartSector(CurrentWeightIndex);
		
		if (weight == -1 || maxSectors == 0) {
			printf("Error: Cannot start ADC - Weight[%d] not properly configured\r\n", CurrentWeightIndex);
			printf("  Weight=%d, MaxSectors=%lu, StartSector=%lu\r\n", weight, maxSectors, startSector);
			return;
		}
		
		// 设置CurrentSectorPointer为该砝码的起始扇区
		CurrentSectorPointer = startSector;
		
		// 清零当前砝码的实际记录扇区数，确保每次采集都从0开始计数
		Sd_SetRealRecordSectorNumber(CurrentWeightIndex, 0);
		
		// 将当前砝码设置为有效状态，表示开始记录数据
		Sd_SetRecordCellValid(CurrentWeightIndex, RecordDataValidFlag);
		
		AdcEnabled = 1;
		printf("ADC Started for Weight[%d]\r\n", CurrentWeightIndex);
		printf("  Weight=%d, StartSector=%lu, MaxSectors=%lu\r\n", weight, startSector, maxSectors);
		printf("  CurrentSectorPointer set to %lu\r\n", CurrentSectorPointer);
		printf("  RealRecordSectorNumber reset to 0\r\n");
		printf("  RecordDataValid set to Valid\r\n");
	}

 if(Cmd==Cmd_EndAdc)
	{
		AdcEnabled = 0;
		printf("ADC Stopped\r\n");
		
		// 立即更新显示，确保显示砝码质量
		extern u8 CurrentWeightIndex;
		if (CurrentWeightIndex < 16) {
			int16_t weight = Sd_GetRecordCellWeight(CurrentWeightIndex);
			if (weight != -1) {
				float weightValue = (float)weight;
				Display_SendToDigitTube(weightValue);
				printf("Force display update after ADC stop via command: %.4f\r\n", weightValue);
			}
		}
	}

 if(Cmd==Cmd_SetRecordTimesInSeconds)
	{
		// 检查数据长度是否正确（1字节命令 + 4字节无符号整数）
		if(Len >= 5) {
			// 提取4字节无符号整数，低位在前
			u32 recordTimes = (Data[4]<<24)|(Data[3]<<16)|(Data[2]<<8)|Data[1];
			
			// 设置记录时间间隔到备份SRAM
			u32 currentRecordTimes = Sd_GetRecordTimesInSeconds();
			Sd_SetRecordTimesInSeconds(recordTimes);
			
			printf("RecordTimesInSeconds changed from %lu to %lu seconds\r\n", currentRecordTimes, recordTimes);
			
			// 验证设置是否成功
			u32 verifyRecordTimes = Sd_GetRecordTimesInSeconds();
			printf("Verification: RecordTimesInSeconds is now %lu seconds\r\n", verifyRecordTimes);
			
			// 重新计算所有砝码的扇区分配
			printf("Recalculating sector allocations due to RecordTimesInSeconds change...\r\n");
			if (CalculateAllWeightSectorAllocations() == 0) {
				printf("Sector allocations updated successfully\r\n");
			} else {
				printf("Error: Failed to update sector allocations\r\n");
			}
		} else {
			printf("Error: Invalid data length for SetRecordTimesInSeconds command\r\n");
		}
	}

 if(Cmd==Cmd_GetRecordTimesInSeconds)
	{
		u32 recordTimes = Sd_GetRecordTimesInSeconds();
		printf("RecordTimesInSeconds: %lu seconds\r\n", recordTimes);
	}

 if(Cmd==Cmd_SetWeightCellArray)
	{
		// 检查数据长度是否正确（1字节命令 + 16个有符号16位数 = 1 + 32 = 33字节）
		if(Len >= 33) {
			printf("Setting WeightCell array with 16 signed 16-bit values:\r\n");
			
			// 解析16个有符号16位数并写入RecordCell数组
			for(int i = 0; i < 16; i++) {
				// 每个16位数占2字节，低位在前
				int16_t weight = (Data[2*i + 2] << 8) | Data[2*i + 1];
				
				// 设置RecordCell中的Weight值，并将RecordDataValid设为无效
				Sd_SetRecordCellWeight(i, weight);
				
				printf("WeightCell[%d]: Weight=%d, Valid=Invalid\r\n", i, weight);
			}
			
			printf("WeightCell array updated successfully\r\n");
			
			// 重新计算所有砝码的扇区分配
			printf("Recalculating sector allocations...\r\n");
			if (CalculateAllWeightSectorAllocations() == 0) {
				printf("Sector allocations updated successfully\r\n");
			} else {
				printf("Error: Failed to update sector allocations\r\n");
			}
		} else {
			printf("Error: Invalid data length for SetWeightCellArray command (expected 33 bytes, got %d)\r\n", Len);
		}
	}

 if(Cmd==Cmd_GetWeightCellArray)
	{
		printf("RecordCell array contents:\r\n");
		for(int i = 0; i < 16; i++) {
			int16_t weight = Sd_GetRecordCellWeight(i);
			u8 valid = Sd_GetRecordCellValid(i);
			printf("RecordCell[%d]: Weight=%d, Valid=%s\r\n", 
				i, weight, (valid == 0x01) ? "Valid" : "Invalid");
		}
	}

 if(Cmd==Cmd_GetWeightCellsAllInfo)
	{
		printf("=== All RecordCell Data ===\r\n");
		
		// 发送16个RecordCell的完整数据
		for(int i = 0; i < 16; i++) {
			int16_t weight = Sd_GetRecordCellWeight(i);
			u8 valid = Sd_GetRecordCellValid(i);
			u32 startSector = Sd_GetRecordStartSector(i);
			u32 maxSectors = Sd_GetMaxSectorNumber(i);
			u32 realSectors = Sd_GetRealRecordSectorNumber(i);
			
			// 以结构化格式输出每个RecordCell的完整信息
			printf("Cell[%02d]: Weight=%6d, Valid=%s, StartSector=%8lu, MaxSectors=%8lu, RealSectors=%8lu\r\n", 
				i, weight, 
				(valid == 0x01) ? "Valid  " : "Invalid", 
				startSector, 
				maxSectors,
				realSectors);
		}
		
		// 输出系统配置信息
		u32 recordTimes = Sd_GetRecordTimesInSeconds();
		
		printf("=== System Configuration ===\r\n");
		printf("RecordTimesInSeconds: %lu\r\n", recordTimes);
		printf("AdcMakeSectorNumberInOneSecond: %.2f\r\n", AdcMakeSectorNumberInOneSecond);
		
		// 计算并显示扇区分配统计信息
		float sectorsPerWeightFloat = AdcMakeSectorNumberInOneSecond * recordTimes;
		u32 sectorsPerWeight = (u32)ceilf(sectorsPerWeightFloat);
		printf("Calculated sectors per weight: %lu (%.2f)\r\n", sectorsPerWeight, sectorsPerWeightFloat);
		
		printf("=== End of RecordCell Data ===\r\n");
	}

 if(Cmd==Cmd_ResetAllWeightCells)
	{
		printf("=== Resetting All WeightCells ===\r\n");
		
		// 重置所有RecordCell数据到初始状态
		for(int i = 0; i < 16; i++) {
			Sd_SetRecordCellWeight(i, 0);  // 重量设为0
			// RecordDataValid会在Sd_SetRecordCellWeight中自动设为Invalid
			Sd_SetRecordStartSector(i, 0);  // 开始扇区清零
			Sd_SetMaxSectorNumber(i, 0);  // 最大扇区数清零
			Sd_SetRealRecordSectorNumber(i, 0);  // 实际记录扇区数清零
			
			printf("Reset Cell[%02d]: Weight=0, Valid=Invalid, StartSector=0, MaxSectors=0, RealSectors=0\r\n", i);
		}
		
		printf("All WeightCells have been reset to initial state\r\n");
		
		// 重新计算扇区分配
		printf("Recalculating sector allocations...\r\n");
		if (CalculateAllWeightSectorAllocations() == 0) {
			printf("Sector allocations updated successfully\r\n");
		} else {
			printf("Error: Failed to update sector allocations\r\n");
		}
		
		printf("=== Reset Complete ===\r\n");
	}

 if(Cmd==Cmd_SetCurrentWeightIndex)
	{
		// 检查数据长度是否正确（1字节命令 + 1字节砝码索引）
		if(Len >= 2) {
			u8 weightIndex = Data[1];
			
			// 验证砝码索引范围
			if (weightIndex < 16) {
				u8 oldIndex = CurrentWeightIndex;
				CurrentWeightIndex = weightIndex;
				
				// 获取砝码信息用于确认
				int16_t weight = Sd_GetRecordCellWeight(CurrentWeightIndex);
				u32 maxSectors = Sd_GetMaxSectorNumber(CurrentWeightIndex);
				u32 startSector = Sd_GetRecordStartSector(CurrentWeightIndex);
				
				printf("Current weight index changed from %d to %d\r\n", oldIndex, CurrentWeightIndex);
				printf("Selected Weight[%d]: Weight=%d, StartSector=%lu, MaxSectors=%lu\r\n", 
					CurrentWeightIndex, weight, startSector, maxSectors);
				
				// 检查砝码是否已配置
				if (weight == -1 || maxSectors == 0) {
					printf("Warning: Selected weight is not properly configured\r\n");
				} else {
					printf("Weight[%d] is ready for recording\r\n", CurrentWeightIndex);
				}
			} else {
				printf("Error: Weight index %d out of range (0-15)\r\n", weightIndex);
			}
		} else {
			printf("Error: Invalid data length for SetCurrentWeightIndex command\r\n");
		}
	}

 if(Cmd==Cmd_GetCurrentWeightIndex)
	{
		if (CurrentWeightIndex < 16) {
			int16_t weight = Sd_GetRecordCellWeight(CurrentWeightIndex);
			u32 maxSectors = Sd_GetMaxSectorNumber(CurrentWeightIndex);
			u32 startSector = Sd_GetRecordStartSector(CurrentWeightIndex);
			
			printf("Current weight index: %d\r\n", CurrentWeightIndex);
			printf("Weight[%d]: Weight=%d, StartSector=%lu, MaxSectors=%lu\r\n", 
				CurrentWeightIndex, weight, startSector, maxSectors);
		} else {
			printf("Current weight index: %d (No weight selected)\r\n", CurrentWeightIndex);
		}
	}

 if(Cmd==Cmd_GetWeightCellsAllInfoHex)
	{
		// 调用新的十六进制输出函数
		Sd_GetWeightCellsAllInfoHex();
	}

 if(Cmd==Cmd_ForceUpdateDisplay)
	{
		// 强制更新数码管显示
		Display_ForceUpdate();
	}

*/

    //printf("历史最小剩余堆: %u 字节\n", xPortGetMinimumEverFreeHeapSize());

}



/**
 * @brief 检测三个按钮状态并处理相应功能
 * 功能：包含50ms防抖处理，避免按钮抖动误触发
 * 注意：此函数仅负责按钮防抖，具体业务处理应在防抖完成后由调用者实现

 Key_Select_Pin:右边第一个按钮
 Key_Start_Pin:左边第一个按钮
 Key_End_Pin:中间按钮
 */
void CheckButtonStatus(void) {
    // 使用静态变量实现按钮防抖
    static u8 selectBtnCount = 0;       // Key_Select按钮防抖计数器
    static u8 startAdcBtnCount = 0;     // START_ADC按钮防抖计数器  
    static u8 endAdcBtnCount = 0;       // END_ADC按钮防抖计数器
    static u8 selectBtnPressed = 0;     // Key_Select按钮按下标志
    static u8 startAdcBtnPressed = 0;   // START_ADC按钮按下标志
    static u8 endAdcBtnPressed = 0;     // END_ADC按钮按下标志
    
    // Key_Select按钮检测 (低电平有效，内部上拉)
    if (HAL_GPIO_ReadPin(Key_Select_GPIO_Port, Key_Select_Pin) == GPIO_PIN_RESET) {
		//  printf("Key_Select 111\r\n");
        if (selectBtnCount < 8) {      // 50ms防抖
            selectBtnCount++;
        } else{
            // 防抖完成，可添加业务处理逻辑
            if (!selectBtnPressed) {
                selectBtnPressed = 1;
                // 按下Key_Select按钮的业务处理待添加
                printf("Key_Select button pressed\r\n");

								ZeroVerify();
								
            }
        }
    } else {
        selectBtnCount = 0;             // 重置防抖计数器
        selectBtnPressed = 0;           // 重置按下标志
    }
    
    // Key_Start按钮检测 (低电平有效，内部上拉)
    if (HAL_GPIO_ReadPin(Key_Start_GPIO_Port, Key_Start_Pin) == GPIO_PIN_RESET) {
        if (startAdcBtnCount < 8) {    // 50ms防抖
            startAdcBtnCount++;
        } 
        else {
            // 防抖完成，可添加业务处理逻辑
            if (!startAdcBtnPressed) {
                startAdcBtnPressed = 1;
                // 按下Key_Start按钮的业务处理待添加
                printf("Key_Start button pressed\r\n");
								StartMeasure();
            }
        }
    } else {
        startAdcBtnCount = 0;           // 重置防抖计数器
        startAdcBtnPressed = 0;         // 重置按下标志
    }
    
    // Key_End按钮检测 (低电平有效，内部上拉)
    if (HAL_GPIO_ReadPin(Key_End_GPIO_Port, Key_End_Pin) == GPIO_PIN_RESET) {
        if (endAdcBtnCount < 8) {      // 50ms防抖
            endAdcBtnCount++;
        } 
        else {
            // 防抖完成，可添加业务处理逻辑
            if (!endAdcBtnPressed) {
                endAdcBtnPressed = 1;
                // 按下Key_End按钮的业务处理待添加
                printf("Key_End button pressed\r\n");
            }
        }
    } else {
        endAdcBtnCount = 0;             // 重置防抖计数器
        endAdcBtnPressed = 0;           // 重置按下标志
    }
}



/**
 * @brief 发送数据到6位数码管显示
 * @param value 要显示的数值
 * 协议格式：$001,数字#
 * 例如：$001,12.3456#
 */
void Display_SendToDigitTube(float value) {
    extern UART_HandleTypeDef huart3;
    static char displayBuffer[32];

    // ========== 非阻塞“缓存最新值 + 固定节拍刷新”驱动 ==========
    // 目的：
    // - 避免HAL_UART_Transmit阻塞
    // - 避免 HAL_UART_STATE_BUSY 时直接 return 导致“卡顿/卡死直到大变动才刷新”
    // 策略：
    // - 每次调用仅更新 pending 值
    // - UART空闲且到达刷新周期时，发送 pending 的最新值
    // - 大幅变化可触发“提前刷新”，提升体感
    #ifndef DIGIT_TUBE_MIN_PERIOD_MS
    #define DIGIT_TUBE_MIN_PERIOD_MS 50U   // 20Hz，足够顺滑且对9600bps很安全
    #endif
    #ifndef DIGIT_TUBE_FAST_DELTA_V100
    #define DIGIT_TUBE_FAST_DELTA_V100 50  // 0.50g 变化触发提前刷新
    #endif

    static int32_t pending_v100 = 0;
    static uint8_t has_pending = 0U;
    static int32_t last_sent_v100 = 0;
    static uint32_t last_send_tick = 0U;
    
    // 格式化数据为协议格式：$001,数字#
	if(value<=0&&value>=-1)
	{
		  value=0.0f;
	}
	else if(value<0.6f &&value>0.0f)
	{
		   value=0.0f;
	}

    // 每次调用都只更新 pending（不立即发送）
    pending_v100 = (int32_t)(value * 100.0f + (value >= 0.0f ? 0.5f : -0.5f));  // 四舍五入到0.01
    has_pending = 1U;

    // UART忙则直接返回；pending 会保留到下次空闲再发
    if (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {
        return;
    }

    // 固定刷新节拍 + 大变化提前刷新
    uint32_t now = HAL_GetTick();
    uint32_t dt = now - last_send_tick;
    int32_t delta = pending_v100 - last_sent_v100;
    if (delta < 0) delta = -delta;
    if (!has_pending) {
        return;
    }
    if (dt < (uint32_t)DIGIT_TUBE_MIN_PERIOD_MS && delta < (int32_t)DIGIT_TUBE_FAST_DELTA_V100) {
        return;
    }

    // 发送 pending 的最新值（此处确保UART空闲）
    int32_t v100 = pending_v100;

    // 低开销格式化：$001,[-]ipart.frac#
    int len = 0;
    displayBuffer[len++] = '$';
    displayBuffer[len++] = '0';
    displayBuffer[len++] = '0';
    displayBuffer[len++] = '1';
    displayBuffer[len++] = ',';

    int32_t abs_v100 = (v100 < 0) ? -v100 : v100;
    if (v100 < 0) {
        displayBuffer[len++] = '-';
    }

    uint32_t ipart = (uint32_t)(abs_v100 / 100);
    uint32_t frac = (uint32_t)(abs_v100 % 100);

    // ipart 至少输出1位
    char tmp[10];
    int tn = 0;
    do {
        tmp[tn++] = (char)('0' + (ipart % 10U));
        ipart /= 10U;
    } while (ipart != 0U && tn < (int)sizeof(tmp));
    for (int i = tn - 1; i >= 0; --i) {
        displayBuffer[len++] = tmp[i];
    }

    displayBuffer[len++] = '.';
    displayBuffer[len++] = (char)('0' + (frac / 10U));
    displayBuffer[len++] = (char)('0' + (frac % 10U));
    displayBuffer[len++] = '#';

    // 发起非阻塞发送；成功才更新时间戳/last_sent
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t*)displayBuffer, (uint16_t)len) == HAL_OK) {
        last_send_tick = now;
        last_sent_v100 = v100;
        has_pending = 0U;
    }
    
    // printf("Display sent to digit tube: %s\r\n", displayBuffer);
}



/**
 * @brief 发送黑屏命令到数码管
 */
void Display_SendBlackScreen(void) {
    extern UART_HandleTypeDef huart3;
    static char displayBuffer[] = "$001,#";
    
    if (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {
        return;
    }

    // 通过UART3发送黑屏命令到数码管（非阻塞）
    (void)HAL_UART_Transmit_IT(&huart3, (uint8_t*)displayBuffer, (uint16_t)strlen(displayBuffer));
    
    // printf("Display sent to digit tube: %s (Black screen)\r\n", displayBuffer);
}

/**
 * @brief 将INS数据推入FIFO缓冲区（使用四元素替代欧拉角）
 * @param acceleration 三轴加速度数组
 * @param angularVelocity 三轴角速度数组
 * @param quaternion 四元素数组
 */
void PushInsData(double acceleration[3], double angularVelocity[3], double quaternion[4])
{
    // 将数据存入当前位置
    for(int i = 0; i < 3; i++)
    {
        InsFifo.InsFifo[InsFifo.InsFifo_Pointer][i] = acceleration[i];           // 加速度 0-2
        InsFifo.InsFifo[InsFifo.InsFifo_Pointer][i + 3] = angularVelocity[i];    // 角速度 3-5
    }

    for(int i = 0; i < 4; i++)
    {
        InsFifo.InsFifo[InsFifo.InsFifo_Pointer][i + 6] = quaternion[i];         // 四元素 6-9
    }

    // 更新指针，如果达到上限就设置为0（循环缓冲区）
    InsFifo.InsFifo_Pointer++;
    if(InsFifo.InsFifo_Pointer >= InsFifoSize)
    {
        InsFifo.InsFifo_Pointer = 0;
        printf("InsFifo full, pointer reset to 0\r\n");
    }
}


/**
 * @brief 将InsFifo的有效数据复制到AdcUserInsFifo
 * 当ADC采样时调用，将当前InsFifo中的所有有效数据复制到AdcUserInsFifo
 */
void CopyInsFifoToAdcUserInsFifo(void)
{
    // 复制InsFifo的所有数据到AdcUserInsFifo
    for(int i = 0; i < InsFifoSize; i++)
    {
        for(int j = 0; j < 10; j++)
        {
            AdcUserInsFifo.InsFifo[i][j] = InsFifo.InsFifo[i][j];
        }
    }

    // 复制当前指针位置
    AdcUserInsFifo.InsFifo_Pointer = InsFifo.InsFifo_Pointer;
		//--清除InsFifo的数据
		InsFifo.InsFifo_Pointer=0;

    #ifdef Debug_Ins
    printf("InsFifo data copied to AdcUserInsFifo, pointer=%d\r\n", AdcUserInsFifo.InsFifo_Pointer);
    #endif
}


