#include "Config.h"

//xQueueHandle xQueue_FromUart1;

//xQueueHandle xQueue_FromUart2;


UartData_ShortStruct Uart1_ReceiveData;

UartData_ShortStruct Uart2_ReceiveData;

//SemaphoreHandle_t  xSemaphore_Uart2;

// ADC控制标志，默认关闭
u8 AdcEnabled = 0;

// 当前正在记录的砝码索引，255表示未选择砝码
u8 CurrentWeightIndex = 255;

u32 RecordStartSector = 0;
u32 RecordEndSector=0;


InsFifoStruct InsFifo;

InsFifoStruct AdcUserInsFifo;

double InsAvgData[9];

double WeightFilterBuffer[MaxUartReceiveBufferSize];

u16 WeightFilterBufferPointer;

double FilteredrCurrentWeightValue;





