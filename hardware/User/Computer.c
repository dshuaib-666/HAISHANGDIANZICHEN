#include "Computer.h"
#include "Config.h"
#include "ProcessData.h"
#include "MYRTC.h"
#include "SEND.h"
#include "math.h"
#define AdBufferSize 50
#define InsBufferSize AdBufferSize*5

// 注意：C语言里“前导0”的整数常量会按八进制解析（例如 00400 != 400）。
// 这里修正为十进制，避免标定参数被误解析。
double L_Ncc[11] = {-310900, 1200, 44610, 11590, 82680, -8350, 1820, -8640, 400, -1510, -20};
double InsDataArray[9][InsBufferSize];
int SampleIndex=0;
double AdcVauleArray[AdBufferSize];
u8 AdcBufferFull=false;

double ZeroMarkValue=0;
double CurrentAdcValue=0;

u8 ZeroMark=false;

u8 AdcCounter=0;

uint16_t AdBufferPointer=0;
uint16_t InsBufferPointer=0;

double CurrentWeight=0;

u8 FilterBufferFull=false;

// 注意：这里的“跳过计数”单位是“调用次数”，不是毫秒。
// 之前主循环里 FilterWeightValue() 实际常按 HX711 新数据就绪节拍调用（10Hz/80Hz），
// 所以 80 次可能意味着 8s（10Hz）或 1s（80Hz）的“强制不更新”。
#define FILTER_SKIP_COUNT 80  // 跳过前80次不稳定数据（仅用于旧的非模型链路）
u16 FilterSkipCounter=0;      // 跳过计数器

#define ZERO_UPDATE_COUNT 200   // 动态零点校正累计次数，可调整

float disfilter=0.0f;

extern volatile float ScaleModel_yFused ;
void DynamicZeroUpdate(void)
{
    static int zeroCounter = 0;           // 计数器
    static double zeroAccumulator = 0.0;  // 累加器
    static u8 validFlag = 1;              // 标志位，1表示有效，0表示中途失效

    // 计算当前差值映射
    double diffValue = (FilteredrCurrentWeightValue - ZeroMarkValue) ;

    if (fabs(diffValue) <= 1.0f) {
        if (validFlag) {
            zeroAccumulator += FilteredrCurrentWeightValue;
            zeroCounter++;
        }
    } else {
        // 一旦超出范围，标志位清零，计数器和累加器重置
        validFlag = 0;
        zeroCounter = 0;
        zeroAccumulator = 0.0;
    }

    // 达到累计次数，且标志位有效
    if (zeroCounter >= ZERO_UPDATE_COUNT && validFlag) {
        ZeroMarkValue = zeroAccumulator / ZERO_UPDATE_COUNT;
        #ifdef MY_Debug
         printf("Dynamic Zero Updated: ZeroMarkValue=%f\n", ZeroMarkValue);
        #endif
        // 重置状态
        zeroCounter = 0;
        zeroAccumulator = 0.0;
        validFlag = 1;
    }

    // 如果标志位失效，下一次重新开始
    if (!validFlag) {
        validFlag = 1; // 等待下一次重新累计
    }
}

void StartMeasure(void)
{

    AdBufferPointer=0;
    AdcBufferFull=false;

    for(int i=0;i<=AdBufferSize-1;i++)
      AdcVauleArray[i]=0;

    printf("Start Measure\n");

}


void ZeroVerify(void)
{

    ZeroMarkValue=CurrentAdcValue;
    ZeroMark=false;

    AdBufferPointer=0;
    AdcBufferFull=false;

    for(int i=0;i<=AdBufferSize-1;i++)
      AdcVauleArray[i]=0;

    printf("ZeroVerifyStart\n");

}

void PushAdValue(float AdVaule)
{
  AdcVauleArray[AdBufferPointer]=AdVaule;
  AdBufferPointer++;
  if(AdBufferPointer==AdBufferSize)
  {
    AdBufferPointer=0;
    AdcBufferFull=true;
  }
}

void ComputerAdcValue(void)
{
#ifdef ENABLE_SCALE_MODEL
  // ========== 模型模式：移除多层均值滤波，直接做“零点/动态零点” ==========
  // 目标：让响应时间由模型窗口（例如 100帧@50Hz≈2s）主导，而不是再叠加 20点均值 + 50点均值（会拉到5s+）。
  // 说明：
  // - FilterWeightValue() 会把 FilteredrCurrentWeightValue 直接设置为 CurrentWeight(=ScaleModel_yFused)。
  // - 这里仅做一次上电自动零点（取一小段平均），以及输出 = 当前值 - ZeroMarkValue。
  #ifndef MODEL_ZEROMARK_SAMPLES
  #define MODEL_ZEROMARK_SAMPLES 50U  // 50Hz 下约 1s；如需更快可改小
  #endif

  static double zm_sum = 0.0;
  static uint32_t zm_cnt = 0U;

  if (ZeroMark == false)
  {
    zm_sum += FilteredrCurrentWeightValue;
    zm_cnt++;

    if (zm_cnt >= (uint32_t)MODEL_ZEROMARK_SAMPLES)
    {
      ZeroMarkValue = zm_sum / (double)zm_cnt;
      ZeroMark = true;
      zm_sum = 0.0;
      zm_cnt = 0U;
      printf("ZeroMarkValue=%f\n", ZeroMarkValue);
    }

    // 零点尚未建立时，输出先保持为0，避免显示跳动
    my_send.now_real = 0.0f;
    return;
  }

  my_send.now_real = (float)(FilteredrCurrentWeightValue - ZeroMarkValue);
  return;
#else

  double SummerValue=0;
  double RealValue=0;
  ///------没有数据零矫正前的处理
  //注意，这个上电开头就会进行一次校准
  if(ZeroMark==false)
  {
    if(AdcBufferFull==true)
    {

      for(int i=0;i<=AdBufferSize-1;i++)
      {
        SummerValue+=AdcVauleArray[i];
      //PushAdValue(AdcVauleArray[i]);
      }
      RealValue=SummerValue/AdBufferSize;
      ZeroMarkValue=RealValue;
      ZeroMark=true;
      printf("ZeroMarkValue=%f\n",RealValue);
      return;
    }
    else
    {

      printf("Wait for zero mark\n");

      return;
    }

  }

  //------零点已经获取

  if(AdcBufferFull==true)
  {
    for(int i=0;i<=AdBufferSize-1;i++)
    {
      SummerValue+=AdcVauleArray[i];
      //PushAdValue(AdcVauleArray[i]);
    }
    RealValue=SummerValue/AdBufferSize;

    //AdcBufferFull=false;
  }
  else
  {
      for(int i=0;i<=AdBufferPointer-1;i++)
      {

        SummerValue+=AdcVauleArray[i];
      }

    RealValue=SummerValue/AdBufferPointer;
  }

  //CurrentAdcValue=RealValue;
   my_send.now_real=(RealValue-ZeroMarkValue);
   
   
//   TIM_GET_TASK_MS(MYADC);
 

  
  // printf("disfilter=%f\r\n,RealValue=%f\r\n ZeroMarkValue=%f\r\n FilteredrCurrentWeightValue=%f \r\n my_send.now_real=%f\r\n",disfilter,RealValue,ZeroMarkValue,FilteredrCurrentWeightValue,my_send.now_real);


  /*if(ZeroMark==false)
  {

    uint32_t ShowValue=RealValue/100;

    printf("ZeroMakerValue=%d\n",ShowValue);
  }
  else
  {
    uint32_t ShowValue=(RealValue-ZeroMarkValue)/100;

    printf("RealValue=%d\n",ShowValue);

  }*/

#endif
}


/**
  * @brief  计算AdcUserInsFifo的平均值到InsAvgData
  * @note   遍历AdcUserInsFifo中所有有效数据，计算每个通道的平均值
  * @param  None
  * @retval None
  */
void ComputeInsAvgData(void)
{
    u8 i, j;
    double sum[9] = {0};

    // 如果FIFO为空，直接返回
    if(AdcUserInsFifo.InsFifo_Pointer == 0)
    {
        return;
    }

    // 计算每个通道的总和
    for(i = 0; i < AdcUserInsFifo.InsFifo_Pointer; i++)
    {
        for(j = 0; j < 9; j++)
        {
            sum[j] += AdcUserInsFifo.InsFifo[i][j];
        }
    }

    // 计算平均值
    for(j = 0; j < 9; j++)
    {
        InsAvgData[j] = sum[j] / AdcUserInsFifo.InsFifo_Pointer;
    }
}




//double L_Ncc_MulityRate=100000;

/*double ZeroMarkValue=0;
char ZeroMarkFlag=0;

double CurrentWeight=0;

char DataInit=0;

int BufferPointer=0;

double Computer_Buffer[BufferSize];

int InnerCounter=0;*/

/**
 * 将惯性传感器数据（加速度、角速度、欧拉角）复制到组合数组InsDataArray中
 * 数据按顺序排列：[0-2]加速度，[3-5]角速度，[6-8]欧拉角
 * 
 * @param None - 使用全局变量作为输入输出
 * @return None - 结果存储在全局InsDataArray中
 */

/*
 void PushData(double Weight)
{
  Computer_Buffer[BufferPointer]=Weight;

  BufferPointer++;
  if(BufferPointer==BufferSize)
  {
    BufferPointer=0;
  }
}*/

/*void MoveInsData()
{
  int Index = 0;

  // 复制三轴加速度数据到InsDataArray的前三个位置
  for (int i = 0; i <= 3 - 1; i++)
  {
    InsDataArray[i + Index] = Acceleration[i];
  }

  Index += 3;

  // 复制三轴角速度数据到InsDataArray的中间三个位置
  for (int i = 0; i <= 3 - 1; i++)
  {
    InsDataArray[i + Index] = AngularVelocity[i];
  }

  Index += 3;

  // 复制欧拉角数据到InsDataArray的最后三个位置
  for (int i = 0; i <= 3 - 1; i++)
  {
    InsDataArray[i + Index] = RealaAngle[i];
  }
}*/

void GetWeightValue()
{
  double Pw=0;
  

  //MoveInsData();

  for(int i=0;i<=9-1;i++)
  {

    Pw+=L_Ncc[i+2]*InsAvgData[i];

  }

  

  CurrentWeight=(AdcValue-L_Ncc[0]-Pw)/L_Ncc[1];
/*
  if(DataInit==0)
  {

    for(int i=0;i<=BufferSize-1;i++)
    Computer_Buffer[i]=CurrentWeight;
    DataInit=1;
  }

  PushData(CurrentWeight);

  InnerCounter++;

  if(InnerCounter==10)
  {
    InnerCounter=0;

    for(int i=0;i<=BufferSize-1;i++)
    {
      SummerValue+=Computer_Buffer[i];
    }

    double AverageValue=SummerValue/BufferSize;
*/
    printf("CurrentWeight=%f\n",CurrentWeight);

   // GetWeightValue();

  

 



 // printf("Pw=%f AdcValue%d   CurrentWeight=%f  \n",Pw,AdcValue,CurrentWeight);


}

/**
 * @brief  对重量值进行滤波处理
 * @note   使用移动平均滤波算法对当前重量值进行平滑处理，减少测量噪声
 * @param  void: 无参数
 * @retval void: 无返回值
 * @details 
 * 该函数实现了一个移动平均滤波器，用于对重量测量值进行滤波处理:
 * 1. 将当前重量值(CurrentWeight)存入滤波缓冲区
 * 2. 当缓冲区未满时，计算已存入数据的平均值
 * 3. 当缓冲区满时，计算整个缓冲区数据的平均值
 * 4. 将计算结果存储在FilteredrCurrentWeightValue变量中
 * 5. 通过串口打印滤波后的重量值
 */

 void SetWeightToAdcValue(void)
 {
  CurrentWeight=ScaleModel_yFused;
 }

void FilterWeightValue(void)
{
#ifdef ENABLE_SCALE_MODEL
  // ========== 模型模式：移除均值滤波链路（20点均值 + 50点均值） ==========
  // 直接使用模型融合输出（CurrentWeight=ScaleModel_yFused），再做零点/动态零点处理。
  FilteredrCurrentWeightValue = CurrentWeight;

  // 仅在零点建立后才进行动态零点校正，避免与上电零点流程相互干扰
  if (ZeroMark) {
    DynamicZeroUpdate();
  }

  ComputerAdcValue();
  return;
#else

  double SummerValue=0;

  // 跳过前几个不稳定的数据
  if(FilterSkipCounter < FILTER_SKIP_COUNT) {
    FilterSkipCounter++;
    //printf("Skipping unstable data #%d: CurrentWeight=%f\n", FilterSkipCounter, CurrentWeight);
    return;
  }

  // 将当前重量值存入滤波缓冲区的当前位置
  WeightFilterBuffer[WeightFilterBufferPointer]=CurrentWeight;

  // 更新缓冲区指针
  WeightFilterBufferPointer++;

  // 检查缓冲区指针是否到达最大值，如果是则重置指针并标记缓冲区已满
  if(WeightFilterBufferPointer==MaxFilterBufferSize)
  {
    WeightFilterBufferPointer=0;
    FilterBufferFull=true;
  }
  
  // 根据缓冲区是否已满，计算缓冲区中所有数据的总和
  if (FilterBufferFull == false)
  {
    for (int i = 0; i <= WeightFilterBufferPointer - 1; i++)
    {
      SummerValue += WeightFilterBuffer[i];
    }
    // 计算平均值并保存为滤波后的重量值
    FilteredrCurrentWeightValue=SummerValue/WeightFilterBufferPointer;

    // printf("FilteredrCurrentWeightValue=%f (sum=%f, count=%d)\n",FilteredrCurrentWeightValue, SummerValue, WeightFilterBufferPointer);
  }
  else
  {
    for (int i = 0; i <= MaxFilterBufferSize - 1; i++)
    {
      SummerValue += WeightFilterBuffer[i];
    }
    // 计算平均值并保存为滤波后的重量值
    FilteredrCurrentWeightValue=SummerValue/MaxFilterBufferSize;
       //printf("FilteredrCurrentWeightValue=%f (sum=%f, count=%d, NOW=%f FULL)\n",FilteredrCurrentWeightValue, SummerValue, MaxFilterBufferSize,CurrentWeight);

      PushAdValue(FilteredrCurrentWeightValue);
      
      //   // PushAdValue(AdcValue);
      DynamicZeroUpdate();
      //   if(AdcCounter<=5-1) //10ms计算一次
      //   {
  
      //     AdcCounter++;
  
      //   }
      //   else
      //   {
  
      //     AdcCounter=0;
  
          ComputerAdcValue();
  
  
        // }

  }

  }
#endif
}

//简单处理,也就是不滤波，直接解算完发出去
void simgle_dispose(void)
{
  if(FilterSkipCounter < FILTER_SKIP_COUNT) {
    FilterSkipCounter++;
    //printf("Skipping unstable data #%d: CurrentWeight=%f\n", FilterSkipCounter, CurrentWeight);
    return;
  }
  FilteredrCurrentWeightValue=AdcValue; 
  PushAdValue(FilteredrCurrentWeightValue);
     

  DynamicZeroUpdate();


  ComputerAdcValue();
}


/*
void GetWeightValue()
{
  double SummerValue=0;

  PushData(AdcValue);

  InnerCounter++;

  if(InnerCounter==10)
  {
    InnerCounter=0;

    for(int i=0;i<=BufferSize-1;i++)
    {
      SummerValue+=Computer_Buffer[i];
    }

    double AverageValue=SummerValue/BufferSize;

    if(ZeroMarkFlag==1)
    {
      ZeroMarkValue=AverageValue;
      ZeroMarkFlag=0;
    }

    printf("AverageValue=%f   ZeroMarkValue=%f\n",AverageValue,ZeroMarkValue);

    double CurrentAdcValue=AdcValue-ZeroMarkValue;
    printf("CurrentAdcValue=%f\n",CurrentAdcValue);
  }

  if(InnerCounter==10)
  {
    InnerCounter=0;

    for(int i=0;i<=BufferSize-1;i++)
    {
      SummerValue+=Computer_Buffer[i];
    }

    double AverageValue=SummerValue/BufferSize;

    printf("AverageValue=%f\n",AverageValue);


  }
}
*/

