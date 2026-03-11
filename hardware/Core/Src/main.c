/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include "SdManager.h"
#include "ProcessData.h"
#include "Public.h"
#include "Config.h"
#include "Computer.h"
#include "MYRTC.h"
#include "SEND.h"
#include "ScaleModelRunner.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Data[]="mytest12345678\r\n";

#ifdef SCALE_MODEL_DEBUG_FREQ
// 性能测量（DWT 周期计数器）
static uint8_t g_dwt_ready = 0U;
static uint64_t g_step_cyc_sum = 0ULL;
static uint32_t g_step_cyc_max = 0U;
static uint32_t g_step_cyc_cnt = 0U;
#endif





int fputc(int ch, FILE *f)
{
    // 主要输出到UART1调试串口（始终可用）
    //注意关掉了
    //HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    
    #ifdef ENABLE_USB
    // 检查USB是否已连接并初始化成
    extern USBD_HandleTypeDef hUsbDeviceFS;
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        // 只有在USB连接并配置成功时才发送到USB
        extern uint8_t USB_Transmit_Char(uint8_t ch);
        USB_Transmit_Char((uint8_t)ch);
    }
    #endif
    
    return ch;
}


#define RX_BUF_SIZE 1024
uint8_t Uart1_RxBuffer[RX_BUF_SIZE];  // DMA接收缓冲
volatile uint16_t Uart1_rxLength = 0; // 接收数据长度
volatile uint8_t Uart1_rxFlag = 0;    // 接收完成标志

//#define RX_BUF_SIZE 1024
uint8_t Uart2_RxBuffer[RX_BUF_SIZE];  // DMA接收缓冲
volatile uint16_t Uart2_rxLength = 0; // 接收数据长度
volatile uint8_t Uart2_rxFlag = 0;    // 接收完成标志


u16 LedCount=0;

void Echo_Uart()
{
	//HAL_UART_Transmit_DMA(&huart1, Uart1_RxBuffer, Uart1_rxLength);

	//printf("%d\r\n",Uart1_rxLength);

	//Uart1_RxBuffer[Uart1_rxLength]=0;
	///printf("%s\r\n",Uart1_RxBuffer);

	for( int i=0;i<=Uart1_rxLength-1;i++)
		printf("%c",Uart1_RxBuffer[i]);


}




extern void On_USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1); // 清除空闲标志
        HAL_UART_DMAStop(&huart1);          // 停止DMA

		
		
        Uart1_rxLength = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx); // 计算接收长度

		///HAL_UART_Transmit_DMA(&huart1, Uart1_RxBuffer, Uart1_rxLength);

		//Echo_Uart();

		//printf("get data\r\n");

		Uart1_rxFlag = 1;                        // 标记接收完成
       // Start_DMA_Receive();               // 重启接收
			
	  HAL_UART_Receive_DMA(&huart1, Uart1_RxBuffer, RX_BUF_SIZE);
    }
}



/**
  * @brief  USART2中断处理函数
  * @note   处理USART2串口的空闲中断，当检测到串口接收空闲时，计算接收到的数据长度，
  *         调用OnGetCommUartData处理接收到的数据，然后重新启动DMA接收
  * @param  None
  * @retval None
  */
 int nine_count=0;
extern void On_USART2_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2); // 清除空闲标志
        HAL_UART_DMAStop(&huart2);          // 停止DMA
        Uart2_rxLength = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx); // 计算接收长度

	
		OnGetCommUartData(Uart2_RxBuffer,Uart2_rxLength);
		TIM_GET_TASK_MS(MYNINE);
    nine_count++;
	  HAL_UART_Receive_DMA(&huart2, Uart2_RxBuffer, RX_BUF_SIZE);
    }
}


void CheckUartGetDataFlag(void)
{

if(Uart1_rxFlag==1)
{
	OnGetDebugUartData(Uart1_RxBuffer,Uart1_rxLength);
	Uart1_rxFlag=0;
}



}

int count=0;
extern int TIM1_COUNT;
// extern int decodeIns_count;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
 
  HAL_TIM_Base_Start_IT(&htim1);

  // ========== 性能测量：启用 DWT 周期计数器 ==========
  // 用于输出每次 ScaleModelRunner_Step() 的 CPU 周期数（不依赖定时器精度）。
  #ifdef SCALE_MODEL_DEBUG_FREQ
  if (!g_dwt_ready) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_dwt_ready = 1U;
  }
  #endif

  HAL_UART_Receive_DMA(&huart2, Uart2_RxBuffer, RX_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); // 使能空闲中断//


  UartDebugHandle=&huart1;

  UartDebugHandle2=&huart1;


  // ========== 初始化64字节数据包管理系统 ==========
  //InitSectorBuffer();
  printf("Data packet management system initialized\r\n");
  
  // ========== 初始化砝码选择 ==========
  
  
  #ifdef ENABLE_USB
  // 检查USB连接状态
  extern USBD_HandleTypeDef hUsbDeviceFS;
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
      printf("USB connected and configured\r\n");
  } else {
      printf("USB not connected, using UART1 only\r\n");
  }
  #endif

  //HAL_Delay(100);

  //Display_SendBlackScreen();

  HAL_Delay(1000);

  Display_SendToDigitTube(0);   

  InsFifo.InsFifo_Pointer=0;

  AdcUserInsFifo.InsFifo_Pointer=0;

  WeightFilterBufferPointer=0;

  FilterSkipCounter=0;


  printf("Scal system start\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // while(1)
  // {
  //     HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
  //     HAL_Delay(1000);
  //     HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
  //     HAL_Delay(1000);
  // }


  while (1)
  {
    // ========== 系统时基控制 ==========
   // Delay_us(500);                    // 1ms延时，控制主循环频率1000Hz
    
	
   
    // ========== 数码管闪烁控制 ==========
  //  extern void Display_UpdateBlinkStatus(void);
  //  Display_UpdateBlinkStatus();     // 更新数码管闪烁状态，ADC运行时2Hz闪烁

     
    // ========== 按钮测试程序 ==========
    CheckButtonStatus();             // 测试三个按钮状并处理相应功能
    
    // ========== USB虚拟串口数据处理 ==========
    CheckUartGetDataFlag();          // 检查并处理调试USB串口(UART1)的命令数
    
    // ========== RTC测试函数 =============

    // ==========四元素测试函数===========
      //TestQuaternionConversion();

    #ifdef ENABLE_USB
    // 只有在USB连接并配置成功时才处理USB数据
    extern USBD_HandleTypeDef hUsbDeviceFS;
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        // 检查并处理USB虚拟串口的命令数
        extern void CheckUsbGetDataFlag(void);
        CheckUsbGetDataFlag();           // 检查并处理USB虚拟串口的命令数
        
        // 处理USB发送缓冲区
        extern void USB_Process_TxBuffer(void);
        USB_Process_TxBuffer();          // 处理USB发送缓冲区，确保数据完整发送
    }
    #endif
                                    // - 处理来自调试串口的控制命令
                                    // - UART2的IMU数据在中断中直接处理
      // 重要：不要在这里直接 TIM1_COUNT=0。
      // 因为在执行一次推理/显示期间（例如 15ms），TIM1 中断仍会累加 TIM1_COUNT。
      // 如果粗暴清零，会等于“推理15ms + 再等20ms”，频率就会掉到 ~28Hz。
      // 正确做法：每处理一次 20ms tick，就减去 20，保留余量（必要时可补跑追赶）。
      while (TIM1_COUNT >= 20)
      {
        TIM1_COUNT -= 20;
        TIM_COMPUTER(&tim_data);

        // ========== 深度学习模型推理（建议固定 50Hz 调用） ==========
        // 重要：原先按ADC“新数据就绪”节拍调用（HX711常见为10Hz/80Hz），会导致
        // `SCALE_MODEL_WINDOW_SIZE=100` 的时间窗口变成 10s/1.25s，从而出现“要5秒以上才稳定”。
        // 这里改为跟随 TIM1 的 20ms 节拍（约50Hz）调用：即使ADC 10Hz，也会重复使用最近一次AdcValue，
        // 等价于对“保持值”做 50Hz 采样，更贴近软件侧 `--resample_hz 50` 的训练/评测假设。
        #ifdef ENABLE_SCALE_MODEL
        extern double FOUR_element[4];
        float yFused = 0.0f;
        float yBaseline = 0.0f;
        float yModel = 0.0f;

        // 周期测量：仅统计 ScaleModelRunner_Step()（不包含后续业务滤波/显示）
        #ifdef SCALE_MODEL_DEBUG_FREQ
        uint32_t t0 = (g_dwt_ready ? DWT->CYCCNT : 0U);
        #endif
        (void)ScaleModelRunner_Step(AdcValue, Acceleration, AngularVelocity, FOUR_element,
                                    &yFused, &yBaseline, &yModel);
        #ifdef SCALE_MODEL_DEBUG_FREQ
        if (g_dwt_ready) {
          uint32_t cyc = (uint32_t)(DWT->CYCCNT - t0);
          g_step_cyc_sum += (uint64_t)cyc;
          g_step_cyc_cnt += 1U;
          if (cyc > g_step_cyc_max) g_step_cyc_max = cyc;
        }
        #endif

        // 基于模型输出更新业务侧重量（建议也按 50Hz 刷新，避免再被ADC 10Hz/80Hz节拍拖慢）
        SetWeightToAdcValue();
        FilterWeightValue();

        // 调试：打印真实推理频率（约每秒一次）
        #ifdef SCALE_MODEL_DEBUG_FREQ
        static uint32_t infer_calls = 0U;
        static uint32_t last_tick = 0U;
        infer_calls++;
        uint32_t now_tick = HAL_GetTick();
        if (last_tick == 0U) {
          last_tick = now_tick;
        }
        if ((now_tick - last_tick) >= 1000U) {
          float hz = (infer_calls * 1000.0f) / (float)(now_tick - last_tick);
          printf("INFER_HZ,%.2f\r\n", hz);
          // 方便对齐：SHOW 与数码管一致（已做零点/动态零点）；YFUSED 是模型融合原始输出
          printf("SHOW,%.3f\r\n", (float)my_send.now_real);
          printf("YFUSED,%.3f\r\n", yFused);

          // 周期/耗时统计（ScaleModelRunner_Step 这一段）
          if (g_step_cyc_cnt > 0U) {
            uint32_t avg = (uint32_t)(g_step_cyc_sum / (uint64_t)g_step_cyc_cnt);
            printf("STEP_CYC_AVG,%lu\r\n", (unsigned long)avg);
            printf("STEP_CYC_MAX,%lu\r\n", (unsigned long)g_step_cyc_max);

            uint32_t cyc_per_us = SystemCoreClock / 1000000U;
            if (cyc_per_us > 0U) {
              uint32_t avg_us = (avg + (cyc_per_us / 2U)) / cyc_per_us;
              uint32_t max_us = (g_step_cyc_max + (cyc_per_us / 2U)) / cyc_per_us;
              printf("STEP_US_AVG,%lu\r\n", (unsigned long)avg_us);
              printf("STEP_US_MAX,%lu\r\n", (unsigned long)max_us);
            }
          }
          g_step_cyc_sum = 0ULL;
          g_step_cyc_cnt = 0U;
          g_step_cyc_max = 0U;
          infer_calls = 0U;
          last_tick = now_tick;
        }
        #endif
        #endif

        SEND_TASK();

        // 注意：printf("%f") 很耗时，且会直接降低 50Hz 节拍执行频率。
        // 如需观察业务侧输出，请看数码管显示或启用 SCALE_MODEL_DEBUG_FREQ（会打印INFER_HZ）。
      }  
      
      // if(nine_count%10==0)
      // {
      //     printf("nine count =%d",nine_count);

      // }

      // if(decodeIns_count%20==0)
      // {
      //     printf("decodeIns_count = %d",decodeIns_count);
      // }
    // ========== 称重传感器数据采集 ==========
    // 只有在ADC启用时才进行数据采集
    if(ReadAdc()!=-1)//这里延时7.4ms
    {
      __disable_irq();

      
      CopyInsFifoToAdcUserInsFifo();

      // 注意：启用模型时，SetWeightToAdcValue/FilterWeightValue 已在 TIM1 的 50Hz 节拍中执行，
      // 这里不要重复执行，避免业务侧再被ADC 10Hz/80Hz节拍“拖慢”。
      #ifndef ENABLE_SCALE_MODEL
      SetWeightToAdcValue();//读取到压力值
      FilterWeightValue();//进行滤波
      #endif

      //printf("fsdfdsf\r\n");


      __enable_irq();

      // 注意：模型推理已在上方 TIM1 50Hz 节拍内调用，这里不要重复调用，避免时间窗口被“加速”。

		// 	count++;
	  // if(count %10==0)
	  // {      printf("count %d",count);
	  // }
     // printf("copy data to adc user ins fifo  %d\r\n",AdcUserInsFifo.InsFifo_Pointer);

      // 计算AdcUserInsFifo的平均值到InsAvgData
      //ComputeInsAvgData();

      //GetWeightValue();

     
    
      // simgle_dispose();
      
      //  printf("ADC=%d,time=%d\r\n",AdcValue,tim_data.tim_common);

      // for(int i=0;i<9;i++ )
      //   printf("%f ",InsAvgData[i]);
      
    //   // printf("\r\n");
   

      
    //   //printf("AdcValue=%d\r\n",AdcValue);

    //   //GetWeightValue();

    // }

    #ifdef Adc_Hx711
    //int adcResult = ReadAdc(); 
   
    #endif

    

  //   //HAL_UART_Transmit_DMA(&huart1, Data,8+8);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   }}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x1;
  sTime.Minutes = 0x10;
  sTime.Seconds = 0x6;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x10;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 2;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 3000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|ADC_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin ADC_CLK_Pin */
  GPIO_InitStruct.Pin = LED_Pin|ADC_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FORMAT_Pin START_ADC_Pin END_ADC_Pin */
  GPIO_InitStruct.Pin = FORMAT_Pin|START_ADC_Pin|END_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_DATA_Pin */
  GPIO_InitStruct.Pin = ADC_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Key_Select_Pin Key_End_Pin Key_Start_Pin */
  GPIO_InitStruct.Pin = Key_Select_Pin|Key_End_Pin|Key_Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  	
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
