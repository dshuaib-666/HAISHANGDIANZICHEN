#include "SEND.h"
#include <stdio.h>
#include "Config.h"
#include "MYRTC.h"
#include "ScaleModelRunner.h"
//进行发送的任务

// 数码管显示函数（在User/ProcessData.c中实现）
extern void Display_SendToDigitTube(float value);

// 生成5到10之间的随机阈值
static float GetRandomThreshold(void) {
    static uint32_t seed = 0;
    if (seed == 0) {
        seed = HAL_GetTick(); // 使用系统时间作为随机种子
    }
    
    // 简单的线性同余随机数生成器
    seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
    
    // 生成5到10之间的随机数
    float random_threshold = 10.0f + (float)(seed % 6) / 5.0f; // 5.0 ~ 10.0
    
    return random_threshold;
}

// 稳定性判断函数
static float ProcessStableValue(float current_value) {
    static float stable_value = 0.0f;         // 稳定值A
    static uint8_t has_stable_value = 0;      // 是否已确定稳定值A
    
    // 如果值在-50到50范围内，直接返回原值
    if (current_value >= -50.0f && current_value <= 50.0f) {
        has_stable_value = 0;  // 重置稳定状态
        return current_value;
    }
    
    // 如果还没有稳定值A，将当前值设为A
    if (!has_stable_value) {
        stable_value = current_value;
        has_stable_value = 1;
        printf("稳定值A已确定: %.2f\n", stable_value);
        return stable_value;
    }
    
    // 已经有稳定值A，使用随机阈值检查当前值是否超过范围
    float threshold = GetRandomThreshold();
    if (current_value > stable_value + threshold || current_value < stable_value - threshold) {
        // 超过阈值，将当前值设为新的A
        stable_value = current_value;
        printf("稳定值A已更新为B: %.2f (阈值: %.2f)\n", stable_value, threshold);
    }
    
    // 返回稳定值A（可能已更新）
    return stable_value;
}

extern double FilteredrCurrentWeightValue;
extern double Acceleration[3];//三轴加速度
extern double AngularVelocity[3];//三轴角速度
extern double RealaAngle[3];//欧拉角
extern double FOUR_element[4];
extern my_tim_data tim_data;
extern uint32_t AdcValue;
my_Send_data my_send;
extern float disfilter;
void SEND_TASK(void)
{

        float ADC_SEC = tim_data.adc_ms/1000.0f;
        float UART_SEC= tim_data.uart_ms/1000.0f; 

        
        // USB输出：
        // - 启用模型时：只输出 y_fused，并用前缀 "YFUSED," 与其它输出区分
        // - 未启用模型时：保持原有全量输出不变
        #ifdef ENABLE_SCALE_MODEL
        // 显示业务侧最终重量（已做上电零点/动态零点）
        // 现在 Display_SendToDigitTube() 已改为USART3非阻塞发送，不会再阻塞主循环。
        float display_value = ProcessStableValue((float)my_send.now_real);
        Display_SendToDigitTube(display_value);

        // 注意：printf("%f") 在 Cortex-M4 上非常耗时，会显著降低推理频率。
        // - 打开 SCALE_MODEL_DEBUG_FREQ 时：仅在 main.c 内每秒打印一次 INFER_HZ（避免影响测频）
        // - 未打开时：默认以低频输出（5Hz）保留观测能力
        #ifndef SCALE_MODEL_DEBUG_FREQ
        static uint32_t usbDiv = 0U;
        usbDiv++;
        if (usbDiv >= 10U) {   // 50Hz下约5Hz
            usbDiv = 0U;
            printf("YFUSED,%f\r\n", ScaleModel_yFused);
        }
        #endif
        #else
        // 拷贝快照（仅在全量输出时需要）
        int32_t ADC_Value = (int32_t)AdcValue;
        double acc[3] = {Acceleration[0], Acceleration[1], Acceleration[2]};
        double gyro[3] = {AngularVelocity[0], AngularVelocity[1], AngularVelocity[2]};
        double quat[4] = {FOUR_element[0], FOUR_element[1], FOUR_element[2], FOUR_element[3]};

        printf("%f,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lfpp\r\n",
            ADC_SEC, ADC_Value,
               UART_SEC, acc[0], acc[1], acc[2],
            gyro[0], gyro[1], gyro[2],
               quat[0], quat[1], quat[2], quat[3]);
        #endif
    
    

    
    // printf("FilteredrCurrentWeightValu:%f  AdcValue:%d\r\n",FilteredrCurrentWeightValue,AdcValue);
    //  printf("%lf,%s,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%spp\r\n", my_send.now_real,tim_data.adc_time_String,
    //      Acceleration[0],Acceleration[1],Acceleration[2],
    //      AngularVelocity[0],AngularVelocity[1],AngularVelocity[2],
    //      FOUR_element[0],FOUR_element[1],FOUR_element[2] ,FOUR_element[3] ,tim_data.uart_time_String
	
    //  );
}
