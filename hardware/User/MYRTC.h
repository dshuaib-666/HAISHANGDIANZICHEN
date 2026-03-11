#ifndef __MY_RTC_H
#define __MY_RTC_H

//RTC 有三组，一组是秒级
//一组是ms
//一组是秒与ms结合后发送
//搞一个指针
#include "main.h"
#include "Config.h"
		  
typedef struct {
    RTC_TimeTypeDef GetTime;          /*!< File path prefix associated with the filesystem. */
    uint32_t      now_Sec;           //当前秒
    const char* partition_label;    /*!< Optional, label of SPIFFS partition to use. If set to NULL, first partition with subtype=spiffs will be used. */
    uint32_t      now_ms;
    uint32_t      now_time;
    uint32_t      now_min;
    uint32_t      now_hour;
} my_rtc_data;

typedef struct {
    uint32_t      now_Sec;           //当前秒
    const char* partition_label;    /*!< Optional, label of SPIFFS partition to use. If set to NULL, first partition with subtype=spiffs will be used. */
    uint32_t      now_ms;
    uint32_t      now_time;
    uint32_t      now_min;
    uint32_t      now_hour;
    uint32_t      Ten_ms_count;

    uint32_t      adc_ms;
    uint32_t      uart_ms;


    
    char        adc_time_String[20];
    char        uart_time_String[20];
    uint32_t      tim_common;

} my_tim_data;

// void MYRTC_GET_DATA(my_rtc_data * rtc_data);//获取到当前的rtc值

// void MYRTC_GET_TEST(void);//测试函数
typedef enum     //选择是哪个led                 
{   	MYADC                     = 0,
		MYNINE                      = 1,   
        NO                          =2,                                                                     
}GET_MODE;
// void MYTIM_GET(u8 staus);
void TIM_MS_COMPUTER(my_tim_data * tim_data);

void TIM_COMPUTER(my_tim_data * tim_data);

//adc uart 获取ms值函数
void TIM_GET_TASK_MS(u8 staus);
extern my_tim_data tim_data;

#endif // !__RTC_
