#include "MYRTC.h"
#include <stdio.h>
#include "Config.h"
#include "string.h"
my_rtc_data rtc_data;
extern RTC_HandleTypeDef hrtc;
my_tim_data tim_data;

// void MYRTC_GET_DATA(my_rtc_data * rtc_data)//获取到当前的rtc值
// {
  
    
//     // RTC_TimeTypeDef GetTime;   //获取时间结构体
//     HAL_RTC_WaitForSynchro(&hrtc);
//     HAL_RTC_GetTime(&hrtc, &rtc_data->GetTime, RTC_FORMAT_BCD);
//     // HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);
    
    

//     // HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
//     rtc_data->now_time=hrtc.Instance->TR;
  
//     uint32_t sec_units = (rtc_data->now_time & 0xF);          // [0:3] SU
//     uint32_t sec_tens  = (rtc_data->now_time >> 4) & 0x7;     // [4:6] ST
//     rtc_data->now_Sec  = sec_tens * 10 + sec_units;

//     // 分钟
//     uint32_t min_units = (rtc_data->now_time >> 8) & 0xF;     // [8:11] MNU
//     uint32_t min_tens  = (rtc_data->now_time >> 12) & 0x7;    // [12:14] MNT
//     rtc_data->now_min  = min_tens * 10 + min_units;

//     // 小时
//     uint32_t hour_units = (rtc_data->now_time >> 16) & 0xF;   // [16:19] HU
//     uint32_t hour_tens  = (rtc_data->now_time >> 20) & 0x3;   // [20:21] HT
//     rtc_data->now_hour  = hour_tens * 10 + hour_units;

//     rtc_data->now_ms=rtc_data->GetTime.SubSeconds*(1000.0f/256.0f);//获取到当前ms值
 
	
	 
//     /* Get the RTC current Date */
    


// }

void TIM_MS_COMPUTER(my_tim_data * tim_data)
{
    tim_data->Ten_ms_count++;

}

void TIM_COMPUTER(my_tim_data * tim_data)
{
    uint32_t ms=tim_data->Ten_ms_count;
    tim_data->now_hour = ms / 3600000; 
    ms %= 3600000;
    tim_data->now_min = ms / 60000; 
    ms %= 60000; 
    tim_data->now_Sec = ms / 1000;
    
}
//adc uart 获取ms值函数
void TIM_GET_TASK_MS(u8 staus)
{
    // printf("tim_data.now_ms %d",tim_data.Ten_ms_count);
    u32 ms=tim_data.Ten_ms_count;
    if (staus == MYADC)
    {
        tim_data.adc_ms=ms;
        return;        
    }
    else if(staus==MYNINE)
    {
        tim_data.uart_ms=ms;
        return;  
    }
}
// }
// //换方案吧，RTC不靠谱，而且麻烦
// //使用定时器得了
// void TIM_GET_DATA(my_tim_data * tim_data)
// {
// 	//5ms
    
//     if(tim_data->Ten_ms_count>=200)
//     {   tim_data->Ten_ms_count=0;
//         tim_data->now_Sec++;
//     }
//     if(tim_data->now_Sec>=60)
//     {    tim_data->now_Sec=0;
//         tim_data->now_min++;     }
//     if(tim_data->now_min>=60)
// 	{   tim_data->now_min=0;
//         tim_data->now_hour++;    }
//     if(tim_data->now_hour>=24)
//     {    tim_data->now_hour=0;   }
//     tim_data->now_ms= tim_data->Ten_ms_count*5;
//     tim_data->tim_common=tim_data->tim_common+5;
// }
// void MYRTC_GET_TEST(void)//测试函数
// {
//     TIM_GET_DATA(&tim_data);

//     //随后进行打印
//     #ifdef MY_Debug
//         printf("TIM:%d,%d,%d\r\n",(int)tim_data.now_min,(int)tim_data.now_Sec,(int)tim_data.now_ms);


//     #endif // !MY_Debug
    
// }
// void MYTIM_GET(u8 staus)
// {
//     if (staus == MYADC)
//     {
//         TIM_GET_DATA(&tim_data);
//         snprintf(tim_data.adc_time_String, sizeof(tim_data.adc_time_String), 
//         "%02d--%02d--%02d--%03d", 
//         (int)tim_data.now_hour, 
//         (int)tim_data.now_min, 
//         (int)tim_data.now_Sec, 
//         (int)tim_data.now_ms);
        
//     }
//     else if(staus==MYNINE)
//     {
//         TIM_GET_DATA(&tim_data);
//         snprintf(tim_data.uart_time_String, sizeof(tim_data.uart_time_String), 
//         "%02d--%02d--%02d--%03d", 
//         (int)tim_data.now_hour, 
//         (int)tim_data.now_min, 
//         (int)tim_data.now_Sec, 
//         (int)tim_data.now_ms);
//     }
//     else if(staus==NO)
//     {
        
//         TIM_GET_DATA(&tim_data);
//     }
// }



