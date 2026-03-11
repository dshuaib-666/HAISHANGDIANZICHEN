#ifndef _SEND_H
#define _SEND_H

void SEND_TASK(void);

void MAIN_TASK(void);

typedef struct {
    float      now_real;           //处理之后的真实值
    float      partition_label;          /*!< Optional, label of SPIFFS partition to use. If set to NULL, first partition with subtype=spiffs will be used. */


} my_Send_data;


extern my_Send_data my_send;


#endif // !_SEND_H
