#include "TM6137.h"
#include "Public.h"

unsigned char SigNum[24]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0X6F,\
                          0x77,0x7C,0x39,0x5E,0x79,0x71,\
                          0x08,0x48,0x49,\
                          0x76,0x38,0x0E,0x50,0x37};//0123456789ABCDEF一二三HLJRS


/*void osDelay(unsigned  int Nus)
{
 for(;Nus>0;Nus--)
	{
	  __nop();
	}
}*/



void TM1637_Start(void)
{
      
    TM1637_CLK(1);
    TM1637_DIO(1);
    osDelay(2);
    TM1637_DIO(0);
    osDelay(2);
    TM1637_CLK(0);

}

void TM1637_Stop(void)
{
	
	TM1637_CLK(0);
	osDelay(2);
	TM1637_DIO(0);
	osDelay(2);
	TM1637_CLK(1);
	osDelay(2);
	TM1637_DIO(1);
}



void TM1637_Ack(void)
{
    TM1637_CLK(0);
   // TM1637_DIO_OUT(); 

    TM1637_DIO(0);
    osDelay(2);

    TM1637_CLK(1);
    osDelay(2);
    TM1637_CLK(0);
}

void TM1637_WriteByte(unsigned char oneByte)
{
    unsigned char i;
    //TM1637_DIO_OUT();
    
    for(i=0;i<8;i++)
    {
        TM1637_CLK(0);
        if(oneByte&0x01)
        {
            TM1637_DIO(1);
        }
        else
        {
        TM1637_DIO(0);
        }
        osDelay(3);
        oneByte = oneByte>>1;
        TM1637_CLK(1);
        osDelay(3);
    }
}



void TM1637_Set_Bright(uint8_t state,uint8_t bright)
{
    TM1637_Start();
    
    TM1637_WriteByte(0x80 + (state ? 0x80:0x00) + bright);

    TM1637_Ack();
    
    TM1637_Stop();


}

void TM1637_DisplayChar(unsigned char ch,unsigned char p)
{
	if(ch>23)ch=0;//防止数组越界
	TM1637_Start();
	TM1637_WriteByte(0x44);//0x44固定模式有利于控制显示位，0x40则采用显示地址自加模式这里不使用
	TM1637_Ack();
	TM1637_Stop();

	TM1637_Start();
	
	TM1637_WriteByte(0xC0+p);//0X00地址开始显示
	TM1637_Ack();
		
	TM1637_WriteByte(SigNum[ch]);//显示
	TM1637_Ack();

	TM1637_Stop();
		
}
