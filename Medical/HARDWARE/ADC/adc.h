#ifndef __ADC_H
#define __ADC_H
#include "sys.h"

void MY_ADC_Init(void);                 //初始化ADC
u16 Get_Adc(u32 ch);                    //获得ADC值
u16 Get_Adc_Average(u32 ch,u8 times);   //获取指定通道的转换值

#endif