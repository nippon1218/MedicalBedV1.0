#ifndef __HX711_H__
#define __HX711_H__
#include "sys.h"

//串口通讯线由管脚PD_SCK和DOUT组成，用来输出数据，选择输入通道和增益

/****************传感器时钟口配置函数***********************/

#define HX711_SCK_INPUT()   {GPIOB->MODER &=~(3<<(5*2));GPIOB->MODER |=0<<(5*2);}	  //PB5输入模式
#define HX711_SCK_OUTPUT()  {GPIOB->MODER &=~(3<<(5*2));GPIOB->MODER |=1<<(5*2);}     //PB5输出模式

/****************传感器数据口配置函数***********************/

#define HX711_DOUT_INPUT()  {GPIOH->MODER &=~(3<<(10*2));GPIOH->MODER |=0<<(10*2);}	  //PH10输入模式
#define HX711_DOUT_OUTPUT() {GPIOH->MODER &=~(3<<(10*2));GPIOH->MODER |=1<<(10*2);}   //PH10输出模式
 
/*********************IO操作函数****************************/

#define	HX711_SCK_OUT PBout(5)         //SCK端口输出    PB5
#define	HX711_SCK_IN  PBin(5)          //SCK端口输入    PB5 

#define	HX711_DOUT_OUT PHout(10)       //DOUT端口输出	PH10
#define	HX711_DOUT_IN  PHin(10)        //DOUT端口输入	PH10

unsigned long HX711_Read(void);        //传感器采集数据函数
unsigned long Get_Weight(void);        //称重函数
unsigned long filter(void);            //数据处理函数
void HX711_Init(void);                 //初始化HX711的IO口函数

#endif















