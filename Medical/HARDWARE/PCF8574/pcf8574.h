#ifndef __PCF8574_H
#define __PCF8574_H
#include "sys.h"
#include "myiic.h"

#define PCF8574_INT  PBin(12)   //PCF8574 INT脚
#define PCF8574_ADDR 	0X40	//PCF8574地址(左移了一位)

/*******************PCF8574各个IO的功能******************/

#define BEEP_IO           0		//蜂鸣器控制引脚  	    P0
//#define AP_INT_IO       1   	//AP3216C中断引脚	    P1
//#define DCMI_PWDN_IO    2    	//DCMI的电源控制引脚	    P2
#define USB_PWR_IO        3    	//USB电源控制引脚	    P3
//#define EX_IO      	  4    	//扩展IO,自定义使用   	P4
//#define MPU_INT_IO      5   	//MPU9250中断引脚	    P5
#define RS485_RE_IO       6    	//RS485_RE引脚		    P6
#define ETH_RESET_IO      7    	//以太网复位引脚		    P7

#define EXIO1       1   	//AP3216C中断引脚	    P1
#define EXIO2       2    	//DCMI的电源控制引脚	    P2
#define EXIO3      	4    	//扩展IO,自定义使用   	P4
#define EXIO4       5   	//MPU9250中断引脚	    P5


u8 PCF8574_Init(void);                       //PCF8574初始化函数
u8 PCF8574_ReadOneByte(void);                //读取PCF8574的8位IO值
void PCF8574_WriteOneByte(u8 DataToWrite);   //向PCF8574写入8位IO值 
void PCF8574_WriteBit(u8 bit,u8 sta);        //设置PCF8574某个IO的高低电平
u8 PCF8574_ReadBit(u8 bit);                  //读取PCF8574的某个IO的值 

void BeepRun(u8 num,u16 time);


#endif

