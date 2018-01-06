#ifndef __PUMP
#define __PUMP
#include "sys.h"

/****************水箱***********************/

#define DIR_SB      PBout(11)          //水泵PB12

//#define DIR_SB     PEout(3)           //水泵PB12,原来的线

#define DIR_QB     PHout(2)           //气泵
#define DIR_HG     PBout(10)          //烘干
#define DIR_XZFPQ   PEout(3)          //旋转分配器
//#define DIR_JRAM   PBout(11)          //按摩
#define DIR_JR     PHout(3)           //加热-继电器
#define DIR_WD     PHout(8)           //温度传感器

void Pump_Init(void);                 //水箱IO口初始化

/************冲洗烘干推杆********************/

#define DIR_CXHG      PCout(1)          //冲洗烘干推杆方向口
#define PWM_CXHG      PBout(13)         //冲洗烘干脉冲方向口

void Push_Rod_Swash_Dry_Init(void);     //冲洗烘干电动杆
void Push_Rod_Swash_Dry(u8 dir,u32 n);  //冲洗烘干电动杆
void Push_Rod_Swash_Dry1(u8 dir,u32 n); //冲洗烘干电动杆

void Push_Rod_Swash(u8 dir,u32 n);      //冲洗电动杆
void Push_Rod_Dry(u8 dir,u32 n);        //烘干电动杆

//串口函数
void Uart_Push_Rod_Swash_Dry(u8 dir,u32 n);  //冲洗烘干电动杆
void Uart_Push_Rod_Swash(u8 dir,u32 n);      //冲洗电动杆
void Uart_Push_Rod_Dry(u8 dir,u32 n);        //烘干电动杆

/***************光电开关********************/

#define GD3_Start       PEin(6)       //3号电机光电开关
#define GD3_Left_End    PBin(15)      //左翻终止
#define GD3_Right_End   PBin(14)      //右翻终止

#define GD4_Start       PEin(5)       //4号电机光电开关
#define GD4_Left_End    PEin(4)       //左翻终止
#define GD4_Right_End   PHin(6)       //右翻终止

#define GD5_Start       PEin(2)       //5号电机光电开关
#define GD5_Left_End    PCin(13)      //左翻终止
#define GD5_Right_End   PHin(9)   	  //右翻终止

#define GD6_Start       PCin(7)       //6号电机光电开关
#define GD6_End   		PCin(6) 
#define GD7_Start       PCin(8)       //7号电机光电开关
#define GD7_End       	PCin(9) 

#define Liq_Sensor      PHin(7)       //液位传感器

#define GD3S 1
#define GD3LE 2
#define GD3RE 3

#define GD4S	4
#define GD4LE 5
#define GD4RE 6

#define GD5S 7
#define GD5LE 8
#define GD5RE	9

#define GD6S 10
#define GD6E 11

#define GD7S 12
#define GD7E 13

#define GD34S	14
#define GD34LE	15
#define GD34RE	16

void Sensor_Init(void);               //光电及传感器初始化

#endif


