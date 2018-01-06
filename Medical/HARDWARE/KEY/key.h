#ifndef _KEY_H
#define _KEY_H
#include "sys.h"

#define WK_UP       PAin(0) //WKUP按键PA0
#define WKUP_PRES   2       //WK_UP 按下时的返回值

//利用按键扫描的方法记录电机传上来实际运行脉冲值
#define Motor3_Tim      PGin(14)    //3号电机
#define Motor3_Alm      PGin(13)

#define Motor4_Tim      PIin(4)     //5号电机
#define Motor4_Alm      PGin(11)
#define Motor5_Tim      PIin(1)     //4号电机
#define Motor5_Alm      PCin(5)

#define Motor6_Tim      PHin(14)    //6号电机
#define Motor6_Alm      PHin(15)
#define Motor7_Tim      PGin(12)    //7号电机


//#define Motor7_Alm      PAin(12)

//利用按键扫描的方法记录电机传上来实际运行脉冲值
#define Motor3_Tim_PRES 3
#define Motor3_Alm_PRES 4
#define Motor4_Tim_PRES 5
#define Motor4_Alm_PRES 6
#define Motor5_Tim_PRES 7
#define Motor5_Alm_PRES 8
#define Motor6_Tim_PRES 9
#define Motor6_Alm_PRES 10
#define Motor7_Tim_PRES 11
#define Motor7_Alm_PRES 12

#define M3TIM 1
#define M3ALM 2
#define M4TIM 3
#define M4ALM 4
#define M5TIM 5
#define M5ALM 6
#define M6TIM 7
#define M6ALM 8
#define M7TIM 9
#define M7ALM 10


void KEY_Init(void);    //按键IO口初始化函数
u8 KEY_Scan(u8 mode);   //按键处理函数

u8 KeyCheck(u8 num);
void KeyCheckAll(void);

#endif
