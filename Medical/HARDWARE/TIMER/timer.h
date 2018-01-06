#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

extern TIM_HandleTypeDef TIM2_Handler;  //定时器2句柄 
extern TIM_HandleTypeDef TIM7_Handler;  //定时器7句柄
extern TIM_HandleTypeDef TIM9_Handler;  //定时器9句柄
extern TIM_HandleTypeDef TIM10_Handler; //定时器10句柄 


void TIM2_Init(u16 arr,u16 psc);
void TIM7_Init(u16 arr,u16 psc);        //定时器7初始化函数
void TIM9_Init(u16 arr,u16 psc);        //定时器7初始化函数
void TIM10_Init(u16 arr,u16 psc);       //定时器10初始化函数

void TIM10_Stop(void);                  //关闭定时器10
void TIM9_Stop(void);                   //关闭定时器9
void TIM2_Stop(void);                   //关闭定时器2
#endif




