#ifndef _DELAY_H
#define _DELAY_H
#include <sys.h>	  

void delay_init(u8 SYSCLK);   //初始化函数
void delay_ms(u16 nms);       //延时ms函数
void delay_us(u32 nus);       //延时us函数
#endif

