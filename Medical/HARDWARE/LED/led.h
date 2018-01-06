#ifndef _LED_H
#define _LED_H
#include "sys.h"
 	

#define LED0 PBout(1)   //LED0
#define LED1 PBout(0)   //LED1

void LED_Init(void);    //LED初始化函数
void LedAlm(u16 time,u8* str);		//Led警报函数
#endif
