#include "led.h"
#include "usart.h"
#include "delay.h"
/***********************************************************************
 函数名      ：LED_Init(void)  
 函数功能    ：LED初始化函数
 输入        ：无
 输出        ：无
                           
************************************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;          //定义结构体变量GPIO_Initure
    __HAL_RCC_GPIOB_CLK_ENABLE();           //开启GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1; //PB1,0
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	//PB1置1 
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB0置1  
}

void LedAlm(u16 time,u8* str)
{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(time);
		u2_printf("%s",str);
		LED0=1;
		LED1=1;
}


