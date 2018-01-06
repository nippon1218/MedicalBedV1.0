#include "pwm.h"
#include "led.h"
#include "delay.h"
 	 
TIM_HandleTypeDef TIM_Handler;          //定时器PWM句柄 
TIM_OC_InitTypeDef TIM_CHXHandler;	    //定时器通道句柄


/***********************************************************************
 函数名      ：TIM8_PWM_CHANNEL_1_START(u16 arr,u16 psc)  
 函数功能    ：TIM8通道 1 PWM输出函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM8_PWM_CHANNEL_1_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM8;                              //定时器8
    TIM_Handler.Init.Prescaler=psc;                         //定时器分频
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //向上计数模式
    TIM_Handler.Init.Period=arr;                            //自动重装载值
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                         //初始化PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                  //模式选择PWM1
    TIM_CHXHandler.Pulse=arr/2;                             //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;           //输出比较极性为低 
	HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_1);//配置TIM8通道1
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_1);          //开启PWM通道1
}

/***********************************************************************
 函数名      ：TIM8_PWM_CHANNEL_2_START(u16 arr,u16 psc)  
 函数功能    ：TIM8通道 2 PWM输出函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM8_PWM_CHANNEL_2_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM8;                               //定时器8
    TIM_Handler.Init.Prescaler=psc;                          //定时器分频
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //向上计数模式
    TIM_Handler.Init.Period=arr;                             //自动重装载值
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                          //初始化PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                   //模式选择PWM1 
    TIM_CHXHandler.Pulse=arr/2;                              //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;            //输出比较极性为低 
    HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_2);//配置TIM8通道2
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_2);           //开启PWM通道2
}

/***********************************************************************
 函数名      ：TIM8_PWM_CHANNEL_3_START(u16 arr,u16 psc)  
 函数功能    ：TIM8通道 3 PWM输出函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM8_PWM_CHANNEL_3_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM8;                              //定时器8
    TIM_Handler.Init.Prescaler=psc;                         //定时器分频
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //向上计数模式
    TIM_Handler.Init.Period=arr;                            //自动重装载值
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                         //初始化PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                  //模式选择PWM1
    TIM_CHXHandler.Pulse=arr/2;                             //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;           //输出比较极性为低 
    HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_3);//配置TIM8通道3
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_3);          //开启PWM通道3
}

/***********************************************************************
 函数名      ：TIM8_PWM_CHANNEL_4_START(u16 arr,u16 psc)  
 函数功能    ：TIM8通道 4 PWM输出函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM8_PWM_CHANNEL_4_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM8;                               //定时器8
    TIM_Handler.Init.Prescaler=psc;                          //定时器分频
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //向上计数模式
    TIM_Handler.Init.Period=arr;                             //自动重装载值
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                          //初始化PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                   //模式选择PWM1
    TIM_CHXHandler.Pulse=arr/2;                              //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;            //输出比较极性为低 
	  HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_4);  //配置TIM8通道4
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_4);           //开启PWM通道4
}

/***********************************************************************
 函数名      ：TIM5_PWM_CHANNEL_4_START(u16 arr,u16 psc)  
 函数功能    ：TIM5通道 4 PWM输出函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM5_PWM_CHANNEL_4_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM5;                               //定时器5
    TIM_Handler.Init.Prescaler=psc;                          //定时器分频
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //向上计数模式
    TIM_Handler.Init.Period=arr;                             //自动重装载值
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                          //初始化PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                   //模式选择PWM1
    TIM_CHXHandler.Pulse=arr/2;                              //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;            //输出比较极性为低 
	  HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_4);  //配置TIM5通道4
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_4);           //开启PWM通道4
}

/***********************************************************************
 函数名      ：TIM3_PWM_CHANNEL_2_START(u16 arr,u16 psc)  
 函数功能    ：TIM3通道 2 PWM输出函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM3_PWM_CHANNEL_2_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM3;                              //定时器3
    TIM_Handler.Init.Prescaler=psc;                         //定时器分频
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //向上计数模式
    TIM_Handler.Init.Period=arr;                            //自动重装载值
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                         //初始化PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                  //模式选择PWM1
    TIM_CHXHandler.Pulse=arr/2;                             //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;           //输出比较极性为低 
	HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_2);   //配置TIM3通道2
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_2);          //开启PWM通道2
}

/***********************************************************************
 函数名      ：TIM1_PWM_CHANNEL_4_START(u16 arr,u16 psc)  
 函数功能    ：TIM1通道 4 PWM输出函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM1_PWM_CHANNEL_4_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM1;                                //定时器1
    TIM_Handler.Init.Prescaler=psc;                           //定时器分频
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;          //向上计数模式
    TIM_Handler.Init.Period=arr;                              //自动重装载值
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                            //初始化PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                     //模式选择PWM1
    TIM_CHXHandler.Pulse=arr/2;                                //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;              //输出比较极性为低 
	  HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_4);//配置TIM1通道4
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_4);             //开启PWM通道4
}

/***********************************************************************
 函数名      ：HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) 
 函数功能    ：定时器底层驱动，时钟使能，引脚配置
			   此函数会被HAL_TIM_PWM_Init()调用
 输入        ：htim:定时器句柄
 输出        ：无
                           
************************************************************************/
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM1_CLK_ENABLE();			//使能定时器1
	__HAL_RCC_TIM3_CLK_ENABLE();			//使能定时器3
	__HAL_RCC_TIM5_CLK_ENABLE();			//使能定时器5
	__HAL_RCC_TIM8_CLK_ENABLE();			//使能定时器8
    __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
	__HAL_RCC_GPIOI_CLK_ENABLE();           //开启GPIOI时钟
	  
	GPIO_Initure.Pin=GPIO_PIN_11;           //PA11
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF1_TIM1;	//PC复用为TIM1_CH4	
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
		
    GPIO_Initure.Pin=GPIO_PIN_7;            //PA7
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF2_TIM3;  //PC复用为TIM3_CH2	
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
		
	GPIO_Initure.Pin=GPIO_PIN_0;            //PI0
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF2_TIM5;  //PC复用为TIM5_CH4	
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7; //PI2/5/6/7
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;     //复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;         //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;    //高速
	GPIO_Initure.Alternate=GPIO_AF3_TIM8;  //PC复用为TIM8
	HAL_GPIO_Init(GPIOI,&GPIO_Initure);	
}





