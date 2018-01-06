#include "timer.h"
#include "led.h"

extern vu16 UART4_RX_LEN;            //串口4接收状态标志位

TIM_HandleTypeDef TIM10_Handler;     //定时器10句柄，用于功能函数定时时间控制
TIM_HandleTypeDef TIM9_Handler;		 //定时器2 句柄
TIM_HandleTypeDef TIM7_Handler;      //定时器7 句柄，用于WiFi
TIM_HandleTypeDef TIM2_Handler;      //定时器2 句柄，用于烘干冲洗推杆

/***********************************************************************
 函数名      ：TIM2_Init(u16 arr,u16 psc)  
 函数功能    ：定时器初始化函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM2_Init(u16 arr,u16 psc)
{
	__HAL_RCC_TIM2_CLK_ENABLE();			                  //使能定时器2
	
	TIM2_Handler.Instance=TIM2;                               //通用定时器2
    TIM2_Handler.Init.Prescaler=psc;                          //分频系数
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //向上计数器
    TIM2_Handler.Init.Period=arr;                             //自动装载值
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   //时钟分频因子
    HAL_TIM_Base_Init(&TIM2_Handler);    
    HAL_TIM_Base_Start_IT(&TIM2_Handler); //使能定时器2和定时器2更新中断：TIM_IT_UPDATE									 
}

/***********************************************************************
 函数名      ：TIM7_Init(u16 arr,u16 psc)  
 函数功能    ：基本定时器7中断初始化函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM7_Init(u16 arr,u16 psc)
{
	__HAL_RCC_TIM7_CLK_ENABLE();			                  //使能定时器7
	
	TIM7_Handler.Instance=TIM7;                               //通用定时器7
    TIM7_Handler.Init.Prescaler=psc;                          //分频系数
    TIM7_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //向上计数器
    TIM7_Handler.Init.Period=arr;                             //自动装载值
    TIM7_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   //时钟分频因子
    HAL_TIM_Base_Init(&TIM7_Handler);    
    HAL_TIM_Base_Start_IT(&TIM7_Handler); //使能定时器7和定时器7更新中断：TIM_IT_UPDATE									 
}

/***********************************************************************
 函数名      ：TIM9_Init(u16 arr,u16 psc)  
 函数功能    ：基本定时器7中断初始化函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM9_Init(u16 arr,u16 psc)
{
	__HAL_RCC_TIM9_CLK_ENABLE();			                  //使能定时器9
	
	TIM9_Handler.Instance=TIM9;                               //通用定时器9
    TIM9_Handler.Init.Prescaler=psc;                          //分频系数
    TIM9_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //向上计数器
    TIM9_Handler.Init.Period=arr;                             //自动装载值
    TIM9_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   //时钟分频因子
    HAL_TIM_Base_Init(&TIM9_Handler);    
    HAL_TIM_Base_Start_IT(&TIM9_Handler); //使能定时器9和定时器9更新中断：TIM_IT_UPDATE									 
}

/***********************************************************************
 函数名      ：TIM10_Init(u16 arr,u16 psc) 
 函数功能    ：定时器10初始化函数
 输入        ：arr：自动重装值；psc：时钟预分频数
 输出        ：无
                           
************************************************************************/
void TIM10_Init(u16 arr,u16 psc)
{
    __HAL_RCC_TIM10_CLK_ENABLE();			                  //使能定时器10
	
		TIM10_Handler.Instance=TIM10;                             //通用定时器10
    TIM10_Handler.Init.Prescaler=psc;                         //分频系数
    TIM10_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //向上计数器
    TIM10_Handler.Init.Period=arr;                            //自动重装载值
    TIM10_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;  //时钟分频因子
    HAL_TIM_Base_Init(&TIM10_Handler);   
    HAL_TIM_Base_Start_IT(&TIM10_Handler); //使能定时器10和定时器10更新中断：TIM_IT_UPDATE   
}


/***********************************************************************
 函数名      ：HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)  
 函数功能    ：定时器底册驱动，开启时钟，设置中断优先级
               此函数会被HAL_TIM_Base_Init()函数调用
 输入        ：定时器
 输出        ：无
                           
************************************************************************/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM7)
	{
		__HAL_RCC_TIM7_CLK_ENABLE();            //使能TIM7时钟
		HAL_NVIC_SetPriority(TIM7_IRQn,0,1);    //设置中断优先级，抢占优先级0，子优先级1
		HAL_NVIC_EnableIRQ(TIM7_IRQn);          //开启TIM7中断   
	}		
}


/***********************************************************************
 函数名      ：TIM7_IRQHandler(void)
 函数功能    ：定时器7中断服务程序
 输入        ：无
 输出        ：无
                           
************************************************************************/
void TIM7_IRQHandler(void)
{ 	    		    
	UART4_RX_LEN|=1<<15;	                                       //标记接收完成标志位
	__HAL_TIM_CLEAR_FLAG(&TIM7_Handler,TIM_EventSource_Update );   //清除TIM7更新中断标志  
	TIM7->CR1&=~(1<<0);     			                           //关闭定时器7     											 
} 

/***********************************************************************
 函数名      ：TIM2_Stop(void)
 函数功能    ：关闭定时器2
 输入        ：无
 输出        ：无
                           
************************************************************************/
void TIM2_Stop(void)
{
	  HAL_TIM_Base_Stop_IT(&TIM2_Handler);  //关闭定时器10和定时器10更新中断
}

/***********************************************************************
 函数名      ：TIM2_Stop(void)
 函数功能    ：关闭定时器2
 输入        ：无
 输出        ：无
                           
************************************************************************/
void TIM9_Stop(void)
{
	  HAL_TIM_Base_Stop_IT(&TIM9_Handler);  //关闭定时器10和定时器10更新中断
}


/***********************************************************************
 函数名      ：TIM10_Stop(void)
 函数功能    ：关闭定时器10
 输入        ：无
 输出        ：无
                           
************************************************************************/
void TIM10_Stop(void)
{
	  HAL_TIM_Base_Stop_IT(&TIM10_Handler);  //关闭定时器10和定时器10更新中断
}

