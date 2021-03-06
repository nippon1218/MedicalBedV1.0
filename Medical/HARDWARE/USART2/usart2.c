#include "sys.h"
#include "usart2.h"	  
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "timer.h"  

u8 aRxBuffer[BUFFSIZE];//HAL库使用的串口接收缓冲
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//接收缓冲,最大USART2_MAX_RECV_LEN个字节.
u8  USART2_TX_BUF[USART2_MAX_SEND_LEN]; 			//发送缓冲,最大USART2_MAX_SEND_LEN字节

//通过判断接收连续2个字符之间的时间差不大于10ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过10ms,则认为不是1次连续数据.也就是超过10ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
vu16 USART2_RX_STA=0;  
UART_HandleTypeDef UART2_Handler; //UART句柄
//TIM_HandleTypeDef TIM7_Handler;   //TIM句柄
void USART2_IRQHandler(void)
{
	u8 res;	      
	if(__HAL_UART_GET_FLAG(&UART2_Handler,UART_FLAG_RXNE)!=RESET)//接收到数据
	{	 
		HAL_UART_Receive(&UART2_Handler,&res,1,1000);
//		res=USART2->DR; 			 
		if((USART2_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
			if(USART2_RX_STA<USART2_MAX_RECV_LEN)	//还可以接收数据
			{
//				__HAL_TIM_SetCounter(&TIM7_Handler,0);	
				TIM7->CNT=0;         				//计数器清空	
				if(USART2_RX_STA==0) 				//使能定时器7的中断 
				{
//					__HAL_RCC_TIM7_CLK_ENABLE();            //使能TIM7时钟
					TIM7->CR1|=1<<0;     			//使能定时器7
				}
				USART2_RX_BUF[USART2_RX_STA++]=res;	//记录接收到的值	 
			}else 
			{
				USART2_RX_STA|=1<<15;				//强制标记接收完成
			} 
		}
	}  				 											 
}   

void usart2_init(u32 bound)
{  	 
	//UART 初始化设置
	UART2_Handler.Instance=USART2;					    //USART1
	UART2_Handler.Init.BaudRate=bound;				    //波特率
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART2_Handler);					    //HAL_UART_Init()会使能UART2
	TIM7_Int_Init(1000-1,8400-1);		//100ms中断
	HAL_UART_Receive_IT(&UART2_Handler, (u8 *)aRxBuffer, BUFFSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
	USART2_RX_STA=0;		//清零
	TIM7->CR1&=~(1<<0);        //关闭定时器7
//	__HAL_TIM_DISABLE(&TIM7_Handler);      //关闭定时器7
}
//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOA时钟
	__HAL_RCC_USART2_CLK_ENABLE();			//使能USART1时钟
	
	GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
	GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
	GPIO_Initure.Alternate=GPIO_AF7_USART2;	//复用为USART1
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB10

	GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB11
	
  __HAL_UART_DISABLE_IT(huart,UART_IT_TC);
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//开启接收中断
	HAL_NVIC_EnableIRQ(USART2_IRQn);				//使能USART2中断
	HAL_NVIC_SetPriority(USART2_IRQn,2,3);			//抢占优先级3，子优先级3	
}
//串口2,printf 函数
//确保一次发送数据不超过USART2_MAX_SEND_LEN字节
void u2_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART2_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART2_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((USART2->SR&0X40)==0);			//循环发送,直到发送完毕   
		USART2->DR=USART2_TX_BUF[j];  
	} 
}