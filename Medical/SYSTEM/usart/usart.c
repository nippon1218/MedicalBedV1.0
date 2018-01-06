#include "usart.h"
#include "modbus_master.h"
#include "led.h"
#include "vk3214.h"
 	 
//如果使用os,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//os 使用	  
#endif
  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 

//#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

u8 aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲

//串口发送缓存区
__align(8) u8 USART6_TX_BUF[USART6_MAX_SEND_LEN]; 	//发送缓冲,最大USART4_MAX_SEND_LEN字节 
__align(8) u8 UART4_TX_BUF[UART4_MAX_SEND_LEN]; 	//发送缓冲,最大USART4_MAX_SEND_LEN字节 
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//发送缓冲,最大USART3_MAX_SEND_LEN字节 
__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//发送缓冲,最大USART2_MAX_SEND_LEN字节 

//串口接收缓存区
u8 USART6_RX_BUF[USART6_MAX_RECV_LEN]; 		   	    //接收缓冲,最大USART4_MAX_RECV_LEN个字节.
u8 UART4_RX_BUF[UART4_MAX_RECV_LEN]; 		   	    //接收缓冲,最大USART4_MAX_RECV_LEN个字节.
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART3_MAX_RECV_LEN个字节.
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//接收缓冲,最大USART2_MAX_RECV_LEN个字节.


u8 modbus_ready;

//通过判断接收连续2个字符之间的时间差不大于100ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过100ms,则认为不是1次连续数据.也就是超过100ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
u16 USART6_RX_LEN=0; 
u16 UART4_RX_LEN=0; 
u16 USART3_RX_LEN=0; 
u16 USART2_RX_LEN=0;
UART_HandleTypeDef UART1_Handler; //UART句柄
UART_HandleTypeDef UART2_Handler; //UART句柄
UART_HandleTypeDef UART3_Handler; //UART句柄
UART_HandleTypeDef UART4_Handler; //UART句柄
UART_HandleTypeDef UART6_Handler; //UART句柄

/***********************************************************************
 函数名      ：uart_init()  
 函数功能    ：初始化IO 串口1 
 输入        ：bound:波特率
 输出        ：无
                           
************************************************************************/
void uart_init(u32 bound)
{	
	//UART 初始化设置
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //波特率
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()会使能UART1
}

/***********************************************************************
 函数名      ：usart2_init()  
 函数功能    ：初始化IO 串口2 
 输入        ：bound:波特率
 输出        ：无
                           
************************************************************************/
void usart2_init(u32 bound)
{	
	//UART 初始化设置
	UART2_Handler.Instance=USART2;					    //USART2
	UART2_Handler.Init.BaudRate=bound;				    //波特率
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART2_Handler);					    //HAL_UART_Init()会使能UART1
}

/***********************************************************************
 函数名      ：usart3_init()  
 函数功能    ：初始化IO 串口3 
 输入        ：bound:波特率
 输出        ：无
                           
************************************************************************/
void usart3_init(u32 bound)
{	
	//UART 初始化设置
	UART3_Handler.Instance=USART3;					    //USART3
	UART3_Handler.Init.BaudRate=bound;				    //波特率
	UART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART3_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART3_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART3_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART3_Handler);					    //HAL_UART_Init()会使能UART1
}

/***********************************************************************
 函数名      ：usart4_init()  
 函数功能    ：初始化IO 串口4
 输入        ：bound:波特率
 输出        ：无
                           
************************************************************************/
void uart4_init(u32 bound)
{	
	//UART 初始化设置
	UART4_Handler.Instance=UART4;					    //USART4
	UART4_Handler.Init.BaudRate=bound;				    //波特率
	UART4_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART4_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART4_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART4_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART4_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART4_Handler);					    //HAL_UART_Init()会使能UART1
}

/***********************************************************************
 函数名      ：usart6_init()  
 函数功能    ：初始化IO 串口6 
 输入        ：bound:波特率
 输出        ：无
                           
************************************************************************/
void usart6_init(u32 bound)
{	
	//UART 初始化设置
	UART6_Handler.Instance=USART6;					    //USART3
	UART6_Handler.Init.BaudRate=bound;				    //波特率
	UART6_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART6_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART6_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART6_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART6_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART6_Handler);					    //HAL_UART_Init()会使能UART1
}

/***********************************************************************
 函数名      ：HAL_UART_MspInit()  
 函数功能    ：UART底层初始化，时钟使能，引脚配置，中断配置 
 输入        ：huart:串口句柄
 输出        ：无
 说明        ：此函数会被HAL_UART_Init()调用 
                           
************************************************************************/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if(huart==(&UART1_Handler))
	{
    //GPIO端口设置
		GPIO_InitTypeDef GPIO_Initure;
	
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9	
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//复用为USART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA10
	
    __HAL_UART_DISABLE_IT(huart,UART_IT_TC);
#if EN_USART1_RX
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//开启接收中断
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//抢占优先级3，子优先级3
#endif	
	}
	
	if(huart==(&UART2_Handler))
	{
    //GPIO端口设置
		GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART2_CLK_ENABLE();			//使能USART2时钟
	
		GPIO_Initure.Pin=GPIO_PIN_2;			//PA2	
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART2;	//复用为USART2
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

		GPIO_Initure.Pin=GPIO_PIN_3;			//PA3
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA3
	
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//开启接收中断
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//使能USART2中断
		HAL_NVIC_SetPriority(USART2_IRQn,1,3);			//抢占优先级1，子优先级3
	}
		
	if(huart==(&UART3_Handler))
	{
		  //GPIO端口设置
		GPIO_InitTypeDef GPIO_Initure;
	
		__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOB时钟
		__HAL_RCC_USART3_CLK_ENABLE();			//使能USART3时钟
	
		GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART3;	//复用为USART3
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB10

		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB11
	
//		__HAL_UART_DISABLE_IT(huart,UART_IT_TC);
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//开启接收中断
		HAL_NVIC_EnableIRQ(USART3_IRQn);				//使能USART3中断
		HAL_NVIC_SetPriority(USART3_IRQn,2,3);			//抢占优先级2，子优先级3	
	}
	
		if(huart==(&UART4_Handler))
		{
		  //GPIO端口设置
		GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_GPIOC_CLK_ENABLE();			//使能GPIOC时钟
		__HAL_RCC_UART4_CLK_ENABLE();			//使能UART4时钟
	
		GPIO_Initure.Pin=GPIO_PIN_10;			//PC10,U4 TX
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF8_UART4;	//复用为UART4
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//初始化PC10

		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11,U4 RX
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//初始化PB11		
		
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//开启接收中断
		HAL_NVIC_EnableIRQ(UART4_IRQn);				    //使能UART4中断
		HAL_NVIC_SetPriority(UART4_IRQn,0,3);			//抢占优先级2，子优先级3	
		TIM7_Init(1000-1,9000-1);		                //100ms中断
		UART4_RX_LEN=0;		                            //清零
		TIM7->CR1&=~(1<<0);                             //关闭定时器7			
		}	
	
	if(huart==(&UART6_Handler))
	{
		  //GPIO端口设置
		GPIO_InitTypeDef GPIO_Initure;
	
		__HAL_RCC_GPIOC_CLK_ENABLE();			//使能GPIOB时钟
		__HAL_RCC_USART6_CLK_ENABLE();			//使能USART6时钟
	
		GPIO_Initure.Pin=GPIO_PIN_6;			//PC6,U6 TX
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF8_USART6;	//复用为USART6
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//初始化PB10

		GPIO_Initure.Pin=GPIO_PIN_7;			//PC7,U6 RX
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//初始化PB11
	
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//开启接收中断
		HAL_NVIC_EnableIRQ(USART6_IRQn);				//使能USART3中断
		HAL_NVIC_SetPriority(USART3_IRQn,4,3);			//抢占优先级4，子优先级3	
	}	
}

/***********************************************************************
 函数名      ：u2_printf()  
 函数功能    ：串口2,printf 函数
 输入        ：无
 输出        ：无
 说明        ：确保一次发送数据不超过USART2_MAX_SEND_LEN字节
                           
************************************************************************/
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

/***********************************************************************
 函数名      ：u3_printf()  
 函数功能    ：串口3,printf 函数
 输入        ：无
 输出        ：无
 说明        ：确保一次发送数据不超过USART3_MAX_SEND_LEN字节
                           
************************************************************************/
void u3_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((USART3->SR&0X40)==0);			//循环发送,直到发送完毕   
		USART3->DR=USART3_TX_BUF[j];  
	} 
}

/***********************************************************************
 函数名      ：u4_printf()  
 函数功能    ：串口4,printf 函数
 输入        ：无
 输出        ：无
 说明        ：确保一次发送数据不超过USART4_MAX_SEND_LEN字节
                           
************************************************************************/
void u4_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)UART4_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)UART4_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((UART4->SR&0X40)==0);			//循环发送,直到发送完毕   
		UART4->DR=UART4_TX_BUF[j];  
	} 
}

/***********************************************************************
 函数名      ：u6_printf()  
 函数功能    ：串口6,printf 函数
 输入        ：无
 输出        ：无
 说明        ：确保一次发送数据不超过USART6_MAX_SEND_LEN字节
                           
************************************************************************/
void u6_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART6_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART6_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((USART6->SR&0X40)==0);			//循环发送,直到发送完毕   
		USART6->DR=USART6_TX_BUF[j];  
	} 
}

/***********************************************************************
 函数名      ：USART1_IRQHandler()  
 函数功能    ：串口1中断服务程序
 输入        ：无
 输出        ：无
                           
************************************************************************/
void USART1_IRQHandler(void)                	
{ 
		u32 timeout=0;
	u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
#endif
	
	HAL_UART_IRQHandler(&UART1_Handler);	//调用HAL库中断处理公用函数
	
	timeout=0;
    while (HAL_UART_GetState(&UART1_Handler) != HAL_UART_STATE_READY)//等待就绪
	{
	 timeout++;////超时处理
   if(timeout>maxDelay) break;			
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
	 timeout++; //超时处理
	 if(timeout>maxDelay) break;	
	}
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
#endif
}
 
/***********************************************************************
 函数名      ：USART2_IRQHandler()  
 函数功能    ：串口2中断服务程序
 输入        ：无
 输出        ：无
                           
************************************************************************/
void USART2_IRQHandler(void)                	
{ 
	u8 Res;
	if((__HAL_UART_GET_FLAG(&UART2_Handler,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
        HAL_UART_Receive(&UART2_Handler,&Res,1,1000); 
		if((USART2_RX_LEN&0x8000)==0)//接收未完成
		{
			if(USART2_RX_LEN&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART2_RX_LEN=0;//接收错误,重新开始
				else USART2_RX_LEN|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART2_RX_LEN|=0x4000;
				else
				{
					USART2_RX_BUF[USART2_RX_LEN&0X3FFF]=Res ;
					USART2_RX_LEN++;
					if(USART2_RX_LEN>(USART2_MAX_RECV_LEN-1))USART2_RX_LEN=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
	}
	HAL_UART_IRQHandler(&UART2_Handler);	
} 

/***********************************************************************
 函数名      ：USART3_IRQHandler()  
 函数功能    ：串口3中断服务程序
 输入        ：无
 输出        ：无
                           
************************************************************************/
void USART3_IRQHandler(void)
{
	u8 Res;
	if((__HAL_UART_GET_FLAG(&UART3_Handler,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
        HAL_UART_Receive(&UART3_Handler,&Res,1,1000); 
		if((USART3_RX_LEN&0x8000)==0)//接收未完成
		{
			if(USART3_RX_LEN&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART3_RX_LEN=0;//接收错误,重新开始
				else USART3_RX_LEN|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART3_RX_LEN|=0x4000;
				else
				{
					USART3_RX_BUF[USART3_RX_LEN&0X3FFF]=Res ;
					USART3_RX_LEN++;
					if(USART3_RX_LEN>(USART3_MAX_RECV_LEN-1))USART3_RX_LEN=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
	}
	HAL_UART_IRQHandler(&UART3_Handler);	
}  

/***********************************************************************
 函数名      ：USART4_IRQHandler()  
 函数功能    ：串口4中断服务程序
 输入        ：无
 输出        ：无
                           
************************************************************************/
void UART4_IRQHandler(void)                	
{ 
	u8 res;	      
	if(__HAL_UART_GET_FLAG(&UART4_Handler,UART_FLAG_RXNE)!=RESET)//接收到数据
	{	 
		res=UART4->DR; 			 
		if((UART4_RX_LEN&(1<<15))==0)               //接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
			if(UART4_RX_LEN<UART4_MAX_RECV_LEN)	    //还可以接收数据
			{
				TIM7->CNT=0;         				//计数器清空	
				if(UART4_RX_LEN==0) 				//使能定时器7的中断 
				{
					TIM7->CR1|=1<<0;     			//使能定时器7
				}
				UART4_RX_BUF[UART4_RX_LEN++]=res;	//记录接收到的值
				
			}
			else 
			{
				UART4_RX_LEN|=1<<15;			    //强制标记接收完成
			} 
		}
	}  				 											 
}  
 
/***********************************************************************
 函数名      ：USART6_IRQHandler()  
 函数功能    ：串口6中断服务程序
 输入        ：无
 输出        ：无
                           
************************************************************************/
void USART6_IRQHandler(void)
{
	u32 timeout=0;
	u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
#endif
	
	HAL_UART_IRQHandler(&UART6_Handler);	//调用HAL库中断处理公用函数
	
	timeout=0;
    while (HAL_UART_GetState(&UART6_Handler) != HAL_UART_STATE_READY)//等待就绪
	{
	 timeout++;	//超时处理
   if(timeout>maxDelay) break;			
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART6_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
	 timeout++; //超时处理
	 if(timeout>maxDelay) break;	
	}
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
#endif
}  

/***********************************************************************
 函数名      ：usmart_pow1()  
 函数功能    ：m^n函数
 输入        ：
 输出        ：m^n次方
                           
************************************************************************/
int long usmart_pow1(u8 m,u8 n)
{
	int long result=1;	 
	while(n--)result*=m;    
	return result;
}

/***********************************************************************
 函数名      ：Fabs()  
 函数功能    ：计算绝对值
 输入        ：
 输出        ：两个数的绝对值
                           
************************************************************************/
u16 abs(u16 data1,u16 data2)
{
  if((data1-data2)>0)
	{
	  return (data1-data2);
	}
	
	else
	{
	  return (data2-data1);
	}
}

/***********************************************************************
 函数名      ：usmart_strnum()  
 函数功能    ：把字符串转为数字
 输入        ：*str:数字字符串指针
 输出        ：0，错误代码.
                           
************************************************************************/
int long   usmart_strnum(u8*str)
{
	u32 t;
	u8  bnum=0;	//数字的位数
	u8 *p;		  
	u8 hexdec=10;//默认为十进制数据
	p=str;
	int long  res=0;//清零.
	while(1)    //计算字符串中数字的个数
	{		
		if(*p<='9'&&*p>='0') 	//字符串中存在数字
		 {
			 bnum++;					//位数增加.
		 }
		else if(*p=='\0')
		{
			break;	//碰到结束符,退出.
		}
		p++; 
	} 
	p=str;			    //重新定位到字符串开始的地址.
	if(bnum==0) return 0;//位数为0，直接退出.	  
	while(1)
	{
		if((*p<='9'&&*p>='0') && (bnum>0))
		{
			bnum--;
			t=*p-'0';	//得到数字的值
			res+=t*usmart_pow1(hexdec,bnum);		 //将得到的数字转化为十进制  
		}   	
		p++;
		if(*p=='\0')
		{
			break;//数据都查完了.	
		}
	}
	return res;//成功转换,并将转换后的十进制数返回
}



/***********************************************************************
 函数名      ：usmart_strnum()  
 函数功能    ：把字符串转为数字
 输入        ：*str:数字字符串指针
 输出        ：0，错误代码.
                           
************************************************************************/
u32 usmart_strnum2(u8*str)
{
	u32 t;
	u8  bnum=0;	//数字的位数
	u8 *p;		  
	u8 hexdec=10;//默认为十进制数据
	p=str;
	u32  res=0;//清零.
	while(1)    //计算字符串中数字的个数
	{		
		if(*p<='9'&&*p>='0') 	//字符串中存在数字
		 {
			 bnum++;					//位数增加.
		 }
		else if(*p=='\0')
		{
			break;	//碰到结束符,退出.
		}
		p++; 
	} 
	p=str;			    //重新定位到字符串开始的地址.
	if(bnum==0) return 0;//位数为0，直接退出.	  
	while(1)
	{
		if((*p<='9'&&*p>='0') && (bnum>0))
		{
			bnum--;
			t=*p-'0';	//得到数字的值
			res+=t*usmart_pow1(hexdec,bnum);		 //将得到的数字转化为十进制  
		}   	
		p++;
		if(*p=='\0')
		{
			break;//数据都查完了.	
		}
	}
	return res;//成功转换,并将转换后的十进制数返回
}
