#include "pump.h"
#include "delay.h"
#include "sys.h"
#include "timer.h"
#include "usart.h"
#include "function.h"
#include "led.h"
#include "motor.h"

/*********************************************************************
*函数名       ：Pump_Init()
*函数功能     ：水箱IO口初始化
*输入         ：无
*输出         ：无

***********************************************************************/
void Pump_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();                         //开启GPIOB时钟
	__HAL_RCC_GPIOH_CLK_ENABLE();                         //开启GPIOH时钟
	
	
    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3;               //PH2/3/7
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;                //推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;                      //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                   //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	  
	GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11;             //PB10/11
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;                //推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;                      //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                   //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);		
	
	GPIO_Initure.Pin=GPIO_PIN_3;                          //PE3
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;                //推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;                      //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                   //高速
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);	
	
	GPIO_Initure.Pin=GPIO_PIN_1;             
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;                //推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;                      //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                   //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);		
}

/*********************************************************************
*函数名       ：Sensor_Init()
*函数功能     ：传感器IO口初始化
*输入         ：无
*输出         ：无

***********************************************************************/
void Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();                       //开启GPIOB时钟
	__HAL_RCC_GPIOC_CLK_ENABLE();                       //开启GPIOB时钟
	__HAL_RCC_GPIOE_CLK_ENABLE();                       //开启GPIOE时钟 
	__HAL_RCC_GPIOH_CLK_ENABLE();                       //开启GPIOH时钟 

    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15;           //PB14/15
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //输入浮空
    GPIO_Initure.Pull=GPIO_PULLUP;                      //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);

	GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_13;  //PC6/7/8/9/13
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //输入浮空
    GPIO_Initure.Pull=GPIO_PULLUP;                      //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	
	  
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;  //PE2/4/5/6
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //输入浮空
    GPIO_Initure.Pull=GPIO_PULLUP;                      //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //高速
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
 	
	GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_9;             //PH679
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //输入浮空
    GPIO_Initure.Pull=GPIO_PULLUP;                      //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_7;  						//PH7
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //输入浮空
    GPIO_Initure.Pull=GPIO_PULLUP;                      //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);	
}

/*********************************************************************
*函数名       ：Push_Rod_Swash_Dry_Init()
*函数功能     ：冲洗烘干推杆方向口和脉冲口初始化
*输入         ：无
*输出         ：无

***********************************************************************/
void Push_Rod_Swash_Dry_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           //开启GPIOB时钟
	__HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟

	GPIO_Initure.Pin=GPIO_PIN_13;           //PB13
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_1;            //PC1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	
	
}

/*********************************************************************
*函数名       ：Push_Rod_Swash_Dry()
*函数功能     ：冲洗烘干推杆驱动，
*输入         ：dir(方向信号：1：伸出；0：缩回)
*输出         ：无

***********************************************************************/
void Push_Rod_Swash_Dry(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //推杆初始化	
	u8 len;
	int j,flag=1;                   //flag为脉冲高低标志位	
	DIR_CXHG=dir;                   //判断冲洗烘干推杆伸出或缩回
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
	delay_ms(100);
	TIM2_Init(1800-1,motor_timer_freq); //3600                                    //打开定时器
	if(((1==dir)&&(push_rod_runed_pulse<push_rod_pulse_lim))||((0==dir)&&(push_rod_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //等待定时时间到，时间到跳出循环
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //清除定时器中断标志位					
			PWM_CXHG=flag;                                              //冲洗烘干电机脉冲输出口高/低电平
			if(UART4_RX_LEN&0x8000)
			{			
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;
				if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				push_rod_runed_pulse++;
			}
			else
			{
				push_rod_runed_pulse--;					
			}
			//发送冲洗烘干推杆动画指令
			if(push_rod_runed_pulse==1)	
			{
				u2_printf("Cartoon_Push_Rod_1");
			}
			else if(push_rod_runed_pulse==(u16)(push_rod_pulse_lim/3))	
			{
				u2_printf("Cartoon_Push_Rod_2");				
			}
			else if(push_rod_runed_pulse==(u16)(2*(push_rod_pulse_lim/3)))
			{
				u2_printf("Cartoon_Push_Rod_3");
			}
			else if(((push_rod_runed_pulse==push_rod_pulse_lim)&&(1==dir))||((push_rod_runed_pulse==push_rod_pulse_lim-1)&&(0==dir)))	
			{
				u2_printf("Cartoon_Push_Rod_4");
			}
		}
	}	
	PWM_CXHG=0;                                        //冲洗烘干电机对应脉冲输出口拉低  
	TIM2_Stop();                                       //关闭定时器
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);         //清除串口4缓冲区
	UART4_RX_LEN=0;
}	
//调试函数
void Push_Rod_Swash_Dry1(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //推杆初始化	
	u8 len;
	int j,flag=1;                   //flag为脉冲高低标志位	
	DIR_CXHG=dir;                   //判断冲洗烘干推杆伸出或缩回
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;		
	for(j=0;j<n;j++)
	{ 	
		flag = !flag;
		PWM_CXHG=flag;                                 //冲洗烘干电机脉冲输出口高/低电平
		delay_ms(1);      //收线两个电机
	}	
	PWM_CXHG=0;                                        //冲洗烘干电机对应脉冲输出口拉低  
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);         //清除串口4缓冲区
	UART4_RX_LEN=0;
}	


/*********************************************************************
*函数名       ：Push_Rod_Swash()
*函数功能     ：冲洗推杆驱动，
*输入         ：dir(方向信号：1：伸出；0：缩回)
*输出         ：无

***********************************************************************/
void Push_Rod_Swash(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //推杆初始化
	static u16 k;                   //传第k张动画
	static u8 kj;
	u8 break_flag;
	u8 len;                         //接收的字符串长度	
	int j,flag=1;                   //flag为脉冲高低标志位	
	DIR_CXHG=dir;                   //判断冲洗烘干推杆伸出或缩回
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //打开定时器
	if(((1==dir)&&(swash_dry_runed_pulse<swash_dry_pulse_lim))||((0==dir)&&(swash_dry_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //等待定时时间到，时间到跳出循环
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //清除定时器中断标志位					
			PWM_CXHG=flag;                                              //冲洗烘干电机脉冲输出口高/低电平
			if(UART4_RX_LEN&0x8000)
			{			
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;
				//若接收到Stop,则跳出循环	
				if(((strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))|| 
					(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))||
							(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone")))&&(1==swash_hand_flag))
				{
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
					break;
				}
				if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				swash_dry_runed_pulse++;
			}
			else
			{
				swash_dry_runed_pulse--;					
			}
			//发送冲洗推杆动画指令						
			if(swash_dry_runed_pulse==(u16)(swash_dry_pulse_lim/4))	
			{
				u2_printf("Cartoon_Push_Rod_Swash_2");
			}
			else if(swash_dry_runed_pulse==(u16)(2*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_3");
			}
			else if(swash_dry_runed_pulse==(u16)(3*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_4");
			}			
			else if(((swash_dry_runed_pulse==swash_dry_pulse_lim)&&(1==dir))||((swash_dry_runed_pulse==swash_dry_pulse_lim-1)&&(0==dir)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_5");
			}
		}
	}	
	PWM_CXHG=0;                                        //冲洗烘干电机对应脉冲输出口拉低  
	TIM2_Stop();                                       //关闭定时器
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);         //清除串口4缓冲区
	UART4_RX_LEN=0;
}	


/*********************************************************************
*函数名       ：Push_Rod_Dry()
*函数功能     ：烘干推杆驱动，
*输入         ：dir(方向信号：1：伸出；0：缩回)
*输出         ：无

***********************************************************************/
void Push_Rod_Dry(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //推杆初始化	
	u8 len;
	u8 break_flag;
	int j,flag=1;                   //flag为脉冲高低标志位	
	DIR_CXHG=dir;                   //判断冲洗烘干推杆伸出或缩回
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //打开定时器
	if(((1==dir)&&(swash_dry_runed_pulse<swash_dry_pulse_lim))||((0==dir)&&(swash_dry_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //等待定时时间到，时间到跳出循环
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //清除定时器中断标志位					
			PWM_CXHG=flag;                                              //冲洗烘干电机脉冲输出口高/低电平
			if(UART4_RX_LEN&0x8000)
			{			
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;				
				//若接收到Stop,则跳出循环	
				if(((strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))|| 
					(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))||
							(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone")))&&(1==dry_hand_flag))
				{
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
					break;
				}
				if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				swash_dry_runed_pulse++;
			}
			else
			{
				swash_dry_runed_pulse--;					
			}
			//发送烘干推杆动画指令
			if(swash_dry_runed_pulse==(u16)(swash_dry_pulse_lim/4))	
			{
				u2_printf("Cartoon_Push_Rod_Dry_2");				
			}
			else if(swash_dry_runed_pulse==(u16)(2*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Dry_3");
			}
			else if(swash_dry_runed_pulse==(u16)(3*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Dry_4");
			}			
			else if(((swash_dry_runed_pulse==swash_dry_pulse_lim)&&(1==dir))||((swash_dry_runed_pulse==swash_dry_pulse_lim-1)&&(0==dir)))				
			{
				u2_printf("Cartoon_Push_Rod_Dry_5");
			}
		}
	}	
	PWM_CXHG=0;                                        //冲洗烘干电机对应脉冲输出口拉低  
	TIM2_Stop();                                       //关闭定时器
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);         //清除串口4缓冲区
	UART4_RX_LEN=0;
}	

/*********************************************************************
*函数名       ：Uart_Push_Rod_Swash_Dry()
*函数功能     ：冲洗烘干推杆驱动，
*输入         ：dir(方向信号：1：伸出；0：缩回)
*输出         ：无

***********************************************************************/
void Uart_Push_Rod_Swash_Dry(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //推杆初始化	
	u8 len;
	int j,flag=1;                   //flag为脉冲高低标志位	
	RELAY6=1;                       //继电器得电，常开触点闭合，坐便袋扎紧电机得电

	DIR_CXHG=dir;                   //判断冲洗烘干推杆伸出或缩回
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //打开定时器
	if(((1==dir)&&(push_rod_runed_pulse<push_rod_pulse_lim))||((0==dir)&&(push_rod_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //等待定时时间到，时间到跳出循环
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //清除定时器中断标志位					
			PWM_CXHG=flag;                                              //冲洗烘干电机脉冲输出口高/低电平
			if(USART2_RX_LEN&0x8000)
			{			
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;
				if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				push_rod_runed_pulse++;
			}
			else
			{
				push_rod_runed_pulse--;					
			}
			//发送冲洗烘干推杆动画指令
//			if(push_rod_runed_pulse==1)	
//			{
//				u2_printf("Cartoon_Push_Rod_1");				
//			}
//			else if(push_rod_runed_pulse==(u16)(push_rod_pulse_lim/3))	
//			{
//				u2_printf("Cartoon_Push_Rod_2");				
//			}
//			else if(push_rod_runed_pulse==(u16)(2*(push_rod_pulse_lim/3)))
//			{
//				u2_printf("Cartoon_Push_Rod_3");
//			}			
//			else if(((push_rod_runed_pulse==push_rod_pulse_lim)&&(1==dir))||((push_rod_runed_pulse==push_rod_pulse_lim-1)&&(0==dir)))	
//			{
//				u2_printf("Cartoon_Push_Rod_4");
//			}
		}
	}	
	PWM_CXHG=0;                                        //冲洗烘干电机对应脉冲输出口拉低  
	TIM2_Stop();                                       //关闭定时器
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //清除串口4缓冲区
	USART2_RX_LEN=0;
}	

/*********************************************************************
*函数名       ：Uart_Push_Rod_Swash()
*函数功能     ：冲洗推杆驱动，
*输入         ：dir(方向信号：1：伸出；0：缩回)
*输出         ：无

***********************************************************************/
void Uart_Push_Rod_Swash(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //推杆初始化
	static u16 k;                   //传第k张动画
	static u8 kj;
	u8 break_flag;
	u8 len;                         //接收的字符串长度	
	int j,flag=1;                   //flag为脉冲高低标志位	
	DIR_CXHG=dir;                   //判断冲洗烘干推杆伸出或缩回
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //打开定时器
//	if(((1==dir)&&(swash_dry_runed_pulse<swash_dry_pulse_lim))||((0==dir)&&(swash_dry_runed_pulse>0)))	
//	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //等待定时时间到，时间到跳出循环
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //清除定时器中断标志位					
			PWM_CXHG=flag;                                              //冲洗烘干电机脉冲输出口高/低电平
			if(USART2_RX_LEN&0x8000)
			{			
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;
				 //若接收到Stop,则跳出循环	
				if(((strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))|| 
					(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))||
							(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone")))&&(1==swash_hand_flag))
				{
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
					break;
				}
				if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //清除串口4缓冲区
					USART2_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				swash_dry_runed_pulse++;
			}
			else
			{
				swash_dry_runed_pulse--;					
			}
			//发送冲洗推杆动画指令						
			if(swash_dry_runed_pulse==(u16)(swash_dry_pulse_lim/4))	
			{
				u2_printf("Cartoon_Push_Rod_Swash_2");
			}
			else if(swash_dry_runed_pulse==(u16)(2*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_3");
			}
			else if(swash_dry_runed_pulse==(u16)(3*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_4");
			}			
			else if(((swash_dry_runed_pulse==swash_dry_pulse_lim)&&(1==dir))||((swash_dry_runed_pulse==swash_dry_pulse_lim-1)&&(0==dir)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_5");
			}
		}
//	}	
	PWM_CXHG=0;                                        //冲洗烘干电机对应脉冲输出口拉低  
	TIM2_Stop();                                       //关闭定时器
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //清除串口2缓冲区
	USART2_RX_LEN=0;
}	


/*********************************************************************
*函数名       ：Uart_Push_Rod_Dry()
*函数功能     ：烘干推杆驱动，
*输入         ：dir(方向信号：1：伸出；0：缩回)
*输出         ：无

***********************************************************************/
void Uart_Push_Rod_Dry(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //推杆初始化	
	u8 len;
	u8 break_flag;
	int j,flag=1;                   //flag为脉冲高低标志位	
	DIR_CXHG=dir;                   //判断冲洗烘干推杆伸出或缩回
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //打开定时器
	if(((1==dir)&&(swash_dry_runed_pulse<swash_dry_pulse_lim))||((0==dir)&&(swash_dry_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //等待定时时间到，时间到跳出循环
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //清除定时器中断标志位					
			PWM_CXHG=flag;                                              //冲洗烘干电机脉冲输出口高/低电平
			if(USART2_RX_LEN&0x8000)
			{			
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;			
				//若接收到Stop,则跳出循环	
				if(((strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))|| 
					(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))||
							(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone")))&&(1==dry_hand_flag))
				{
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
					break;
				}
				if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //清除串口4缓冲区
					USART2_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				swash_dry_runed_pulse++;
			}
			else
			{
				swash_dry_runed_pulse--;					
			}
			//发送烘干推杆动画指令
			if(swash_dry_runed_pulse==(u16)(swash_dry_pulse_lim/4))	
			{
				u2_printf("Cartoon_Push_Rod_Dry_2");				
			}
			else if(swash_dry_runed_pulse==(u16)(2*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Dry_3");
			}
			else if(swash_dry_runed_pulse==(u16)(3*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Dry_4");
			}			
			else if(((swash_dry_runed_pulse==swash_dry_pulse_lim)&&(1==dir))||((swash_dry_runed_pulse==swash_dry_pulse_lim-1)&&(0==dir)))				
			{
				u2_printf("Cartoon_Push_Rod_Dry_5");
			}
		}
	}	
	PWM_CXHG=0;                                        //冲洗烘干电机对应脉冲输出口拉低  
	TIM2_Stop();                                       //关闭定时器
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //清除串口4缓冲区
	USART2_RX_LEN=0;
}	




