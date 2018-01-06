#include "motor.h"
#include "sys.h"
#include "pwm.h"
#include "delay.h"
#include "usart.h"
#include "function.h"
#include "pump.h"
#include "usart.h"
#include "LED.h"
#include "pcf8574.h"
#include "key.h"
#include "check.h"


u8 Desk1st2st=1;
u8 DeskSwitch=0;
u8 DeskLastJudgeFlg=0;
u8 DeskFrontFlg=0;


extern unsigned int motor_hang_freq;    //吊挂电机自动重装载值，控制电机运行速度 
extern unsigned int motor_timer_freq;   //定时器分频值psc   定时器频率=90M/50

/*********************************************************************
*函数名       ：Motor_Dir_Init()
*函数功能     ：6台电机方向口初始化
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_Dir_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOA时钟
	__HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟
	__HAL_RCC_GPIOD_CLK_ENABLE();           //开启GPIOD时钟
	__HAL_RCC_GPIOG_CLK_ENABLE();           //开启GPIOG时钟 
	__HAL_RCC_GPIOH_CLK_ENABLE();           //开启GPIOH时钟	
	
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6;            //PA5/6  
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;             //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;                     //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	GPIO_Initure.Pin= GPIO_PIN_4|GPIO_PIN_12;           //PC4/12
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;              //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;                      //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	
	
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7;  //PD2/3/7
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;              //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;                      //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //高速
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);		
			
	GPIO_Initure.Pin=GPIO_PIN_13;                       //PH13
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;              //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;                      //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);	
	
//坐便袋扎紧电机继电器，控制电机的通断：常开触点，高电平有效
	
	GPIO_Initure.Pin=GPIO_PIN_12;                       //PH12
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;              //推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;                    //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
}

/*********************************************************************
*函数名       ：Push_Rod_Init()
*函数功能     ：电动推杆方向口和脉冲口初始化-支背、曲腿
*输入         ：无
*输出         ：无
***********************************************************************/
void Push_Rod_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOI_CLK_ENABLE();           //开启GPIOI时钟
	__HAL_RCC_GPIOG_CLK_ENABLE(); 	        //开启GPIOG时钟
	
	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_3|GPIO_PIN_7;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);

	GPIO_Initure.Pin=GPIO_PIN_10;           //PG10
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);		
}

/*********************************************************************
*函数名       ：Hang_Init()
*函数功能     ：吊挂电机方向口、脉冲口初始化
*输入         ：无
*输出         ：无
***********************************************************************/
void Hang_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           //开启GPIOB时钟
	__HAL_RCC_GPIOD_CLK_ENABLE();           //开启GPIOD时钟  
    __HAL_RCC_GPIOG_CLK_ENABLE();           //开启GPIOG时钟
	__HAL_RCC_GPIOH_CLK_ENABLE();           //开启GPIOH时钟 

	  GPIO_Initure.Pin=GPIO_PIN_12;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速	
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	
    GPIO_Initure.Pin=GPIO_PIN_6 | GPIO_PIN_7|  GPIO_PIN_8|GPIO_PIN_9;  //PB6/7/8//9
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速	
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
  	GPIO_Initure.Pin=GPIO_PIN_13;               //PD13
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速	
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);

	GPIO_Initure.Pin=GPIO_PIN_11;              //PH11
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;     //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;             //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;        //高速	
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_3|GPIO_PIN_6;    //PG3/6
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;     //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;             //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;        //高速
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);
}

/*********************************************************************
*函数名       ：Push_Rod_Start()
*函数功能     ：电动推杆驱动函数
*输入         ：dir：1：下曲腿；0：上曲腿
*输出         ：无
***********************************************************************/
void Push_Rod_Start(u8 dir)
{
	Push_Rod_Init();
	if(dir==1)
	{
		DIR2_DOWN=0 ;   //下曲腿
		DIR2_UP=1 ; 		
	}
	else
	{
		DIR2_DOWN=1 ;   //上曲腿
		DIR2_UP=0 ; 	
	}
}
/*********************************************************************
*函数名       ：Push_Rod_Stop()
*函数功能     ：电动推杆停止函数
*输入         ：无
*输出         ：无
***********************************************************************/
void Push_Rod_Stop(void)
{
	DIR2_DOWN=0 ;     //对应脉冲口拉低
	DIR2_UP=0 ; 	
}

/*********************************************************************
*函数名       ：Motor_1_START()
*函数功能     ：支背电动推杆驱动函数
*输入         ：dir：1：下曲腿；0：上曲腿
*输出         ：无
***********************************************************************/
void Motor_1_START(u8 dir)
{
	Push_Rod_Init();
	if(dir==1)
	{
		DIR1_DOWN=0 ;   //支背上行
		DIR1_UP=1 ;  		
	}
	else
	{
		DIR1_DOWN=1 ;   //支背下行
		DIR1_UP=0 ; 	
	}
}
/*********************************************************************
*函数名       ：Motor_1_STOP()
*函数功能     ：支背电动推杆停止函数
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_1_STOP(void)
{
	DIR1_DOWN=0 ;     //对应脉冲口拉低
	DIR1_UP=0 ; 	
}

/*********************************************************************
*函数名       ：Motor_3_START()
*函数功能     ：3号电机驱动
*输入         ：arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor_3_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM8_PWM_CHANNEL_2_START(arr,psc);     //TIM8通道 2 PWM输出函数 	
}
/*********************************************************************
*函数名       ：Motor_5_START()
*函数功能     ：5号电机驱动
*输入         ：arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor_5_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM8_PWM_CHANNEL_1_START(arr,psc);     //TIM8通道 1 PWM输出函数 
	
}
/*********************************************************************
*函数名       ：Motor_4_START()
*函数功能     ：4号电机驱动
*输入         ：arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor_4_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM5_PWM_CHANNEL_4_START(arr,psc);     //TIM5通道4 PWM输出函数 
}

/*********************************************************************
*函数名       ：Motor_4_Compensate()
*函数功能     ：4号电机驱动
*输入         ：arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor_4_Compensate(u8 dir,u16 time_arr,u16 arr,u16 psc)
{
	DIR4=dir;
	Motor_4_START(arr,psc);
	TIM10_Init(time_arr,timer10_freq);
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); 
	while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)) )
	{

	}
	Motor_4_STOP();
}


/*********************************************************************
*函数名       ：Motor_6_START()
*函数功能     ：6号电机驱动
*输入         ：arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor_6_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM3_PWM_CHANNEL_2_START(arr,psc);     //TIM8通道 2 PWM输出函数 
}

/*********************************************************************
*函数名       ：Motor_6_1_START()
*函数功能     ：坐便器袋子收紧电机驱动
*输入         ：arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor_6_1_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM8_PWM_CHANNEL_4_START(arr,psc);     //TIM8通道 4 PWM输出函数 
}

/*********************************************************************
*函数名       ：Motor_6_2_START()
*函数功能     ：坐便器袋子收紧前推杆驱动，
*输入         ：dir(运行方向)，pulse(运行脉冲数)
*输出         ：无
***********************************************************************/
void Motor_6_2_START(u8 dir,u32 pulse)
{	
	Motor_Dir_Init();
	int i,flag=1;          //flag为脉冲高低标志位
    DIR6_2=dir;            //坐便袋收紧驱动电机转动方向控制，1高电平正转，0低电平反转
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
    TIM10_Init(3600-1,motor_timer_freq);         //打开定时器
	if(((1==dir)&&(push_rod_tig_runed_pulse<push_rod_tig_pulse_lim))||((0==dir)&&(push_rod_tig_runed_pulse>0)))
    {
		for(i=0;i<pulse;i++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
			flag = !flag ;
			PWM6_2=flag;          //坐便袋收紧驱动电机脉冲输出口高/低电平
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
			//根据上下行判断脉冲累计
			if(1==dir)           //上行，脉冲+
			{
				push_rod_tig_runed_pulse++;
			}
			else                 //下行，脉冲-
			{
				push_rod_tig_runed_pulse--;
			}
			//发送动画指令
			if(push_rod_tig_runed_pulse==1)
			{
				u2_printf("Cartoon_Push_Rod_Tig_1");
			}
			if(push_rod_tig_runed_pulse==(u16)(pulse/2))
			{
				u2_printf("Cartoon_Push_Rod_Tig_2");
			}
			if(((push_rod_tig_runed_pulse==pulse)&&(1==dir))||((push_rod_tig_runed_pulse==pulse-1)&&(0==dir)))
			{
				u2_printf("Cartoon_Push_Rod_Tig_3");
			} 
		}
	}
	PWM6_2=0;                      //坐便袋收紧驱动电机对应脉冲输出口拉低  
	TIM10_Stop();                  //关闭定时器
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);   //清除中断标志位
}

/*********************************************************************
*函数名       ：Motor_7_START()
*函数功能     ：7号电机驱动
*输入         ：arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor_7_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM1_PWM_CHANNEL_4_START(arr,psc);     //TIM8通道 2 PWM输出函数 
}

/*********************************************************************
*函数名       ：Motor_3_4_5_START()
*函数功能     ：345号电机同时驱动，完成翻身
*输入         ：arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor_3_4_5_START_left(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM8_PWM_CHANNEL_2_START(arr,psc);     //TIM8通道 2 PWM输出函数 ，3号电机
	TIM8_PWM_CHANNEL_1_START(arr,psc);     //TIM8通道 1 PWM输出函数 ,5号电机
	TIM5_PWM_CHANNEL_4_START((u16)(arr*1.2),psc);     //TIM5通道 4 PWM输出函数 	，4号电机
}

/*********************************************************************
*函数名       ：Motor_3_4_5_START()
*函数功能     ：345号电机同时驱动，完成翻身
*输入         ：arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor_3_4_5_START_right(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM8_PWM_CHANNEL_1_START(arr,psc);     //TIM8通道 2 PWM输出函数,5号电机 
	TIM8_PWM_CHANNEL_2_START((u16)(arr*1.4),psc);     //TIM8通道 2 PWM输出函数 ，3号电机
	
	TIM5_PWM_CHANNEL_4_START(arr,psc);     //TIM8通道 2 PWM输出函数 	，4号电机
}

/*********************************************************************
*函数名       ：Motor_3_STOP()
*函数功能     ：3号电机停止
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_3_STOP(void)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM8->CCER&=~(1<<4);                   //定时器8通道2
}
/*********************************************************************
*函数名       ：Motor_5_STOP()
*函数功能     ：5号电机停止
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_5_STOP(void)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM8->CCER&=~(1<<0);                   //定时器8通道1
}
/*********************************************************************
*函数名       ：Motor_5_STOP()
*函数功能     ：5号电机停止
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_4_STOP(void)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM5->CCER&=~(1<<12);                  //定时器5通道4
}
/*********************************************************************
*函数名       ：Motor_6_STOP()
*函数功能     ：6号电机停止
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_6_STOP(void)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM3->CCER&=~(1<<4);                   //定时器3通道2
}

/*********************************************************************
*函数名       ：Motor_6_1_STOP()
*函数功能     ：坐便器袋子收紧电机停止
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_6_1_STOP(void)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM8->CCER&=~(1<<12);                  //定时器8通道4
}
/*********************************************************************
*函数名       ：Motor_7_STOP()
*函数功能     ：7号电机停止
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_7_STOP(void)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM1->CCER&=~(1<<12);                  //定时器1通道4
}
/*********************************************************************
*函数名       ：Motor_3_4_5_STOP()
*函数功能     ：345号电机同时停止
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_3_4_5_STOP(void)
{
	Motor_Dir_Init();                      //电机方向口初始化函数
	TIM8->CCER&=~(1<<4);                   //定时器8通道2
	TIM8->CCER&=~(1<<0);                   //定时器8通道1
	TIM5->CCER&=~(1<<12);	               //定时器5通道4
}

/*********************************************************************
*函数名       ：Motor_All_Stop()
*函数功能     ：所有电机停止
*输入         ：无
*输出         ：无
***********************************************************************/
void Motor_All_Stop(void)
{      
	Motor_1_STOP();                //支背电机停止函数
	Push_Rod_Stop();               //曲腿电机停止函数
	Motor_3_STOP();                //背部电机停止函数
	Motor_4_STOP();                //腰部支背电机停止函数
	Motor_5_STOP();                //侧翻电机停止函数
	Motor_3_4_5_STOP();            //左/右翻身电机停止函数	
	Motor_6_STOP();                //坐便器电机停止函数
	Motor_6_1_STOP();              //坐便袋扎紧电机停止函数
	Motor_7_STOP();                //小桌子电机停止函数	
}

/*********************************************************************
*函数名       ：Auto_Hang_1()
*函数功能     ：1号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Auto_Hang_1(u16 dir,u32 pulse)
{
    Hang_Init();                      //电机方向口初始化函数
	int i,flag=1;                     //flag为脉冲高低标志位
	int j=0,k=0;
    HANG_DIR1=dir;                    //1号吊挂电机转动方向控制，高电平正转，低电平反转
    memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位			
		HANG_PWM1=flag;              //1号吊挂电机脉冲输出口高电平 
		if(arm_fore_left_flag==1)    //左胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_fore_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_left_runed--;
			}
			//发送左胳膊吊挂动画指令
			j=arm_fore_left_runed/(arm_fore_left_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_15");
				}				
			}									
		}
		else if( leg_fore_left_flag==1)    //左腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_fore_left_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_fore_left_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_fore_left_runed/(leg_fore_left_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_15");
				}		
			}
		}
	}
	HANG_PWM1=0;                      //1号吊挂电机对应脉冲输出口拉低  
	TIM10_Stop();                     //关闭定时器
}
/*********************************************************************
*函数名       ：Auto_Hang_3()
*函数功能     ：3号吊挂电机驱动
*输入         ：dir(方向:1正0反)，n(脉冲数)
*输出         ：无
***********************************************************************/
void Auto_Hang_3(u16 dir,u32 pulse)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;
	if(1==dir)
	{
		HANG_DIR3=0;       //3号吊挂电机转动方向控制，高电平正转，低电平反转   
	}
	else
	{
		HANG_DIR3=1;       //3号吊挂电机转动方向控制，高电平正转，低电平反转   
	}
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;   
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM3=flag;               //3号吊挂电机脉冲输出口高电平
		if(arm_fore_right_flag==1)    //右胳膊
		{
			//通过上下行判断脉冲累加
			if(0==dir)     //上行，脉冲+
			{
				arm_fore_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_right_runed--;
			}			
			//发送左胳膊吊挂动画指令
			j=arm_fore_right_runed/(arm_fore_right_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Right_15");
				}				
			}									
		}
		if( leg_fore_right_flag==1)    //右腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_fore_right_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_fore_right_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_fore_right_runed/(leg_fore_right_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Right_15");
				}		
			}
		}		 
	}
	HANG_PWM3=0;              //3号吊挂电机对应脉冲输出口拉低  
	TIM10_Stop();             //关闭定时器
}
/*********************************************************************
*函数名       ：Auto_Hang_1_2()
*函数功能     ：1/2号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Auto_Hang_1_2(u16 dir,u32 pulse)
{
	Hang_Init();        //电机方向口初始化函数
	int i,flag=1;       //flag为脉冲高低标志位
	u8 j=0,k=0;
    if(dir==1)
     {
		HANG_DIR1=1;    //1号吊挂电机转动方向控制，高电平正转
		HANG_DIR2=0;    //2号吊挂电机转动方向控制，高电平正转
     }
    else
     {
		HANG_DIR1=0;   //1号吊挂电机转动方向控制，低电平反转
		HANG_DIR2=1;   //2号吊挂电机转动方向控制，高电平正转
     }
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;    
    TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)));    //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位
		
		HANG_PWM1=flag; 
		HANG_PWM2=flag;
		if(arm_fore_left_flag==1)    //左胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_left_runed--;
			}
			//发送左胳膊吊挂动画指令	
			j=arm_left_runed/(arm_left_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}			
			}	
		}
		if( leg_fore_left_flag==1)    //左腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_left_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_left_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_left_runed/(leg_left_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}			
			}			
		}		
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	HANG_PWM2=0;           //2号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Auto_Hang_1_3()
*函数功能     ：1/3号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Auto_Hang_1_3(u16 dir,u32 pulse)
{
	Hang_Init();        //电机方向口初始化函数
	int i,flag=1;       //flag为脉冲高低标志位
	int j=0,k=0;
	if(dir==1)
     {
		HANG_DIR1=1;    //1号吊挂电机转动方向控制，高电平正转
		HANG_DIR3=0;    //2号吊挂电机转动方向控制，高电平正转
     }
    else
     {
		HANG_DIR1=0;   //1号吊挂电机转动方向控制，低电平反转
		HANG_DIR3=1;   //2号吊挂电机转动方向控制，高电平正转
     }
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;    
	TIM10_Init(motor_hang_freq,motor_timer_freq);//打开定时器3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM1=flag;   //1号吊挂电机脉冲输出口高/低电平
		HANG_PWM3=flag;   //2号吊挂电机脉冲输出口高/低电平	
 		if(arm_fore_left_right_flag==1)    //左右胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_fore_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_left_right_runed--;
			}
			//发送左胳膊吊挂动画指令
			j=arm_fore_left_right_runed/(arm_fore_left_right_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_Right_15");
				}				
			}									
		}
		if( leg_fore_left_right_flag==1)    //左右腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_fore_left_right_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_fore_left_right_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_fore_left_right_runed/(leg_fore_left_right_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_Right_15");
				}		
			}
		}  
	}
	HANG_PWM1=0;          //1号吊挂电机脉冲输出口低电平
	HANG_PWM3=0;          //3号吊挂电机脉冲输出口高电平 
	TIM10_Stop();         //关闭定时器
}


/*********************************************************************
*函数名       ：Auto_Hang_3_4()
*函数功能     ：3/4号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Auto_Hang_3_4(u16 dir,u32 pulse)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	u8 j=0,k=0;	
	if(dir==1)
     {
		HANG_DIR3=0;    //3号吊挂电机转动方向控制，高电平正转
		HANG_DIR4=1;    //4号吊挂电机转动方向控制，高电平正转
     }
    else
     {
		HANG_DIR3=1;   //3号吊挂电机转动方向控制，低电平反转
		HANG_DIR4=0;   //4号吊挂电机转动方向控制，高电平正转
     }
    memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(motor_hang_freq,motor_timer_freq);//打开定时器3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
			
		HANG_PWM3=flag;          //3号吊挂电机脉冲输出口高电平
		HANG_PWM4=flag;          //4号吊挂电机脉冲输出口高电平
		if(arm_fore_right_flag==1)    //右胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_right_runed--;
			}
			//发送右胳膊吊挂动画指令	
			j=arm_right_runed/(arm_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}			
			}	
		}
		if( leg_fore_right_flag==1)    //右腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_right_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_right_runed--;
			}
			//发送右腿吊挂动画指令						
			j=leg_right_runed/(leg_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}			
			}			
		}		
	}
	HANG_PWM3=0;           //3号吊挂电机脉冲输出口低电平
	HANG_PWM4=0;           //4号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Auto_Hang_1_2_3_4()
*函数功能     ：1/2/3/4号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Auto_Hang_1_2_3_4(u16 dir,u32 pulse)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	u8 j=0,k=0;
	if(dir==1)
    {
		HANG_DIR1=1;    //1号吊挂电机转动方向控制，高电平正转
		HANG_DIR2=0;    //2号吊挂电机转动方向控制，高电平正转
		HANG_DIR3=0;    //1号吊挂电机转动方向控制，高电平正转
		HANG_DIR4=1;    //2号吊挂电机转动方向控制，高电平正转
    }
    else
    {
		HANG_DIR1=0;   //1号吊挂电机转动方向控制，低电平反转
		HANG_DIR2=1;   //2号吊挂电机转动方向控制，高电平正转
		HANG_DIR3=1;    //1号吊挂电机转动方向控制，高电平正转
		HANG_DIR4=0;    //2号吊挂电机转动方向控制，高电平正转
    }
     memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM1=flag;   //1号吊挂电机脉冲输出口高/低电平
		HANG_PWM2=flag;   //2号吊挂电机脉冲输出口高/低电平
		HANG_PWM3=flag;   //3号吊挂电机脉冲输出口高/低电平
		HANG_PWM4=flag;   //4号吊挂电机脉冲输出口高/低电平	
		if(arm_fore_left_right_flag==1)    //左右胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_left_right_runed--;
			}
			//发送左胳膊吊挂动画指令	
			j=arm_left_right_runed/(arm_left_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}			
			}	
		}
		if( leg_fore_left_right_flag==1)    //左右腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_left_right_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_left_right_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_left_right_runed/(leg_left_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}			
			}			
		}	  		 
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	HANG_PWM2=0;           //2号吊挂电机脉冲输出口高电平 
	HANG_PWM3=0;           //3号吊挂电机脉冲输出口低电平
	HANG_PWM4=0;           //4号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Hand_Hang_1()
*函数功能     ：1号吊挂电机：左肢小臂
*输入         ：dir:方向口；n：脉冲口
*输出         ：无
***********************************************************************/
void Hand_Hang_1(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;            //flag为脉冲高低标志位	
	static int k=0;	
    Hang_Init();               //电机方向口初始化函数
	HANG_DIR1=dir;  		   //1正转，0反转  
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器10
	for(i=0;i<pulse;i++)
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
		HANG_PWM1=flag; 	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;		
			 //若接收到停止指令,则跳出循环
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftUpHand"))||    
			   (strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftDownHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftUpHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftDownHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftUpHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftDownHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftUpHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}			
		}
		//发送左小臂动画指令
		if((1==arm_fore_left_flag)||(1==arm_fore_post_left_flag))        //左小臂、左大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_fore_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_left_runed--;
			}
			j=arm_fore_left_runed/(arm_fore_left_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_12");
				}				
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_15");
				}				
			}
		}
		else if((1==leg_fore_left_flag)||(1==leg_fore_post_left_flag))   //左小腿、左大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_fore_left_runed++;
			}
			else           //下行，脉冲-
			{
				leg_fore_left_runed--;
			}
			j=leg_fore_left_runed/(leg_fore_left_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_15");
				}				
			}			
		}
		
		
		if(((arm_fore_left_runed==0)&&((1==arm_fore_left_flag)||(1==arm_fore_post_left_flag)) )||
			((leg_fore_left_runed==0)&&((1==leg_fore_left_flag)||(1==leg_fore_post_left_flag))))
		{
			k=0;     
		}		
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Hand_Hang_1_2()
*函数功能     ：1/2号吊挂电机联动：左肢
*输入         ：m:方向口；pulse：脉冲口
*输出         ：无
***********************************************************************/
void Hand_Hang_1_2(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;     //flag为脉冲高低标志位
	static int k=0;
    Hang_Init();        //电机方向口初始化函数

	if(1==dir)
	{
		HANG_DIR1=1;  	//1正转，2反转
		HANG_DIR2=0;  
	}
	else
	{
		HANG_DIR1=0;  	//1正转，2反转
		HANG_DIR2=1; 
	}	

	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器3
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位
		
		HANG_PWM1=flag; 
		HANG_PWM2=flag; 
	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftDownHand")))    
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}
		}
		//发送左胳膊动画指令
		if(1==arm_fore_left_flag)        //左小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_left_runed--;
			}
			j=arm_left_runed/(arm_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}
			}
		}
		else if((1==arm_post_left_flag)||(1==arm_fore_post_left_flag))   //左大臂、左大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_post_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_post_left_runed--;
			}
			j=arm_post_left_runed/(arm_post_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}
			}
		}		
		//发送左腿动画指令
		else if(1==leg_fore_left_flag)   //左小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_left_runed++;
			}
			else           //下行，脉冲-
			{
				leg_left_runed--;
			}
			j=leg_left_runed/(leg_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}
			}			
		}
		else if((1==leg_post_left_flag)||(1==leg_fore_post_left_flag))   //左大腿、左大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_post_left_runed++;
			}
			else           //下行，脉冲-
			{
				leg_post_left_runed--;
			}
			j=leg_post_left_runed/(leg_post_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}
			}			
		}
		if(((arm_left_runed==0)&&(1==arm_fore_left_flag))||
			((arm_post_left_runed==0)&&((1==arm_post_left_flag)||(1==arm_fore_post_left_flag)))||
			((leg_left_runed==0)&&(1==leg_fore_left_flag))||
			((0==leg_post_left_runed)&&((1==leg_post_left_flag)||(1==leg_fore_post_left_flag))))
		{
			k=0;     
		}
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	HANG_PWM2=0;           //2号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Hand_Hang_3()
*函数功能     ：3号吊挂电机：右肢小臂
*输入         ：m:方向口；n：脉冲口
*输出         ：无
***********************************************************************/

void Hand_Hang_3(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;          //flag为脉冲高低标志位	
	static int k=0;		
    Hang_Init();             //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR3=0;  		 //1正转，2反转  
	}
	else
	{
		HANG_DIR3=1;  		 //1正转，2反转  
	}
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
		HANG_PWM3=flag; 	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}
		}
		//发送右小臂动画指令
		if((1==arm_fore_right_flag)||(1==arm_fore_post_right_flag))         //右小臂、右大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_fore_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_right_runed--;
			}
			j=arm_fore_right_runed/(arm_fore_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Right_12");
				}				
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Right_15");
				}				
			}
		}
		else if((1==leg_fore_right_flag)||(1==leg_fore_post_right_flag))    //右小腿、右大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_fore_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_fore_right_runed--;
			}
			j=leg_fore_right_runed/(leg_fore_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Right_15");
				}				
			}			
		}
		if(((arm_fore_right_runed==0)&&((1==arm_fore_right_flag)||(1==arm_fore_post_right_flag)))||
			((leg_fore_right_runed==0)&&((1==leg_fore_right_flag)||(1==leg_fore_post_right_flag))))
		{
			k=0;     
		}		
		
  }
	HANG_PWM3=0;           //3号吊挂电机脉冲输出口低电平
	TIM10_Stop();          //关闭定时器
}


/*********************************************************************
*函数名       ：Hand_Hang_3_4()
*函数功能     ：3/4号吊挂电机联动：右肢
*输入         ：dir:方向口；pulse：脉冲数
*输出         ：无
***********************************************************************/
void Hand_Hang_3_4(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flag为脉冲高低标志位
	static int k=0;	
    Hang_Init();          //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR3=0;  	  //1正转，2反转
		HANG_DIR4=1;
	}
	else
	{
		HANG_DIR3=1;  	  //1正转，2反转
		HANG_DIR4=0;
	}  
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
		HANG_PWM3=flag; 
		HANG_PWM4=flag; 	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}			
		}
		//发送右胳膊动画指令
		if(1==arm_fore_right_flag)            //右小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_right_runed--;
			}
			j=arm_right_runed/(arm_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}
			}
		}
		else if(1==leg_fore_right_flag)       //右小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_right_runed--;
			}
			j=leg_right_runed/(leg_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}
			}			
		}
		
		else if((1==arm_post_right_flag)||(1==arm_fore_post_right_flag))    //右大臂、右大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_post_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_post_right_runed--;
			}
			j=arm_post_right_runed/(arm_post_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}
			}
		}
		else if((1==leg_post_right_flag)||(1==leg_fore_post_right_flag))    //右大腿、右大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_post_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_post_right_runed--;
			}
			j=leg_post_right_runed/(leg_post_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}
			}			
		}


		if(((arm_right_runed==0)&&(1==arm_fore_right_flag))||
			((0==leg_right_runed)&&(1==leg_fore_right_flag))||
			((arm_post_right_runed==0)&&((1==arm_post_right_flag)||(1==arm_fore_post_right_flag)))||
			((0==leg_post_right_runed)&&((1==leg_post_right_flag)||(1==leg_fore_post_right_flag))))
		{
			k=0;     
		}		
   }
	HANG_PWM3=0;           //3号吊挂电机脉冲输出口低电平
	HANG_PWM4=0;           //4号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Hand_Hang_1_3()
*函数功能     ：1/3号吊挂电机联动：左右肢小臂/小腿
*输入         ：m:方向口；pulse：脉冲口
*输出         ：无
***********************************************************************/
void Hand_Hang_1_3(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flag为脉冲高低标志位
	static int k=0;		
    Hang_Init();          //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR1=1;  	  //1正转，2反转
		HANG_DIR3=0; 
	}
	else
	{
		HANG_DIR1=0;  	  //1正转，2反转
		HANG_DIR3=1; 
	}

	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器3
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位
		
		HANG_PWM1=flag; 
		HANG_PWM3=flag; 
	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			} 
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}
		}
		//发送左右胳膊动画指令
		if((1==arm_fore_left_right_flag)||(1==arm_fore_post_left_right_flag))          //左右小臂、左右大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_fore_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_left_right_runed--;
			}
			j=arm_fore_left_right_runed/(arm_fore_left_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_Right_15");
				}				
			}
		}
		else if((1==leg_fore_left_right_flag)||(1==leg_fore_post_left_right_flag))     //左右小腿、左右大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_fore_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_fore_left_right_runed--;
			}
			j=leg_fore_left_right_runed/(leg_fore_left_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_Right_15");
				}				
			}			
		}
		if(((arm_fore_left_right_runed==0)&&((1==arm_fore_left_right_flag)||(1==arm_fore_post_left_right_flag)))||
			((0==leg_fore_left_right_runed)&&((1==leg_fore_left_right_flag)||(1==leg_fore_post_left_right_flag))))
		{
			k=0;     
		}			
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	HANG_PWM3=0;           //2号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Hand_Hang_1_2_3_4()
*函数功能     ：1/2/3/4号吊挂电机联动：左右肢
*输入         ：dir:方向口；pulse：脉冲数
*输出         ：无
***********************************************************************/
void Hand_Hang_1_2_3_4(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flag为脉冲高低标志位
	static int k=0;	
    Hang_Init();          //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR1=1;
		HANG_DIR2=0;  	  //1正转，2反转
		HANG_DIR3=0;
		HANG_DIR4=1; 
	}
	else
	{
		HANG_DIR1=0;
		HANG_DIR2=1;  	  //1正转，2反转
		HANG_DIR3=1;
		HANG_DIR4=0; 
	}	  
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
		HANG_PWM1=flag;
	    HANG_PWM2=flag;
	    HANG_PWM3=flag;
		HANG_PWM4=flag; 	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			} 
			else if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");       //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}
		}
		//发送左右胳膊动画指令
		if(1==arm_fore_left_right_flag)          //左右小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_left_right_runed--;
			}
			j=arm_left_right_runed/(arm_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}
			}
		}
		else if(1==leg_fore_left_right_flag)     //左右小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_left_right_runed--;
			}
			j=leg_left_right_runed/(leg_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}
			}			
		}
		else if((1==arm_post_left_right_flag)||(1==arm_fore_post_left_right_flag))  //左右大臂、左右大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_post_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_post_left_right_runed--;
			}
			j=arm_post_left_right_runed/(arm_post_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}
			}
		}
		else if((1==leg_post_left_right_flag)||(1==leg_fore_post_left_right_flag))  //左右大腿、左右大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_post_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_post_left_right_runed--;
			}
			j=leg_post_left_right_runed/(leg_post_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}
			}			
		}		
		if(((arm_left_right_runed==0)&&(1==arm_fore_left_right_flag))||
			((0==leg_left_right_runed)&&(1==leg_fore_left_right_flag))||
			((arm_post_left_right_runed==0)&&((1==arm_post_left_right_flag)||(1==arm_fore_post_left_right_flag)))||
			((0==leg_post_left_right_runed)&&((1==leg_post_left_right_flag)||(1==leg_fore_post_left_right_flag))))
		{
			k=0;     
		}		
  }
	HANG_PWM1=0; 
	HANG_PWM2=0;           //3号吊挂电机脉冲输出口低电平
	HANG_PWM3=0; 
	HANG_PWM4=0;           //4号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}


/*********************************************************************

						吊挂串口函数

***********************************************************************/

/*********************************************************************
*函数名       ：Uart_Auto_Hang_1()
*函数功能     ：1号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Uart_Auto_Hang_1(u16 dir,u32 pulse)
{
  Hang_Init();                      //电机方向口初始化函数
	int i,flag=1;                     //flag为脉冲高低标志位
	int j=0,k=0;
	if(1==dir)
	{
		HANG_DIR1=1;                    //1号吊挂电机转动方向控制，高电平正转，低电平反转		
	}
	else
	{
		HANG_DIR1=0;                    //1号吊挂电机转动方向控制，高电平正转，低电平反转		
	}
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
    TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位			
		HANG_PWM1=flag;              //1号吊挂电机脉冲输出口高电平 
		if(arm_fore_left_flag==1)    //左胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_fore_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_left_runed--;
			}
			//发送左胳膊吊挂动画指令
			j=arm_fore_left_runed/(arm_fore_left_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_15");
				}				
			}									
		}
		else if( leg_fore_left_flag==1)    //左腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_fore_left_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_fore_left_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_fore_left_runed/(leg_fore_left_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_15");
				}		
			}
		}
	}
	HANG_PWM1=0;                      //1号吊挂电机对应脉冲输出口拉低  
	TIM10_Stop();                     //关闭定时器
}
/*********************************************************************
*函数名       ：Uart_Auto_Hang_3()
*函数功能     ：3号吊挂电机驱动
*输入         ：dir(方向:1正0反)，n(脉冲数)
*输出         ：无
***********************************************************************/
void Uart_Auto_Hang_3(u16 dir,u32 pulse)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;
	if(1==dir)
	{
		HANG_DIR3=0;       //3号吊挂电机转动方向控制，高电平正转，低电平反转   		
	}
 	else
	{
		HANG_DIR3=1;       //3号吊挂电机转动方向控制，高电平正转，低电平反转   		
	}	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);				
	USART2_RX_LEN=0;  
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM3=flag;          //3号吊挂电机脉冲输出口高电平
		if(arm_fore_right_flag==1)    //右胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_fore_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_right_runed--;
			}			
			//发送左胳膊吊挂动画指令
			j=arm_fore_right_runed/(arm_fore_right_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Right_15");
				}				
			}									
		}
		if( leg_fore_right_flag==1)    //右腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_fore_right_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_fore_right_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_fore_right_runed/(leg_fore_right_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Right_15");
				}		
			}
		}		 
	}
	HANG_PWM3=0;              //3号吊挂电机对应脉冲输出口拉低  
	TIM10_Stop();             //关闭定时器
}
/*********************************************************************
*函数名       ：Uart_Auto_Hang_1_2()
*函数功能     ：1/2号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Uart_Auto_Hang_1_2(u16 dir,u32 pulse)
{
	Hang_Init();        //电机方向口初始化函数
	int i,flag=1;       //flag为脉冲高低标志位
	u8 j=0,k=0;
    if(dir==1)
     {
		HANG_DIR1=1;    //1号吊挂电机转动方向控制，高电平正转
		HANG_DIR2=0;    //2号吊挂电机转动方向控制，高电平正转
     }
    else
     {
		HANG_DIR1=0;   //1号吊挂电机转动方向控制，低电平反转
		HANG_DIR2=1;   //2号吊挂电机转动方向控制，高电平正转
     }
    memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
    TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)));    //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位
		
		HANG_PWM1=flag; 
		HANG_PWM2=flag;
		if(arm_fore_left_flag==1)    //左胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_left_runed--;
			}
			//发送左胳膊吊挂动画指令	
			j=arm_left_runed/(arm_left_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}			
			}	
		}
		if( leg_fore_left_flag==1)    //左腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_left_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_left_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_left_runed/(leg_left_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}			
			}			
		}		
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	HANG_PWM2=0;           //2号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Uart_Auto_Hang_1_3()
*函数功能     ：1/3号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Uart_Auto_Hang_1_3(u16 dir,u32 pulse)
{
	Hang_Init();        //电机方向口初始化函数
	int i,flag=1;       //flag为脉冲高低标志位
	int j=0,k=0;
	if(dir==1)
     {
		HANG_DIR1=1;    //1号吊挂电机转动方向控制，高电平正转
		HANG_DIR3=0;    //2号吊挂电机转动方向控制，高电平正转
     }
    else
     {
		HANG_DIR1=0;   //1号吊挂电机转动方向控制，低电平反转
		HANG_DIR3=1;   //2号吊挂电机转动方向控制，高电平正转
     }
    
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	 
	TIM10_Init(motor_hang_freq,motor_timer_freq);//打开定时器3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM1=flag;   //1号吊挂电机脉冲输出口高/低电平
		HANG_PWM3=flag;   //2号吊挂电机脉冲输出口高/低电平	
 		if(arm_fore_left_right_flag==1)    //左右胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_fore_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_left_right_runed--;
			}
			//发送左胳膊吊挂动画指令
			j=arm_fore_left_right_runed/(arm_fore_left_right_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_Right_15");
				}				
			}									
		}
		if( leg_fore_left_right_flag==1)    //左右腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_fore_left_right_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_fore_left_right_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_fore_left_right_runed/(leg_fore_left_right_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_Right_15");
				}		
			}
		}  
	}
	HANG_PWM1=0;          //1号吊挂电机脉冲输出口低电平
	HANG_PWM3=0;          //3号吊挂电机脉冲输出口高电平 
	TIM10_Stop();         //关闭定时器
}


/*********************************************************************
*函数名       ：Uart_Auto_Hang_3_4()
*函数功能     ：3/4号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Uart_Auto_Hang_3_4(u16 dir,u32 pulse)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	u8 j=0,k=0;	
	if(dir==1)
     {
		HANG_DIR3=0;    //3号吊挂电机转动方向控制，高电平正转
		HANG_DIR4=1;    //4号吊挂电机转动方向控制，高电平正转
     }
    else
     {
		HANG_DIR3=1;   //3号吊挂电机转动方向控制，低电平反转
		HANG_DIR4=0;   //4号吊挂电机转动方向控制，高电平正转
     }
   
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	 
	TIM10_Init(motor_hang_freq,motor_timer_freq);//打开定时器3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
			
		HANG_PWM3=flag;          //3号吊挂电机脉冲输出口高电平
		HANG_PWM4=flag;          //4号吊挂电机脉冲输出口高电平
		if(arm_fore_right_flag==1)    //右胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_right_runed--;
			}
			//发送右胳膊吊挂动画指令	
			j=arm_right_runed/(arm_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}			
			}	
		}
		if( leg_fore_right_flag==1)    //右腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_right_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_right_runed--;
			}
			//发送右腿吊挂动画指令						
			j=leg_right_runed/(leg_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}			
			}			
		}		
	}
	HANG_PWM3=0;           //3号吊挂电机脉冲输出口低电平
	HANG_PWM4=0;           //4号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Uart_Auto_Hang_1_2_3_4()
*函数功能     ：1/2/3/4号吊挂电机驱动
*输入         ：dir(方向:1正0反)，pulse(脉冲数)
*输出         ：无
***********************************************************************/
void Uart_Auto_Hang_1_2_3_4(u16 dir,u32 pulse)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	u8 j=0,k=0;
	if(dir==1)
    {
		HANG_DIR1=1;    //1号吊挂电机转动方向控制，高电平正转
		HANG_DIR2=0;    //2号吊挂电机转动方向控制，高电平正转
		HANG_DIR3=0;    //3号吊挂电机转动方向控制，高电平正转
		HANG_DIR4=1;    //4号吊挂电机转动方向控制，高电平正转
    }
    else
    {
		HANG_DIR1=0;   //1号吊挂电机转动方向控制，低电平反转
		HANG_DIR2=1;   //2号吊挂电机转动方向控制，高电平正转
		HANG_DIR3=1;   //3号吊挂电机转动方向控制，高电平正转
		HANG_DIR4=0;   //4号吊挂电机转动方向控制，高电平正转
    }
  
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM1=flag;   //1号吊挂电机脉冲输出口高/低电平
		HANG_PWM2=flag;   //2号吊挂电机脉冲输出口高/低电平
		HANG_PWM3=flag;   //3号吊挂电机脉冲输出口高/低电平
		HANG_PWM4=flag;   //4号吊挂电机脉冲输出口高/低电平	
		if(arm_fore_left_right_flag==1)    //左右胳膊
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{
				arm_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_left_right_runed--;
			}
			//发送左胳膊吊挂动画指令	
			j=arm_left_right_runed/(arm_left_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}			
			}	
		}
		if( leg_fore_left_right_flag==1)    //左右腿
		{
			//通过上下行判断脉冲累加
			if(1==dir)     //上行，脉冲+
			{				
				leg_left_right_runed++; 
			}
			else           //下行，脉冲-
			{
				leg_left_right_runed--;
			}
			//发送左腿吊挂动画指令						
			j=leg_left_right_runed/(leg_left_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}			
			}			
		}	  		 
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	HANG_PWM2=0;           //2号吊挂电机脉冲输出口高电平 
	HANG_PWM3=0;           //3号吊挂电机脉冲输出口低电平
	HANG_PWM4=0;           //4号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Uart_Hand_Hang_1()
*函数功能     ：1号吊挂电机：左肢小臂
*输入         ：dir:方向口；n：脉冲口
*输出         ：无
***********************************************************************/
void Uart_Hand_Hang_1(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;            //flag为脉冲高低标志位	
	static int k=0;	
    Hang_Init();               //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR1=1;  		   //1正转，0反转 
	}
	else
	{
		HANG_DIR1=0;  		   //1正转，0反转 
	}	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器10
	for(i=0;i<pulse;i++)
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
		HANG_PWM1=flag; 	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;				
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftUpHand"))||    
			   (strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftDownHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftUpHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftDownHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftUpHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftDownHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftUpHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftDownHand")))	
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("USART2_RX_BUF=%s",USART2_RX_BUF);
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(USART2_RX_BUF,0,len);   //清除接收
				USART2_RX_LEN=0; 
			}			
		}
		//发送左小臂动画指令
		if((1==arm_fore_left_flag)||(1==arm_fore_post_left_flag))        //左小臂、左大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_fore_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_left_runed--;
			}
			j=arm_fore_left_runed/(arm_fore_left_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_12");
				}				
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_15");
				}				
			}
		}
		else if((1==leg_fore_left_flag)||(1==leg_fore_post_left_flag))   //左小腿、左大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_fore_left_runed++;
			}
			else           //下行，脉冲-
			{
				leg_fore_left_runed--;
			}
			j=leg_fore_left_runed/(leg_fore_left_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_15");
				}				
			}			
		}
		
		
		if(((arm_fore_left_runed==0)&&((1==arm_fore_left_flag)||(1==arm_fore_post_left_flag)) )||
			((leg_fore_left_runed==0)&&((1==leg_fore_left_flag)||(1==leg_fore_post_left_flag))))
		{
			k=0;     
		}		
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Uart_Hand_Hang_1_2()
*函数功能     ：1/2号吊挂电机联动：左肢
*输入         ：m:方向口；pulse：脉冲口
*输出         ：无
***********************************************************************/
void Uart_Hand_Hang_1_2(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;     //flag为脉冲高低标志位
	static int k=0;
    Hang_Init();        //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR1=1;  	//1正转，2反转
		HANG_DIR2=0;
	}
	else
	{
		HANG_DIR1=0;  	//1正转，2反转
		HANG_DIR2=1;
	}	   
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器3
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位
		
		HANG_PWM1=flag; 
		HANG_PWM2=flag; 
	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftDownHand"))) 	
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(USART2_RX_BUF,0,len);   //清除接收
				USART2_RX_LEN=0; 
			}
		}
		//发送左胳膊动画指令
		if(1==arm_fore_left_flag)        //左小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_left_runed--;
			}
			j=arm_left_runed/(arm_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}
			}
		}
		else if((1==arm_post_left_flag)||(1==arm_fore_post_left_flag))   //左大臂、左大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_post_left_runed++;
			}
			else           //下行，脉冲-
			{
				arm_post_left_runed--;
			}
			j=arm_post_left_runed/(arm_post_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}
			}
		}		
		//发送左腿动画指令
		else if(1==leg_fore_left_flag)   //左小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_left_runed++;
			}
			else           //下行，脉冲-
			{
				leg_left_runed--;
			}
			j=leg_left_runed/(leg_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}
			}			
		}
		else if((1==leg_post_left_flag)||(1==leg_fore_post_left_flag))   //左大腿、左大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_post_left_runed++;
			}
			else           //下行，脉冲-
			{
				leg_post_left_runed--;
			}
			j=leg_post_left_runed/(leg_post_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}
			}			
		}
		if(((arm_left_runed==0)&&(1==arm_fore_left_flag))||
			((arm_post_left_runed==0)&&((1==arm_post_left_flag)||(1==arm_fore_post_left_flag)))||
			((leg_left_runed==0)&&(1==leg_fore_left_flag))||
			((0==leg_post_left_runed)&&((1==leg_post_left_flag)||(1==leg_fore_post_left_flag))))
		{
			k=0;     
		}
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	HANG_PWM2=0;           //2号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Uart_Hand_Hang_3()
*函数功能     ：3号吊挂电机：右肢小臂
*输入         ：m:方向口；n：脉冲口
*输出         ：无
***********************************************************************/
void Uart_Hand_Hang_3(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;          //flag为脉冲高低标志位	
	static int k=0;		
    Hang_Init();             //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR3=0;  		 //1正转，2反转 
	}
	else
	{
		HANG_DIR3=1;  		 //1正转，2反转 
	}
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
		HANG_PWM3=flag; 	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightDownHand")))	
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(USART2_RX_BUF,0,len);
				USART2_RX_LEN=0; 
			}
		}
		//发送右小臂动画指令
		if((1==arm_fore_right_flag)||(1==arm_fore_post_right_flag))         //右小臂、右大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_fore_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_right_runed--;
			}
			j=arm_fore_right_runed/(arm_fore_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Right_12");
				}				
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Right_15");
				}				
			}
		}
		else if((1==leg_fore_right_flag)||(1==leg_fore_post_right_flag))    //右小腿、右大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_fore_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_fore_right_runed--;
			}
			j=leg_fore_right_runed/(leg_fore_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Right_15");
				}				
			}			
		}
		if(((arm_fore_right_runed==0)&&((1==arm_fore_right_flag)||(1==arm_fore_post_right_flag)))||
			((leg_fore_right_runed==0)&&((1==leg_fore_right_flag)||(1==leg_fore_post_right_flag))))
		{
			k=0;     
		}		
		
  }
	HANG_PWM3=0;           //3号吊挂电机脉冲输出口低电平
	TIM10_Stop();          //关闭定时器
}


/*********************************************************************
*函数名       ：Uart_Hand_Hang_3_4()
*函数功能     ：3/4号吊挂电机联动：右肢
*输入         ：dir:方向口；pulse：脉冲数
*输出         ：无
***********************************************************************/
void Uart_Hand_Hang_3_4(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flag为脉冲高低标志位
	static int k=0;	
    Hang_Init();          //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR3=0;  	  //1正转，2反转
		HANG_DIR4=1;
	}
	else
	{
		HANG_DIR3=1;  	  //1正转，2反转
		HANG_DIR4=0;
	}	 
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
		HANG_PWM3=flag; 
		HANG_PWM4=flag; 	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightDownHand")))	
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(USART2_RX_BUF,0,len);
				USART2_RX_LEN=0; 
			}			
		}
		//发送右胳膊动画指令
		if(1==arm_fore_right_flag)            //右小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_right_runed--;
			}
			j=arm_right_runed/(arm_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}
			}
		}
		else if(1==leg_fore_right_flag)       //右小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_right_runed--;
			}
			j=leg_right_runed/(leg_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}
			}			
		}
		
		else if((1==arm_post_right_flag)||(1==arm_fore_post_right_flag))    //右大臂、右大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_post_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_post_right_runed--;
			}
			j=arm_post_right_runed/(arm_post_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}
			}
		}
		else if((1==leg_post_right_flag)||(1==leg_fore_post_right_flag))    //右大腿、右大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_post_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_post_right_runed--;
			}
			j=leg_post_right_runed/(leg_post_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}
			}			
		}


		if(((arm_right_runed==0)&&(1==arm_fore_right_flag))||
			((0==leg_right_runed)&&(1==leg_fore_right_flag))||
			((arm_post_right_runed==0)&&((1==arm_post_right_flag)||(1==arm_fore_post_right_flag)))||
			((0==leg_post_right_runed)&&((1==leg_post_right_flag)||(1==leg_fore_post_right_flag))))
		{
			k=0;     
		}		
   }
	HANG_PWM3=0;           //3号吊挂电机脉冲输出口低电平
	HANG_PWM4=0;           //4号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Uart_Hand_Hang_1_3()
*函数功能     ：1/3号吊挂电机联动：左右肢小臂/小腿
*输入         ：m:方向口；pulse：脉冲口
*输出         ：无
***********************************************************************/
void Uart_Hand_Hang_1_3(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flag为脉冲高低标志位
	static int k=0;		
    Hang_Init();          //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR1=1;  	  //1正转，2反转
		HANG_DIR3=0;
	}
	else
	{
		HANG_DIR1=0;  	  //1正转，2反转
		HANG_DIR3=1;
	}	   
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器3
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位
		
		HANG_PWM1=flag; 
		HANG_PWM3=flag; 
	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightDownHand")))	
			{			
				u2_printf("break_flag==1");
				break;
			} 
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(USART2_RX_BUF,0,len);
				USART2_RX_LEN=0; 
			}
		}
		//发送左右胳膊动画指令
		if((1==arm_fore_left_right_flag)||(1==arm_fore_post_left_right_flag))          //左右小臂、左右大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_fore_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_fore_left_right_runed--;
			}
			j=arm_fore_left_right_runed/(arm_fore_left_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_Right_15");
				}				
			}
		}
		else if((1==leg_fore_left_right_flag)||(1==leg_fore_post_left_right_flag))     //左右小腿、左右大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_fore_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_fore_left_right_runed--;
			}
			j=leg_fore_left_right_runed/(leg_fore_left_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_Right_15");
				}				
			}			
		}
		if(((arm_fore_left_right_runed==0)&&((1==arm_fore_left_right_flag)||(1==arm_fore_post_left_right_flag)))||
			((0==leg_fore_left_right_runed)&&((1==leg_fore_left_right_flag)||(1==leg_fore_post_left_right_flag))))
		{
			k=0;     
		}			
	}
	HANG_PWM1=0;           //1号吊挂电机脉冲输出口低电平
	HANG_PWM3=0;           //2号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Uart_Hand_Hang_1_2_3_4()
*函数功能     ：1/2/3/4号吊挂电机联动：左右肢
*输入         ：dir:方向口；pulse：脉冲数
*输出         ：无
***********************************************************************/
void Uart_Hand_Hang_1_2_3_4(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flag为脉冲高低标志位
	static int k=0;	
    Hang_Init();          //电机方向口初始化函数
	if(1==dir)
	{
		HANG_DIR1=1;
		HANG_DIR2=0;  	  //1正转，2反转
		HANG_DIR3=0;
		HANG_DIR4=1; 
	}
	else
	{
		HANG_DIR1=0;
		HANG_DIR2=1;  	  //1正转，2反转
		HANG_DIR3=1;
		HANG_DIR4=0; 
	}	
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //打开定时器10
	for(i=0;i<pulse;i++)
  { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
		HANG_PWM1=flag;
	    HANG_PWM2=flag;
	    HANG_PWM3=flag;
		HANG_PWM4=flag; 	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightDownHand")))		
			{			
				u2_printf("break_flag==1");
				break;
			} 
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //若接收到Stop,则跳出循环	
			{	}
			else
			{
				u2_printf("NotRun");       //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
				memset(USART2_RX_BUF,0,len);
				USART2_RX_LEN=0; 
			}
		}
		//发送左右胳膊动画指令
		if(1==arm_fore_left_right_flag)          //左右小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_left_right_runed--;
			}
			j=arm_left_right_runed/(arm_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}
			}
		}
		else if(1==leg_fore_left_right_flag)     //左右小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_left_right_runed--;
			}
			j=leg_left_right_runed/(leg_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}
			}			
		}
		else if((1==arm_post_left_right_flag)||(1==arm_fore_post_left_right_flag))  //左右大臂、左右大小臂
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				arm_post_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				arm_post_left_right_runed--;
			}
			j=arm_post_left_right_runed/(arm_post_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}
			}
		}
		else if((1==leg_post_left_right_flag)||(1==leg_fore_post_left_right_flag))  //左右大腿、左右大小腿
		{
			//通过上下行判断脉冲累计
			if(1==dir)      //上行，脉冲+
			{
				leg_post_left_right_runed++;
			}
			else           //下行，脉冲-
			{
				leg_post_left_right_runed--;
			}
			j=leg_post_left_right_runed/(leg_post_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}
			}			
		}		
		if(((arm_left_right_runed==0)&&(1==arm_fore_left_right_flag))||
			((0==leg_left_right_runed)&&(1==leg_fore_left_right_flag))||
			((arm_post_left_right_runed==0)&&((1==arm_post_left_right_flag)||(1==arm_fore_post_left_right_flag)))||
			((0==leg_post_left_right_runed)&&((1==leg_post_left_right_flag)||(1==leg_fore_post_left_right_flag))))
		{
			k=0;     
		}		
  }
	HANG_PWM1=0; 
	HANG_PWM2=0;           //3号吊挂电机脉冲输出口低电平
	HANG_PWM3=0; 
	HANG_PWM4=0;           //4号吊挂电机脉冲输出口高电平 
	TIM10_Stop();          //关闭定时器
}

/*********************************************************************
*函数名       ：Uart_Motor_6_2_START()
*函数功能     ：坐便器袋子收紧前推杆驱动，
*输入         ：dir(运行方向)，pulse(运行脉冲数)
*输出         ：无
***********************************************************************/
void Uart_Motor_6_2_START(u8 dir,u32 pulse)
{
	
	Motor_Dir_Init();
	int i,flag=1;          //flag为脉冲高低标志位	
    DIR6_2=dir;            //坐便袋收紧驱动电机转动方向控制，1高电平正转，0低电平反转 

	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
    TIM10_Init(3600-1,motor_timer_freq);                       //打开定时器
//	if(((1==dir)&&(push_rod_tig_runed_pulse<push_rod_tig_pulse_lim))||((0==dir)&&(push_rod_tig_runed_pulse>0)))
//    {
		for(i=0;i<pulse;i++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
			flag = !flag ;
			PWM6_2=flag;          //坐便袋收紧驱动电机脉冲输出口高/低电平
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位		
			//根据上下行判断脉冲累计
			if(1==dir)           //上行，脉冲+
			{
				push_rod_tig_runed_pulse++;
			}
			else                 //下行，脉冲-
			{
				push_rod_tig_runed_pulse--;
			}
			//发送动画指令
			if(push_rod_tig_runed_pulse==1)
			{
				u2_printf("Cartoon_Push_Rod_Tig_1");
			}
			if(push_rod_tig_runed_pulse==(u16)(pulse/2))
			{
				u2_printf("Cartoon_Push_Rod_Tig_2");
			}
			if(((push_rod_tig_runed_pulse==pulse)&&(1==dir))||((push_rod_tig_runed_pulse==pulse-1)&&(0==dir)))
			{
				u2_printf("Cartoon_Push_Rod_Tig_3");
			} 
		}
//	}
	PWM6_2=0;                      //坐便袋收紧驱动电机对应脉冲输出口拉低  
	TIM10_Stop();                  //关闭定时器
}



/*********************************************************************
*函数名       ：MotorStart()
*函数功能     ：坐便器袋子收紧前推杆驱动，
*输入         ：MotorID-电机ID，dir-方向，定时器重装载值arr(自动重装载值)
*输出         ：无
***********************************************************************/
void MotorStart(u8 MotorID,u8 dir,u16 arr)
{
	switch(MotorID)
	{
		case 1:
			Motor_1_START(dir);
			
			
			break;
		case 2:
			Push_Rod_Start(dir);
			break;
		case 3:
			DIR3=dir;
			Motor_3_START(arr,motor_timer_freq);
			break;
		case 4:
			DIR4=dir;
			Motor_4_START(arr,motor_timer_freq);
			break;
		case 5:
			DIR5=dir;
			Motor_5_START(arr,motor_timer_freq);
			break;
		case 6:
			DIR6=dir;
			Motor_6_START(arr,motor_timer_freq);		
			break;
		case 7:
			DIR7=dir;
			Motor_7_START(arr,motor_timer_freq);
			break;
		case 8:
			RELAY6=1; 
			Uart_Push_Rod_Swash(dir,arr);   //1:冲洗烘干推杆伸出,0:冲洗烘干推杆收回
			RELAY6=0; 
			break;
		case 9:
			RELAY6=1;             //继电器得电
			Uart_Motor_6_2_START(dir,arr);      //1:收线推杆伸出	,0:收线推杆缩回
			RELAY6=0;  
			break;
		case 10:
			UartWashletTig(dir,13000,arr);		//1:坐便袋收线电机收线,0放线
			break;
	}
}

/*********************************************************************
*函数名       ：Motor345Start(u8 dir,u16 M3Arr,u16 M4Arr,u16 M5Arr,u16 psc)
*函数功能     ：坐便器袋子收紧前推杆驱动，
*输入         ：MotorID-电机ID，dir-方向，定时器重装载值arr(自动重装载值)，psc(分频值)
*输出         ：无
***********************************************************************/
void Motor345Start(u8 dir,u16 M3Arr,u16 M4Arr,u16 M5Arr)
{
	Motor_Dir_Init();                     	 //电机方向口初始化函数
	TIM8_PWM_CHANNEL_2_START(M3Arr,motor_timer_freq);     //TIM8通道 2 PWM输出函数 ，3号电机
	TIM5_PWM_CHANNEL_4_START(M4Arr,motor_timer_freq);		 //TIM5通道 4 PWM输出函数 ，4号电机
	TIM8_PWM_CHANNEL_1_START(M5Arr,motor_timer_freq);     //TIM8通道 1 PWM输出函数 ,5号电机

	DIR3=dir;
	DIR4=dir;
	DIR5=dir;
}


/*********************************************************************
*函数名       ：MotorStop()
*函数功能     ：坐便器袋子收紧前推杆驱动，
*输入         ：MotorID-电机ID
*输出         ：无
***********************************************************************/ 
void MotorStop(u8 MotorID)
{
	switch(MotorID)
	{
		case 1:
			Motor_1_STOP();
		case 2:
			Push_Rod_Stop();
		case 3:
			Motor_3_STOP();
			break;
		case 4:
			Motor_4_STOP();
			break;
		case 5:
			Motor_5_STOP();
			break;
		case 6:
			Motor_6_STOP();	
			break;
		case 7:
			Motor_7_STOP();	
			break;
		case 8:
			Motor_3_STOP();
			Motor_4_STOP();
			Motor_5_STOP();
			break;
	}
}

/*********************************************************************
*函数名       ：WashletRun()
*函数功能     ：坐便袋运作
*输入         ：
*输出         ：无
***********************************************************************/ 
void WashletRun(u8 dir,u16 TimArr,u16 TimPsc)
{
	u8 err=0;
	u8 i=0;
	u8 breakflg=0;
	if((leg_up_flag==0)&&(body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))		//连锁，坐便器执行一定要在翻身和上曲腿复位时才行
	{
//		if((washlet_flag==0)&&((1==GD6_Start)||(1==Motor6_Alm)))	//判断初始位置光电是否在位
//		{
//			delay_us(100);
//			if(((1==GD6_Start)||(1==Motor6_Alm)))
//			{
//				err=1;
//				u2_printf("GD6SErr");
//				BeepRun(2,300);
//				//此处可以写下复位函数....						
//			}
//		}
//		else
//		{err=0;}
		
		//正常情况下的执行
		if(err==0)
		{
			MotorStart(6,!dir,motor_washlet_freq);
			TIM10_Init(TimArr,TimPsc); 
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);			
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				//光电限位
				if((0==GD6_End)&&(1==dir))
				{
					delay_us(100);
					if(0==GD6_End)
					{
						breakflg=1;
						u2_printf("GD6End"); 
						break ;
					}						
				}
				if((0==GD6_Start)&&(0==dir))
				{
					delay_us(100);
					if(0==GD6_Start)
					{
						breakflg=1;
						u2_printf("GD6Start"); 
						break ; 
					}
				}
			}		
			//跳出while循环，判断光电有没有到位
			if(dir==0)
			{
				GDCheckDealy(GD6S,300);	//检测GD6Start,若光电没到，则延时300ms	
				if(GDCheckAlm(GD6S,2))
				{
					//此处写未到位的函数		
				}
			}		
			else if(dir==1)
			{
				GDCheckDealy(GD6E,300);
				if(GDCheckAlm(GD6E,2))
				{
					//此处写未到位的函数		
				}
			}	
			MotorStop(6);	//6号电机停止
			TIM10_Stop();		
			breakflg=0;
			//最后判断有没有到位
			//if((GD6_Start==0)&&(dir==0))		//复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))			
			{washlet_flag=0;}	 		//标志位清零
			//else if((GD6_End==0)&&(dir==1))	//终了位置
			else if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==dir))	//终了位置
			{washlet_flag=1;	}	
			else
			{
				washlet_flag=1;
				if(dir==0)
					{u2_printf("GD6_SFalse\r\n");}
				else
					{u2_printf("GD6_EFalse\r\n");}
			}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);			
		}
	}
	else{LedAlm(300,"Uart_Washlet_Auto_Interfere");}//若不满足条件，LED0/LED1闪一下
}

/*********************************************************************
*函数名       ：BodyLeftRun()
*函数功能     ：翻身
*输入         ：
*输出         ：无
***********************************************************************/ 
void BodyLeftRun(u8 dir,u16 M3Arr,u16 M4Arr,u16 Angle)
{
	u8 err=0;
	u16 arr_now;
	static u8 motor5_run_flag; 
  u8 breakflag;

	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{
//		if((body_left_flag==0)&&(body_right_flag==0))
//		{
//			if((GD3_Start==1)||(GD4_Start==1)||(GD5_Start==1)||(1==Motor5_Alm))
//			{
//				delay_us(100);
//				if((GD3_Start==1)||(GD4_Start==1)||(GD5_Start==1)||(1==Motor5_Alm))
//				{
//					err=1;
//					if(	GD3_Start==1)
//					{u2_printf("GD3SErr");BeepRun(2,300);}	
//					else if(GD4_Start==1)
//					{u2_printf("GD4SErr");BeepRun(2,300);}
//					else if(GD5_Start==1)
//					{u2_printf("GD4SErr");BeepRun(2,300);}
//				}
//			}
//			else
//			{err=0;}
//		}
//		else{err=0;}		
		if(err==0)
		{
			if((body_left_flag==0)&&(dir==1))  //如果复位到初始状态，才执行左翻起
			{
				body_left_flag=1;
				motor5_run_flag=1;
				u2_printf("BodyLeftStart");		delay_ms(200);		
				MotorStart(5,1,motor_body_freq);
				TIM10_Init(body_angle_to_arr(Angle),timer10_freq); 
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)) )
				{
						//光电限位
//						if((0==GD5_Left_End)&&(1==dir))
//						{
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								//breakflag=1;
//								u2_printf("GD5LE"); 
//								break ;
//							}						
//						}
						//电机故障、故障诊断
//						if(1==Motor5_Alm)       
//						{	
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("BodyLeftOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;							
//							}	             
//						}													
				}	
				GDCheckDealy(GD5LE,300);
				if(GDCheckAlm(GD5LE,2))
				{
					//此处写未到位的函数		
				}
				Motor_5_STOP();            //电机5停止
				TIM10_Stop();              //定时器关闭
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
				Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);	//调用补偿函数
			}	
			//翻身345号电机动作	
			if((dir==1)||(motor5_run_flag==1))
			{
				if(body_angle_to_arr(Angle)>body_left_runed_arr)
				{
					u2_printf("345Up");
					motor5_run_flag=0;					
					body_left_dir_flag=1;
					Motor345Start(0,M3Arr,M4Arr,motor_body_freq);
					TIM10_Init(body_angle_to_arr(Angle)-body_left_runed_arr,timer10_freq);	
				}
			}
			else if(dir==0)
			{
				u2_printf("345Down");
				if(body_left_runed_arr>0)
				{
					Motor345Start(1,M3Arr,M4Arr,motor_body_freq);
					TIM10_Init(body_left_runed_arr,timer10_freq);	
				}
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位			
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;

			if(((body_left_runed_arr!=body_angle_to_arr(Angle))&&(1==dir))||((0!=body_left_runed_arr)&&(0==dir)))
			{
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  
				{
					//光电限位
						if(((0==GD3_Start)||(0==GD4_Start))&&(0==dir))
						{
							delay_us(100);
							if(0==GD3_Start)
							{
								u2_printf("GD3S"); 
								break ;
							}
							else if(0==GD4_Start)
							{
								u2_printf("GD4S"); 
								break ;
							}
						}
						else if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==dir))
						{
							delay_us(100);
							if(0==GD3_Left_End)
							{
								u2_printf("GD3LE"); 
								break ;
							}
							else if(0==GD4_Left_End)
							{
								u2_printf("GD4LE"); 
								break ;
							}
						}									
						if(UsartCheck2("BodyLeftUpNew","BodyLeftDownNew"))
						{
							u2_printf("Stop\r\n");
							breakflag=1;
							break;
						}
				}				
				if((breakflag==0)&&(dir==0))
				{
					GDCheckDealy(GD34S,300);
					if(GDCheckAlm(GD34S,2))
					{
						//此处写未到位的函数		
					}					
				}				
				MotorStop(8);	//3,4,5电机停止	
				breakflag=0;				
			//判断复位			
			//if(((GD3_Start==0)||(GD4_Start)||(GD5_Start))&&(dir=0))
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))
				{
					arr_now=0;
					body_left_flag=0;
				}	

				else
				{					
					arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now
					body_left_flag=1;
	//				W25QXX_Write((u8*)&body_left_flag,33,1);														
				}
				//通过上下行判断脉冲累计
				if(dir==1)           //上行，脉冲++
				{
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{
						body_left_runed_arr=body_angle_to_arr(Angle);
						u2_printf("LeftEnd\r\n");
						u2_printf("dir=1,时间到\r\n");
						u2_printf("body_left_runed_arr=%d\r\n",body_left_runed_arr);
					}
					else
					{
						body_left_runed_arr=body_left_runed_arr+arr_now;
						u2_printf("body_left_runed_arr=%d\r\n",body_left_runed_arr);
						u2_printf("运行到这边了1\r\n");
					}		
				}	
				else     //下行，脉冲-
				{			
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{
						body_left_runed_arr=0;
						u2_printf("body_left_runed_arr=%d\r\n",body_left_runed_arr);
						u2_printf("运行到这边了2\r\n");
					}
					else
					{
						body_left_runed_arr=body_left_runed_arr-arr_now;	
						u2_printf("body_left_runed_arr=%d\r\n",body_left_runed_arr);	
						u2_printf("运行到这边了3\r\n");						
					}
				}					
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //清除中断标志位				

				if(body_left_flag==0) 
				{
					Motor_4_Compensate(1,bodyleft_compleate,motor_body_freq,motor_timer_freq);
					u2_printf("5号电机开始复位");
					MotorStart(5,0,motor_body_freq);
					
					body_left_runed_arr=0;
					TIM10_Init(body_angle_to_arr(Angle),timer10_freq);                     //打开定时器				
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
					{
						if((0==GD5_Start)&&(0==dir))
						{
							delay_us(100);
							if(0==GD5_Start)
							{
								//breakflag=1;
								u2_printf("GD5S"); 
								break ;
							}						
						}
					}
					GDCheckDealy(GD5S,300);
					if(GDCheckAlm(GD5S,2))
					{
						//此处写未到位的函数		
					}							
					Motor_5_STOP();       //电机停止
					TIM10_Stop();         //关闭定时器
					breakflag=0;         //清除标志位
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
					delay_ms(200);
					u2_printf("body_left_flag==0");
					delay_ms(200);
					u2_printf("BodyLeftRes");
					delay_ms(200);
				}
			}	
		}
	}
	else
	{		LedAlm(300,"BodyLeftInterfere");	}
}

/*********************************************************************
*函数名       ：BodyRightRun()
*函数功能     ：翻身
*输入         ：
*输出         ：无
***********************************************************************/ 
void BodyRightRun(u8 dir,u16 M3Arr,u16 M4Arr,u16 Angle)
{
	u8 err=0;
	u16 arr_now;
	static u8 motor5_run_flag; 
  u8 breakflag;
	if((lock_flag==1)&&(back_flag==0)&&(body_left_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{
//		if((body_left_flag==0)&&(body_right_flag==0))
//		{
//			if((GD3_Start==1)||(GD4_Start==1)||(GD5_Start==1)||(1==Motor5_Alm))
//			{
//				delay_us(100);
//				if((GD3_Start==1)||(GD4_Start==1)||(GD5_Start==1)||(1==Motor5_Alm))
//				{
//					err=1;
//					if(	GD3_Start==1)
//					{u2_printf("GD3SErr");BeepRun(2,300);}	
//					else if(GD4_Start==1)
//					{u2_printf("GD4SErr");BeepRun(2,300);}
//					else if(GD5_Start==1)
//					{u2_printf("GD4SErr");BeepRun(2,300);}
//				}
//			}
//			else
//			{err=0;}
//		}
//		else{err=0;}		
		if(err==0)
		{
			if((body_right_flag==0)&&(dir==1))  //如果复位到初始状态，才执行左翻起
			{
				body_right_flag=1;
				motor5_run_flag=1;
				u2_printf("BodyRightStart");		delay_ms(200);		
				MotorStart(5,0,motor_body_freq);
				TIM10_Init(body_angle_to_arr(Angle),timer10_freq); 
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)) )
				{
						//光电限位
						if((0==GD5_Right_End)&&(1==dir))
						{
							delay_us(100);
							if(0==GD5_Right_End)
							{
								breakflag=1;
								u2_printf("GD5RE"); 
								break ;
							}						
						}
						//电机故障、故障诊断
//						if(1==Motor5_Alm)       
//						{	
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("BodyLeftOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;							
//							}	             
//						}													
				}	
				GDCheckDealy(GD5RE,300);
				Motor_5_STOP();            //电机5停止
				TIM10_Stop();              //定时器关闭
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
				Motor_4_Compensate(1,bodyright_compleate,motor_body_freq,motor_timer_freq);	//调用补偿函数
			}	
			//翻身345号电机动作	
			if((dir==1)||(motor5_run_flag==1))
			{
				if(body_angle_to_arr(Angle)>body_right_runed_arr)
				{
					u2_printf("345Up");
					motor5_run_flag=0;					
					body_right_dir_flag=1;
					Motor345Start(1,M3Arr,M4Arr,motor_body_freq);
					TIM10_Init(body_angle_to_arr(Angle)-body_right_runed_arr,timer10_freq);	
				}
			}
			else if(dir==0)
			{
				u2_printf("345Down");
				if(body_right_runed_arr>0)
				{
					Motor345Start(1,M3Arr,M4Arr,motor_body_freq);
					TIM10_Init(body_right_runed_arr,timer10_freq);	
				}
			}
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位
			if(((body_right_runed_arr!=body_angle_to_arr(Angle))&&(1==dir))||((0!=body_right_runed_arr)&&(0==dir)))
			{
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  
				{
					//光电限位
						if(((0==GD3_Start)||(0==GD4_Start))&&(0==dir))
						{
							delay_us(100);
							if(0==GD3_Start)
							{
								breakflag=1;
								u2_printf("GD3S"); 
								break ;
							}
							else if(0==GD4_Start)
							{
								breakflag=1;
								u2_printf("GD4S"); 
								break ;
							}
						}
						else if(((0==GD3_Right_End)||(0==GD4_Right_End))&&(1==dir))
						{
							delay_us(100);
							if(0==GD3_Right_End)
							{
								u2_printf("GD3RE"); 
								break ;
							}
							else if(0==GD4_Right_End)
							{
								u2_printf("GD4RE"); 
								break ;
							}
						}									
						if(UsartCheck2("BodyRightUpNew","BodyRightDownNew"))
						{
							u2_printf("Stop\r\n");
							breakflag=1;
							break;
						}
				}				
				if((breakflag==0)&&(dir==0))
				{
					GDCheckDealy(GD34S,300);
				}				
				MotorStop(8);	//3,4,5电机停止
				breakflag=0;				
			//判断复位			
			//if(((GD3_Start==0)||(GD4_Start)||(GD5_Start))&&(dir=0))
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))
				{
					arr_now=0;
					body_right_flag=0;
				}	
				else
				{					
					arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now
					body_right_flag=1;
	//				W25QXX_Write((u8*)&body_left_flag,33,1);														
				}
				//通过上下行判断脉冲累计
				if(dir==1)           //上行，脉冲+
				{
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{
						body_right_runed_arr=body_angle_to_arr(Angle);
						u2_printf("RightEnd\r\n");
						u2_printf("body_right_runed_arr=%d\r\n",body_right_runed_arr);
					}
					else
					{
						body_right_runed_arr=body_right_runed_arr+arr_now;
						u2_printf("body_right_runed_arr=%d\r\n",body_right_runed_arr);
					}		
				}	
				else     //下行，脉冲-
				{			
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{
						body_right_runed_arr=0;
						u2_printf("body_right_runed_arr=%d\r\n",body_right_runed_arr);
					}
					else
					{
						body_right_runed_arr=body_right_runed_arr-arr_now;	
						u2_printf("body_right_runed_arr=%d\r\n",body_right_runed_arr);						
					}
				}					
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //清除中断标志位				

				if(body_right_flag==0) 
				{
					Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);
					u2_printf("5号电机开始复位");
					MotorStart(5,1,motor_body_freq);
					
					body_right_runed_arr=0;
					TIM10_Init(body_angle_to_arr(Angle),timer10_freq);                     //打开定时器				
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
					{
						if((0==GD5_Start)&&(0==dir))
						{
							delay_us(100);
							if(0==GD5_Start)
							{
								//breakflag=1;
								u2_printf("GD5S"); 
								break ;
							}						
						}
					}
					GDCheckDealy(GD5S,300);
					Motor_5_STOP();       //电机停止
					TIM10_Stop();         //关闭定时器
					breakflag=0;         //清除标志位
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
					delay_ms(200);
					u2_printf("body_right_flag==0");
					delay_ms(200);
					u2_printf("BodyRightRes");
					delay_ms(200);
				}
			}	
		}
	}
	else
	{		LedAlm(300,"BodyRightInterfere");	}
}


/*********************************************************************
*函数名       ：DeskRun()
*函数功能     ：小桌子
*输入         ：
*输出         ：无
***********************************************************************/ 
void DeskRun(u8 dir,u16 HalfDist)
{
	u8 err=0;
	u8 breakflg=0;
	u16 arr_now; 
//	DeskLastJudgeFlg=0;
	
//	DeskSwitch=0;
	
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{
//		if((desk_flag==0)&&((1==GD7_Start)||(1==Motor7_Alm)))
//		{
//			delay_us(100);
//			if((1==GD7_Start)||(1==Motor7_Alm))
//			{	err=1;
//				u2_printf("GD67SErr");
//				BeepRun(2,300);
//				//此处可以写下复位函数....		
//			}
//			else
//			{err=0;}
//		}
//		else
//		{err=0;}
		
		if(err==0)
		{
			if(dir==1)
			{
				u2_printf("DeskUp\r\n");
				if(desk_distance_to_arr(HalfDist)>desk_runed_arr)
				{
					MotorStart(7,dir,motor_desk_freq);
					TIM10_Init(desk_distance_to_arr(HalfDist)-desk_runed_arr,timer10_freq); 				
				}
			}
			else if(dir==0)
			{
				u2_printf("DeskDown\r\n");
				if(desk_runed_arr>0)
				{
					MotorStart(7,dir,motor_desk_freq);
					TIM10_Init(desk_runed_arr,timer10_freq); 				
				}
			}
			
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;	
			
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);	
		 if(((desk_runed_arr!=desk_distance_to_arr(HalfDist))&&(1==dir))||((0!=desk_runed_arr)&&(0==dir)))	
		 { 
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{
					//光电限位
//					if((0==GD7_End)&&(1==dir))
//					{
//						delay_us(100);
//						if(0==GD6_End)
//						{
//							breakflg=1;
//							u2_printf("GD7End"); 
//							break ;
//						}						
//					}
//					if((0==GD7_Start)&&(0==dir))
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							breakflg=1;
//							u2_printf("GD7Start"); 
//							break ; 
//						}
//					}
						if(UsartCheck2("DeskRunUpNew","DeskRunDownNew"))
						{
							u2_printf("Stop");
							breakflg=2;
							break;
						}
				}

				if((breakflg==0)&&(Desk1st2st==1)&&(dir==1))
				{
					DeskSwitch=1;
					Desk1st2st=2;
					arr_now=0;
					desk_runed_arr=0;
					u2_printf("1切换2\r\n");
					DeskRun(dir,HalfDist);
				}
				else if((breakflg==0)&&(Desk1st2st==2)&&(dir==0))
				{
					DeskSwitch=1;
					Desk1st2st=1;
					desk_runed_arr=desk_distance_to_arr(HalfDist);
					u2_printf("2切换1\r\n");
					//arr_now=desk_distance_to_arr(HalfDist);
					DeskRun(dir,HalfDist);
				}
				else
				{u2_printf("无切换\r\n");}
				
				if(breakflg!=2)
				{
					if(dir==0)
					{GDCheckDealy(GD7S,300);}
					else if(dir==1)
					{GDCheckDealy(GD7E,300);}
				}	
				
				
				Motor_7_STOP();     //电机停止
				TIM10_Stop();       //定时器关闭
				breakflg=0;
				
				
				if((DeskSwitch==0)||(DeskLastJudgeFlg==0))
				{	
					DeskLastJudgeFlg=1;
					DeskSwitch=0;
					//最后判断有没有到位
					//if((GD7_Start==0)&&(dir==0))		//复位
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))			
					{
						desk_flag=0;	 			//标志位清零
						arr_now=0;  
					}
					else
					{
						arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now		
						desk_flag=1;
					}
					
					if(dir==1)
					{
						//if(0==GD7_End)
						if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
						{
							desk_runed_arr=desk_distance_to_arr(HalfDist);
							u2_printf("DeskLim");
							delay_ms(200);				
						}
						else
						{  desk_runed_arr=desk_runed_arr+arr_now;	}
					}
					else if(dir==0)
					{	//if(0==GD7_Start)
						if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
						{
							desk_runed_arr=0;			
							u2_printf("DeskRes");
							delay_ms(200);
						}
						else
						{	desk_runed_arr=desk_runed_arr-arr_now;	}

					}
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); 
					u2_printf("Desk1st2st=%d",Desk1st2st);
				}
				else
				{DeskLastJudgeFlg=1;
					
				}
			}
		 else
		 {u2_printf("ERR\r\n");}
		}
	}
	else
	{		LedAlm(300,"DeskInterfere");	}
}



void DeskRun1(u8 dir,u16 Dist)
{
	u8 err=0;
	u8 breakflg=0;
	u16 arr_now; 

	
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{
//		if((desk_flag==0)&&((1==GD7_Start)||(1==Motor7_Alm)))
//		{
//			delay_us(100);
//			if((1==GD7_Start)||(1==Motor7_Alm))
//			{	err=1;
//				u2_printf("GD7SErr");
//				BeepRun(2,300);
//				//此处可以写下复位函数....		
//			}
//			else
//			{err=0;}
//		}
//		else
//		{err=0;}
		
		if(err==0)
		{	
			if(desk_flag==0)
			{
				if(dir==1)
				{
					u2_printf("第一阶段开始\r\n");
					MotorStart(7,!dir,motor_desk_freq);
					TIM10_Init((u16)(desk_distance_to_arr(Dist)/2.6),timer10_freq);
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //清除中断标志位
					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) 
					{
						
					}
					Motor_7_STOP();     //电机停止
					TIM10_Stop();       //定时器关闭
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
					u2_printf("电机7第一阶段运行完成\r\n");
				}
			}
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
			
			if((dir==1))
			{
				u2_printf("DeskUp\r\n");
				if(desk_distance_to_arr(Dist)>desk_runed_arr)
				{
					MotorStart(7,!dir,motor_desk_freq);
					TIM10_Init(desk_distance_to_arr(Dist)-desk_runed_arr,timer10_freq); 				
				}
			}
			else if(dir==0)
			{
				if(desk_runed_arr>0)    //下行时，比较值为校准后的当量装载值与0,且不能为0
				{
					MotorStart(7,!dir,motor_desk_freq);
					TIM10_Init(desk_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
				}
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
			if(((desk_runed_arr!=desk_distance_to_arr(Dist))&&(1==dir))||((0!=desk_runed_arr)&&(0==dir)))
			{
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{
					//光电限位											   						
//					if((0==GD7_End)&&(1==dir))
//					{
//						delay_us(100);
//						if(0==GD7_End)
//						{
//							u2_printf("GD7End");					
//							break;					
//						}								
//					}

					
					if(UsartCheck2("DeskRunUpNew","DeskRunDownNew"))
					{
						u2_printf("Stop");
						breakflg=1;
						break;
					}			
				}
				
				if(breakflg==0)
				{
					if(dir==1)
					{
						GDCheckDealy(GD7E,300);
						if(GDCheckAlm(GD7E,2))
						{
							//此处写未到位的函数		
						}	
					}
				}
				Motor_7_STOP();     //电机停止
				TIM10_Stop();       //定时器关闭
				breakflg=0;
	
				//判断复位
				//if((GD7_Start==0)&&(dir==0))
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))   //判断是否处于复位状态，复位状态的前提是下行的定时器走完
				{
					arr_now=0;         //此时处于复位状态，将状态值都设为0；
					desk_flag=0;
				}
				else
				{
					arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now				
					desk_flag=1;
				}
				
				//通过上下行判断脉冲累计
				if(dir==1)        //如果是小桌子前进，则用+
				{
					//if(0==GD7_End)
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{ 					
						desk_runed_arr=desk_distance_to_arr(Dist);
						u2_printf("DeskLim");
						delay_ms(200);
					}
					else
					{  desk_runed_arr=desk_runed_arr+arr_now;	}				
				}
				else if(dir==0)                 //如果是小桌子后退，则用-
				{
					//if(0==GD7_Start)
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{						
						desk_runed_arr=0; 
						u2_printf("DeskRes");
						delay_ms(200);
					}
					else
					{	 
						desk_runed_arr=desk_runed_arr-arr_now;
					}						
				}				
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
				u2_printf("desk_runed_arr=%d",desk_runed_arr);
				
				if(desk_flag==0)
				{
					MotorStart(7,1,motor_desk_freq);
					desk_runed_arr=0;
					Motor_7_START(motor_desk_freq,motor_timer_freq);
					TIM10_Init((u16)(desk_distance_to_arr(Dist)/2.6),timer10_freq);
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //清除中断标志位
					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) 
					{
						if((0==GD7_Start)&&(0==dir))
						{
//							delay_us(100);
//							if(0==GD7_Start)
//							{
//								u2_printf("GD7Start");					
//								break;						
//							}					
						}						
					}				
					if(dir==0)
					{
						GDCheckDealy(GD7S,300);
						if(GDCheckAlm(GD7S,2))
						{
							//此处写未到位的函数		
						}	
					}
					Motor_7_STOP();     //电机停止
					TIM10_Stop();       //定时器关闭
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
					
					u2_printf("电机7第一阶段返回完成");					
					}
			}
		}
	}
	else
	{		LedAlm(300,"DeskInterfere");	}
}















