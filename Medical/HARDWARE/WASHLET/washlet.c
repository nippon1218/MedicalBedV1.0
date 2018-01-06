#include "sys.h"
#include "usart.h"
#include "washlet.h"
#include "led.h"
#include "key.h"
#include "motor.h"
#include "delay.h"
#include "pump.h"

void WashLetAuto_V1(u8 dir,u16 arr)
{



}


/***********************************************************************
 函数名      ：Uart_Washlet(void)  
 函数功能    ：按键执行坐便器功能
 输入        ：dir: 1(打开坐便)；0（关闭坐便）
 输出        ：无
                           
************************************************************************/
void WashLet_V1(u8 dir,u16 arr)
{
	u16 arr_send;
	u16 j=0;	
	static u16 k=0; 
	static u8 kj;
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;

	washlet_flag=1;
	if((leg_up_flag==0)&&(body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{
		DIR6=!dir;		//DIR6=0时打开，DIR6=1时关闭
		u2_printf("坐便器开始动作");
		Motor_6_START(motor_washlet_freq,motor_timer_freq);           //电机启动
		TIM10_Init(arr,timer10_freq);                     //打开定时器35000
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位
		if(1==dir)
		{
			washlet_flag=1;	
			u2_printf("Cartoon_Washlet_1");
			delay_ms(100);  
		}
		else
		{
			washlet_picture_k=24;
			u2_printf("Cartoon_Washlet_25");	
			delay_ms(100);
		}
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //等待定时时间到，时间到跳出循环
		{	
			//光电限位
//				if((0==GD6_End)&&(0==dir))
//				{
//					delay_us(100);
//					if(0==GD6_End)
//					{
//						break_flag=1;
//						u2_printf("GD6End"); 
//						break ;
//					}						
//				}
//				if((0==GD6_Start)&&(1==dir))
//				{
//					delay_us(100);
//					if(0==GD6_Start)
//					{
//						break_flag=1;
//						u2_printf("GD6Start"); 
//						break ; 
//					}
//				}
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
				}
			
				//电机故障、故障诊断
//				if(1==Motor6_Alm)     
//				{
//					delay_us(100);
//					if(1==Motor6_Alm)  
//					{
//						washlet_auto_overload=1;
//						u2_printf("WashletAutoOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;		
//					}						
//				}	
				
			//发送动画指令
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(arr/24);
			if(0==dir)
			{
				j=24-j;
			}
			k=washlet_picture_k;
			if(k!=j)
			{
				kj=abs(k,j);				
				if(kj<2)
				{
					k=j;   washlet_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Washlet_2");
								break;
						case 2:	u2_printf("Cartoon_Washlet_3");
								break;					
						case 3:	u2_printf("Cartoon_Washlet_4");
								break;					
						case 4:	u2_printf("Cartoon_Washlet_5");
								break;	
						case 5:	u2_printf("Cartoon_Washlet_6");
								break;	
						case 6:	u2_printf("Cartoon_Washlet_7");
								break;
						case 7:	u2_printf("Cartoon_Washlet_8");
								break;	
						case 8:	u2_printf("Cartoon_Washlet_9");
								break;	
						case 9:	u2_printf("Cartoon_Washlet_10");
								break;
						case 10:u2_printf("Cartoon_Washlet_11");
								break;	
						case 11:u2_printf("Cartoon_Washlet_12");
								break;	
						case 12:u2_printf("Cartoon_Washlet_13");
								break;		
						case 13:u2_printf("Cartoon_Washlet_14");
								break;	
						case 14:u2_printf("Cartoon_Washlet_15");
								break;	
						case 15:u2_printf("Cartoon_Washlet_16");
								break;	
						case 16:u2_printf("Cartoon_Washlet_17");
								break;		
						case 17:u2_printf("Cartoon_Washlet_18");
								break;	
						case 18:u2_printf("Cartoon_Washlet_19");
								break;
						case 19:u2_printf("Cartoon_Washlet_20");
								break;	
						case 20:u2_printf("Cartoon_Washlet_21");
								break;	
						case 21:u2_printf("Cartoon_Washlet_22");
								break;		
						case 22:u2_printf("Cartoon_Washlet_23");
								break;	
						case 23:u2_printf("Cartoon_Washlet_24");
								break;								
					}
				}
			}							
		}
			Motor_6_STOP();    //6号电机停止
			TIM10_Stop();      //关闭定时器	
		//判断复位
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))//判断是否处于复位状态，复位状态的前提是下行的定时器走完
		{		
			washlet_flag=0;	 
			delay_ms(200);
			u2_printf("Cartoon_Washlet_1");		
		}
		 else if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==dir))
		 {
			washlet_flag=1;    
			delay_ms(200);
			u2_printf("Cartoon_Washlet_25");
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
		u2_printf("坐便器结束动作");
	}
	else
	 {
			LED0=0;          //若不满足条件，LED0/LED1闪一下
			LED1=0;
			delay_ms(100);
			LED0=1;
			LED1=1;    	
	 }	
}




void WashLet_V2(u8 dir,u16 arr)
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	
	//联锁功能，只有在上曲腿、左右翻身复位后，才能执行座便器功能	
	if((lock_flag==1)&&(0==body_left_flag)&&(0==body_right_flag)) 
	{
		if(0==washlet_auto_flag)
		{			
			u2_printf("Uart_Washlet_Auto_Start");
			delay_ms(200);
		}          
	
		if((dir==1)&&(0==washlet_auto_flag))     //自动坐便正行程
		{
			u2_printf("washlet_auto_flag==1");			 
			washlet_auto_flag=1;
			delay_ms(200);
			if((1==back_flag)&&(0==leg_down_flag))                 //此时已处于支背支起状态
			{
				Push_Rod_Start(1);                                 //下曲腿下	
				leg_down_state_flag=1;                             //下曲腿需要动作，支背不需要 
				back_state_flag=0;
				u2_printf("Cartoon_Washlet_Leg_Down_1");
				delay_ms(200);
			}
			else if((0==back_flag)&&(1==leg_down_flag))            //此时已处于下曲腿状态
			{
				Motor_1_START(1);                                   //支背上行
				back_dir_flag=1;
				back_state_flag=1;                                  //支背需要动作，下曲腿不需要 
				leg_down_state_flag=0;
				u2_printf("Cartoon_Washlet_Back_1");
				delay_ms(200);
			}
			else if((0==back_flag)&&(0==leg_down_flag))            //此时处于支背及下曲腿复位状态
			{
				Motor_1_START(1);                                  //支背上行   
				Push_Rod_Start(1);                                 //下曲腿下
				back_dir_flag=1;
				leg_down_state_flag=1;                             //支背需要动作，下曲腿需要动作       
				back_state_flag=1;
				u2_printf("Cartoon_Washlet_Back_Leg_Down_1");
				delay_ms(200);
			}
			else if((1==back_flag)&&(1==leg_down_flag))            //此时已处于支背支起及下曲腿状态
			{
				leg_down_state_flag=0;            
				back_state_flag=0; 
			}
		}		
		else if((dir==0)&&(1==washlet_auto_flag))         //自动坐便复位-支背、下曲腿复位
		{
			u2_printf("恢复了恢复了,开始恢复\r\n");
			back_dir_flag=0; 
			leg_down_state_flag=1;            
			back_state_flag=1;			
			Motor_1_START(0);                                      //支背下行       
			Push_Rod_Start(0);                                     //下曲腿上			
			u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
			delay_ms(200);
		}
		
		if((1==leg_down_state_flag)||(1==back_state_flag))
		{	
			if(1==leg_down_state_flag){ leg_down_flag=1; }
			if(1==back_state_flag)    { back_flag=1;     }
			Uart_Back_Leg();
		}
		
		if(dir==1)
		{
			//Uart_Washlet(0);	            //坐便打开		
			u2_printf("坐便打开\r\n");			
			WashLet_V1(1,arr);
			delay_ms(500);
			u2_printf("开始重物检测\r\n");		
			Uart_Washlet_Weight();          //重物检测
			u2_printf("结束重物检测\r\n");	
			//delay_ms(1000);

			Uart_Swash_Dry();             //冲洗烘干			
			
			
			WashLet_V1(0,arr);	            //坐便关闭
			u2_printf("坐便关闭\r\n");	
			
			if(washlet_flag==0)             //判断坐便是否处于复位状态，再进行坐便袋收紧
			{									

//				washlet_flag=1;
//				RELAY6=1;
//				u2_printf("继电器得电\r\n");
				//继电器得电
				//Uart_Motor_6_2_START(1,21000);   //收线推杆伸出
//				u2_printf("收线推杆伸出\r\n");
//				RELAY6=0; 
//				u2_printf("继电器失电\r\n");
//				washlet_flag=0;
							
				delay_ms(100);
				Uart_Washlet_Tig(1);        //坐便袋收紧				
				//Uart_Washlet_Auto();        //再次调用该函数，使标志位取反，复位
//				washlet_flag=1;
//				RELAY6=1;
//				Uart_Motor_6_2_START(0,21000);   //收线推杆缩回
//				RELAY6=0;
//				washlet_flag=0;
//				u2_printf("收线推杆缩回\r\n");
				
				
				//小桌子
				u2_printf("\r\n\r\n*****小桌子******\r\n\r\n");
				DeskRun1(1,100);
				delay_ms(6000);  	delay_ms(4000);  
				DeskRun1(0,100);
				delay_ms(1000);				
							
				WashLet_V2(0,arr);
				
				leg_down_flag=0;            //清除标志位
				back_flag=0;
				washlet_auto_flag=0;
				leg_down_state_flag=0;
				back_state_flag=0;
				back_dir_flag=0;
				delay_ms(200);
				u2_printf("washlet_auto_flag==0");
				delay_ms(200);
				u2_printf("WashletAutoRes");				
			}	
		}				
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Uart_Washlet_Auto_Interfere");
		LED0=1;
		LED1=1;  
	}	
}


