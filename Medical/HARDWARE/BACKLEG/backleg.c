#include "sys.h"
#include "backleg.h"
#include "function.h"
#include "led.h"
#include "motor.h"
#include "delay.h"
#include "check.h"




void BackRun(u8 dir,u8 angle)
{
	u8 len;
	u16 arr_now;              //当前一次运行脉冲数
	
//实现上位机实时显示护理床当前运动状态
	static u8 k;             //传第k张动画
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //当前一次运行脉冲值
	static u8 back_limit_flag; //支背运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位后，才能进行支背
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		if(dir==1)		//如果为支背上行
		{
			if(back_angle_to_arr(angle)>back_runed_arr)  //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{				
				back_dir_flag=1;      
				if(back_flag==0)
				{
					back_flag=1;
					delay_ms(200);
					u2_printf("back_flag==1");
					delay_ms(200);
					u2_printf("BackStart");
					delay_ms(200);
					u2_printf("Cartoon_Back_1");
					delay_ms(200);
				}
				Motor_1_START(1);                                                          //支背上行
				TIM10_Init(back_angle_to_arr(angle)-back_runed_arr,timer10_freq); //打开定时器
			}
		}
		else if(dir==0)
		{
			if(back_runed_arr>0)    //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				back_dir_flag=0; 
				if(1==back_limit_flag)
				{
					back_limit_flag=0;
					delay_ms(200);
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
				}
				Motor_1_START(0);                                                          //支背上行
				TIM10_Init(back_runed_arr,timer10_freq); //打开定时器
			}
		}
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
		
	 	if(((back_runed_arr!=back_angle_to_arr(angle))&&(1==dir))||((0!=back_runed_arr)&&(0==dir)))
		{					
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环   
			{		
					//停止指令
				if(UsartCheck2("BackUpNew","BackDownNew"))
					{
						u2_printf("Stop\r\n");
						break;
					}

				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);      //当前一次脉冲值
				//传输动画指令
				if(dir==1)
				{
					j=(back_runed_arr+arr_send)/(back_angle_to_arr(angle)/19);
				}
				else
				{
					j=abs(back_runed_arr,arr_send)/(back_angle_to_arr(angle)/19);
				}
				k=back_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);					
					if(kj<2)
					{
						k=j;   back_picture_k=k;					
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Back_2");									
									break;
							case 2:	u2_printf("Cartoon_Back_3");
									break;					
							case 3:	u2_printf("Cartoon_Back_4");
									break;					
							case 4:	u2_printf("Cartoon_Back_5");
									break;	
							case 5:	u2_printf("Cartoon_Back_6");
									break;	
							case 6:	u2_printf("Cartoon_Back_7");
									break;
							case 7:	u2_printf("Cartoon_Back_8");
									break;
							case 8:	u2_printf("Cartoon_Back_9");
									break;						
							case 9:	u2_printf("Cartoon_Back_10");
									break;												
							case 10:u2_printf("Cartoon_Back_11");
									break;
							case 11:u2_printf("Cartoon_Back_12");									
									break;
							case 12:u2_printf("Cartoon_Back_13");
									break;					
							case 13:u2_printf("Cartoon_Back_14");
									break;					
							case 14:u2_printf("Cartoon_Back_15");
									break;	
							case 15:u2_printf("Cartoon_Back_16");
									break;	
							case 16:u2_printf("Cartoon_Back_17");
									break;
							case 17:u2_printf("Cartoon_Back_18");
									break;
							case 18:u2_printf("Cartoon_Back_19");
									break;		
						}
					}				
				}	
			}		
		Motor_1_STOP();    //电机停止
		TIM10_Stop();      //关闭定时器
		//判断复位
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))   //判断是否处于复位状态，复位状态的前提是下行的定时器走完
		{				
			arr_now=0;         //此时处于复位状态，将状态值都设为0；
			back_flag=0;
			delay_ms(200);
			u2_printf("back_flag==0");
			delay_ms(200);			
		}		
		else
		{
			arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now				
			back_flag=1;
		}
      //通过上下行判断脉冲累计		
		if(	dir==1)        //如果是支背上行，则用+
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{  
				back_runed_arr=back_angle_to_arr(angle);
				back_limit_flag=1; 
				delay_ms(200);
				u2_printf("Cartoon_Back_20");
				delay_ms(200);
				u2_printf("BackLim");
				delay_ms(200);
			}
			else
			{  back_runed_arr+=arr_now;	}				
		}
		else                //如果是支背下行，则用-
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{	
				back_runed_arr=0;
				delay_ms(200);
				u2_printf("Cartoon_Back_1");				
				delay_ms(200);
				u2_printf("BackRes");
				delay_ms(200);
			}
			else
			{
				back_runed_arr-=arr_now;
			}						
		}		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
	}
}
	else
	{
		LedAlm(300,"BackInterfere");
	}	
}

void LegUpRun(u8 dir,u8 angle)
{
	u16 arr_now;              //当前一次运行脉冲数
	u8 len;                   //WiFi串口接收字符串长度
//实现上位机实时显示护理床当前运动状态
	static u8 k=0;            //传输第k张动画指令
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;               //当前一次运行脉冲值
	static u8 leg_up_limit_flag;//上曲腿运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位及下曲腿复位后才能进行上曲腿
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_down_flag==0)&&(lock_flag==1))
	{		
		if(dir==1)  //上曲腿上行	
		{				   
			if(leg_angle_to_arr(angle)>leg_up_runed_arr)             //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				if(leg_up_flag==0)
				{
					leg_up_flag=1;
					delay_ms(200);					
					u2_printf("leg_up_flag==1");
					delay_ms(200);
					u2_printf("LegUpStart");
					delay_ms(200);
					u2_printf("Cartoon_Leg_Up_1");
					delay_ms(200);
				}				
				Push_Rod_Start(0);
				TIM10_Init(leg_angle_to_arr(angle)-leg_up_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz					
			} 
		}
		else if(dir==0)  //上曲腿下行
		{			
			if(leg_up_runed_arr>0)                          //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				if(leg_up_limit_flag==1)
				{
					leg_up_limit_flag=0;
					delay_ms(200);
					u2_printf("Cartoon_Leg_Up_8");
					delay_ms(200);
				}										
				Push_Rod_Start(1);
				TIM10_Init(leg_up_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;				
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //清除中断标志位
	 	if(((leg_up_runed_arr!=leg_angle_to_arr(angle))&&(1==dir))||((0!=leg_up_runed_arr)&&(0==dir)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{
					//停止指令
					if(UsartCheck2("LegUpUpNew","LegUpDownNew"))
					{
						u2_printf("Stop\r\n");
						break;
					}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//判断传输动画指令
				if(dir==1)
				{
					j=(leg_up_runed_arr+arr_send)/(leg_angle_to_arr(leg_up_angle_max)/7);
				}
				else
				{
					j=abs(leg_up_runed_arr,arr_send)/(leg_angle_to_arr(leg_up_angle_max)/7);
				}
				k=leg_up_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);					
					if(kj<2)
					{
						k=j;    leg_up_picture_k=k;						
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Leg_Up_2");									
									break;
							case 2:	u2_printf("Cartoon_Leg_Up_3");
									break;					
							case 3:	u2_printf("Cartoon_Leg_Up_4");
									break;					
							case 4:	u2_printf("Cartoon_Leg_Up_5");
									break;	
							case 5:	u2_printf("Cartoon_Leg_Up_6");
									break;	
							case 6:	u2_printf("Cartoon_Leg_Up_7");
									break;
						}
					}					
				}				
			}				//等待定时时间到，时间到跳出循环
	
		Push_Rod_Stop();    //推杆停止
		TIM10_Stop();       //关闭定时器
		//判断复位
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0)) //判断是否处于复位状态，复位状态的前提是下行的定时器走完
		{
			arr_now=0;                  //此时处于复位状态，将状态值都设为0；
			leg_up_flag=0;
			delay_ms(200);
			u2_printf("leg_up_flag==0");
			delay_ms(200);
		}
		else
		{
			arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);       
			leg_up_flag=1;
		}			
		 //通过上下行判断脉冲累计	
		if(dir==1)    //如果是上曲腿上行，则用+
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{
				leg_up_runed_arr=leg_angle_to_arr(angle);
				leg_up_limit_flag=1;	
				delay_ms(200);
				u2_printf("Cartoon_Leg_Up_8");
				delay_ms(200);
				u2_printf("LegUpLim");
				delay_ms(200);
			}
			 else
			{
				leg_up_runed_arr=leg_up_runed_arr+arr_now;
			}	
		}
		else     //如果是上曲腿下行，则用-
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{
				leg_up_runed_arr=0;	
				delay_ms(200);
				u2_printf("Cartoon_Leg_Up_1");
				delay_ms(200);
				u2_printf("LegUpRes");
				delay_ms(200);
			}
			 else
			{
				leg_up_runed_arr=leg_up_runed_arr-arr_now;
			}			
		}	
		 __HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 		
	 }
   }	
	else
	{
		LedAlm(300,"LegUpInterfere");
	}
}


void LegDownRun(u8 dir,u8 angle)
{
	u16 arr_now;         //当前一次运行脉冲数   
	u8 len;              //接收的字符串长度
	
//实现上位机实时显示护理床当前运动状态
	static u8 k=0;       //发送第K张动画指令
	static u8 kj=0;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;        //当前一次运行脉冲数
	static u8 leg_down_limit_flag;//下曲腿运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位及下曲腿复位后才能进行上曲腿
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(lock_flag==1))
	{
		if(dir==1)   //下曲腿下行
		{				
			if(leg_angle_to_arr(angle)>leg_down_runed_arr)             //下行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				if(leg_down_flag==0)
				{
					leg_down_flag=1;
					delay_ms(200);
					u2_printf("leg_down_flag==1");
					delay_ms(200);
					u2_printf("LegDownStart");
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_1");
					delay_ms(200);
				}							
				Push_Rod_Start(1);
				TIM10_Init(leg_angle_to_arr(angle)-leg_down_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
				leg_down_flag=1;
			}	
		}
		else if(dir==0)  //下曲腿上行
		{
			
			if(leg_down_runed_arr>0)     //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				if(leg_down_limit_flag==1)
				{
					leg_down_limit_flag=0;
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_20");
					delay_ms(200);
				}							
				Push_Rod_Start(0);
				TIM10_Init(leg_down_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //清除中断标志位	

	   if(((leg_down_runed_arr!=leg_angle_to_arr(angle))&&(1==dir))||((0!=leg_down_runed_arr)&&(0==dir)))
	   {			
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{			
				
					//停止指令
					if(UsartCheck2("LegDownUpNew","LegDownDownNew"))
					{
						u2_printf("Stop\r\n");
						break;
					}			
				
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//发送动画指令
				if(dir==1)
				{
					j=(leg_down_runed_arr+arr_send)/(leg_angle_to_arr(leg_down_angle_max)/19);
				}
				else
				{
					j=abs(leg_down_runed_arr,arr_send)/(leg_angle_to_arr(leg_down_angle_max)/19);
				}
				k=leg_down_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;   leg_down_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Leg_Down_2");									
									break;
							case 2:	u2_printf("Cartoon_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Leg_Down_12");									
									break;
							case 12:u2_printf("Cartoon_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Leg_Down_15");
									break;	
							case 15:u2_printf("Cartoon_Leg_Down_16");
									break;	
							case 16:u2_printf("Cartoon_Leg_Down_17");
									break;
							case 17:u2_printf("Cartoon_Leg_Down_18");
									break;
							case 18:u2_printf("Cartoon_Leg_Down_19");
									break;																		
						}
					}				
				}				 
			}				   //等待定时时间到，时间到跳出循环	      
			Push_Rod_Stop();   //推杆停止
			TIM10_Stop();      //关闭定时器
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))
			{
				arr_now=0;                 //此时处于复位状态，将状态值都设为0；
				leg_down_flag=0;
				delay_ms(200);			
				u2_printf("leg_down_flag==0");
				delay_ms(200);
			}
			else
			{			
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);     
				leg_down_flag=1;
			}			
			//通过上下行判断脉冲累计
			if(dir==1)    //如果是下曲腿下行，则用+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_down_runed_arr=leg_angle_to_arr(angle);
					leg_down_limit_flag=1;	   
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_20");
					delay_ms(200);
					u2_printf("LegDownLim");
					delay_ms(200);
				}
				 else
				{
					leg_down_runed_arr=leg_down_runed_arr+arr_now;
				}	
			}
			else		//如果是下曲腿上行，则用-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_down_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_1");
					delay_ms(200);
					u2_printf("LegDownRes");
					delay_ms(200);
				}
				 else
				{
					leg_down_runed_arr=leg_down_runed_arr-arr_now;
				}			
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 					
		}
	}
	else
	{
		LedAlm(300,"LegDownInterfere");
	}
}






