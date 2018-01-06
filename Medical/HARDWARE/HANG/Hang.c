#include "Hang.h"
#include "sys.h"
#include "usart.h"
#include "led.h"
#include "motor.h"
#include "function.h"
#include "pcf8574.h"
#include "delay.h"

void HangRun(u8 dir,u8 height,u8 hang1,u8 hang2,u8 hang3,u8 hang4)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;
	
	HANG_DIR1=dir;       //1号吊挂电机转动方向控制，高电平Up，低电平Down
	HANG_DIR2=!dir;			 //2号吊挂电机转动方向控制，高电平Down，低电平Up
	HANG_DIR3=dir;			 //3号吊挂电机转动方向控制，高电平Up，低电平Down
	HANG_DIR4=!dir;		  	//4号吊挂电机转动方向控制，高电平Down，低电平Up
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(70-1,motor_timer_freq);                       //打开定时器3
	
    for(i=0;i<100000*height;i++)   //最终确定为3000000
    { 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
			flag = !flag ;
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
			if(hang1==1)
			{
				HANG_PWM1=flag;               //3号吊挂电机脉冲输出口高电平
			}
			if(hang2==1)
			{
				HANG_PWM2=flag;               //3号吊挂电机脉冲输出口高电平
			}
			if(hang3==1)
			{
				HANG_PWM3=flag;               //3号吊挂电机脉冲输出口高电平
			}
			if(hang4==1)
			{
				HANG_PWM4=flag;               //3号吊挂电机脉冲输出口高电平
			}
	  }
	HANG_PWM1=0;              //3号吊挂电机对应脉冲输出口拉低 
	HANG_PWM2=0; 
	HANG_PWM3=0; 
	HANG_PWM4=0; 
	TIM10_Stop();             //关闭定时器
		u2_printf("%d\r\n",height);
			if(hang1==1)
			{
				u2_printf("hang1\r\n");
			}
			if(hang2==1)
			{
				u2_printf("hang2\r\n");
			}
			if(hang3==1)
			{
				u2_printf("hang3\r\n"); 
			}
			if(hang4==1)
			{
				u2_printf("hang4\r\n");
			}
}




void HangRunAuto(u8 dir,u8 turn,u8 ArmLeg,u8 times,u16 height)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;					
	
	HANG_DIR1=dir;       //1号吊挂电机转动方向控制，高电平Up，低电平Down
	HANG_DIR2=!dir;			 //2号吊挂电机转动方向控制，高电平Down，低电平Up
	HANG_DIR3=dir;			 //3号吊挂电机转动方向控制，高电平Up，低电平Down
	HANG_DIR4=!dir;		  	//4号吊挂电机转动方向控制，高电平Down，低电平Up
	
	TIM10_Init(70-1,motor_timer_freq);                       //打开定时器3

	for(i=0;i<100000*height;i++)   //最终确定为3000000
	{
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );
		//flag=!flag;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); 
		if(turn==0)
		{
			HANG_PWM2=!HANG_PWM2;
			if(i%times==0)
			{HANG_PWM1=!HANG_PWM1;}
			
			if(ArmLeg==0)	//臂，0
			{
				if(1==dir)
				{arm_left_runed++;}
				else
				{arm_left_runed--;}
				
				j=arm_left_runed/(100000*height/4);
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
			else		//腿，1
			{
				if(1==dir)
				{leg_left_runed++;}
				else
				{leg_left_runed--;}
				
				j=leg_left_runed/(100000*height/4);
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
		else
		{
			HANG_PWM3=!HANG_PWM3;
			if(i%times==0)
			{HANG_PWM4=!HANG_PWM4  ;}
			
			if(ArmLeg==0)	//臂，0
			{
				if(1==dir)
				{arm_right_runed++;}
				else
				{arm_right_runed--;}
				
				j=arm_right_runed/(100000*height/4);
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
			else		//腿，1
			{
				if(1==dir)
				{leg_right_runed++;}
				else
				{leg_right_runed--;}
				
				j=leg_right_runed/(100000*height/4);
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

	}
	HANG_PWM1=0;              //3号吊挂电机对应脉冲输出口拉低 
	HANG_PWM2=0; 
	HANG_PWM3=0; 
	HANG_PWM4=0; 		
	TIM10_Stop();             //关闭定时器
	u2_printf("\r\ndir=%d,turn=%d,ArmLeg=%d,times=%d,height=%d\r\n",dir,turn,ArmLeg,times,height);
}

void Uart_Auto_Arm_Leg(u8 dir,u8 turn,u8 ArmLeg,u8 times,u16 height)
{
	int j;
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   { 
		 PCF8574_WriteBit(EXIO1,0);	
		 if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftAuto"))
		 {
			 if(leg_fore_left_flag==0)       //防止按键误触发，导致标志位置位
				{	
					arm_fore_left_flag=1;
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmLeftAutoStart");				
				}
		 }
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftAuto"))    //左腿
		{
			if(arm_fore_left_flag==0)      //防止按键误触发，导致标志位置位
			{	
				leg_fore_left_flag=1;
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegLeftAutoStart");
				delay_ms(200);
			}			
		}		 
		HangRunAuto(1,0,0,2,15);
		delay_ms(1000);	 
		for(j=0;j<1;j++)                   //进行t此左肢康复训练
		{
			Uart_Auto_Hang_1(1,75000);     //向上运动
			delay_ms(1000);
			Uart_Auto_Hang_1(0,75000);     //向下运动
			delay_ms(1000);	
		}	
		 
		 
	 }


}



