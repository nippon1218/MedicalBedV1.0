#include "function.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "motor.h"
#include "pump.h"
#include "ds18b20.h"
#include "hx711.h"
#include "timer.h"
#include "pwm.h"
#include "key.h"
#include "modbus_master.h"
#include "bsp_user_lib.h"
#include "nand.h"
#include "pcf8574.h"
#include "common.h"
#include "w25qxx.h"
#include "check.h"
#include "hang.h"
#include "washlet.h"

//称重
unsigned long weight1[10]; 
unsigned long weight2[10];
unsigned long weight3[10];
unsigned long u1=0,u2=0,u3=0;

unsigned int i;
unsigned int k,w,num=1;

u8 device_num;                      //用于计算当前WiFi连接设备数
u8 _step=0;

/********************机构限制运行最大角度********************************/

u8  back_angle_max=85;               //上位机设定支背运行角度
u8  leg_up_angle_max=30;             //上位机设定上曲腿运行角度
u8  leg_down_angle_max=80;           //上位机设定下曲腿运行角度
u8  body_left_angle_max=45;          //上位机设定左翻运行角度
u8  body_right_angle_max=40;         //上位机设定右翻运行角度
u8  desk_distance_max=100;           //上位机设定娱乐小桌子运动距离
u8  swash_dry_time_max=1;            //上位机设定娱乐坐便冲洗烘干时间/分钟
u16 washlet_arr_lim=32950;           //坐便器运行下，计时器的arr（重装载值）12000

/********************已运行脉冲值********************************/

u16 back_runed_arr=0;                //支背，调整后的自动重装载值
u16 leg_up_runed_arr=0;              //上曲腿，调整后的自动重装载值
u16 leg_down_runed_arr=0;            //下曲腿，调整后的自动重装载值
u16 body_left_runed_arr=0;           //左翻，调整后的自动重装载值
u16 body_right_runed_arr=0;          //右翻，调整后的自动重装载值
u16 washlet_runed_arr=0;             //坐便器，调整后的自动重装载值
u16 desk_runed_arr=0;                //桌子，调整后的自动重装载值
u16 back_nursing_left_runed_arr=0;   //左背护理，调整后的自动重装载值
u16 back_nursing_right_runed_arr=0;  //右背护理，调整后的自动重装载值
u16 waist_nursing_left_runed_arr=0;  //左腰护理，调整后的自动重装载值
u16 waist_nursing_right_runed_arr=0; //右腰护理，调整后的自动重装载值

u16 add_arr=5000;                    //为保证复位到初始位置，另加额外脉冲值

/********************吊挂康复脉冲数计算********************************/

//左肢
u32 arm_left_runed=0;                  //相对起始位置已运行脉冲数             
u32 arm_left_lim=2000000;              //极限脉冲值 
u32 leg_left_runed=0;                  //相对起始位置已运行脉冲数             
u32 leg_left_lim=3000000;              //极限脉冲值 

//右肢
u32 arm_right_runed=0;                 //相对起始位置已运行脉冲数            
u32 arm_right_lim=2000000;             //极限脉冲值 
u32 leg_right_runed=0;                 //相对起始位置已运行脉冲数            
u32 leg_right_lim=3000000;             //极限脉冲值 

//左右肢
u32 arm_left_right_runed=0;            //相对起始位置已运行脉冲数        
u32 arm_left_right_lim=2000000;        //极限脉冲值 
u32 leg_left_right_runed=0;            //相对起始位置已运行脉冲数        
u32 leg_left_right_lim=3000000;        //极限脉冲值 

//左小臂/小腿
u32 arm_fore_left_runed=0;             //相对起始位置已运行脉冲数        
u32 arm_fore_left_lim=2000000;         //极限脉冲值 
u32 leg_fore_left_runed=0;             //相对起始位置已运行脉冲数        
u32 leg_fore_left_lim=3000000;         //极限脉冲值 

//左大臂/大腿
u32 arm_post_left_lim=2000000;         //极限脉冲值       
u32 arm_post_left_runed=0;             //相对起始位置已运行脉冲数
u32 leg_post_left_lim=3000000;         //极限脉冲值       
u32 leg_post_left_runed=0;             //相对起始位置已运行脉冲数


//右小臂/小腿
u32 arm_fore_right_lim=2000000;        //极限脉冲值      
u32 arm_fore_right_runed=0;            //相对起始位置已运行脉冲数
u32 leg_fore_right_lim=3000000;        //极限脉冲值      
u32 leg_fore_right_runed=0;            //相对起始位置已运行脉冲数

//右大臂/大腿
u32 arm_post_right_lim=2000000;        //极限脉冲值      
u32 arm_post_right_runed=0;            //相对起始位置已运行脉冲数
u32 leg_post_right_lim=3000000;        //极限脉冲值      
u32 leg_post_right_runed=0;            //相对起始位置已运行脉冲数


//左右小臂/腿
u32 arm_fore_left_right_lim=2000000;   //极限脉冲值 
u32 arm_fore_left_right_runed=0;       //相对起始位置已运行脉冲数
u32 leg_fore_left_right_lim=3000000;   //极限脉冲值 
u32 leg_fore_left_right_runed=0;       //相对起始位置已运行脉冲数

//左右大臂/大腿
u32 arm_post_left_right_lim=2000000;   //极限脉冲值 
u32 arm_post_left_right_runed=0;       //相对起始位置已运行脉冲数
u32 leg_post_left_right_lim=3000000;   //极限脉冲值 
u32 leg_post_left_right_runed=0;       //相对起始位置已运行脉冲数

/********************冲洗烘干推杆***************************/

u32 push_rod_runed_pulse=0;                   //相对起始位置已运行脉冲数  
u32 push_rod_pulse_lim=30000;                  //极限脉冲

u32 swash_dry_runed_pulse=0;                  //相对起始位置已运行脉冲数  
u32 swash_dry_pulse_lim=25000;                 //极限脉冲

/********************坐便袋收紧前移动的推杆***************************/

u32 push_rod_tig_runed_pulse=0;              //相对起始位置已运行脉冲数  
u32 push_rod_tig_pulse_now=0;                //当前一次运行脉冲数
u32 push_rod_tig_pulse_lim=5000;             //极限脉冲

/*****************复位状态标志位，1：未复位 ； 0：复位到初始状态***********/

u8 back_flag=0;               //支背
u8 leg_up_flag=0;             //上曲腿
u8 leg_down_flag=0;           //下曲腿
u8 body_left_flag=0;          //左翻
u8 body_right_flag=0;         //右翻
u8 back_nursing_left_flag=0;  //左背部护理
u8 back_nursing_right_flag=0; //右背部护理
u8 waist_nursing_left_flag=0; //左腰部护理
u8 waist_nursing_right_flag=0;//右腰部护理
u8 washlet_flag=0;            //坐便器
u8 washlet_auto_flag=0;       //自动坐便器
u8 desk_flag=0;               //就餐娱乐一体桌
u8 jram_flag=0;               //肌肉按摩
u8 swash_dry_flag=0;          //冲洗烘干
u8 lock_flag=1;               //一键锁定程序
u8 fault_flag=0;              //电机故障标志位

u8 swash_hand_flag=0;         //手动冲洗标志位
u8 dry_hand_flag=0;           //手动烘干标志位

//吊挂
u8 armleg_left_flag=0;        //手动左肢
u8 armleg_right_flag=0;       //手动右肢
u8 armleg_left_right_flag=0;  //手动左右肢

u8 arm_fore_left_flag=0;      //左小臂
u8 leg_fore_left_flag=0;      //左小腿

u8 arm_fore_right_flag=0;     //右小臂
u8 leg_fore_right_flag=0;     //右小腿

u8 arm_post_left_flag=0;      //左大臂
u8 leg_post_left_flag=0;      //左大腿

u8 arm_post_right_flag=0;     //右大臂
u8 leg_post_right_flag=0;     //右大腿

u8 arm_fore_post_left_flag=0;       //左大小臂
u8 leg_fore_post_left_flag=0;       //左大小腿

u8 arm_fore_post_right_flag=0;      //右大小臂
u8 leg_fore_post_right_flag=0;      //右大小腿

u8 arm_fore_left_right_flag=0;      //左右小臂
u8 leg_fore_left_right_flag=0;      //左右小腿

u8 arm_post_left_right_flag=0;      //左右大臂
u8 leg_post_left_right_flag=0;      //左右大腿

u8 arm_fore_post_left_right_flag=0; //左右大小臂
u8 leg_fore_post_left_right_flag=0; //左右大小腿

/****************发送图片******************************/
u8 back_picture_k=0;                //支背
u8 leg_up_picture_k=0;              //上曲腿
u8 leg_down_picture_k=0;            //下曲腿
u8 desk_picture_k=0;                //小桌子
u8 washlet_picture_k=0;             //坐便器
u8 body_left_picture_k=0;           //左翻身
u8 left_motor5_picture_m=0;         //左小侧翻
u8 body_right_picture_k=0;          //右翻身
u8 right_motor5_picture_m=0;        //右小侧翻

u8 back_nursing_left_picture_k=0;   //左背部护理
u8 waist_nursing_left_picture_k=0;  //左腰部护理
u8 back_nursing_right_picture_k=0;  //右背部护理
u8 waist_nursing_right_picture_k=0; //右腰部护理

/****************错误状态标志位******************************/

//驱动器返回-电机过载
u8 body_left_overload_3=0;          //左翻身
u8 body_left_overload_4=0;          
u8 body_left_overload_5=0;         
u8 body_right_overload_3=0;         //右翻身
u8 body_right_overload_4=0;         
u8 body_right_overload_5=0;        
u8 washlet_auto_overload=0;         //自动坐便
u8 desk_overload=0;                 //就餐娱乐一体桌
u8 back_nursing_left_overload=0;    //左背部护理
u8 back_nursing_right_overload=0;   //右背部护理
u8 waist_nursing_left_overload=0;   //左腰部护理
u8 waist_nursing_right_overload=0;  //右腰部护理

//驱动器返回-电机失步
u8 body_left_losepulse=0;            //左翻身         
u8 body_right_losepulse=0;           //右翻身         
u8 washlet_auto_losepulse=0;         //自动坐便
u8 desk_losepulse=0;                 //就餐娱乐一体桌
u8 back_nursing_left_losepulse=0;    //左背部护理
u8 back_nursing_right_losepulse=0;   //右背部护理
u8 waist_nursing_left_losepulse=0;   //左腰部护理
u8 waist_nursing_right_losepulse=0;  //右腰部护理

//程序运行出现动作干涉标志位
u8 back_interfere=0;                 //支背
u8 leg_up_interfere=0;               //上曲腿
u8 leg_down_interfere=0;             //下曲腿    
u8 leg_interfere=0;                  //曲腿
u8 body_left_interfere=0;            //左翻身
u8 body_right_interfere=0;           //右翻身
u8 washlet_auto_interfere=0;         //坐便器
u8 desk_interfere=0;                 //小桌子
u8 back_nursing_left_interfere=0;    //左背部护理
u8 back_nursing_right_interfere=0;   //右背部护理
u8 waist_nursing_left_interfere=0;   //左腰部护理
u8 waist_nursing_right_interfere=0;  //右腰部护理

/****************方向标志位******************************/

u8 back_dir_flag=0;                    //支背方向标志位
u8 leg_up_dir_flag=0;                  //上曲腿方向标志位
u8 leg_down_dir_flag=0;                //下曲腿方向标志位
u8 body_left_dir_flag=0;               //左翻方向标志位
u8 body_right_dir_flag=0;              //右翻方向标志位
u8 back_nursing_left_dir_flag=0;       //左背部护理方向标志位
u8 back_nursing_right_dir_flag=0;      //右背部护理方向标志位
u8 waist_nursing_left_dir_flag=0;      //左腰部护理方向标志位
u8 waist_nursing_right_dir_flag=0;     //右腰部护理方向标志位
u8 washlet_dir_flag=0;                 //坐便器方向标志位
u8 washlet_auto_dir_flag=0;            //自动坐便器方向标志位
u8 desk_dir_flag=0;                    //小桌子方向标志
u8 muscle_massager_flag=0;             //肌肉按摩方向标志
u8 lock_dir_flag=0;                    //键锁标志

u8 leg_down_state_flag=0;              //自动坐便时记录曲腿是否已处于动作状态
u8 back_state_flag=0;                  //自动坐便时记录支背是否已处于动作状态

unsigned int temp_flag_1=1;            //温度标志
unsigned int temp_flag_2=1;            //温度标志

/*************电机运行自动重装载值，控制电机运行速度**********************/

unsigned int motor_back_freq=1000-1;     //支背1000
unsigned int motor_body_freq=1400-1;     //翻身1250
unsigned int motor_washlet_freq=1400-1;  //坐便 700
unsigned int motor_desk_freq=700-1;      //小桌子700
unsigned int motor_hang_freq=70-1;      //吊挂 800

/****************************定时器分频值********************************/

unsigned int motor_timer_freq=25-1;      //定时器分频值psc，控制电机运行速度
u16 timer10_freq=65000;                  //定时器分频值psc，控制电机运行速度
unsigned int timer10_freq_1=45000-1;     //定时器3分频值psc
unsigned int timer10_arr_1s=2000-1;      //定时器3分预装载值arr:psc*arr/90M=1s

u16 bodyleft_compleate=1400;
u16 bodyright_compleate=2200;

/***********************************************************************
 函数名      ：Fun_Back(void)   
 函数功能    ：执行支背操作-电动推杆
 输入        ：无
 输出        ：无                           
************************************************************************/
void Fun_Back(void)
{
	u8 direct,len;
	u16 arr_now;               //当前一次运行脉冲数
	
	//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;           //判断程序是否是从break跳出来 
	u16 repeat_num;
	static u8 dir_fore;        //记录上一次的运动方向
	static u8 dir_change;      //判断运动方向是否发生变化
	static u16 k;              //传第k张动画
	static u8 kj;
	u8 i=0;
	u16 j=0;	
	u16 arr_send;              //当前一次运行脉冲值
	static u8 back_limit_flag; //支背运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位后，才能进行支背
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackUpPhone"))
		{			
			if(back_angle_to_arr(back_angle_lim)>back_runed_arr)  //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				direct=1;
				back_dir_flag=1;      
				if(back_flag==0)
				{
					back_flag=1;
					delay_ms(200);
					u2_printf("back_flag==1");
					delay_ms(200);
					u2_printf("BackStart");
					delay_ms(200);
					Wifi_Send("RunStart");
					delay_ms(200);
					u2_printf("Cartoon_Back_1");
					delay_ms(200);
				}
				Motor_1_START(1);                                                          //支背上行
				TIM10_Init(back_angle_to_arr(back_angle_lim)-back_runed_arr,timer10_freq); //打开定时器
			}				
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackDownPhone"))
		{
			if(back_runed_arr>0)    //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				direct=0;
				back_dir_flag=0; 
				if(1==back_limit_flag)
				{
					back_limit_flag=0;					
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
				}
				Motor_1_START(0);                           //支背下行
				TIM10_Init(back_runed_arr,timer10_freq);    //打开定时器
			}
		}	
		
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;									
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
		
	 	if(((back_runed_arr!=back_angle_to_arr(back_angle_lim))&&(1==direct))||((0!=back_runed_arr)&&(0==direct)))
		{					
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环   
			{							
				for(repeat_num=0;repeat_num<700;repeat_num++)
				{			
					//停止指令
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"BackUpPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"BackDownPhone")))    //若接收到Stop,则跳出循环	
						{					
							break_flag=1;
							memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
							UART4_RX_LEN=0;	
							break;						
						}
						else if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))
						{	}
						else 
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数							
							memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
							UART4_RX_LEN=0;
						}
					 }
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);      //当前一次脉冲值
				//传输动画指令
				if(direct==1)
				{
					j=(back_runed_arr+arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				else
				{
					j=abs(back_runed_arr,arr_send)/(back_angle_to_arr(back_angle_max)/19);
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
			break_flag=0;      //标志位清零
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //判断是否处于复位状态，复位状态的前提是下行的定时器走完
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
			if(	direct==1)        //如果是支背上行，则用+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{  
					back_runed_arr=back_angle_to_arr(back_angle_lim);
					back_limit_flag=1; 
					delay_ms(200);
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("BackLim");
				}
				else
				{  
					back_runed_arr+=arr_now;	
				}				
			}
			else                //如果是支背下行，则用-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{	
					back_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Back_1");
					delay_ms(200);
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("BackRes");
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
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("BackInterfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Fun_Leg_Up(void)   
 函数功能    ：执行上曲腿操作
 输入        ：无
 输出        ：无                           
************************************************************************/
void Fun_Leg_Up(void)
{
	u16 arr_now;              //当前一次运行脉冲数
	u8 len;                   //WiFi串口接收字符串长度
	u8 direct;	              //运行方向标志位
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //判断程序是否通过break跳出 
	static u8 dir_fore;       //记录上一次的运动方向
	static u8 dir_change=0;   //判断运动方向是否发生变化
	static u16 k=0;           //传输第k张动画指令
	static u8 kj;
	u8 i=0;
	u16 j=0;	
	u16 arr_send;               //当前一次运行脉冲值
	static u8 leg_up_limit_flag;//上曲腿运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位及下曲腿复位后才能进行上曲腿
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_down_flag==0)&&(lock_flag==1))
	{		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpUpPhone"))  //上曲腿上行	
		{				   
			if(leg_angle_to_arr(leg_up_angle_lim)>leg_up_runed_arr)            //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				direct=1; 
				if(leg_up_flag==0)
				{
					leg_up_flag=1;
					delay_ms(200);					
					u2_printf("leg_up_flag==1");					
					delay_ms(200);
					u2_printf("LegUpStart");
					delay_ms(200);
					Wifi_Send("RunStart");
					delay_ms(200);
					u2_printf("Cartoon_Leg_Up_1");
					delay_ms(200);
				}											
				Push_Rod_Start(0);
				TIM10_Init(leg_angle_to_arr(leg_up_angle_lim)-leg_up_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz					
			} 
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpDownPhone"))  //上曲腿下行
		{			
			if(leg_up_runed_arr>0)                          //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				direct=0;
				if(leg_up_limit_flag==1)
				{
					leg_up_limit_flag=0;
					u2_printf("Cartoon_Leg_Up_8");
					delay_ms(200);
				}										
				Push_Rod_Start(1);
				TIM10_Init(leg_up_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
			}
		}		
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;		
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //清除中断标志位
	 	if(((leg_up_runed_arr!=leg_angle_to_arr(leg_up_angle_lim))&&(1==direct))||((0!=leg_up_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)	
				{	
					//停止指令
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpUpPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpDownPhone")))    //若接收到Stop,则跳出循环	
						{
							break_flag=1;	
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
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//判断传输动画指令
				if(direct==1)
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
			break_flag=0;       //标志位清零
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0)) //判断是否处于复位状态，复位状态的前提是下行的定时器走完
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
			if(direct==1)    //如果是上曲腿上行，则用+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_up_runed_arr=leg_angle_to_arr(leg_up_angle_lim);
					leg_up_limit_flag=1;	
					delay_ms(200);
					u2_printf("Cartoon_Leg_Up_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("LegUpLim");
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
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("LegUpRes");
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
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("LegUpInterfere");		
		LED0=1;
		LED1=1;	
	}
}

/***********************************************************************
 函数名      ：Phone_Leg_Down(void)   
 函数功能    ：执行下曲腿操作
 输入        ：无
 输出        ：无                           
************************************************************************/
void Fun_Leg_Down(void)
{
	u16 arr_now;         //当前一次运行脉冲数   
	u8 len;              //接收的字符串长度
	u8 direct;           //代表某个动作运行的方向标志：1-正向运行；0-反向运行
	
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //判断是否通过break跳出循环 
	static u8 dir_fore;       //记录上一次的运动方向
	static u8 dir_change=0;   //判断运动方向是否发生变化
	static u16 k=0;           //发送第K张动画指令
	static u16 kj=0;
	u8 i=0;
	u16 j=0;	
	u16 arr_send;             //当前一次运行脉冲数
	static u8 leg_down_limit_flag;//下曲腿运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位及下曲腿复位后才能进行上曲腿
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(lock_flag==1))
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownDownPhone"))   //下曲腿下行
		{				
			if(leg_angle_to_arr(leg_down_angle_lim)>leg_down_runed_arr)             //下行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				direct=1;
				if(leg_down_flag==0)
				{
					leg_down_flag=1;
					delay_ms(200);
					u2_printf("leg_down_flag==1");					
					delay_ms(200);
					u2_printf("LegDownStart");
					delay_ms(200);
					Wifi_Send("RunStart");
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_1");
					delay_ms(200);
				}			
				Push_Rod_Start(1);
				TIM10_Init(leg_angle_to_arr(leg_down_angle_lim)-leg_down_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
				leg_down_flag=1;
			}	
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownUpPhone"))  //下曲腿上行
		{
			if(leg_down_runed_arr>0)     //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				direct=0;
				if(leg_down_limit_flag==1)
				{
					leg_down_limit_flag=0;
					u2_printf("Cartoon_Leg_Down_20");
					delay_ms(200);
				}			
				Push_Rod_Start(0);
				TIM10_Init(leg_down_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
			}
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //清除中断标志位	

	   if(((leg_down_runed_arr!=leg_angle_to_arr(leg_down_angle_lim))&&(1==direct))||((0!=leg_down_runed_arr)&&(0==direct)))
	   {			
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{						
					//停止指令
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownDownPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownUpPhone")))    //若接收到Stop,则跳出循环	
						{				
							break_flag=1;
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
				 }
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//发送动画指令
				if(direct==1)
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
						k=j;    leg_down_picture_k=k;
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
			break_flag=0;	   //清除break标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
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
			if(direct==1)    //如果是下曲腿下行，则用+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim);
					leg_down_limit_flag=1;	
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_20");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("LegDownLim");
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
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("LegDownRes");
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
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("LegDownInterfere");		
		LED0=1;
		LED1=1;	
	}
}

/***********************************************************************
 函数名      ：Fun_Body_Left(void)  
 函数功能    ：执行左翻身操作
 输入        ：无
 输出        ：无                           
************************************************************************/
void Fun_Body_Left(void)
{
	u8 direct,key;
	u16 arr_now;                 //当前一次运行脉冲数,用于脉冲累计
	u8 len;                      //接收的字符串长度
	u16 arr_feed;                //计算程序中当前一次运行脉冲数，用于判断电机失步故障
	u16 pulse_num=0;             //电机理论接收到的脉冲值
	u16 num1=0,num2=0,num3=0;    //电机实际运行的脉冲值	
	static u8 motor5_run_flag;   //判断小侧翻是否已经动作，若动作该位置1 
	
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //判断程序是否从break跳出
	static u16 k=0,m=0;
	u8 i=0,i1=0;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;   //当前一次运行脉冲数
	static u8 kj;
	static u8 M345_Start;     //345电机第一次运行
	static u8 M345_End;       //345电机运行到上极限位置
	static u8 mn;
		
	//联锁功能，只有在支背、上下曲腿、座便、桌子复位后，才能执行左翻身功能
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{			
		//小侧翻起来
		if(body_left_flag==0)   //如果复位到初始状态，才执行左翻起
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftUpPhone"))
			{
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				//5号侧翻起			
				DIR5=1; 
				body_left_flag=1;
				motor5_run_flag=1;
				W25QXX_Write((u8*)&body_left_flag,33,1); 
				u2_printf("body_left_flag==1");				
				delay_ms(200);
				u2_printf("BodyLeftStart");
				delay_ms(200);
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);	 //电机启动	
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);    //打开定时器20000
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);                 //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{	
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{					
						//光电限位
//						if((0==GD5_Left_End)&&(1==body_left_flag))        //碰到光电开关跳出循环，电机停转 
//						{						
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;	
//							}
//						}
						  //判断有没有收到上位机指令		
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
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送动画指令
				    arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					m=left_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);						
						if(mn<2)
						{
							m=n;   left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;												
							}
						}
					}
				}
				Motor_5_STOP();            //电机5停止
				TIM10_Stop();              //定时器关闭
				break_flag=0;              //清除标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);				
				Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);	//调用补偿函数
			}
		}	
		//翻身345号电机动作	
		if((strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftUpPhone"))||(1==motor5_run_flag))
		{				
			if(body_angle_to_arr(body_left_angle_lim)>body_left_runed_arr)
			{
			     motor5_run_flag=0;
				 
				//345联动左翻起			
				 DIR3=0;DIR4=0;DIR5=0;direct=1;body_left_dir_flag=1;
				 if(M345_Start==0)
				 {
					 delay_ms(200);
					 M345_Start=1;
					 delay_ms(200);
					 u2_printf("Cartoon_Body_Left_1");
					 delay_ms(200);
				 }				 
				 Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);				
				 TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);			
			}
		}		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftDownPhone"))
		{					
			if(body_left_runed_arr>0)
			{			   
				//345联动左翻起	
			     DIR3=1;DIR4=1;DIR5=1;direct=0;body_left_dir_flag=0;
				 if(M345_End==1)
				 {
					 M345_End=0;
					 delay_ms(200);
					 u2_printf("Cartoon_Body_Left_8");	
					 delay_ms(200);
				 }					 
			     Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	//电机启动			
			     TIM10_Init(body_left_runed_arr,timer10_freq);			//关闭定时器
			}
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //清除中断标志位
		 if(((body_left_runed_arr!=body_angle_to_arr(body_left_angle_lim))&&(1==direct))||((0!=body_left_runed_arr)&&(0==direct)))
		 {
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{
					//光电限位
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //如果碰到光电开关，跳出循环，停止运行
//					{					
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");		
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");		
//						}
//						break_flag=1;
//						break;
//					}					
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))    //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if(0==GD3_Left_End)
//						{
//							u2_printf("GD3LeftEnd");
//						}
//						if(0==GD4_Left_End)
//						{
//							u2_printf("GD4LeftEnd");
//						}
//						break_flag=1;					
//						break;
//					}
					//停止指令
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftUpPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftDownPhone")))//若接收到Stop,则跳出循环	
						{
							break_flag=1;	
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
					//电机故障、故障诊断
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))       
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_left_overload_3=1;
//							u2_printf("BodyLeftOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_left_overload_4=1;
//							u2_printf("BodyLeftOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_left_overload_5=1;
//							u2_printf("BodyLeftOverload5");
//						}					
//						Breakdown_Treatment();
//						break_flag=1;				
//						break;												
//					}						
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_left_runed_arr+arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				else
				{
					j=abs(body_left_runed_arr,arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}	
				k=body_left_picture_k;
				if(k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{	
						k=j;  body_left_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Left_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Left_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Left_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Left_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Left_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Left_7");
									break;
						}
					}				
				}				
			}							
			Motor_3_4_5_STOP();    //电机停止
			TIM10_Stop();          //关闭定时器
			break_flag=0;		   //清除标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;
				body_left_flag=0;
				W25QXX_Write((u8*)&body_left_flag,33,1);				
			}					
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now
				body_left_flag=1;									
			}
			//通过上下行判断脉冲累计
			if(direct==1)     //上行，脉冲+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Left_End)||(0==GD4_Left_End))
				{					
					body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
					M345_End=1; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("BodyLeftLim");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr+arr_now;
				}		
			}	
			else     //下行，脉冲-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))
				{										
					body_left_runed_arr=0;
					M345_Start=0;   
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_1");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr-arr_now;
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
		 //若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_left_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //光电3没到位
//				{
//					DIR3=1;
//					Motor_3_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD3Start");
//							break;																		
//						}
//					}			
//					Motor_3_STOP();
//					TIM10_Stop();         //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //光电4没到位
//				{
//					DIR4=1;
//					Motor_4_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD4Start");
//							break;																		
//						}
//					}			
//					Motor_4_STOP();       //电机停止
//					TIM10_Stop();         //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //光电3/4都没到位
//				{
//					DIR3=1; DIR4=1; DIR5=1;
//					Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //运行时碰到光电开关，跳出循环 
//						{
//							body_left_flag=0;
//							W25QXX_Write((u8*)&body_left_flag,33,1);
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//							}
//							break;																		
//						}
//					}			
//					Motor_3_4_5_STOP();   //电机停止
//					TIM10_Stop();         //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
//			}
			//5号电机复位
			if(body_left_flag==0)     //345联动复位到初始状态，才复位5号电机
			{			
				//5号侧翻复位
				Motor_4_Compensate(1,bodyleft_compleate,motor_body_freq,motor_timer_freq);
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;		
				DIR5=0;
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);  //5号电机启动
				body_left_runed_arr=0;			
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);                     //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{				
						//光电限位
//						if(((0==GD5_Start)&&(0==body_left_flag)))   //碰到光电开关跳出循环，电机停转 
//						{												
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;	
//							}								
//						}
						  //判断有没有收到上位机指令		
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
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送动画指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);				
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					n=9-n;
					m=left_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;  left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}
						}
					}
				}      
				Motor_5_STOP();       //电机停止
				TIM10_Stop();         //关闭定时器
				break_flag=0;         //清除标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位					
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				u2_printf("body_left_flag==0");				
				delay_ms(200);
				Wifi_Send("RunRes");
				delay_ms(200);
				u2_printf("BodyLeftRes");
				//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=0;
//					Motor_5_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD5Start");
//							break;																		
//						}
//					}			
//					Motor_5_STOP();
//					TIM10_Stop();
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
			}				
		}
	}
	else
	{
		LED0=0;   //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("BodyLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Fun_Body_Right(void)  
 函数功能    ：执行右翻身操作
 输入        ：无
 输出        ：无                           
************************************************************************/
void Fun_Body_Right(void)
{
	u8 direct,key;
	u16 arr_now;        //当前一次运行脉冲数，用于脉冲累计
	u8 len;             //接收的字符串长度
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num1=0,num2=0,num3=0;
	
	static u8 motor5_run_flag;  //小侧翻已运行标志位
	
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //判断程序是否从break跳出 
	static u16 k=0,m=0;
	static u8 M345R_Start=0;  //345电机从初始位置运行
	static u8 M345R_End=0;    //345电机到达上极限位置
	u8 mn;
	u8 kj;

	u8 i=0,i1=0;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;	  //当前一次运行脉冲数
	
	//联锁功能，只有在支背、上下曲腿、座便、桌子复位后，才能执行右左翻身功能
	if((lock_flag==1)&&(back_flag==0)&&(body_left_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_right_flag==0)&&(waist_nursing_right_flag==0))
	{
		if(body_right_flag==0)   //如果复位到初始状态，才执行右翻起
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightUpPhone"))
			{
			 //5号侧翻起
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				DIR5=0; 
				body_right_flag=1;
				motor5_run_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);
				delay_ms(200);
				u2_printf("body_right_flag==1");
				delay_ms(200);
				u2_printf("BodyRightStart");
				delay_ms(200);
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);	//电机启动	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);   //打开定时器  
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{	
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{
						//光电限位
//						if((0==GD5_Right_End)&&(1==body_right_flag))                     //碰到光电开关跳出循环，电机停转 
//						{		
//							delay_us(100);
//							if(0==GD5_Right_End)
//							{
//								break_flag=1;
//								u2_printf("GD5RightEnd"); 
//								break;		
//							}								
//						}
						  //判断有没有收到上位机指令		
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
						//电机故障、故障诊断
//						if(1==Motor5_Alm)        
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("BodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;	
//							}								
//						}											
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送图片指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					m=right_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch(m)
							{								
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;
							}
						}
					}
				}					      
				Motor_5_STOP();       //电机停止
				TIM10_Stop();         //关闭定时器
				break_flag=0;         //清除break标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(1,bodyright_compleate,motor_body_freq,motor_timer_freq);//调用补偿函数
			}			
		}	
		//翻身345号电机动作	
		if((strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightUpPhone"))||(1==motor5_run_flag))  //右翻身上
		{		
			if(body_angle_to_arr(body_right_angle_lim)>body_right_runed_arr)
			{
				motor5_run_flag=0;
				//345联动左翻起
				DIR3=1;DIR4=1;DIR5=1;direct=1;body_right_dir_flag=1;
				if(M345R_Start==0)
				{
					M345R_Start=1;
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");	
					delay_ms(200);
				}			
				Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //电机启动	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim)-body_right_runed_arr,timer10_freq);	//定时器打开			
			}
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightDownPhone"))  //右翻身下
		{			
			if(body_right_runed_arr>0)
			{
				//345联动左翻起
			   DIR3=0;DIR4=0;DIR5=0;direct=0;body_right_dir_flag=0;
			   if(M345R_End==1)
			   {
					M345R_End=0; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
			   }			
													
			   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //电机启动		
			   TIM10_Init(body_right_runed_arr,timer10_freq);		  //打开定时器
			}	
		}				
		  memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		  UART4_RX_LEN=0;
			
		  __HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位	 	

		 if(((body_right_runed_arr!=body_angle_to_arr(body_right_angle_lim))&&(1==direct))||((0!=body_right_runed_arr)&&(0==direct)))
		 {
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //等待定时时间到，时间到跳出循环
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//光电限位
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))                    //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");
//							break_flag=1;					
//							break;
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");
//							break_flag=1;					
//							break;
//						}
//					}
//					if(((0==GD3_Right_End)||(0==GD4_Right_End))&&(1==direct))           //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if(0==GD3_Right_End)
//						{
//							u2_printf("GD3RightEnd");
//							break_flag=1;					
//							break;
//						}
//						if(0==GD4_Right_End)
//						{
//							u2_printf("GD4RightEnd");
//							break_flag=1;					
//							break;
//						}
//					}
					//停止指令
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightUpPhone"))|| 
								(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightDownPhone")))  //若接收到Stop,则跳出循环	
						{
							break_flag=1;
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
					//电机故障、故障诊断
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//					{											
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_right_overload_3=1;
//							u2_printf("BodyRightOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_right_overload_4=1;
//							u2_printf("BodyRightOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_right_overload_5=1;
//							u2_printf("BodyRightOverload5");
//						}
//						Breakdown_Treatment();	
//						break_flag=1;				
//						break;
//					}										
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_right_runed_arr+arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				else
				{
					j=abs(body_right_runed_arr,arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				k=body_right_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;  body_right_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_7");
									break;		
						}
					}				
				}				
			}				
				 
			Motor_3_4_5_STOP();    //电机停止
			TIM10_Stop();          //定时器关闭
			break_flag=0;		   //清除标志位
			//判断复位	
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;
				body_right_flag=0;					
				W25QXX_Write((u8*)&body_right_flag,34,1);				
			}					
			else
			{							
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				body_right_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);					
			}
			//通过上下行判断脉冲累计
			if(direct==1)    //翻身上行，则用+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Right_End)||(0==GD4_Right_End))
				{					
					body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
					M345R_End=1; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("BodyRightLim");
				}
				else
				{				
					body_right_runed_arr=body_right_runed_arr+arr_now;
				}
			}
			else		//翻身下行，则用-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))
				{					
					body_right_runed_arr=0;	
					M345R_Start=0; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
					delay_ms(200);
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr-arr_now;
				}			
			}			
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
			//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_right_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start)&&(0==direct))   //光电3没到位
//				{
//					DIR3=0;
//					Motor_3_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //运行时碰到光电开关，跳出循环 
//						{	
//							u2_printf("GD3Start");			
//							break;																		
//						}
//					}			
//					Motor_3_STOP();   //电机停止
//					TIM10_Stop();     //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//				}
//				if((0==GD3_Start)&&(1==GD4_Start)&&(0==direct))         //光电4没到位
//				{
//					DIR4=0;
//					Motor_4_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD4Start");
//							break;																		
//						}
//					}			
//					Motor_4_STOP();    //电机停止
//					TIM10_Stop();      //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start)&&(0==direct))   //光电3/4都没到位
//				{
//					DIR3=0; DIR4=0; DIR5=0;
//					Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //运行时碰到光电开关，跳出循环 
//						{
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//							}
//							break;																		
//						}
//					}			
//					Motor_3_4_5_STOP();   //电机停止
//					TIM10_Stop();         //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
//			}
			 //侧翻复位
			if(0==body_right_flag)      //只有翻身复位到初始状态，小侧翻才复位
			{			
				//5号侧翻复位
				Motor_4_Compensate(0,bodyright_compleate,motor_body_freq,motor_timer_freq);//调用补偿函数
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;		
				DIR5=1;
				delay_ms(200);			
				u2_printf("Cartoon_Body_Right_Motor5_10");	
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);    //电机启动
				body_right_runed_arr=0;
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);                     //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{
						//光电限位
//						if(((0==GD5_Start)&&(0==body_right_flag)) )           //碰到光电开关跳出循环，电机停转 
//						{						
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break_flag=1;
//								break;
//							}
//						}
						  //判断有没有收到上位机指令		
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
						//电机故障、故障诊断
//						if(1==Motor5_Alm)       
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("BodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;
//							}
//						}				
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送动画指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					n=9-n;	
					m=right_motor5_picture_m;
					if(	m!=n)
					{
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}
						}
					}
				}	     
				Motor_5_STOP();      //电机停止
				TIM10_Stop();        //定时器关闭
				break_flag=0;    
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位		   
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);			
				u2_printf("body_right_flag==0");
				delay_ms(200);
				Wifi_Send("RunRes");
				delay_ms(200);
				u2_printf("BodyRightRes");
				//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_right_flag))
//				{
//					DIR5=1;
//					Motor_5_START(motor_body_freq,motor_timer_freq);    //电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD5Start");
//							break;																		
//						}
//					}			
//					Motor_5_STOP();    //电机停止
//					TIM10_Stop();      //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}			
			 }	
		}
	}
	else
	{
		LED0=0;   //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("BodyRightInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Fun_Desk(void)  
 函数功能    ：小桌子
 输入        ：无
 输出        ：无                           
************************************************************************/
void Fun_Desk(void)
{
	u8 direct,key;    //表示电机运行方向，1：小桌子前进；0：小桌子后退
	u16 arr_now;      //本次运行脉冲值
	u8 len;           //表示接收的字符串的长度
	u16 arr_feed;     //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;  //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //判断程序是否从break跳出 
	static u16 k=0;           //发送第k张图片
	static u16 kj=0;
	u8 i=0;
	u16 j=0;	
	u16 arr_send;                 //当前一次运行脉冲数
	static u8 desk_limit_flag;    //判断小桌子是否运行到极限位置，若是发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位后，才能进行小桌子移动
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskUpPhone"))
		{
			DIR7=0;
			direct=1;
			if(desk_flag==0)
			{
				desk_flag=1;
				u2_printf("desk_flag==1");
				delay_ms(200);
				u2_printf("DeskStart");
				delay_ms(200);
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("Cartoon_Desk_1");
				delay_ms(200);
			}
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskDownPhone"))
		{
			DIR7=1;
			direct=0;
			if(1==desk_limit_flag)
			{
				desk_limit_flag=0;
				u2_printf("Cartoon_Desk_20");
				delay_ms(200);
			}
		}		
		if(direct==1)   //如果是小桌子向前
		{		
			if(desk_distance_to_arr(desk_distance_lim)>desk_runed_arr)  //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_distance_to_arr(desk_distance_lim)-desk_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
			}				
		}
		else       //如果是小桌子后退
		{
			if(desk_runed_arr>0)    //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);//启动电机
				TIM10_Init(desk_runed_arr,timer10_freq);        //打开定时器			
			}
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
	    if(((desk_runed_arr!=desk_distance_to_arr(desk_distance_lim))&&(1==direct))||((0!=desk_runed_arr)&&(0==direct)))
		 {	 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环  
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//光电限位											   						
//					if((0==GD7_End)&&(1==direct))
//					{
//						delay_us(100);
//						if(0==GD7_End)
//						{
//							u2_printf("GD7End");
//							break_flag=1;				
//							break;	
//						}
//					}
//					if((0==GD7_Start)&&(0==direct))
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							u2_printf("GD7Start");
//							break_flag=1;					
//							break;
//						}							
//					}
					//停止指令
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"DeskUpPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"DeskDownPhone")))    //若接收到Stop,则跳出循环	
						{
							break_flag=1;
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
					//电机故障、故障诊断
//					if(1==Motor7_Alm)        
//					{						
//						delay_us(100);
//						if(1==Motor7_Alm)
//						{
//							desk_overload=1;
//							u2_printf("DeskOverload");
//							Breakdown_Treatment();
//							break_flag=1;					
//							break;
//						}							
//					}								
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(desk_runed_arr+arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				else
				{
					j=abs(desk_runed_arr,arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				k=desk_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;   desk_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Desk_2");									
									break;
							case 2:	u2_printf("Cartoon_Desk_3");
									break;					
							case 3:	u2_printf("Cartoon_Desk_4");
									break;					
							case 4:	u2_printf("Cartoon_Desk_5");
									break;	
							case 5:	u2_printf("Cartoon_Desk_6");
									break;	
							case 6:	u2_printf("Cartoon_Desk_7");
									break;
							case 7:	u2_printf("Cartoon_Desk_8");
									break;
							case 8:	u2_printf("Cartoon_Desk_9");
									break;						
							case 9:	u2_printf("Cartoon_Desk_10");
									break;												
							case 10:u2_printf("Cartoon_Desk_11");
									break;	
							case 11:u2_printf("Cartoon_Desk_12");									
									break;
							case 12:u2_printf("Cartoon_Desk_13");
									break;					
							case 13:u2_printf("Cartoon_Desk_14");
									break;					
							case 14:u2_printf("Cartoon_Desk_15");
									break;	
							case 15:u2_printf("Cartoon_Desk_16");
									break;	
							case 16:u2_printf("Cartoon_Desk_17");
									break;
							case 17:u2_printf("Cartoon_Desk_18");
									break;
							case 18:u2_printf("Cartoon_Desk_19");
									break;						
						}
					}					
				}				
			}				    
			Motor_7_STOP();     //电机停止
			TIM10_Stop();       //定时器关闭
			break_flag=0;		//清除break标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //判断是否处于复位状态，复位状态的前提是下行的定时器走完
			{
				arr_now=0;         //此时处于复位状态，将状态值都设为0；
				desk_flag=0;
				delay_ms(200);
				u2_printf("desk_flag==0");			
			}			
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now				
				desk_flag=1;
			}	
			//通过上下行判断脉冲累计
			if(	direct==1)        //如果是小桌子前进，则用+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_End))
				{ 
					desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
					desk_limit_flag=1;	
					delay_ms(200); 
					u2_printf("Cartoon_Desk_20");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("DeskLim");
				}
				else
				{  
					desk_runed_arr=desk_runed_arr+arr_now;	
				}				
			}
			else                //如果是小桌子后退，则用-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_Start))
				{	
					desk_runed_arr=0; 
					delay_ms(200);
					u2_printf("Cartoon_Desk_1");
					delay_ms(200);
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("DeskRes");
				}
				else
				{	
					desk_runed_arr=desk_runed_arr-arr_now;
				}						
			}				
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	   			
			//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段,并删除清除中断语句）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD7_Start==1)&&(direct==0)&&(desk_flag==1))
//			{   
//				u2_printf("附加脉冲运行");
//				DIR7=1;
//				Motor_7_START(motor_desk_freq,motor_timer_freq);
//			    TIM10_Init(add_arr,timer10_freq);   //打开定时器
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//				{			
//					if(0==GD7_Start)  //运行时碰到光电开关，跳出循环 
//					{				
//						u2_printf("GD7Start");
//						break;																		
//					}
//				}			
//				Motor_7_STOP();   //电机停止
//				TIM10_Stop();     //关闭定时器
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//			}			
		}
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("DeskInterfere");
		LED0=1;
		LED1=1;	

	}		
}

/***********************************************************************
 函数名      ：Fun_Back_Nursing_Left(void)  
 函数功能    ：左背部护理
 输入        ：无
 输出        ：无 

************************************************************************/
void Fun_Back_Nursing_Left(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;  
	u8 break_flag=0;    //判断程序是否从break跳出
	static u16 k=0;     //发送第k张动画指令
	u8 i=0;
	u16 j=0;	
	u16 arr_send;	    //当前一次运行脉冲数
	static u8 kj;
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
	
	//联锁功能，只有在已执行左翻身功能且左腰部护理复位后，才能进行左背部护理
	if((lock_flag==1)&&(body_left_flag==1)&&(waist_nursing_left_flag==0))
	{
		back_nursing_left_flag=!back_nursing_left_flag;	
		back_nursing_left_dir_flag=!back_nursing_left_dir_flag;			
		if(back_nursing_left_dir_flag==1)
		{ 
			DIR3=1; direct=1;
			delay_ms(200);
			u2_printf("back_nursing_left_flag==1");			
			delay_ms(200);
			u2_printf("BackNursingLeftStart");
			delay_ms(200);
			Wifi_Send("RunStart");
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_1");
			delay_ms(200);
		}
		else
		{ 
			DIR3=0; direct=0;
			u2_printf("Cartoon_Back_Nursing_Left_20");
			delay_ms(200);
		}

		Motor_3_START(motor_body_freq,motor_timer_freq);	       //电机启动
		TIM10_Init(body_left_runed_arr,timer10_freq);              //打开定时器body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环   
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//光电限位
//				if((0==GD3_Left_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD3_Left_End)
//					{
//						u2_printf("GD3LeftEnd");
//						break_flag=1;
//						break;	
//					}
//				}
//				if((0==GD3_Start)&&(0==direct))     //落下
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						u2_printf("GD3Start");
//						break_flag=1;					
//						break;	
//					}						
//				}
				  //判断有没有收到上位机指令		
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
				//电机故障、故障诊断
//				if(1==Motor3_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor3_Alm)
//					{
//						back_nursing_left_overload=1;
//						u2_printf("BackNursingLeftOverload");
//						Breakdown_Treatment();
//						break_flag=1;					
//						break;
//					}						
//				}			
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//发送动画指令
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_left_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=back_nursing_left_picture_k;
			if(k!=j)
			{
				kj=abs(k,j);
				if(kj<2)
				{
					k=j;  back_nursing_left_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Back_Nursing_Left_2");									
								break;
						case 2:	u2_printf("Cartoon_Back_Nursing_Left_3");
								break;					
						case 3:	u2_printf("Cartoon_Back_Nursing_Left_4");
								break;					
						case 4:	u2_printf("Cartoon_Back_Nursing_Left_5");
								break;	
						case 5:	u2_printf("Cartoon_Back_Nursing_Left_6");
								break;	
						case 6:	u2_printf("Cartoon_Back_Nursing_Left_7");
								break;
						case 7:	u2_printf("Cartoon_Back_Nursing_Left_8");									
								break;
						case 8:	u2_printf("Cartoon_Back_Nursing_Left_9");
								break;					
						case 9:	u2_printf("Cartoon_Back_Nursing_Left_10");
								break;					
						case 10:u2_printf("Cartoon_Back_Nursing_Left_11");
								break;	
						case 11:u2_printf("Cartoon_Back_Nursing_Left_12");
								break;	
						case 12:u2_printf("Cartoon_Back_Nursing_Left_13");
								break;
						case 13:u2_printf("Cartoon_Back_Nursing_Left_14");
								break;	
						case 14:u2_printf("Cartoon_Back_Nursing_Left_15");
								break;	
						case 15:u2_printf("Cartoon_Back_Nursing_Left_16");
								break;
						case 16:u2_printf("Cartoon_Back_Nursing_Left_17");
								break;	
						case 17:u2_printf("Cartoon_Back_Nursing_Left_18");
								break;
						case 18:u2_printf("Cartoon_Back_Nursing_Left_19");
								break;						
					}
				}
			}
		}				  
		Motor_3_STOP();     //电机停止
		TIM10_Stop();       //关闭定时器    
		break_flag=0;       //清除break标志位
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		if(back_nursing_left_dir_flag==1)
		{
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_20");
			delay_ms(200);
			Wifi_Send("RunLim");
			delay_ms(200);
			u2_printf("BackNursingLeftLim");
		}
		else
		{			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_1");
			delay_ms(200);
			Wifi_Send("RunRes");
			delay_ms(200);
			u2_printf("BackNursingLeftRes");
			delay_ms(200);
			u2_printf("back_nursing_left_flag==0");
		}	
	}
	else
	{
		LED0=0;        //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("BackNursingLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Fun_Back_Nursing_Right(void)
 函数功能    ：右背部护理
 输入        ：无
 输出        ：无                           
************************************************************************/
void Fun_Back_Nursing_Right(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;  
	u8 break_flag=0;     //判断程序从break跳出
	static u16 k=0;      //发送第k张动画指令
	u8 i=0;
	u16 j=0;	
	u16 arr_send;	    //当前一次运行脉冲数
	static u8 kj;
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;		
	
	//联锁功能，只有在已执行右翻身功能且右腰部护理复位后，才能进行右背部护理
	if((lock_flag==1)&&(1==body_right_flag)&&(waist_nursing_right_flag==0))
	{	
		back_nursing_right_flag=!back_nursing_right_flag;
		back_nursing_right_dir_flag=!back_nursing_right_dir_flag;	
		if(back_nursing_right_dir_flag==1)
		{ 
			DIR3=0; direct=1;
			delay_ms(200);
			u2_printf("back_nursing_right_flag==1");
			delay_ms(200);
			u2_printf("BackNursingRightStart");
			delay_ms(200);
			Wifi_Send("RunStart");
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_1");
			delay_ms(200);
		}
		else
		{ 
			DIR3=1; direct=0;
			u2_printf("Cartoon_Back_Nursing_Right_20");
			delay_ms(200);
		}		
		Motor_3_START(motor_body_freq*1.4,motor_timer_freq);              //电机启动
		TIM10_Init(body_right_runed_arr,timer10_freq);                //打开定时器
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //等待定时时间到，时间到跳出循环 
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//光电控制
//				if((0==GD3_Right_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD3_Right_End)
//					{
//						u2_printf("GD3RightEnd");
//						break_flag=1;
//						break;
//					}						
//				}
//				if((0==GD3_Start)&&(0==direct))     //落下
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						u2_printf("GD3Start");
//						break_flag=1;
//						break;
//					}						
//				}
				  //判断有没有收到上位机指令		
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
				//电机故障、故障诊断
//				if(1==Motor3_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor3_Alm)
//					{
//						back_nursing_right_overload=1;
//						u2_printf("BackNursingRightOverload");
//						Breakdown_Treatment();
//						break_flag=1;
//						break;
//					}						
//				}						
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//发送动画指令
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_right_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=back_nursing_right_picture_k;
			if(	k!=j)
			{	
				kj=abs(k,j);				
				if(kj<2)
				{
					k=j;  back_nursing_right_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Back_Nursing_Right_2");									
								break;
						case 2:	u2_printf("Cartoon_Back_Nursing_Right_3");
								break;					
						case 3:	u2_printf("Cartoon_Back_Nursing_Right_4");
								break;					
						case 4:	u2_printf("Cartoon_Back_Nursing_Right_5");
								break;	
						case 5:	u2_printf("Cartoon_Back_Nursing_Right_6");
								break;	
						case 6:	u2_printf("Cartoon_Back_Nursing_Right_7");
								break;						
						case 7:	u2_printf("Cartoon_Back_Nursing_Right_8");									
								break;
						case 8:	u2_printf("Cartoon_Back_Nursing_Right_9");
								break;					
						case 9:	u2_printf("Cartoon_Back_Nursing_Right_10");
								break;					
						case 10:u2_printf("Cartoon_Back_Nursing_Right_11");
								break;	
						case 11:u2_printf("Cartoon_Back_Nursing_Right_12");
								break;	
						case 12:u2_printf("Cartoon_Back_Nursing_Right_13");
								break;
						case 13:u2_printf("Cartoon_Back_Nursing_Right_14");
								break;	
						case 14:u2_printf("Cartoon_Back_Nursing_Right_15");
								break;	
						case 15:u2_printf("Cartoon_Back_Nursing_Right_16");
								break;
						case 16:u2_printf("Cartoon_Back_Nursing_Right_17");
								break;	
						case 17:u2_printf("Cartoon_Back_Nursing_Right_18");
								break;
						case 18:u2_printf("Cartoon_Back_Nursing_Right_19");
								break;	
					}
				}
			}			
		}			
		Motor_3_STOP();      //电机停止
		TIM10_Stop();        //关闭定时器
		break_flag=0;        //清除break标志位
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		
		if(back_nursing_right_dir_flag==1)
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_20");
			delay_ms(200);
			Wifi_Send("RunLim");
			delay_ms(200);
			u2_printf("BackNursingRightLim");
		}
		else
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_1"); 
			delay_ms(200);
			Wifi_Send("RunRes");
			delay_ms(200);
			u2_printf("BackNursingRightRes");
			delay_ms(200);
			u2_printf("back_nursing_right_flag==0");
		}				
	}
	else
	{
		LED0=0;        //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("BackNursingRightInterfere");
		LED0=1;	
		LED1=1;		
	}
}

/***********************************************************************
 函数名      ：Fun_Waist_Nursing_Left(void)  
 函数功能    ：左腰部护理
 输入        ：无
 输出        ：无                           
************************************************************************/
void Fun_Waist_Nursing_Left(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;  
	u8 break_flag=0;    //判断程序从break跳出
	static u16 k=0;     //发送第k张动画指令
	u8 i=0;
	u16 j=0;	 
	u16 arr_send;	    //当前一次运行脉冲数
	static u8 kj;
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;		
	
	//联锁功能，只有在已执行左翻身功能且左背部护理复位后，才能进行左腰部护理
	if((lock_flag==1)&&(1==body_left_flag)&&(back_nursing_left_flag==0))
	{
		waist_nursing_left_flag=!waist_nursing_left_flag;
		waist_nursing_left_dir_flag=!waist_nursing_left_dir_flag;		
		if(waist_nursing_left_dir_flag==1)
		{ 
			DIR4=1; direct=1;
			delay_ms(200);
			u2_printf("waist_nursing_left_flag==1");			
			delay_ms(200);
			u2_printf("WaistNursingLeftStart");
			Wifi_Send("RunStart");
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_1");
			delay_ms(200);
		}
		else
		{ 
			DIR4=0; direct=0;			
			u2_printf("Cartoon_Waist_Nursing_Left_20");
			delay_ms(200);
		}
			
		Motor_4_START((u16)(motor_body_freq*1.2),motor_timer_freq);             //电机启动
		TIM10_Init(body_left_runed_arr,timer10_freq);                //打开定时器
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //等待定时时间到，时间到跳出循环 
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//光电限位
//				if((0==GD4_Left_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD4_Left_End)
//					{
//						u2_printf("GD4LeftEnd");
//						break_flag=1;
//						break;	
//					}
//				}
//				if((0==GD4_Start)&&(0==direct))      //落下
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("GD4Start");
//						break_flag=1;
//						break;
//					}						
//				}
				  //判断有没有收到上位机指令		
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
				//电机故障、故障诊断
//				if(1==Motor4_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor4_Alm)
//					{
//						waist_nursing_left_overload=1;
//						u2_printf("WaistNursingLeftOverload");
//						Breakdown_Treatment();
//						break_flag=1;
//						break;	
//					}						
//				}						
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
            //发送动画指令			
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_left_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=waist_nursing_left_picture_k;
			if(	k!=j)
			{
				kj=abs(k,j);			
				if(kj<2)
				{
					k=j;   waist_nursing_left_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Waist_Nursing_Left_2");									
								break;
						case 2:	u2_printf("Cartoon_Waist_Nursing_Left_3");
								break;					
						case 3:	u2_printf("Cartoon_Waist_Nursing_Left_4");
								break;					
						case 4:	u2_printf("Cartoon_Waist_Nursing_Left_5");
								break;	
						case 5:	u2_printf("Cartoon_Waist_Nursing_Left_6");
								break;	
						case 6:	u2_printf("Cartoon_Waist_Nursing_Left_7");
								break;
						case 7:	u2_printf("Cartoon_Waist_Nursing_Left_8");									
								break;
						case 8:	u2_printf("Cartoon_Waist_Nursing_Left_9");
								break;					
						case 9:	u2_printf("Cartoon_Waist_Nursing_Left_10");
								break;					
						case 10:u2_printf("Cartoon_Waist_Nursing_Left_11");
								break;	
						case 11:u2_printf("Cartoon_Waist_Nursing_Left_12");
								break;	
						case 12:u2_printf("Cartoon_Waist_Nursing_Left_13");
								break;
						case 13:u2_printf("Cartoon_Waist_Nursing_Left_14");
								break;	
						case 14:u2_printf("Cartoon_Waist_Nursing_Left_15");
								break;	
						case 15:u2_printf("Cartoon_Waist_Nursing_Left_16");
								break;
						case 16:u2_printf("Cartoon_Waist_Nursing_Left_17");
								break;	
						case 17:u2_printf("Cartoon_Waist_Nursing_Left_18");
								break;	
						case 18:u2_printf("Cartoon_Waist_Nursing_Left_19");
								break;		
					}
				}
			}			
		}				     
		Motor_4_STOP();     //电机停止
		TIM10_Stop();       //定时器关闭
		break_flag=0;       //清除break标志位
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		
		if(waist_nursing_left_dir_flag==1)
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_20");  
			delay_ms(200);
			Wifi_Send("RunLim");
			delay_ms(200);
			u2_printf("WaistNursingLeftLim");
		}
		else
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_1");
			delay_ms(200);
			u2_printf("waist_nursing_left_flag==0");			
			delay_ms(200);
			Wifi_Send("RunRes");
			delay_ms(200);
			u2_printf("WaistNursingLeftRes");
		}				
	}
	else
	{
		LED0=0;        //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("WaistNursingLeftInterfere");
		LED0=1;	
		LED1=1;
	}
}

/***********************************************************************
 函数名      ：Fun_Waist_Nursing_Right(void)  
 函数功能    ：右腰部护理
 输入        ：无
 输出        ：无                           
************************************************************************/
void Fun_Waist_Nursing_Right(void)
{
	u8 direct,key,len;
	u16 arr_feed;      //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;   //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;  
	u8 break_flag=0;      //判断程序从break跳出
	static u16 k=0;       //发送第k张动画指令
	u8 i=0;
	u16 j=0;	
	u16 arr_send;	      //当前一次运行脉冲数
	static u8 kj;
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
	
	//联锁功能，只有在已执行右翻身功能且右背部护理复位后，才能进行右腰部护理
	if((lock_flag==1)&&(body_right_flag==1)&&(back_nursing_right_flag==0))
	{
		waist_nursing_right_flag=!waist_nursing_right_flag;	
		waist_nursing_right_dir_flag=!waist_nursing_right_dir_flag;		
		if(waist_nursing_right_dir_flag==1)
		{ 
			DIR4=0; direct=1;
			delay_ms(200);
			u2_printf("waist_nursing_right_flag==1");
			delay_ms(200);
			u2_printf("WaistNursingRightStart");
			Wifi_Send("RunStart");
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_1");
			delay_ms(200);
		}
		else
		{ 
			DIR4=1; direct=0;
			u2_printf("Cartoon_Waist_Nursing_Right_20");
			delay_ms(200);
		}
			
		Motor_4_START(motor_body_freq,motor_timer_freq);             //电机启动
		TIM10_Init(body_right_runed_arr,timer10_freq);               //打开定时器
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //等待定时时间到，时间到跳出循环
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//光电限位
//				if((0==GD4_Right_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD4_Right_End)
//					{
//						u2_printf("GD4RightEnd");
//						break_flag=1;
//						break;	
//					}
//				}
//				if((0==GD4_Start)&&(0==direct))     //落下
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("GD4Start");
//						break_flag=1;
//						break;	
//					}						
//				}
				  //判断有没有收到上位机指令		
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
				//电机故障、故障诊断
//				if(1==Motor4_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor4_Alm)
//					{
//						waist_nursing_right_overload=1;
//						u2_printf("WaistNursingRightOverload");
//						Breakdown_Treatment();
//						break_flag=1;
//						break;	
//					}						
//				}			
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}	
			//发送动画指令
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_right_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=waist_nursing_right_picture_k;
			if(	k!=j)
			{
				kj=abs(k,j);							
				if(kj<2)	
				{
					k=j;   waist_nursing_right_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Waist_Nursing_Right_2");									
								break;
						case 2:	u2_printf("Cartoon_Waist_Nursing_Right_3");
								break;					
						case 3:	u2_printf("Cartoon_Waist_Nursing_Right_4");
								break;					
						case 4:	u2_printf("Cartoon_Waist_Nursing_Right_5");
								break;	
						case 5:	u2_printf("Cartoon_Waist_Nursing_Right_6");
								break;	
						case 6:	u2_printf("Cartoon_Waist_Nursing_Right_7");
								break;
						case 7:	u2_printf("Cartoon_Waist_Nursing_Right_8");									
								break;
						case 8:	u2_printf("Cartoon_Waist_Nursing_Right_9");
								break;					
						case 9:	u2_printf("Cartoon_Waist_Nursing_Right_10");
								break;					
						case 10:u2_printf("Cartoon_Waist_Nursing_Right_11");
								break;	
						case 11:u2_printf("Cartoon_Waist_Nursing_Right_12");
								break;	
						case 12:u2_printf("Cartoon_Waist_Nursing_Right_13");
								break;
						case 13:u2_printf("Cartoon_Waist_Nursing_Right_14");
								break;	
						case 14:u2_printf("Cartoon_Waist_Nursing_Right_15");
								break;	
						case 15:u2_printf("Cartoon_Waist_Nursing_Right_16");
								break;
						case 16:u2_printf("Cartoon_Waist_Nursing_Right_17");
								break;
						case 17:u2_printf("Cartoon_Waist_Nursing_Right_18");
								break;	
						case 18:u2_printf("Cartoon_Waist_Nursing_Right_19");
								break;						
					}
				}
			}
		}				    
		Motor_4_STOP();    //电机停止
		TIM10_Stop();      //关闭定时器 
		break_flag=0;      //清除break标志位
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		
		if(waist_nursing_right_dir_flag==1)
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_20");  
			delay_ms(200);
			Wifi_Send("RunLim");
			delay_ms(200);
			u2_printf("WaistNursingRightLim");
		}
		else
		{			
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_1");  
			delay_ms(200);
			Wifi_Send("RunRes");
			delay_ms(200);
			u2_printf("WaistNursingRightRes");
			delay_ms(200);
			u2_printf("waist_nursing_right_flag==0");
		}		
	}
	else
	{
		LED0=0;        //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("WaistNursingRightInterfere");		
		LED0=1;	
		LED1=1;
	}
}

/***********************************************************************
 函数名      ：Washlet_Auto(void)   
 函数功能    ：按键执行自动坐便器功能
 输入        ：无
 输出        ：无 
                          
************************************************************************/
void Washlet_Auto(void)
{
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
	
	//联锁功能，只有在上曲腿、左右翻身复位后，才能执行座便器功能	
	if((lock_flag==1)&&(0==body_left_flag)&&(0==body_right_flag)) 
	{
		if(0==washlet_auto_flag)
		{
			//Wifi_Send("RunStart");
			//delay_ms(200);
			u2_printf("WashletAutoStart");
			delay_ms(200);
		}                         
		washlet_auto_dir_flag=!washlet_auto_dir_flag; 
		
		if((washlet_auto_dir_flag==1)&&(0==washlet_auto_flag))     //自动坐便正行程
		{
			u2_printf("washlet_auto_flag==1");
			delay_ms(200);
			back_dir_flag=1; 
			washlet_auto_flag=1;
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
				Motor_1_START(1);                                  //支背上行
				back_state_flag=1;                                 //支背需要动作，下曲腿不需要 
				leg_down_state_flag=0;
				u2_printf("Cartoon_Washlet_Back_1");
				delay_ms(200);
			}
			else if((0==back_flag)&&(0==leg_down_flag))            //此时处于支背及下曲腿复位状态
			{
				Motor_1_START(1);                                  //支背上行  
				Push_Rod_Start(1);                                 //下曲腿下
				leg_down_state_flag=1;                             //支背需要动作，下曲腿需要动作       
				back_state_flag=1;
				u2_printf("Cartoon_Washlet_Back_Leg_Down_1");
				delay_ms(200);
			}
			else if((1==back_flag)&&(1==leg_down_flag))                 //此时已处于支背支起及下曲腿状态
			{
				leg_down_state_flag=0;            
				back_state_flag=0; 
			}
		}		
		else if((washlet_auto_dir_flag==0)&&(1==washlet_auto_flag))         //自动坐便复位-支背、下曲腿复位
		{
			back_dir_flag=0; 
			leg_down_state_flag=1;            
			back_state_flag=1;			
			Motor_1_START(0);             //支背下行       
			Push_Rod_Start(0);            //下曲腿上			
			u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
			delay_ms(200);
		}
		if((1==leg_down_state_flag)||(1==back_state_flag))
		{	
			if(1==leg_down_state_flag){ leg_down_flag=1; }
			if(1==back_state_flag)    { back_flag=1; }
			Back_Leg();
		}
		if(washlet_auto_dir_flag==1)
		{
			Washlet(0);	            //坐便打开			
			delay_ms(1000);
//			Washlet_Weight();       //重物检测
//			delay_ms(1000);
			Swash_Dry();            //冲洗烘干			
			Washlet(1);	            //坐便关闭
			if(washlet_flag==0)     //判断坐便是否处于复位状态，再进行坐便袋收紧
			{									
				Washlet_Tig(1);             //坐便袋收紧
				delay_ms(100);
				Washlet_Auto();             //再次调用该函数，使标志位取反，复位
				leg_down_flag=0;
				back_flag=0;
				washlet_auto_flag=0;
				leg_down_state_flag=0;
				back_state_flag=0;
				delay_ms(200);
				Wifi_Send("RunRes");
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
		Wifi_Send("WashletAutoInterfere");
		LED0=1;
		LED1=1;  
	}	
}


/***********************************************************************
 函数名      ：Washlet(void)  
 函数功能    ：按键执行坐便器功能
 输入        ：dir: 0(打开坐便)；1（关闭坐便）
 输出        ：无
                           
************************************************************************/
void Washlet(u8 dir)
{
	u8 direct;       //代表某个动作运行的方向标志：1-正向运行；0-反向运行
	u8 key;          //按键扫描函数返回值,用于判断电机失步故障
	u16 num,len;
	u16 arr_feed;    //计算程序中当前一次运行脉冲数
	u16 pulse_num;   //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //程序从break跳出标志位
	static u16 k=0;           //发送第k张图片指令
	u8 i=0;
	u16 j=0;	
	u16 arr_send;             //当前一次运行脉冲数
	static u8 kj;
	
	washlet_flag=1;
	//联锁功能，只有在上曲腿、左右翻身复位后，才能执行座便器功能
	if((leg_up_flag==0)&&(body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{
		DIR6=dir;	   
		direct=!dir;

		Motor_6_START(motor_washlet_freq,motor_timer_freq);           //电机启动
		TIM10_Init(washlet_arr_lim,timer10_freq);                     //打开定时器35000
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位
		if(1==direct)
		{
			delay_ms(200);
			u2_printf("Cartoon_Washlet_1");
			delay_ms(200);
		}
		else
		{
			u2_printf("Cartoon_Washlet_25");
			delay_ms(200);
		}
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //等待定时时间到，时间到跳出循环
		{						
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{	
				//光电限位
//				if((0==GD6_End)&&(1==direct))
//				{
//					delay_us(100);
//					if(0==GD6_End)
//					{
//						u2_printf("GD6End");
//						break_flag=1;
//						break ; 
//					}
//				}
//				if((0==GD6_Start)&&(0==direct))
//				{
//					delay_us(100);
//					if(0==GD6_Start)
//					{
//						u2_printf("GD6Start");
//						break_flag=1;
//						break ;
//					}
//				}
				  //判断有没有收到上位机指令		
				if(USART2_RX_LEN&0x8000)
				{
					len=USART2_RX_LEN&0x3fff;				
					USART2_RX_BUF[len]=0;
					if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))
					{	}
					else 
					{
						u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
						memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
						UART4_RX_LEN=0;
					}												
				}							
				//电机故障、故障诊断
//				if(1==Motor6_Alm)     
//				{	
//					delay_us(100);
//					if(1==Motor6_Alm) 
//					{
//						washlet_auto_overload=1;
//						Wifi_Send("WashletAutoOverload");
//						Breakdown_Treatment();
//						break_flag=1;
//					}
//					break;		             
//				}						
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//发送动画指令
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(washlet_arr_lim/24);	
			if(0==direct)
			{
				j=24-j;
			}
			k=washlet_picture_k;
			if(k!=j)
			{
				kj=abs(k,j);
				if(kj<2)
				{
					k=j;  washlet_picture_k=k;
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
		break_flag=0;      //清除break标志位
		//判断复位
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))//判断是否处于复位状态，复位状态的前提是下行的定时器走完
		{		
			washlet_flag=0;	  
			delay_ms(200);
			u2_printf("Cartoon_Washlet_1");	
				
		}				
		 else
		{
			washlet_flag=1;   
			delay_ms(200);
			u2_printf("Cartoon_Washlet_25");
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//		//使电机复位到初始状态（光电安装后直接打开此段）
//		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD6_Start)&&(direct==0)&&(washlet_flag==1))
//		{
//			DIR6=1;
//			Motor_6_START(motor_washlet_freq,motor_timer_freq);           //电机启动
//			TIM10_Init(add_arr,timer10_freq);                             //打开定时器35000
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位	 	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )  //等待定时时间到，时间到跳出循环
//			{
//				if(0==GD6_Start)  //如果电机过载或碰到光电开关，则跳出循环，电机停止转动 
//				{  				
//					u2_printf("GD6Start");
//					break; 											 
//				}				
//			}				                                 			
//			Motor_6_STOP();      //电机停止
//			TIM10_Stop();        //关闭定时器
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位
//		}
	}
	else
	 {
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("WashletInterfere");
		LED0=1;
		LED1=1;    	
	 }	
}


/***********************************************************************
 函数名      ：Back_Leg()   
 函数功能    ：支背、下曲腿同时运行函数
 输入        ：无
 输出        ：无 
                          
************************************************************************/
void Back_Leg(void)
{
	u8 len;
	u16 arr_now;               //当前一次运行脉冲数
	
	//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;           //判断程序是否是从break跳出来 
	u16 repeat_num;
	static u8 k;               //传第k张动画
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //当前一次运行脉冲值
		
	//联锁功能，只有在左右翻身功能复位后，才能进行支背、下曲腿
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;
		
		//运行支背或下曲腿或支背和下曲腿同时运行
		TIM10_Init(leg_angle_to_arr(leg_down_angle_lim),timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )    //定时时间到
		{
			 //判断有没有收到别的指令
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
			 //发送支背动画指令
			if((1==back_state_flag)&&(0==leg_down_state_flag))           //只执行支背
			{
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);				
				j=arr_send/(back_angle_to_arr(back_angle_max)/19);
				if(0==washlet_auto_dir_flag)
				{
					j=19-j;
				}
				k=back_picture_k;
				if(k!=j)
				{
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;  back_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Back_2");							
									break;
							case 2:	u2_printf("Cartoon_Washlet_Back_3");											
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Back_4");											
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Back_5");											
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Back_6");											
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Back_7");											
									break;
							case 7:	u2_printf("Cartoon_Washlet_Back_8");										
									break;
							case 8:	u2_printf("Cartoon_Washlet_Back_9");											
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Back_10");											
									break;												
							case 10:u2_printf("Cartoon_Washlet_Back_11");											
									break;
							case 11:u2_printf("Cartoon_Washlet_Back_12");																		
									break;
							case 12:u2_printf("Cartoon_Washlet_Back_13");											
									break;					
							case 13:u2_printf("Cartoon_Washlet_Back_14");											
									break;					
							case 14:u2_printf("Cartoon_Washlet_Back_15");										
									break;	
							case 15:u2_printf("Cartoon_Washlet_Back_16");																		
									break;
							case 16:u2_printf("Cartoon_Washlet_Back_17");											
									break;					
							case 17:u2_printf("Cartoon_Washlet_Back_18");											
									break;					
							case 18:u2_printf("Cartoon_Washlet_Back_19");										
									break;							
						}
					}
				}
			}
			//发送曲腿动画指令
			else if((0==back_state_flag)&&(1==leg_down_state_flag))      //只执行下曲腿
			{
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				j=arr_send/(leg_angle_to_arr(leg_down_angle_max)/19);
				if(0==washlet_auto_dir_flag)
				{
					j=19-j;
				}
				k=leg_down_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;  leg_down_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Leg_Down_2");	
									break;
							case 2:	u2_printf("Cartoon_Washlet_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Washlet_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Washlet_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Washlet_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Washlet_Leg_Down_12");	
									break;
							case 12:u2_printf("Cartoon_Washlet_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Washlet_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Washlet_Leg_Down_15");
									break;	
							case 15:u2_printf("Cartoon_Washlet_Leg_Down_16");
									break;	
							case 16:u2_printf("Cartoon_Washlet_Leg_Down_17");	
									break;
							case 17:u2_printf("Cartoon_Washlet_Leg_Down_18");
									break;					
							case 18:u2_printf("Cartoon_Washlet_Leg_Down_19");
									break;					
							case 19:u2_printf("Cartoon_Washlet_Leg_Down_20");
									break;	
						}
					}				
				}			
			}			
			
			//发送支背和下曲腿同时运行动画指令
			else if((1==back_state_flag)&&(1==leg_down_state_flag))      //支背和下曲腿同时运行
			{				
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				j=arr_send/(back_angle_to_arr(back_angle_max)/14);
				if(0==washlet_auto_dir_flag)
				{
					j=14-j;
				}
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Back_Leg_Down_2");	
									break;
							case 2:	u2_printf("Cartoon_Washlet_Back_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Back_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Back_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Back_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Back_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Washlet_Back_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Washlet_Back_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Back_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Washlet_Back_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Washlet_Back_Leg_Down_12");	
									break;
							case 12:u2_printf("Cartoon_Washlet_Back_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Washlet_Back_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
									break;																											
						}
					}				
				}								
			}										
		}
		Push_Rod_Stop();            //下曲腿停止
		TIM10_Stop();		        //关闭定时器	
		//判断下曲腿复位
		if((washlet_auto_dir_flag==0)&&(1==leg_down_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
		{
			leg_down_flag=0; 
			leg_down_runed_arr=0; 
			if(0==back_state_flag)                   //只有下曲腿运行，支背不运行
			{
				k=0;   leg_down_picture_k=0;
				delay_ms(200);
				u2_printf("Cartoon_Washlet_Leg_Down_1");
			}
		}
		if((washlet_auto_dir_flag==1)&&(1==leg_down_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
		{
			leg_down_flag=1; 
			leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim); 
			if(0==back_state_flag)                   //只有下曲腿运行，支背不运行
			{
				k=19;   leg_down_picture_k=19;
				delay_ms(200);
				u2_printf("Cartoon_Washlet_Leg_Down_20");
			}
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //清除中断标志位
		
		//继续运行支背或支背及下曲腿同时继续运行
		if(back_state_flag==1)              //继续运行支背或支背及下曲腿同时继续运行
		{				
			TIM10_Init(back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim),timer10_freq);
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) ) //支背定时时间到
			{	
				 //判断有没有收到别的指令
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
				 if(break_flag==1)
				 {
					u2_printf("break_flag==1");
					break;
				 }
				 //继续发送支背运行动画指令
				 if(0==leg_down_state_flag)        //只有支背运行
				 {
					 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					 if(washlet_auto_dir_flag==1)
					 {
						 j=(leg_angle_to_arr(leg_down_angle_lim)+arr_send)/(back_angle_to_arr(back_angle_max)/19);
					 }
					 else
					 {
						 arr_send=back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim)-arr_send;
						 j=arr_send/(back_angle_to_arr(back_angle_max)/19);
					 }
					 k=back_picture_k;
					 if(k!=j)
					 {				
						kj=abs(k,j);
						if(kj<2)
						{
							k=j;   back_picture_k=k;
							switch (k)
							{						
								case 1:	u2_printf("Cartoon_Washlet_Back_2");							
										break;
								case 2:	u2_printf("Cartoon_Washlet_Back_3");											
										break;					
								case 3:	u2_printf("Cartoon_Washlet_Back_4");											
										break;					
								case 4:	u2_printf("Cartoon_Washlet_Back_5");											
										break;	
								case 5:	u2_printf("Cartoon_Washlet_Back_6");											
										break;	
								case 6:	u2_printf("Cartoon_Washlet_Back_7");											
										break;
								case 7:	u2_printf("Cartoon_Washlet_Back_8");										
										break;
								case 8:	u2_printf("Cartoon_Washlet_Back_9");											
										break;						
								case 9:	u2_printf("Cartoon_Washlet_Back_10");											
										break;												
								case 10:u2_printf("Cartoon_Washlet_Back_11");											
										break;
								case 11:u2_printf("Cartoon_Washlet_Back_12");																		
										break;
								case 12:u2_printf("Cartoon_Washlet_Back_13");											
										break;					
								case 13:u2_printf("Cartoon_Washlet_Back_14");											
										break;					
								case 14:u2_printf("Cartoon_Washlet_Back_15");										
										break;	
								case 15:u2_printf("Cartoon_Washlet_Back_16");																		
										break;
								case 16:u2_printf("Cartoon_Washlet_Back_17");											
										break;					
								case 17:u2_printf("Cartoon_Washlet_Back_18");											
										break;					
								case 18:u2_printf("Cartoon_Washlet_Back_19");										
										break;																																			
							}
						}				
					 }						
				 }
				 //继续发送下曲腿、支背同时运行的动画指令
				 if(1==leg_down_state_flag)       //支背、曲腿都运行
				 { 
					 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					 if(washlet_auto_dir_flag==1)
					 {
						 j=(leg_angle_to_arr(leg_down_angle_lim)+arr_send)/(back_angle_to_arr(back_angle_max)/14);
					 }
					 else
					 {
						 arr_send=back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim)-arr_send;
						 j=arr_send/(back_angle_to_arr(back_angle_max)/14);
					 }
					 k=leg_down_picture_k;
					 if(k!=j)
					 {				
						kj=abs(k,j);
						if(kj<2)
						{
							k=j;  leg_down_picture_k=k;
							switch (k)
							{						
								case 1:	u2_printf("Cartoon_Washlet_Back_Leg_Down_2");	
										break;
								case 2:	u2_printf("Cartoon_Washlet_Back_Leg_Down_3");
										break;					
								case 3:	u2_printf("Cartoon_Washlet_Back_Leg_Down_4");
										break;					
								case 4:	u2_printf("Cartoon_Washlet_Back_Leg_Down_5");
										break;	
								case 5:	u2_printf("Cartoon_Washlet_Back_Leg_Down_6");
										break;	
								case 6:	u2_printf("Cartoon_Washlet_Back_Leg_Down_7");
										break;
								case 7:	u2_printf("Cartoon_Washlet_Back_Leg_Down_8");
										break;
								case 8:	u2_printf("Cartoon_Washlet_Back_Leg_Down_9");
										break;						
								case 9:	u2_printf("Cartoon_Washlet_Back_Leg_Down_10");
										break;												
								case 10:u2_printf("Cartoon_Washlet_Back_Leg_Down_11");
										break;	
								case 11:u2_printf("Cartoon_Washlet_Back_Leg_Down_12");	
										break;
								case 12:u2_printf("Cartoon_Washlet_Back_Leg_Down_13");
										break;					
								case 13:u2_printf("Cartoon_Washlet_Back_Leg_Down_14");
										break;																																									
							}
						}				
					}	
				}						 
			}
			Motor_1_STOP();                                             //支背停止
			TIM10_Stop();                                               //关闭定时器
			//判断支背复位
			if((washlet_auto_dir_flag==0)&&(1==back_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				back_flag=0; 
				back_runed_arr=0;
				if(0==leg_down_state_flag)         //只有支背运行
				{
					k=0;   back_picture_k=0;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_1");
				}
				if(1==leg_down_state_flag)         //支背、曲腿同时运行
				{
					k=0;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_Leg_Down_1");
				}
			}
			if((washlet_auto_dir_flag==1)&&(1==back_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				back_flag=1; 
				back_runed_arr=back_angle_to_arr(back_angle_lim);
				if(0==leg_down_state_flag)         //只有支背运行
				{
					k=19;   back_picture_k=19;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_20");
				}
				if(1==leg_down_state_flag)         //支背、曲腿同时运行
				{
					k=14;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
				}
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //清除中断标志位
		}				
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

/***********************************************************************
 函数名      ：Weight()   
 函数功能    ：称重函数
 输入        ：无
 输出        ：1：重物在变化；0：重物未发生变化 
                          
************************************************************************/
u8 Weight(void)  
{
    //开始检测重物				
	if(num==1)
	{				         			
		num=2;
		u1=filter();
		u2_printf("\r\nu1=%d\r\n",u1);
		delay_ms(1000);
		if(num==2)
		{	
			num=3;
			u2=filter();
			u2_printf("\r\nu2=%d\r\n",u2);
			delay_ms(1000);
			if(num==3)
			{	 
				num=1;
				u3=filter();
				u2_printf("\r\nu3=%d\r\n",u3);
				delay_ms(1000);
			}
		}
	}
	u2_printf("\r\nabs(u1,u2)=%d\r\n",abs(u1,u2));
	u2_printf("\r\nabs(u1,u3)=%d\r\n",abs(u1,u3));
	u2_printf("\r\nabs(u2,u3)=%d\r\n",abs(u2,u3));
	//检测重物质量是否发生变化
	 if((abs(u1,u2)<0x30)&&(abs(u1,u3)<0x30)&&(abs(u2,u3)<0x30))//0x300=768
	 {
		 u2_printf("\r\n重量没有发生变化\r\n");
		 u2_printf("\r\nu1=%d\r\n",u1);
		 u2_printf("\r\nu2=%d\r\n",u2);
		 u2_printf("\r\nu3=%d\r\n",u3);
		 return 0;
	 }
	 else
	 {
		 u2_printf("\r\n重量发生变化\r\n");
		 return 1;
	 }	 	
}	


/***********************************************************************
 函数名      ：Washlet_Weight(void)   
 函数功能    ：按键执行自动坐便器功能
 输入        ：无
 输出        ：无 
                          
************************************************************************/
u8  Washlet_Weight(void)
{
	u8 m=0,i=0;
	while(1)
	{
		TIM10_Init(60000-1,timer10_freq);      //打开定时器30S
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //清除中断标志位
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到
		{
			if(Weight())                //若重物在变化跳出循环，重新开始计时
			{  
				m++;
				switch(m)
				{
					case 1:u2_printf("Cartoon_Washlet_Weight_1");									
							break;
					case 2:u2_printf("Cartoon_Washlet_Weight_2");									
							break;
					case 3:u2_printf("Cartoon_Washlet_Weight_3");									
							break;
					case 4:u2_printf("Cartoon_Washlet_Weight_4");									
							break;
					case 5:u2_printf("Cartoon_Washlet_Weight_5");									
							break;
					case 6:u2_printf("Cartoon_Washlet_Weight_6");									
							break;
					case 7:u2_printf("Cartoon_Washlet_Weight_7");									
							break;
					case 8:u2_printf("Cartoon_Washlet_Weight_8");									
							break;
					case 9:u2_printf("Cartoon_Washlet_Weight_9");									
							break;
					case 10:u2_printf("Cartoon_Washlet_Weight_10");									
							break;
					case 11:u2_printf("Cartoon_Washlet_Weight_11");									
							break;
					case 12:u2_printf("Cartoon_Washlet_Weight_12");									
							break;
					case 13:u2_printf("Cartoon_Washlet_Weight_13");									
							break;
					case 14:u2_printf("Cartoon_Washlet_Weight_14");									
							break;
					case 15:u2_printf("Cartoon_Washlet_Weight_15");									
							break;
				}				
			}
		}
		//如果30S定时间到且重物没有发生变化
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(0==Weight())) 
		{ 
			i++; 
		}
		else { i=0; }                    //否则重新计时
		if(i==4)                         //重物两分钟没有发生变化，开始冲洗烘干
		{
			Wifi_Send("Washlet_Over");   //排便结束
			u1=0;
			u2=0;
			u3=0;
			break; 
		}   
	}	   	 		
}

/***********************************************************************
 函数名      ：Washlet_Tig()   
 函数功能    ：坐便袋收紧
 输入        ：dir:电机运行方向标志；1-正转；0-反转
 输出        ：无
                          
************************************************************************/	
void Washlet_Tig(u8 dir)
{	
	u2_printf("坐便袋收紧开始");
//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;          //判断程序是否是从break跳出来 
	u16 repeat_num;
	u16 k=0;                  //传第k张动画
	u8 kj;
	u16 j=0;	
	u16 arr_send;             //当前一次运行脉冲值	
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;
	u8 len;
	RELAY6=1;                  //继电器得电，常开触点闭合，坐便袋扎紧电机得电
	delay_ms(1000);
	Motor_6_2_START(0,18000);  //收线推杆伸出
	u2_printf("Cartoon_Washlet_Tig_1");		
	DIR6_1=dir;
	Motor_6_1_START(360-1,250-1);                  //将坐便袋扎紧
	TIM2_Init(10500,timer10_freq);                             //打开定时器35000
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);         //清除中断标志位
	while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler, TIM_SR_CC1IF)))  //等待定时时间到
	{
		for(repeat_num=0;repeat_num<700;repeat_num++)
		{ 			
			//判断有没有收到上位机指令		
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
		}
		 arr_send=__HAL_TIM_GET_COUNTER(&TIM2_Handler);      //当前一次脉冲值
		//传输动画指令
		if(dir==1)
		{
			j=arr_send/(10500/6);
		}
		else
		{
			j=arr_send/(10500/6);
			j=6-j;
		}
		if(	k!=j)
		{	
			kj=abs(k,j);
			if(kj<2)
			{
				k=j;
				switch (k)
				{						
					case 1:	u2_printf("Cartoon_Washlet_Tig_2");									
							break;
					case 2:	u2_printf("Cartoon_Washlet_Tig_3");
							break;					
					case 3:	u2_printf("Cartoon_Washlet_Tig_4");
							break;					
					case 4:	u2_printf("Cartoon_Washlet_Tig_5");
							break;	
					case 5:	u2_printf("Cartoon_Washlet_Tig_6");
							break;																				
				}
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);          //清除中断标志位
	Motor_6_1_STOP();                                           //电机停止
	TIM2_Stop();                                                //关闭定时器
	delay_ms(100);
	u2_printf("Cartoon_Washlet_Tig_7");
	delay_ms(1000);
	Motor_6_2_START(1,18000);                                    //收线推杆缩回	
	delay_ms(1000);
	RELAY6=0;                                                   //继电器复位，坐便袋扎紧电机断电
	u2_printf("坐便袋收紧结束");
}	


/***********************************************************************
 函数名      ：Swash_Dry(void)   
 函数功能    ：按键执行冲洗烘干功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Swash_Dry()
{
	u8 num,len;    //接收字符串长度
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	UART4_RX_LEN=0;
	
    //连锁功能，只有在坐便打开时才能进行喷气烘干
	washlet_flag=1;
	if((lock_flag==1)&&(1==washlet_flag))
	{
		Push_Rod_Swash_Dry(1,2000);                 //冲洗烘干推杆伸出  
		delay_ms(200);
		
/********************开始冲洗**********************/
        Swash_Auto();                               //自动冲洗
		
		//自动冲洗结束，等待30S若再次按下冲洗按键，进行手动调节冲洗
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);  //清空接收寄存器
		UART4_RX_LEN=0;
		TIM9_Init(60000,timer10_freq);              //打开定时器，定时30S
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);         //清除中断标志位
		while(!(__HAL_TIM_GET_FLAG(&TIM9_Handler, TIM_SR_CC1IF)))  //等待定时时间到
		{					
			if(UART4_RX_LEN&0x8000)
			{
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;	
				if(strstr((const char *)UART4_RX_BUF,(const char *)"SwashPhone"))   //若接收到Stop,则跳出循环	
				{					
					u2_printf("\r\nSwashPhone按下\r\n");
					Swash_Hand();
				}				
			}
		}		
		TIM9_Stop();                                         //关闭定时器
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);   //清除中断标志位	
		delay_ms(1000);
		
/**********************************开始烘干***********************************/
		
		Dry_Auto();       //自动烘干2分钟
		delay_ms(1000);
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
		UART4_RX_LEN=0;
		
		//自动烘干结束，等待30S若再次按下烘干按键，进行手动调节烘干		
		TIM9_Init(60000,timer10_freq);                             //打开定时器，定时30S
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);         //清除中断标志位
		while(!(__HAL_TIM_GET_FLAG(&TIM9_Handler, TIM_SR_CC1IF)))  //等待定时时间到
		{
			if(UART4_RX_LEN&0x8000)
			{
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;
				if(strstr((const char *)UART4_RX_BUF,(const char *)"DryPhone"))    //若接收到Stop,则跳出循环	
				{
					u2_printf("\r\nDryPhone按下\r\n");
					Dry_Hand();
				}				
			}
		}
		TIM9_Stop();                                        //关闭定时器
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);  //清除中断标志位		
		delay_ms(1000);
		Push_Rod_Swash_Dry(0,2000+swash_dry_runed_pulse);   //冲洗烘干推杆缩回		
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

/***********************************************************************
 函数名      ：Swash_Hand(void)   
 函数功能    ：按键执行冲洗功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Swash_Hand(void)
{
	u8 len;              //WiFi接收字符串长度
	u8 direct;           //方向标志位
	u8 i;
	Pump_Init();
    //连锁功能，只有在坐便打开时才能进行喷水冲洗
	if((lock_flag==1)&&(1==washlet_flag))		
	{								
		//喷水冲洗
		swash_hand_flag=1;
		RELAY6=1;             //继电器得电
		DIR_SB=1;             //水泵开启PB12
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		delay_ms(200);
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
		UART4_RX_LEN=0;		
		for(i=0;i<2*swash_dry_time;i++)      //冲洗swash_dry_time分钟
		{
			TIM10_Init(60000-1,timer10_freq_1);                        //打开定时器,定时器周期为30S					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位 
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环
			{ 					
				if(0==Liq_Sensor)	                        //水位在低水位下，不足一次冲洗，则直接跳出
				{ 
					delay_ms(100);
					Wifi_Send("LiquidLevellow");          //发送给上位机指令信号，表示此时水位偏低
					break;
				}
				if(UART4_RX_LEN&0x8000)
				{
					len=UART4_RX_LEN&0x3fff;				
					UART4_RX_BUF[len]=0;
					if(strstr((const char *)UART4_RX_BUF,(const char *)"PushRodUpPhone"))    //推杆伸出	
					{
						direct=1;
						Push_Rod_Swash(1,swash_dry_pulse_lim); 
					}
					if(strstr((const char *)UART4_RX_BUF,(const char *)"PushRodDown_hone"))  //推杆缩回	
					{ 
						direct=0;
						Push_Rod_Swash(0,swash_dry_pulse_lim); 
					}
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
					UART4_RX_LEN=0;	
				}	
			}						 		
			TIM10_Stop();		                                //关闭定时器
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位  
		}
		swash_hand_flag=0;
		DIR_SB=0;             //水泵关闭PB12
		if(swash_dry_runed_pulse>0)
		{
			direct=0;
			Push_Rod_Swash(0,swash_dry_runed_pulse);
		}		
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		RELAY6=0;             //继电器断电
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("SwashHandInterfere");
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Dry_Hand(void)   
 函数功能    ：按键执行烘干功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Dry_Hand(void)
{
	u8 len,i;
	u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行         
	
    //连锁功能，只有在坐便打开时才能进行喷气烘干
	if((lock_flag==1)&&(1==washlet_flag))		
	{	
		//喷气烘干
		DIR_HG=1;             //烘干喷气阀门打开PB10
		delay_ms(1000);       //等待阀门打开1S
		dry_hand_flag=1;
		RELAY6=1;             //继电器得电
		DIR_QB=1;             //气泵启动PH2
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}
		delay_ms(200);
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
		UART4_RX_LEN=0;		
		for(i=0;i<2*swash_dry_time;i++)      //烘干swash_dry_time分钟
		{
			TIM10_Init(60000-1,timer10_freq_1);                        //打开定时器,定时器周期为30S					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位 
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环
			{ 					
				if(UART4_RX_LEN&0x8000)
				{
					len=UART4_RX_LEN&0x3fff;				
					UART4_RX_BUF[len]=0;
					//判断上下行
					if(strstr((const char *)UART4_RX_BUF,(const char *)"PushRodUpPhone"))    //推杆伸出	
					{
						direct=1;
						Push_Rod_Dry(1,swash_dry_pulse_lim); 
					}
					if(strstr((const char *)UART4_RX_BUF,(const char *)"PushRodDown_hone"))  //推杆缩回	
					{ 
						direct=0;
						Push_Rod_Dry(0,swash_dry_pulse_lim);
					}
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
					UART4_RX_LEN=0;
				}				
			}              		
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位   		
			TIM10_Stop();   //关闭定时器		 	
		}
		dry_hand_flag=0;
		if(0==swash_dry_runed_pulse)
		{
			direct=0;
			Push_Rod_Dry(0,swash_dry_runed_pulse);
		}		
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}
		RELAY6=0;       //继电器断电
		DIR_QB=0;       //气泵关闭PH2
		DIR_HG=0;       //烘干喷气阀关闭PB10		
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("DryHandInterfere");
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Swash_Auto(void)   
 函数功能    ：按键执行冲洗功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Swash_Auto(void)
{
	u8 direct;   //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	u8 flag=0;   //控制推杆方向切换
	u8 i;
	u8 num,len;
	Pump_Init();
    //连锁功能，只有在坐便打开时才能进行喷水冲洗
	if((lock_flag==1)&&(1==washlet_flag))		
	{						
		delay_ms(100);
		//Wifi_Send("\r\n开始冲水\r\n");
		//冲洗之前检测水位若在正常水位以下，发出报警声，并等待水箱注水
		if(0==Liq_Sensor)  
		{
			delay_ms(100);
			u2_printf("Liquid_Level_Low");
			//Wifi_Send("Liquid_Level_Low");  //发送给上位机指令信号，表示此时水位偏低
		}
		delay_ms(200);
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);  //清空接收寄存器
		UART4_RX_LEN=0;
		
		while(0==Liq_Sensor)                        //水箱注水后，才能继续往下执行		
		{
			PCF8574_WriteBit(BEEP_IO,0);	        //控制蜂鸣器报警	
			if(UART4_RX_LEN&0x8000)
			{
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;				
				if(strstr((const char *)UART4_RX_BUF,(const char *)"BeepOff"))
				{
					PCF8574_WriteBit(BEEP_IO,1);    //蜂鸣器停止报警						
					break;
				}
			}			
		}	
		while(0==Liq_Sensor);                //若水箱未注满，等待注满
		PCF8574_WriteBit(BEEP_IO,1);         //蜂鸣器停止报警
		delay_ms(100);
		//Wifi_Send("水箱注满，开始运行");
		u2_printf("水箱注满，开始运行");
		delay_ms(1000);
		RELAY6=1;                            //继电器得电
		//喷水冲洗
		DIR_SB=1;                            //水泵开启PB12	
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		for(i=0;i<10*swash_dry_time;i++)     //冲洗烘干推杆自动循环冲洗swash_dry_time分钟
		{
			flag=!flag;
			Push_Rod_Swash(flag,swash_dry_pulse_lim);      //每次伸出缩回5S钟
			delay_ms(50);			
		}	
		if(0==swash_dry_runed_pulse)
		{                   
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		RELAY6=0;             //继电器断电
		DIR_SB=0;             //水泵关闭PB12		
		delay_ms(100);
	}
	else
	{
		LED0=0;               //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("SwashAutoInterfere");
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Dry_Auto(void)   
 函数功能    ：按键执行烘干功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Dry_Auto(void)
{
	u8 flag=0;        //控制推杆方向切换
	u8 i;
	
    //连锁功能，只有在坐便打开时才能进行喷气烘干
	if((lock_flag==1)&&(1==washlet_flag))		
	{
		delay_ms(100);
		Wifi_Send("\r\n开始喷气烘干\r\n");		
		//喷气烘干
		DIR_HG=1;             //烘干喷气阀门打开PB10
		delay_ms(1000);       //等待阀门打开1S
		RELAY6=1;             //继电器得电
		DIR_QB=1;             //气泵启动PH2
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}		
		for(i=0;i<10*swash_dry_time;i++)     //冲洗烘干推杆自动循环烘干swash_dry_time 分钟
		{
			flag=!flag;
			Push_Rod_Dry(flag,swash_dry_pulse_lim);        //每次伸出缩回5S钟
			delay_ms(50);
		} 
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}
		RELAY6=0;        //继电器断电
		DIR_QB=0;        //气泵关闭PH2
		DIR_HG=0;        //烘干喷气阀关闭PB10
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("DryAutoInterfere");
		LED0=1;
		LED1=1;
	}	
}
			

/***********************************************************************
 函数名      ：IO_TEST(void)   
 函数功能    ：控制板IO口测试
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void IO_TEST(void)
{
	
	GPIO_InitTypeDef GPIO_Initure;          //定义结构体变量GPIO_Initure
    __HAL_RCC_GPIOA_CLK_ENABLE();           //使能GPIOA时钟 
	__HAL_RCC_GPIOB_CLK_ENABLE();           //使能GPIOA时钟 
    __HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟
	__HAL_RCC_GPIOD_CLK_ENABLE();           //开启GPIOC时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();           //开启GPIOE时钟
    __HAL_RCC_GPIOH_CLK_ENABLE();           //开启GPIOH时钟
    __HAL_RCC_GPIOG_CLK_ENABLE();           //开启GPIOG时钟
    __HAL_RCC_GPIOI_CLK_ENABLE();           //开启GPIOI时钟
		
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_12;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;            //PA12
    GPIO_Initure.Mode=GPIO_MODE_INPUT;       //输入
    GPIO_Initure.Pull=GPIO_PULLUP;           //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;      //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);		
	
    GPIO_Initure.Pin=GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13; 
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP; 
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_13; 
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP; 
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6; 
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP; 
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8; //PI4
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);		
}

/***********************************************************************
 函数名      ：Muscle_Massager(void)   
 函数功能    ：执行肌肉按摩功能
 输入        ：t:肌肉按摩次数
 输出        ：无  
                          
***********************************************************************/
void Muscle_Massager(void)
{
	//连锁功能，只有在坐便孔关闭时才可进行按摩功能
	if((lock_flag==1))
	{
		muscle_massager_flag=!muscle_massager_flag;  //肌肉按摩标志位取反,1-开始；0：结束
		if(muscle_massager_flag==1)
		{
			//DIR_JRAM=1;          //按摩喷气阀门打开
			DIR_XZFPQ=1;					//旋转分配器阀门打开
			delay_ms(500);      //等待阀门打开		
			DIR_QB=1;            //气泵启动
			//Wifi_Send("MuscleMassagerStart");
			u2_printf("MuscleMassagerStart");
		}		
		else
		{
			DIR_QB=0;           //气泵停止
			DIR_XZFPQ=0;				//旋转分配器停止工作
			//DIR_JRAM=0;         //按摩喷气阀门关闭
			delay_ms(1000);     //等待阀门关闭
			//Wifi_Send("MuscleMassagerStop");
			u2_printf("MuscleMassagerStop");
		}		
	}		
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		//Wifi_Send("MuscleMassagerInterfere");
		LED0=1;
		LED1=1;
	} 
}

/***********************************************************************
 函数名      ：Heat(void)   
 函数功能    ：执行加热功能
 输入        ：无
 输出        ：无  
                          
***********************************************************************/
void Heat()
{
	short temp;
	if(1==washlet_flag)   	//连锁功能，只有在坐便打开的时候才能进行加热
	{
		LED1=0;
		//加热
		DIR_JR=1;           //启动加热器开始加热PH3
		TIM10_Init(timer10_arr_1s,timer10_freq_1); //打开定时器3,定时器周期为(9000*10000)/90MHZ=1S
		for(i=0;i<20;i++)    //定时20S
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)));  //等待定时时间到，时间到跳出循环
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位   
		}
		TIM10_Stop();       //关闭定时器	 
		DS18B20_Start();    //进行一次温度转换
		delay_ms(750);
			
		DS18B20_Rst();	 
		DS18B20_Write_Byte(0xCC);
		DS18B20_Write_Byte(0xBE);
			
		while(1)
		{
			if(1==temp_flag_1)   //初始化时该标志位为1
			{
				temp=DS18B20_Get_Temp();
				if(temp<=35)
				{
					u2_printf("\r\n%d\r\n",temp);	 
				}
				else
				{
					DIR_JR=0;        //关闭加热器
					u2_printf("\r\n%d\r\n",temp);
					temp_flag_1=0;   //使程序进入else，跳出第一个while循环
				}
			}
			else
			{
				temp_flag_1=1;
				break;
			}
		}			
		while(1)
		{
			if(1==temp_flag_2)              //初始化时该标志位为1
			{
				temp=DS18B20_Get_Temp();    //从ds18b20得到温度值
				u2_printf("\r\n%d\r\n",temp);
				if(temp<=30)
				{
					DIR_JR=1;               //启动加热器加热				
				}
				else if(temp>=37)
				{
					DIR_JR=0;               //关闭加热器停止加热
					temp_flag_2=0;          //使程序进入else，跳出第一个while循环
				}
			}
			else
			{
				temp_flag_1=1;
				temp_flag_2=1;
				break;
			}
		}		
		LED1=1;	
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("HeatInterfere");
		LED0=1;
		LED1=1;
	}	
}



/***********************************************************************
 函数名      ：WriteInUART4(void)  
 函数功能    ：串口4写入函数
 输入        ：要写入UART4_RX_BUF的字符串""
 输出        ：无                           
************************************************************************/
void WriteInUART4(char *p)
{
	u8 len=strlen(p);
	u8 i;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	for(i=0;i<len;i++)
	{
	  UART4_RX_BUF[i]=p[i];
	}	
}

/***********************************************************************
 函数名      ：LDUART4(void)  
 函数功能    ：联动测试函数
 输入        ：无
 输出        ：无                           
************************************************************************/
void LDUART4(void)
{
	//左翻
	WriteInUART4("BodyLeftUpPhone");		
	Fun_Body_Left();           //左翻身	
	delay_ms(1000);
	Fun_Back_Nursing_Left();   //左背部护理	
	delay_ms(1000);
	Fun_Back_Nursing_Left();   //左背部护理复位	
	delay_ms(1000);
	Fun_Waist_Nursing_Left();  //左腰部护理
	delay_ms(1000);
	Fun_Waist_Nursing_Left();  //左腰部护理复位	
	delay_ms(1000);
	WriteInUART4("BodyLeftDownPhone");
	Fun_Body_Left();           //左翻身复位
	delay_ms(1000);

	//右翻
	WriteInUART4("BodyRightUpPhone");
	Fun_Body_Right();          //右翻身
	delay_ms(1000);
	Fun_Back_Nursing_Right();  //右背部护理
	delay_ms(1000);
	Fun_Back_Nursing_Right();  //右背部护理复位
	delay_ms(1000);
	Fun_Waist_Nursing_Right(); //右腰部护理
	delay_ms(1000);
	Fun_Waist_Nursing_Right(); //右腰部护理复位
	delay_ms(1000);
	WriteInUART4("BodyRightDownPhone");
	Fun_Body_Right();          //右翻身复位
	delay_ms(1000);
	
	//上曲腿
	WriteInUART4("LegUpUpPhone");
	Fun_Leg_Up();              //上曲腿
	delay_ms(1000);
	WriteInUART4("LegUpDownPhone");
	Fun_Leg_Up();              //上曲腿复位
	delay_ms(1000);
	
	//形成坐姿-支背、下曲腿、坐便器、小桌子
	
	//支背
//	WriteInUART4("BackUpPhone");
//	Fun_Back();                //支背起
//	delay_ms(1000);
	
	//下曲腿
	WriteInUART4("LegDownDownPhone");
	Fun_Leg_Down();           //下曲腿	
	delay_ms(1000);
	
	//坐便器
	Washlet(0);               //坐便器打开
	delay_ms(1000);
	Washlet(1);               //坐便器关闭
	delay_ms(1000);
	
	//小桌子
	WriteInUART4("DeskUpPhone");
	Fun_Desk();               //小桌子靠近
	delay_ms(1000);
	WriteInUART4("DeskDownPhone");
	Fun_Desk();               //小桌子后退
	delay_ms(1000);	
	
	WriteInUART4("LegDownUpPhone");
	Fun_Leg_Down();           //下曲腿复位
	delay_ms(1000);
	
//	WriteInUART4("BackDownPhone");
//	Fun_Back();               //支背复位			
}

/***********************************************************************
 函数名      ：GB_Back(void)  
 函数功能    ：护栏支背按键联动
 输入        ：无
 输出        ：无
                           
************************************************************************/
void GB_Back(void)
{
	static u8 back_limit_flag; //判断支背是否运行到极限位置，若是发送极限位置图片
	u8 direct,key;
	u16 arr_now;               //当前一次运行脉冲数，用于脉冲累计 
	u8 len;	                   //接受的字符串长度
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //程序从break跳出标志位
	static u16 k=0;           //发送第k张图片指令
	u8 i=0;
	u16 j=0;	
	u16 arr_send;             //当前一次运行脉冲数
	static u8 kj;
	//联锁功能，只有在左右翻身功能复位后，才能进行支背
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		back_dir_flag=!back_dir_flag;
		if(back_dir_flag==1)
		{
			if(back_angle_to_arr(back_angle_lim)>back_runed_arr)  //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				direct=1;
				back_dir_flag=1;      
				if(back_flag==0)
				{
					back_flag=1;
					delay_ms(200);
					u2_printf("back_flag==1");
					delay_ms(200);
					u2_printf("BackStart");
					Wifi_Send("RunStart");
					delay_ms(200);
					u2_printf("Cartoon_Back_1");
					delay_ms(200);
				}
				Motor_1_START(1);                                                          //支背上行
				TIM10_Init(back_angle_to_arr(back_angle_lim)-back_runed_arr,timer10_freq); //打开定时器
			}			
		}
		if(back_dir_flag==0)
		{
			if(back_runed_arr>0)    //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				direct=0;
				back_dir_flag=0; 
				if(1==back_limit_flag)
				{
					back_limit_flag=0;
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
				}
				Motor_1_START(0);                        //支背下行
				TIM10_Init(back_runed_arr,timer10_freq); //打开定时器
			}
		}				
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);            //清除中断标志位	 	
		
		if(((back_runed_arr!=back_angle_to_arr(back_angle_lim))&&(1==direct))||((0!=back_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))     //等待定时时间到，时间到跳出循环 
			{			
				for(repeat_num=0;repeat_num<700;repeat_num++)
				{
					//停止指令
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"BackGB"))||(strstr((const char *)UART4_RX_BUF,(const char *)"Stop")))    //若接收到Stop,则跳出循环	
						{
							break_flag=1;
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
				}	
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送图片指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(back_runed_arr+arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				else
				{
					j=abs(back_runed_arr,arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}			
				k=back_picture_k;			
				if(	k!=j)
				{
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;	back_picture_k=k;
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
			Motor_1_STOP();     //电机停止
			TIM10_Stop();       //关闭定时器
			break_flag=0;	    //清除break标志位	
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //判断是否处于复位状态，复位状态的前提是下行的定时器走完
			{				
				arr_now=0;         //此时处于复位状态，将状态值都设为0；
				back_flag=0;
				delay_ms(200);
				u2_printf("back_flag==0");
			}		
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now				
				back_flag=1;							 
			}	
			//通过上下行判断脉冲累计
			if(	direct==1)        //如果是支背上行，则用+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{  
					back_runed_arr=back_angle_to_arr(back_angle_lim); 
					back_limit_flag=1; 
					delay_ms(200);
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
					Wifi_Send("RunLim"); 
					delay_ms(200);
					u2_printf("GuardbarBackLim");
				}
				else
				{  back_runed_arr=back_runed_arr+arr_now;	}				
			}
			else                //如果是支背下行，则用-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{	
					back_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Back_1");  
					delay_ms(200);
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("GuardbarBackRes");
				}
				else
				{
					back_runed_arr=back_runed_arr-arr_now;
				}						
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 								
		}
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("BackInterfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：GB_Body_Left(void)  
 函数功能    ：护栏左翻身
 输入        ：无
 输出        ：无
                           
************************************************************************/
void GB_Body_Left(void)
{
	u8 direct,key;
	u16 arr_now;               //当前一次运行脉冲数，用于脉冲累计
	u8 len;                    //接受的字符串长度
	u16 arr_feed;              //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;           // 将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较 
	u16 num1=0,num2=0,num3=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //程序从brak跳出标志位      
	static u8 M345_Start;     //345电机从初始位置运动
	static u8 M345_End;       //345电机运行到上极限位置标志位  
	static u16 k=0,m=0;
	u8 i=0;
	u8 mn;
	u8	kj;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;	 //当前一次运行脉冲数	
	
	//联锁功能，只有在支背、上下曲腿、座便、桌子复位后，才能执行左翻身功能
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{			
		if(body_left_flag==0)   //如果复位到初始状态，才执行小侧翻左翻起
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftGB"))
			{
			  //5号侧翻起
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;				
				delay_ms(200);
				u2_printf("GuardbarBodyLeftStart");
				delay_ms(200);
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				u2_printf("body_left_flag=1");
				delay_ms(200);				
				DIR5=1; 				
				body_left_flag=1;
				W25QXX_Write((u8*)&body_left_flag,33,1);				
				Motor_5_START(motor_body_freq,motor_timer_freq);	//电机启动	
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);   //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{	
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{				
						//光电限位
//						if((0==GD5_Left_End)&&(1==body_left_flag))  //运行时碰到光电开关，跳出循环 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;					
//							}								
//						}
						  //判断有没有收到上位机指令		
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
						//电机故障、故障诊断
//						if(1==Motor5_Alm)       
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("GuardbarBodyLeftOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;	
//							}								
//						}
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送图片指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					m=left_motor5_picture_m;
					if(m!=n)
					{
						mn=abs(m,n);					
						if(mn<2)
						{
							m=n;	left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}							
						}
					}					
				}					      
				Motor_5_STOP();       //电机停止
				TIM10_Stop();         //定时器关闭
				break_flag=0;  	      //清除break标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);	//调用补偿函数		
			}	
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;		
		//翻身345电机动作	
		if(1==body_left_flag)
		{
			body_left_dir_flag=!body_left_dir_flag;	
			if(body_left_dir_flag==1)    //左翻起
			{				
				if(body_angle_to_arr(body_left_angle_lim)>body_left_runed_arr)
				{
				   //345联动左翻起				
					 DIR3=0;DIR4=0;DIR5=0;direct=1;
					 if(M345_Start==0)
					 {
						 delay_ms(200);
						 M345_Start=1;
						 u2_printf("Cartoon_Body_Left_1");
						 delay_ms(200);
					 }								 
					 Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	  //电机启动			
					 TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);									 
				}
			}
			else          //左翻落
			{				
				if(body_left_runed_arr>0)
				{
				   //345联动左翻落
				   DIR3=1;DIR4=1;DIR5=1;direct=0;
					if(M345_End==1)
					 {
						 M345_End=0;
						 delay_ms(200);
						 u2_printf("Cartoon_Body_Left_8");	
						 delay_ms(200);
					 }											
				   Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	 //电机启动			
				   TIM10_Init(body_left_runed_arr,timer10_freq);	     //打开定时器		
				} 
			}
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位		
		if(((body_left_runed_arr!=body_angle_to_arr(body_left_angle_lim))&&(1==direct))||((0!=body_left_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //等待定时时间到，时间到跳出循环
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{
					//光电限位
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))         //如果碰到光电开关，跳出循环，停止运行
//					{						
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");
//							break_flag=1;
//							break;
//						}
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))   //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if(0==GD3_Left_End)
//						{
//							u2_printf("GD3LeftEnd");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Left_End)
//						{
//							u2_printf("GD4LeftEnd");
//							break_flag=1;
//							break;
//						}
//					}
					//停止指令
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftGB"))||(strstr((const char *)UART4_RX_BUF,(const char *)"Stop")))  //若接收到Stop,则跳出循环	
						{
							break_flag=1;
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
				
					//电机故障、故障诊断
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_left_overload_3=1;
//							u2_printf("GuardbarBodyLeftOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_left_overload_4=1;
//							u2_printf("GuardbarBodyLeftOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_left_overload_5=1;
//							u2_printf("GuardbarBodyLeftOverload5");
//						}						
//						Breakdown_Treatment();
//						break_flag=1;
//						break;		             
//					}						
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_left_runed_arr+arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				else
				{
					j=abs(body_left_runed_arr,arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				k=body_left_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;   body_left_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Left_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Left_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Left_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Left_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Left_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Left_7");
									break;		
						}
					}				
				}					
			}							
			Motor_3_4_5_STOP();   //电机停止
			TIM10_Stop();         //定时器关闭
			break_flag=0;         //清除break标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))  //判断是否到达复位状态
			{
				arr_now=0;                  
				body_left_flag=0;	
				W25QXX_Write((u8*)&body_left_flag,33,1);			
			}		
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);      //获取当前计数值arr_now
				body_left_flag=1;
			}
			//通过上下行判断脉冲累计
			if(direct==1)       //上行，则用+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Left_End)||(0==GD4_Left_End))            //向上运行到极限位置
				{
					body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
					M345_End=1;	 
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("GuardbarBodyLeftLim");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr+arr_now;
				}		
			}	
			else            //下行，则用-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))    //向下运行到极限位置
				{					
					body_left_runed_arr=0;
					M345_Start=0;	
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_1");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr-arr_now;	
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位
//  		//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_left_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //光电3没到位
//				{
//					DIR3=1;
//					Motor_3_START(motor_body_freq,motor_timer_freq);    //电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD3Start");			
//							break;																		
//						}
//					}			
//					Motor_3_STOP();       //电机停止
//					TIM10_Stop();         //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //光电4没到位
//				{
//					DIR4=1;
//					Motor_4_START(motor_body_freq,motor_timer_freq);   //电机启动
//					TIM10_Init(add_arr,timer10_freq);                  //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD4Start");
//							break;																		
//						}
//					}			
//					Motor_4_STOP();                                     //电机停止
//					TIM10_Stop();                                       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //光电3/4都没到位
//				{
//					DIR3=1; DIR4=1; DIR5=1;
//					Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);//电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //运行时碰到光电开关，跳出循环 
//						{
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//							break;
//							}																		
//						}
//					}			
//					Motor_3_4_5_STOP();   //电机停止
//					TIM10_Stop();         //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
//			}
			//侧翻复位
			if((body_left_flag==0)&&(0==direct))     //345联动复位到初始状态，才复位5号电机
			{			
				//5号侧翻复位
				Motor_4_Compensate(1,bodyleft_compleate,motor_body_freq,motor_timer_freq);
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				DIR5=0;
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq); //电机启动
				body_left_runed_arr=0;
				
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);   //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);                //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{
						//光电限位
//						if(((0==GD5_Start)&&(0==body_left_flag)))            //碰到光电开关跳出循环，电机停转 
//						{								
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;	
//							}								
//						}
						  //判断有没有收到上位机指令		
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
						//电机故障、故障诊断
						if(1==Motor5_Alm)         
						{						
							delay_us(100);
							if(1==Motor5_Alm)
							{
								body_left_overload_5=1;						
								u2_printf("GuardbarBodyLeftOverload5");
								Breakdown_Treatment();
								break_flag=1;
								break;	
							}								
						}						
					}	
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送动画指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					n=9-n;	m=left_motor5_picture_m;
					if(	m!=n)
					{			
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;    left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}	
						}				
					}	
				}      
				Motor_5_STOP();     //电机停止
				TIM10_Stop();       //定时器关闭
				break_flag=0;       //清除break标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");				
				delay_ms(200);
				u2_printf("body_left_flag==0");
				delay_ms(200);
				u2_printf("GuardbarBodyLeftRes");
				Wifi_Send("RunRes");
				//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=0;
//					Motor_5_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD5Start");
//							break;																		
//						}
//					}			
//					Motor_5_STOP();
//					TIM10_Stop();       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}			
			}				
		}
	}
	else
	{
		LED0=0;   //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("BodyLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：GB_Body_Right(void)  
 函数功能    ：护栏右翻身
 输入        ：无
 输出        ：无
                           
************************************************************************/
void GB_Body_Right(void)
{
	u8 direct,key;
	u16 arr_now;       //当前一次运行脉冲数，用于脉冲累计
	u8 len;            //接收的字符串长度
	u16 arr_feed;      //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;   //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num1=0,num2=0,num3=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //程序从break跳出标志位 
	static u16 k=0,m=0;
	static u8 M345R_Start;    //345电机从初始位置启动
	static u8 M345R_End;      //345电机运行到上极限位置
	u8 mn;
	u8 kj;
	u8 i=0;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;	  //当前一次运行脉冲数	
	
	//联锁功能，只有在支背、上下曲腿、座便、桌子复位后，才能执行右翻身功能
	if((lock_flag==1)&&(back_flag==0)&&(body_left_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_right_flag==0)&&(waist_nursing_right_flag==0))
	{			
		if(body_right_flag==0)   //如果复位到初始状态，才执行左翻起
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightGB"))
			{
			  //5号侧翻起
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				delay_ms(200);
				u2_printf("GuardbarBodyRightStart");
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("body_right_flag==1");				
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);
				DIR5=0; 
				body_right_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);				
				Motor_5_START(motor_body_freq,motor_timer_freq);	    //电机启动	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);    //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);                  //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{	
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{				
						//光电限位
//						if((0==GD5_Right_End)&&(1==body_right_flag))                      //运行时碰到光电开关，跳出循环 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Right_End)
//							{
//								break_flag=1;
//								u2_printf("GD5RightEnd");
//								break;		
//							}								
//						}
						  //判断有没有收到上位机指令		
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
						//电机故障、故障诊断
//						if(1==Motor5_Alm)         
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm) 
//							{
//								body_right_overload_5=1;
//								u2_printf("GuardbarBodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;		
//							}								
//						}					
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					} 
					//发送图片指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					m=right_motor5_picture_m;
					if(m!=n)
					{
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}	
						}					
					}		
				}						      
				Motor_5_STOP();        //电机停止
				TIM10_Stop();          //定时器关闭
				break_flag=0;          //清除break标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(1,bodyright_compleate,motor_body_freq,motor_timer_freq);//调用补偿函数
				}	
			}
			memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
			UART4_RX_LEN=0;			
			//翻身345电机动作	
			if(1==body_right_flag)
			{
				body_right_dir_flag=!body_right_dir_flag;
				if(body_right_dir_flag==1)
				{				
					if(body_angle_to_arr(body_right_angle_lim)>body_right_runed_arr)
					{
					   //345联动左翻起		
					   DIR3=1;DIR4=1;DIR5=1;direct=1;
					   if(M345R_Start==0)
					   {
						   delay_ms(200);
						   M345R_Start=1;	
						   u2_printf("Cartoon_Body_Right_1");
						   delay_ms(200);
					   }									 		
					   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq); //电机启动				
					   TIM10_Init(body_angle_to_arr(body_right_angle_lim)-body_right_runed_arr,timer10_freq);//打开定时器			
					}
				}
				else
				{				
					if(body_right_runed_arr>0)
					{
					   //345联动左翻起
					   DIR3=0;DIR4=0;DIR5=0;direct=0;
					   if(M345R_End==1)
						 {
							 M345R_End=0;
							 delay_ms(200);
							 u2_printf("Cartoon_Body_Right_8");	
							 delay_ms(200);
						 }								 	
					   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //电机启动			
					   TIM10_Init(body_right_runed_arr,timer10_freq);	      //打开定时器		
					}
				}
			}
			memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
			UART4_RX_LEN=0;
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //清除中断标志位		
			if(((body_right_runed_arr!=body_angle_to_arr(body_right_angle_lim))&&(1==direct))||((0!=body_right_runed_arr)&&(0==direct)))
			{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//光电限位
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //如果碰到光电开关，跳出循环，停止运行
//					{						
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");	
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");	
//							break_flag=1;
//							break;
//						}
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))    //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if(0==GD3_Left_End)
//						{
//							u2_printf("GD3LeftEnd");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Left_End)
//						{
//							u2_printf("GD4LeftEnd");
//							break_flag=1;
//							break;
//						}
//					}
					//停止指令
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightGB"))||(strstr((const char *)UART4_RX_BUF,(const char *)"Stop")) )    //若接收到Stop,则跳出循环	
						{
							break_flag=1;
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
			
					//电机故障、故障诊断
//					if((1==Motor3_Alm) || (1==Motor4_Alm) || (1==Motor5_Alm))       
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_right_overload_3=1;
//							u2_printf("GuardbarBodyRightOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_right_overload_4=1;
//							u2_printf("GuardbarBodyRightOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_right_overload_5=1;
//							u2_printf("GuardbarBodyRightOverload5");
//						}										
//						Breakdown_Treatment();
//						break_flag=1;
//						break;		             
//					}			
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_right_runed_arr+arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				else
				{
					j=abs(body_right_runed_arr,arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				k=body_right_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);							
					if(kj<2)
					{
						k=j;   body_right_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_7");
									break;	
						}
					}				
				}							
			}							
			Motor_3_4_5_STOP();   //电机停止
			TIM10_Stop();         //定时器关闭
			break_flag=0;	      //清除break标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))  //判断是否到达复位状态
			{
				arr_now=0;                   //此时arr_now为0
				body_right_flag=0;	
				W25QXX_Write((u8*)&body_right_flag,34,1);			
			}		
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);         //获取当前计数值arr_now
				body_right_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);	
			}
			//通过上下行判断脉冲累计
			if(direct==1)      //上行，则用+
			{ 
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Right_End)||(0==GD4_Right_End)) //向上运行到极限位置
				{
					body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
					M345R_End=1;   
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("GuardbarBodyRightLim");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr+arr_now;
				}		
			}	
			else              //下行，则用-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))   //向下运行到极限位置
				{					
					body_right_runed_arr=0;	
					M345R_Start=0;  
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr-arr_now;	
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);       //清除中断标志位
//  		//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_right_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //光电3没到位
//				{
//					DIR3=0;
//					Motor_3_START(motor_body_freq,motor_timer_freq);    //电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD3Start");			
//							break;																		
//						}
//					}			
//					Motor_3_STOP();     //电机停止
//					TIM10_Stop();       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //光电4没到位
//				{
//					DIR4=0;
//					Motor_4_START(motor_body_freq,motor_timer_freq);    //电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD4Start");	
//							break;																		
//						}
//					}			
//					Motor_4_STOP();     //电机停止
//					TIM10_Stop();       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //光电3/4都没到位
//				{
//					DIR3=0; DIR4=0; DIR5=0;
//					Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);//电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //运行时碰到光电开关，跳出循环 
//						{
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//							}
//							break;																		
//						}
//					}			
//					Motor_3_4_5_STOP(); //电机停止
//					TIM10_Stop();       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
//			}
			//侧翻复位
			if((body_right_flag==0)&&(0==direct))     //345联动复位到初始状态，才复位5号电机
			{			
				//5号侧翻复位
				Motor_4_Compensate(0,bodyright_compleate,motor_body_freq,motor_timer_freq);//调用补偿函数
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				DIR5=1;
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);             //电机启动
				body_right_runed_arr=0;
			
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);                              //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //等待定时时间到，
				{				
					for(repeat_num=0;repeat_num<600;repeat_num++)
					{			
						//光电限位
//						if((0==GD5_Start)&&(0==body_right_flag))       //运行时碰到光电开关，跳出循环 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break_flag=1;
//								break;	
//							}								
//						}
						  //判断有没有收到上位机指令		
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
						//电机故障、故障诊断
//						if(1==Motor5_Alm)
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("GuardbarBodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;		
//							}								
//						}				
					}				
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送动画指令
					 arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);			
					 n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					 n=9-n;
					 m=right_motor5_picture_m;
					 if(m!=n)
					 {	
						mn=abs(m,n);			
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}
						}				
					}	
				}      
				Motor_5_STOP();        //电机停止
				TIM10_Stop();		   //定时器关闭
				break_flag=0;          //清除break标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");				
			    delay_ms(200);
				u2_printf("body_right_flag==0");
				delay_ms(200);
				u2_printf("GuardbarBodyRightRes");
				Wifi_Send("RunRes");
				//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=1;
//					Motor_5_START(motor_body_freq,motor_timer_freq);   //电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							u2_printf("GD5Start");
//							break;																		
//						}
//					}			
//					Motor_5_STOP();        //电机停止
//					TIM10_Stop();		   //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}			
			}			
		}
	}

	else
	{
		LED0=0;   //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("BodyRightInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：GB_Back_Nursing(void)  
 函数功能    ：护栏背部护理
 输入        ：无
 输出        ：无
                           
************************************************************************/
void GB_Back_Nursing(void)
{
	if((lock_flag==1)&&(1==body_left_flag))  //如果此时处于左翻身，则进行左背部护理
	{
		Fun_Back_Nursing_Left(); 
	}
	if((lock_flag==1)&&(1==body_right_flag))//如果此时处于右翻身，则进行右背部护理
	{
		Fun_Back_Nursing_Right();
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

/***********************************************************************
 函数名      ：GB_Back_Nursing(void)  
 函数功能    ：护栏腰部护理
 输入        ：无
 输出        ：无
                           
************************************************************************/
void GB_Waist_Nursing(void)
{
	if((lock_flag==1)&&(1==body_left_flag))  //如果此时处于左翻身，则进行左腰部护理
	{
		Fun_Waist_Nursing_Left(); 
	}
	if((lock_flag==1)&&(1==body_right_flag))//如果此时处于右翻身，则进行右腰部护理
	{
		Fun_Waist_Nursing_Right();
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		LED0=1;
		LED1=1;	
		Wifi_Send("Interfere");
	}	
}

/***********************************************************************
 函数名      ：GB_Lock(void)  
 函数功能    ：一键锁定程序
 输入        ：无
 输出        ：无
                           
************************************************************************/
u8 GB_Lock(void)
{	
	lock_dir_flag=!lock_dir_flag;
	if(lock_dir_flag==1)
	{
		lock_flag=0;	//护栏锁定
		Wifi_Send("Fun_Lock");
		u2_printf("GuardbarLock");
	}
	else
	{
		lock_flag=1;   //护栏解锁
		Wifi_Send("Fun_Unlock");
		u2_printf("GuardbarUnLock");
	}
    return 	lock_flag;
}

/***********************************************************************
 函数名      ：Exp_Back(void)  
 函数功能    ：专家系统推理-支背
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Exp_Back(void) 
{
	//支背起
	WriteInUART4("BackUpPhone");
	Fun_Back();                //支背
	delay_ms(1000);	
	//支背复位	
	WriteInUART4("BackDownPhone");
	Fun_Back();                //支背复位	
}

/***********************************************************************
 函数名      ：Exp_Body(void)  
 函数功能    ：专家系统推理-翻身
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Exp_Body(void) 
{
	//左翻身
	WriteInUART4("BodyLeftUpPhone");		
	Fun_Body_Left();           //左翻身	
	delay_ms(1000);
	WriteInUART4("BodyLeftDownPhone");		
	Fun_Body_Left();           //左翻身复位
	delay_ms(1000);
			
	//右翻身
	WriteInUART4("BodyRightUpPhone");
	Fun_Body_Right();          //右翻身
	delay_ms(1000);
	WriteInUART4("BodyRightDownPhone");
	Fun_Body_Right();          //右翻身复位

}

/***********************************************************************
 函数名      ：Exp_Leg(void)  
 函数功能    ：专家系统推理-曲腿
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Exp_Leg(void) 
{
	//上曲腿
	WriteInUART4("LegUpUpPhone");
	Fun_Leg_Up();              //上曲腿
	delay_ms(1000);
	WriteInUART4("LegUpDownPhone");
	Fun_Leg_Up();              //上曲腿复位
	delay_ms(1000);	
	
	//下曲腿
	WriteInUART4("LegDownDownPhone");
	Fun_Leg_Down();            //下曲腿	
	delay_ms(1000);
	WriteInUART4("LegDownUpPhone");
	Fun_Leg_Down();            //下曲腿复位

}

/***********************************************************************
 函数名      ：Exp_Washlet_Auto(void)  
 函数功能    ：专家系统推理-自动坐便
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Exp_Washlet_Auto(void) 
{
	Washlet_Auto();	  //自动坐便
}


/***********************************************************************
 函数名      ：Auto_Arm_Leg_Left(void)  
 函数功能    ：执行左肢康复锻炼
 输入        ：t-康复锻炼次数
 输出        ：无
                           
************************************************************************/
void Auto_Arm_Leg_Left(int t)
{
   u32 pulse;          //吊挂运行脉冲数
   int j;	
	//病人只有平躺在床上才能进行吊挂康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   { 		
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		 if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftAuto"))    //左胳膊
		{
			pulse=2000000;
			if(leg_fore_left_flag==0)       //防止按键误触发，导致标志位置位
			{	
				arm_fore_left_flag=1;
				Wifi_Send("RunStart");
				u2_printf("ArmLeftAutoStart");
			}
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftAuto"))    //左腿
		{
			pulse=3000000;
			if(arm_fore_left_flag==0)      //防止按键误触发，导致标志位置位
			{	
				leg_fore_left_flag=1;
				Wifi_Send("RunStart");
				u2_printf("LegLeftAutoStart");
			}			
		}	        
		Auto_Hang_1_2(1,pulse);       //将左肢抬高
		delay_ms(1000);	     
		   
		for(j=0;j<t;j++)              //进行t此左肢康复训练
		{
			Auto_Hang_1(1,75000);     //向上运动
			delay_ms(1000);
			Auto_Hang_1(0,75000);     //向下运动
			delay_ms(1000);	
		}	
		Auto_Hang_1_2(0,pulse);       //将左肢放平到原来的位置
		//将标志位置位
		if((0==arm_left_runed)&&(1==arm_fore_left_flag))         //左胳膊已运行脉冲为零，复位到初始状态
		{
			arm_fore_left_flag=0;
			Wifi_Send("RunRes"); 
			u2_printf("ArmLeftAutoRes");
		}
		if((0==leg_left_runed)&&(1==leg_fore_left_flag))         //左腿已运行脉冲为零，复位到初始状态
		{
			leg_fore_left_flag=0;
			Wifi_Send("RunRes");
			u2_printf("LegLeftAutoRes");
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}		
}

/***********************************************************************
 函数名      ：Auto_Arm_Leg_Right(void)   
 函数功能    ：执行右肢吊挂康复训练
 输入        ：t吊挂次数
 输出        ：无 
                          
************************************************************************/
void Auto_Arm_Leg_Right(int t)
{
	u32 pulse;     //吊挂运行脉冲数
	//病人只有平躺在床上才能进行吊挂康复训练
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmRightAuto"))    //右胳膊
		{
			pulse=2000000;
			if(leg_fore_right_flag==0)       //防止按键误触发，导致标志位置位
			{	
				arm_fore_right_flag=1;
				Wifi_Send("RunStart");
				u2_printf("ArmRightAutoStart");				
			}			
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegRightAuto"))    //右腿
		{
			pulse=3000000;
			if(arm_fore_right_flag==0)       //防止按键误触发，导致标志位置位
			{	
				leg_fore_right_flag=1;
				Wifi_Send("RunStart");
				u2_printf("LegRightAutoStart");
			}			
		}
	
		int j;
		Auto_Hang_3_4(1,pulse);       //将右肢抬高
		delay_ms(1000);
		
		for(j=0;j<t;j++)              //进行t次右肢康复训练
		{		
			Auto_Hang_3(1,75000);     //向上运动		
			delay_ms(1000);		
			Auto_Hang_3(0,75000);     //向下运动
			delay_ms(1000);		
		}	
		Auto_Hang_3_4(0,pulse);       //将右肢放平到原来的位置
		//将标志位置位
		if((0==arm_right_runed)&&(1==arm_fore_right_flag))        //右胳膊已运行脉冲为零，复位到初始状态
		{
			arm_fore_right_flag=0;
			Wifi_Send("RunRes");
			u2_printf("ArmRightAutoRes");
		}
		if((0==leg_right_runed)&&(1==leg_fore_right_flag))       //右腿已运行脉冲为零，复位到初始状态
		{
			leg_fore_right_flag=0;
			Wifi_Send("RunRes");
			u2_printf("LegRightAutoRes");
		}	
			PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	 {
		LED0=0;                 //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");	
 		 
		LED0=1;
		LED1=1;
	 }	
}

/***********************************************************************
 函数名      ：Auto_Arm_Leg_Left_Right(void)   
 函数功能    ：执行左右肢吊挂康复训练
 输入        ：t吊挂次数
 输出        ：无  
                          
***********************************************************************/
void Auto_Arm_Leg_Left_Right(int t)
{
	u32 pulse;	         //吊挂运行脉冲数
	int j;
	//病人只有平躺在床上才能进行吊挂康复训练
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{		
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftRightAuto"))    //左右胳膊
		{
			pulse=2000000;
			if(leg_fore_left_right_flag==0)       //防止按键误触发，导致标志位置位
			{	
				arm_fore_left_right_flag=1;	
				Wifi_Send("RunStart");
				u2_printf("ArmLeftRightAutoStart");
			}
			else
			{
				arm_fore_left_right_flag=0;	
			}
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftRightAuto"))    //左右腿
		{
			pulse=3000000;
			if(arm_fore_left_right_flag==0)      //防止按键误触发，导致标志位置位
			{	
				leg_fore_left_right_flag=1;
				Wifi_Send("RunStart");
				u2_printf("LegLeftRightAutoStart");
			}
			else
			{
				leg_fore_left_right_flag=0;	
			}
		}	
		Auto_Hang_1_2_3_4(1,pulse);     //将左右肢抬高，小臂抬得比大臂高
		delay_ms(1000);	     
			   
		for(j=0;j<t;j++)                //进行t次康复训练
		{
			LED1=0;                     //在康复训练过程中LED0闪烁
			Auto_Hang_1_3(1,75000);     //小臂上升一定高度
			LED1=1;
			delay_ms(1000);
			LED1=0;
			Auto_Hang_1_3(0,75000);     //小臂下降一定高度
			delay_ms(1000);
			LED1=1;	
		}
			Auto_Hang_1_2_3_4(0,pulse); //将左右肢放平到原来的位置
		//将标志位置位
		if((0==arm_left_right_runed)&&(1==arm_fore_left_right_flag))  //左胳膊已运行脉冲为零，复位到初始状态
		{
			arm_fore_left_right_flag=0;
			Wifi_Send("RunRes"); 
			u2_printf("ArmLeftRightAutoRes");
		}
		if((0==leg_left_right_runed)&&(1==leg_fore_left_right_flag))  //左腿已运行脉冲为零，复位到初始状态
		{
			leg_fore_left_right_flag=0;
			Wifi_Send("RunRes"); 
			u2_printf("LegLeftRightAutoRes");
		}	
			PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;               //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Arm_Fore_Left(void)  
 函数功能    ：手动执行左小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Arm_Fore_Left(void)
{
	 u8 direct;   //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag=0;
	//连锁功能只有在键锁打开、支背、坐便、翻身复位的情况下才能进行小臂/小腿康复训练
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{		
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		if(0==arm_fore_left_flag)                    //先将左肢抬高到一定高度
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("ArmForeLeftHandStart");
				arm_fore_left_flag=1;
				Hand_Hang_1_2(1,arm_left_lim);       //将左肢抬高到一定高度，开始小臂康复训练
				runed_flag=1;
			}
		}				
		//左小臂向上运行
		if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftUpHand"))||(1==runed_flag))
		{	
			runed_flag=0;
			if(arm_fore_left_lim>arm_fore_left_runed)
			{
				Hand_Hang_1(1,arm_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==arm_fore_left_lim)
				{
					arm_fore_left_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmForeLeftHandLim");
				}
				else
				{
					arm_fore_left_flag=1;		
				}								
			}	
		}
		//左小臂向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftDownHand"))
		{	
			if(arm_fore_left_runed>0)
			{
				Hand_Hang_1(0,arm_fore_left_runed);					
			}
			if(0==arm_fore_left_runed)               //若小臂复位，则将胳膊复位
			{
				Hand_Hang_1_2(0,arm_left_runed);     //将左肢复位到水平状态 
			}
			if((0==arm_fore_left_runed)&&(0==arm_left_runed))
			{
				arm_fore_left_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForeLeftHandRes");
			}
		}	
			PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Leg_Fore_Left(void)  
 函数功能    ：手动执行左小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Leg_Fore_Left(void)
{
	 u8 direct;   //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag=0;
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {		
	   PCF8574_WriteBit(EXIO1,0);	       //继电器
		 if(0==leg_fore_left_flag)                    //先将左肢抬高到一定高度
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("LegForeLeftHandStart");
				leg_fore_left_flag=1;
				Hand_Hang_1_2(1,leg_left_lim);      //将左肢抬高到一定高度，开始小臂康复训练
				runed_flag=1;
			}
		}
		if((strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftUpHand"))||(1==runed_flag))
		{	
			runed_flag=0;
			if(leg_fore_left_lim>leg_fore_left_runed)
			{
				Hand_Hang_1(1,leg_fore_left_lim-leg_fore_left_runed);	
				if(leg_fore_left_runed==leg_fore_left_lim)
				{
					leg_fore_left_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegForeLeftHandLim");
				}
				else
				{
					leg_fore_left_flag=1;		
				}								
			}	
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftDownHand"))
		{	
			if(leg_fore_left_runed>0)
			{
				Hand_Hang_1(0,leg_fore_left_runed);					
			}
			if(0==leg_fore_left_runed)             //若小臂已运行脉冲为0，则将胳膊复位
			{
				Hand_Hang_1_2(0,leg_left_runed);   //将左肢复位到水平状态 
			}
			if((0==leg_fore_left_runed)&&(0==leg_left_runed))
			{
				leg_fore_left_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForeLeftHandRes");
			}			
		}	
			PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Arm_Fore_Right(void)  
 函数功能    ：手动执行右小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Arm_Fore_Right(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag=0;
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行小臂/小腿康复训练
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{		
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		if(0==arm_fore_right_flag)                    //先将右肢抬高到一定高度
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("ArmForeRightHandStart");
				arm_fore_right_flag=1;
				Hand_Hang_3_4(1,arm_right_lim);       //将右肢抬高到一定高度，开始小臂康复训练
				runed_flag=1;
			}
		}
		//右小臂向上运行
		if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightUpHand"))||(1==runed_flag))		
		{
			runed_flag=0;
			if(arm_fore_right_lim>arm_fore_right_runed)
			{
				Hand_Hang_3(1,arm_fore_right_lim-arm_fore_right_runed);	
				if(arm_fore_right_runed==arm_fore_right_lim)
				{
					arm_fore_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmForeRightHandLim");
				}
				else
				{
					arm_fore_right_flag=1;						
				}								
			}	
		}			
		//右小臂向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightDownHand"))
		{
			if(arm_fore_right_runed>0)    //右小臂向下运动
			{
				Hand_Hang_3(0,arm_fore_right_runed);					
			}
			if(arm_fore_right_runed==0)    //若右小臂复位，则将右胳膊复位到水平位置
			{
				Hand_Hang_3_4(0,arm_right_runed); 
			}
			if((arm_fore_right_runed==0)&&(0==arm_right_runed))   //标志位置位
			{
				arm_fore_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForeRightHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Leg_Fore_Right(void)  
 函数功能    ：手动执行右小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Leg_Fore_Right(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag;
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {		
	   PCF8574_WriteBit(EXIO1,0);	       //继电器
		 if(0==leg_fore_right_flag)                    //先将左肢抬高到一定高度
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("LegForeRightHandStart");
				leg_fore_right_flag=1;
				Hand_Hang_3_4(1,leg_right_lim);       //将左肢抬高到一定高度，开始小臂康复训练
				runed_flag=1;
			}
		}
		//右小腿上行
	   if((strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightUpHand"))||(runed_flag==1))
		{
			runed_flag=0;
			if(leg_fore_right_lim>leg_fore_right_runed)
			{
				Hand_Hang_3(1,leg_fore_right_lim-leg_fore_right_runed);	
				if(leg_fore_right_runed==leg_fore_right_lim)
				{
					leg_fore_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegForeRightHandLim");
				}
				else
				{
					leg_fore_right_flag=1;						
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightDownHand"))
		{
			if(leg_fore_right_runed>0)     //右小腿向下运行
			{
				Hand_Hang_3(0,leg_fore_right_runed);					
			}
			if(0==leg_fore_right_runed)    //若右小腿复位，则将左肢放平到水平位置
			{
				Hand_Hang_3_4(0,leg_right_runed);
			}
			if((0==leg_fore_right_runed)&&(0==leg_right_runed))  //标志位复位
			{
				leg_fore_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForeRightHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Arm_Post_Left(void)  
 函数功能    ：手动执行左大臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Arm_Post_Left(void)
{
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
	   PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //向上运行		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftUpHand"))
		{
			if(arm_post_left_lim>arm_post_left_runed)
			{
			    if(0==arm_post_left_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmPostLeftHandStart");
				}
				arm_post_left_flag=1;
				Hand_Hang_1_2(1,arm_post_left_lim-arm_post_left_runed);	
				if(arm_post_left_runed==arm_post_left_lim)
				{
					arm_post_left_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmPostLeftHandLim");
				}
				else
				{
					arm_post_left_flag=1;						
				}								
			}	
		}
         //向下运行		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftDownHand"))
		{
			if(arm_post_left_runed>0)
			{
				Hand_Hang_1_2(0,arm_post_left_runed);
				if(0==arm_post_left_runed)
				{					
					arm_post_left_flag=0;
					Wifi_Send("RunRes");
					u2_printf("ArmPostLeftHandRes");
				}
				else
				{
					arm_post_left_flag=1;			
				}					
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_ArmLeg_Post_Left(void)  
 函数功能    ：手动执行左大腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Leg_Post_Left(void)
{
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
	   PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //向上运行		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftUpHand"))
		{
			if(leg_post_left_lim>leg_post_left_runed)
			{
			    if(0==leg_post_left_flag)
				{	
					Wifi_Send("RunStart");
					u2_printf("LegPostLeftHandStart");
				}
				leg_post_left_flag=1;
				Hand_Hang_1_2(1,leg_post_left_lim-leg_post_left_runed);	
				if(leg_post_left_runed==leg_post_left_lim)
				{
					leg_post_left_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegPostLeftHandLim");
				}
				else
				{
					leg_post_left_flag=1;						
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftDownHand"))
		{
			if(leg_post_left_runed>0)
			{
				Hand_Hang_1_2(0,leg_post_left_runed);
				if(0==leg_post_left_runed)
				{					
					leg_post_left_flag=0;
					Wifi_Send("RunRes");
					u2_printf("LegPostLeftHandRes");
				}
				else
				{
					leg_post_left_flag=1;			
				}					
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Arm_Post_Right(void)  
 函数功能    ：手动执行右大臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Arm_Post_Right(void)
{
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //向上运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightUpHand"))
		{
			if(arm_post_right_lim>arm_post_right_runed)
			{
			    if(0==arm_post_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmPostRightHandStart");
				}
				arm_post_right_flag=1;
				Hand_Hang_3_4(1,arm_post_right_lim-arm_post_right_runed);	
				if(arm_post_right_runed==arm_post_right_lim)
				{
					arm_post_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmPostRightHandLim");
				}
				else
				{
					arm_post_right_flag=1;						
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightDownHand"))
		{
			if(arm_post_right_runed>0)
			{
				Hand_Hang_3_4(0,arm_post_right_runed);
				if(0==arm_post_right_runed)
				{
					arm_post_right_flag=0;
					Wifi_Send("RunRes");
					u2_printf("ArmPostRightHandRes");
				}
				else
				{
					arm_post_right_flag=1;			
				}					
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Leg_Post_Right(void)  
 函数功能    ：手动执行右大腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Leg_Post_Right(void)
{
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //同过上下行判断脉冲累计
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightUpHand"))
		{
			if(leg_post_right_lim>leg_post_right_runed)
			{
				if(0==leg_post_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("LegPostRightHandStart");
				}
				leg_post_right_flag=1;
				Hand_Hang_3_4(1,leg_post_right_lim-leg_post_right_runed);	
				if(leg_post_right_runed==leg_post_right_lim)
				{
					leg_post_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegPostRightHandLim");
				}
				else
				{
					leg_post_right_flag=1;					
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightDownHand"))
		{
			if(leg_post_right_runed>0)
			{
				Hand_Hang_3_4(0,leg_post_right_runed);
				if(0==leg_post_right_runed)
				{
					leg_post_right_flag=0;
					Wifi_Send("RunRes");
					u2_printf("LegPostRightHandRes");
				}
				else
				{
					leg_post_right_flag=1;
				}					
			}			
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Arm_Fore_Post_Left(void)  
 函数功能    ：手动执行左大小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Arm_Fore_Post_Left(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
       PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //向上运行		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftUpHand"))
		{
			 //先运行左大臂
			if(arm_post_left_lim>arm_post_left_runed)
			{
				if(0==arm_fore_post_left_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmForePostLeftHandStart");
				}
				arm_fore_post_left_flag=1;
				Hand_Hang_1_2(1,arm_post_left_lim-arm_post_left_runed);	
				if(arm_post_left_runed==arm_post_left_lim)
				{
					arm_fore_post_left_flag=1;
				}
				else
				{
					arm_fore_post_left_flag=1;						
				}								
			}
			//再运行左小臂
			if((arm_fore_left_lim>arm_fore_left_runed)&&(arm_post_left_runed==arm_post_left_lim))  
			{
				Hand_Hang_1(1,arm_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==arm_fore_left_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("ArmForePostLeftHandLim");
				}
			}			
		}	
		//向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftDownHand"))
		{
			//先运行左小臂
			if(arm_fore_left_runed>0)   
			{
				Hand_Hang_1(0,arm_fore_left_runed);					
			}
			//再运行左大臂
			if((arm_post_left_runed>0)&&(0==arm_fore_left_runed))
			{
				Hand_Hang_1_2(0,arm_post_left_runed);	
			}
			if((0==arm_post_left_runed)&&(0==arm_fore_left_runed))
			{
				arm_fore_post_left_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForePostLeftHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_ArmLeg_Fore_Post_Left(void)  
 函数功能    ：手动执行左大小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Leg_Fore_Post_Left(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
       PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //向上运行		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftUpHand"))
		{
			 //先运行左大腿
			if(leg_post_left_lim>leg_post_left_runed)
			{
				if(0==leg_fore_post_left_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("LegForePostLeftHandStart");
				}
				leg_fore_post_left_flag=1;
				Hand_Hang_1_2(1,leg_post_left_lim-leg_post_left_runed);	
				if(leg_post_left_runed==leg_post_left_lim)
				{
					leg_fore_post_left_flag=1;
				}
				else
				{
					leg_fore_post_left_flag=1;						
				}								
			}
			//再运行左小腿
			if((leg_fore_left_lim>leg_fore_left_runed)&&(leg_post_left_runed==leg_post_left_lim))  
			{
				Hand_Hang_1(1,leg_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==leg_fore_left_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("LegForePostLeftHandLim");
				}
			}							
		}
		//向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftDownHand"))
		{
			//先运行左小腿
			if(leg_fore_left_runed>0)   
			{
				Hand_Hang_1(0,leg_fore_left_runed);					
			}
			//再运行左大腿
			if((leg_post_left_runed>0)&&(0==leg_fore_left_runed))
			{
				Hand_Hang_1_2(0,leg_post_left_runed);	
			}			
			if((0==leg_post_left_runed)&&(0==leg_fore_left_runed))
			{
				leg_fore_post_left_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForePostLeftHandRes");
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Arm_Fore_Post_Right(void)  
 函数功能    ：手动执行右大小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Arm_Fore_Post_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //同过上下行判断脉冲累计
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightUpHand"))
		{
			 //先运行右大臂
			if(arm_post_right_lim>arm_post_right_runed)
			{
				if(0==arm_fore_post_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmForePostRightHandStart");
				}
				arm_fore_post_right_flag=1;
				Hand_Hang_3_4(1,arm_post_right_lim-arm_post_right_runed);	
				if(arm_post_right_runed==arm_post_right_lim)
				{
					arm_fore_post_right_flag=1;
				}
				else
				{
					arm_fore_post_right_flag=1;						
				}								
			}
			//再运行右小臂
			if((arm_fore_right_lim>arm_fore_right_runed)&&(arm_post_right_runed==arm_post_right_lim))  
			{
				Hand_Hang_3(1,arm_fore_right_lim-arm_fore_right_runed);	
				if(arm_fore_right_runed==arm_fore_right_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("ArmForePostRightHandLim");
				}
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightDownHand"))
		{
			//先运行右小臂
			if(arm_fore_right_runed>0)   
			{
				Hand_Hang_3(0,arm_fore_right_runed);					
			}
			//再运行右大臂
			if((arm_post_right_runed>0)&&(0==arm_fore_right_runed))
			{
				Hand_Hang_3_4(0,arm_post_right_runed);	
			}
			if((0==arm_post_right_runed)&&(0==arm_fore_right_runed))
			{
				arm_fore_post_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForePostRightHandRes");
			}			
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Leg_Fore_Post_Right(void)  
 函数功能    ：手动执行右大小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Leg_Fore_Post_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //同过上下行判断脉冲累计
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightUpHand"))
		{
			 //先运行右大腿
			if(leg_post_right_lim>leg_post_right_runed)
			{
				if(0==leg_fore_post_right_flag)
				{	
					Wifi_Send("RunStart");
					u2_printf("LegForePostRightHandStart");
				}
				leg_fore_post_right_flag=1;
				Hand_Hang_3_4(1,leg_post_right_lim-leg_post_right_runed);	
				if(leg_post_right_runed==leg_post_right_lim)
				{
					leg_fore_post_right_flag=1;
				}
				else
				{
					leg_fore_post_right_flag=1;						
				}								
			}
			//再运行右小腿
			if((leg_fore_right_lim>leg_fore_right_runed)&&(leg_post_right_runed==leg_post_right_lim))  
			{
				Hand_Hang_3(1,leg_fore_right_lim-leg_fore_right_runed);	
				if(leg_fore_right_runed==leg_fore_right_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("LegForePostRightHandLim");
				}
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightDownHand"))
		{
			//先运行右小腿
			if(leg_fore_right_runed>0)   
			{
				Hand_Hang_3(0,leg_fore_right_runed);					
			}
			//再运行右大腿
			if((leg_post_right_runed>0)&&(0==leg_fore_right_runed))
			{
				Hand_Hang_3_4(0,leg_post_right_runed);	
			}
			if((0==leg_post_right_runed)&&(0==leg_fore_right_runed))
			{
				leg_fore_post_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForePostRightHandRes");
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Hand_Arm_Fore_Left_Right(void)  
 函数功能    ：手动执行左右小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Arm_Fore_Left_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag;
	//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行左右小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {		
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		 if(0==arm_fore_left_right_flag)    //先将左右肢抬到一定高度
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("ArmForeLeftRightHandStart");
				arm_fore_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,arm_left_right_lim);
				runed_flag=1;
			}
		}
		//向上运行
		if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))||(1==runed_flag))
		{	
			if(arm_fore_left_right_lim>arm_fore_left_right_runed)
			{
				runed_flag=0;
				Hand_Hang_1_3(1,arm_fore_left_right_lim-arm_fore_left_right_runed);	
				if(arm_fore_left_right_runed==arm_fore_left_right_lim)
				{
					arm_fore_left_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmForeLeftRightHandLim");
				}
				else
				{
					arm_fore_left_right_flag=1;						
				}								
			}	
		}	
		//向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))
		{
			if(arm_fore_left_right_runed>0)      //左右小臂向下运行
			{
				Hand_Hang_1_3(0,arm_fore_left_right_runed);					
			}
			if(arm_fore_left_right_runed==0)     //若左右小臂复位，则将左右肢复位到水平位置
			{
				Hand_Hang_1_2_3_4(0,arm_left_right_runed);
			}
			if((arm_fore_left_right_runed==0)&&(0==arm_left_right_runed))  //标志位置位
			{
				arm_fore_left_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForeLeftRightHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 函数名      ：Hand_Leg_Fore_Left_Right(void)  
 函数功能    ：手动执行左右小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Leg_Fore_Left_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag;
	//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行左右小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {	    
	   PCF8574_WriteBit(EXIO1,0);	       //继电器
		 if(0==leg_fore_left_right_flag)
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("LegForeLeftRightHandStart");
				leg_fore_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,leg_left_right_lim);
				runed_flag=1;
			}				
		}
		//同过上下行判断脉冲累计
		if((strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightUpHand"))||(1==runed_flag))
		{
			if(leg_fore_left_right_lim>leg_fore_left_right_runed)
			{
				runed_flag=0;
				Hand_Hang_1_3(1,leg_fore_left_right_lim-leg_fore_left_right_runed);	
				if(leg_fore_left_right_runed==leg_fore_left_right_lim)
				{
					leg_fore_left_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegForeLeftRightHandLim");
				}
				else
				{
					leg_fore_left_right_flag=1;						
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightDownHand"))
		{
			if(leg_fore_left_right_runed>0)    //左右小腿向下运行
			{
				Hand_Hang_1_3(0,leg_fore_left_right_runed);					
			}
			if(leg_fore_left_right_runed==0)    //若左右小腿复位，则将左右腿复位到水平位置
			{
				Hand_Hang_1_2_3_4(0,leg_left_right_runed);
			}
			if((leg_fore_left_right_runed==0)&&(0==leg_left_right_runed))   //标志位置位
			{
				leg_fore_left_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForeLeftRightHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 函数名      ：Hand_Arm_Post_Left_Right(void)  
 函数功能    ：手动执行左右大臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Arm_Post_Left_Right(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行左右大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //同过上下行判断脉冲累计		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))
		{
			if(arm_post_left_right_lim>arm_post_left_right_runed)
			{
				if(0==arm_post_left_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmPostLeftRightHandStart");
				}
				arm_post_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,arm_post_left_right_lim-arm_post_left_right_runed);	
				if(arm_post_left_right_runed==arm_post_left_right_lim)
				{
					arm_post_left_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmPostLeftRightHandLim");
				}
				else
				{
					arm_post_left_right_flag=1;						
				}								
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))
		{
			if(arm_post_left_right_runed>0)
			{
				Hand_Hang_1_2_3_4(0,arm_post_left_right_runed);
				if(0==arm_post_left_right_runed)
				{
					arm_post_left_right_flag=0;
					Wifi_Send("RunRes");
					u2_printf("ArmPostLeftRightHandRes");
				}
				else
				{
					arm_post_left_right_flag=1;			
				}					
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 函数名      ：Hand_Leg_Post_Left_Right(void)  
 函数功能    ：手动执行左右大腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Leg_Post_Left_Right(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行左右大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {		
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //同过上下行判断脉冲累计		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightUpHand"))
		{
			if(leg_post_left_right_lim>leg_post_left_right_runed)
			{
				if(0==leg_post_left_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("LegPostLeftRightHandStart");
				}
				leg_post_left_right_flag=1;	
				Hand_Hang_1_2_3_4(1,leg_post_left_right_lim-leg_post_left_right_runed);	
				if(leg_post_left_right_runed==leg_post_left_right_lim)
				{
					leg_post_left_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegPostLeftRightHandLim");
				}
				else
				{
					leg_post_left_right_flag=1;							
				}								
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightDownHand"))
		{
			if(leg_post_left_right_runed>0)
			{
				Hand_Hang_1_2_3_4(0,leg_post_left_right_runed);
				if(0==leg_post_left_right_runed)
				{
					leg_post_left_right_flag=0;	
					Wifi_Send("RunRes");
					u2_printf("LegPostLeftRightHandRes");
				}
				else
				{
					leg_post_left_right_flag=1;			
				}					
			}			
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 函数名      ：Hand_Arm_Fore_Post_Left_Right(void)  
 函数功能    ：手动执行左右大小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Arm_Fore_Post_Left_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
     if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
     {
		PCF8574_WriteBit(EXIO1,0);	       //继电器
			 //向上运动
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))
		{
			 //先运行左右大臂
			if(arm_post_left_right_lim>arm_post_left_right_runed)
			{				
				if(0==arm_fore_post_left_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmForePostLeftRightHandStart");
				}
				arm_fore_post_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,arm_post_left_right_lim-arm_post_left_right_runed);	
				if(arm_post_left_right_runed==arm_post_left_right_lim)
				{
					arm_fore_post_left_right_flag=1;
				}
				else
				{
					arm_fore_post_left_right_flag=1;						
				}								
			}
			//再运行左右小臂
			if((arm_fore_left_right_lim>arm_fore_left_right_runed)&&(arm_post_left_right_runed==arm_post_left_right_lim))  
			{
				Hand_Hang_1_3(1,arm_fore_left_right_lim-arm_fore_left_right_runed);
				if(arm_fore_left_right_runed==arm_fore_left_right_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("ArmForePostLeftRightHandLim");
				}
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))
		{
			//先运行左右小臂
			if(arm_fore_left_right_runed>0)   
			{
				Hand_Hang_1_3(0,arm_fore_left_right_runed);					
			}
			//再运行左右大臂
			if((arm_post_left_right_runed>0)&&(0==arm_fore_left_right_runed))
			{
				Hand_Hang_1_2_3_4(0,arm_post_left_right_runed);	
			}
			if((0==arm_post_left_right_runed)&&(0==arm_fore_left_right_runed))
			{
				arm_fore_post_left_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForePostLeftRightHandRes");
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;	
	}	
}

/***********************************************************************
 函数名      ：Hand_Leg_Fore_Post_Left_Right(void)  
 函数功能    ：手动执行左右大小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Hand_Leg_Fore_Post_Left_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //继电器
		 //同过上下行判断脉冲累计
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))
		{
			 //先运行左右大腿
			if(leg_post_left_right_lim>leg_post_left_right_runed)
			{
				if(0==leg_fore_post_left_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("LegForePostLeftRightHandStart");
				}				
				leg_fore_post_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,leg_post_left_right_lim-leg_post_left_right_runed);	
				if(leg_post_left_right_runed==leg_post_left_right_lim)
				{
					leg_fore_post_left_right_flag=1;
				}
				else
				{
					leg_fore_post_left_right_flag=1;						
				}								
			}
			//再运行左右小腿
			if((leg_fore_left_right_lim>leg_fore_left_right_runed)&&(leg_post_left_right_runed==leg_post_left_right_lim))  
			{
				Hand_Hang_1_3(1,leg_fore_left_right_lim-leg_fore_left_right_runed);	
				if(leg_fore_left_right_runed==leg_fore_left_right_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("LegForePostLeftRightHandLim");
				}
			}
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightDownHand"))
		{
			//先运行左右小腿
			if(leg_fore_left_right_runed>0)   
			{
				Hand_Hang_1_3(0,leg_fore_left_right_runed);					
			}
			//再运行左右大腿
			if((leg_post_left_right_runed>0)&&(0==leg_fore_left_right_runed))
			{
				Hand_Hang_1_2_3_4(0,leg_post_left_right_runed);	
			}
			if((0==leg_post_left_right_runed)&&(0==leg_fore_left_right_runed))
			{
				leg_fore_post_left_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForePostLeftRightHandRes");
			}			
		}
		PCF8574_WriteBit(EXIO1,1);	       //继电器
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;	
	}	
}




/***********************************************************************
 函数名      ：Res_Power_Down(void)  
 函数功能    ：掉电复位,每个功能函数停止条件靠光电开关
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Res_Power_Down(void)
{
	if(lock_flag==1)
	{
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;
//		//读取翻身复位状态标志位
//		u8 	body_left_flag_buf[1];
//		u8 	body_right_flag_buf[1];
//		W25QXX_Read((u8*)body_left_flag_buf,33,1);        //从第33地址开始，读取1个字节
//		W25QXX_Read((u8*)body_right_flag_buf,34,1);       //从第34地址开始，读取1个字节
//		body_left_flag=body_left_flag_buf[0];
//		body_right_flag=body_right_flag_buf[0];
		
		if(1==body_left_flag)     //左翻复位
		{			
			Res_Body_Left();
		}		
		if(1==body_right_flag)    //右翻复位
		{
			Res_Body_Right();
		}
		delay_ms(1000);
		
		washlet_picture_k=24;
		Washlet(1);              //坐便器复位       
		delay_ms(1000);
	
		Res_Back();              //支背复位     
		delay_ms(1000);

		Res_Leg();               //曲腿复位
		delay_ms(1000);	
	
		Res_Desk();              //办公娱乐一体桌复位   
	}
}

/***********************************************************************
 函数名      ：Res_Back(void)  
 函数功能    ：支背复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Res_Back(void)
{
	back_runed_arr=back_angle_to_arr(back_angle_lim);
	back_picture_k=19;
	WriteInUART4("BackDownPhone");
	Fun_Back();                //支背复位		
}

/***********************************************************************
 函数名      ：Res_Leg(void)  
 函数功能    ：曲腿复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Res_Leg(void)
{
	leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim);  //下曲腿上行
	leg_down_picture_k=19;
	WriteInUART4("LegDownUpPhone");
	Fun_Leg_Down();           
	delay_ms(1000);	

	WriteInUART4("LegUpUpPhone");
	Fun_Leg_Up();              //上曲腿
	delay_ms(1000);
	WriteInUART4("LegUpDownPhone");
	Fun_Leg_Up();              //上曲腿复位
	delay_ms(1000);		
}

/***********************************************************************
 函数名      ：Res_Desk(void)  
 函数功能    ：就餐娱乐一体桌复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Res_Desk(void)
{
	desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
	desk_picture_k=19;
	WriteInUART4("DeskDownPhone");
	Fun_Desk();             //小桌子复位
	delay_ms(1000);	
}

	
/***********************************************************************
 函数名      ：Res_Motor5(void)  
 函数功能    ：5号电机复位:
 输入        ：反向复位0;正向复位1
 输出        ：无
                           
************************************************************************/
void Res_Motor5(u8 dir) 
{
	u8 key;
	u16 arr_feed;      //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;   //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	DIR5=dir;  
	u8 direct=0;       
	Motor_5_START(motor_body_freq,motor_timer_freq);	//电机启动	
	TIM10_Init(20000,timer10_freq);                     //打开定时器
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
	while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
	{				
		//光电限位
		if((0==GD5_Start)&&(0==direct))   //碰到光电开关跳出循环，电机停转 
		{		
			delay_us(100);
			if(0==GD5_Start)
			{
				break;		
			}				
		}
		//电机过载
//		if(1==Motor5_Alm)      
//		{
//			delay_us(100);
//			if(1==Motor5_Alm)
//			{
//				if(body_left_flag==1)
//				{
//					body_left_overload_5=1;
//					u2_printf("Reset_Body_Left_Overload_5");
//				}
//				if(body_right_flag==1)
//				{
//					body_right_overload_5=1;
//					u2_printf("Reset_Body_Right_Overload_5");
//				}
//				Breakdown_Treatment();
//				break;
//			}			
//		}
		
//		//电机失步或异常停止
//		key=KEY_Scan(0);              //按键扫描
//		if(key==Motor5_Tim_PRES)
//		{
//			num++;
//		}	
//		arr_feed=__HAL_TIM_GET_COUNTER(&TIM10_Handler); 
//		pulse_num=arr_feed/7.44*timer10_freq*0.36/(motor_body_freq*motor_timer_freq);						

//		if((abs(pulse_num,num)>150)&&pulse_num>0)
//		{
//			if(body_left_flag==1)
//			{
//				body_left_losepulse=1;
//				u2_printf("Reset_Body_Left_Losepulse");
//			}
//			if(body_right_flag==1)
//			{
//				body_right_losepulse=1;
//				u2_printf("Reset_Body_Right_Losepulse");
//			}				
//			Breakdown_Treatment();
//			break;
//		}		
	}		      
	Motor_5_STOP();     //电机停止
	TIM10_Stop();       //关闭定时器
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
	//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段）
//	if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD5_Start==1)&&(direct==0))
//	{  
//		DIR5=dir;
//		Motor_5_START(motor_body_freq,motor_timer_freq);    //电机启动
//		TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//		{			
//			if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//			{
//				break;																		
//			}
//		}			
//		Motor_5_STOP();     //电机停止
//	    TIM10_Stop();       //关闭定时器
//		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//	}	
}


/***********************************************************************
 函数名      ：Res_Body_Left(void)  
 函数功能    ：左翻身复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Res_Body_Left(void)
{	
	if(lock_flag==1)
	{	
		u2_printf("ResetBodyLeftStart");
		//4号电机复位
		lock_flag==1;
		body_left_flag=1;
		back_nursing_left_flag==0;
		waist_nursing_left_dir_flag=1;	
		waist_nursing_left_picture_k=19;
		body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
		Fun_Waist_Nursing_Left();    //左腰部护理复位
		
		//3号电机复位
		lock_flag=1;
		body_left_flag=1;
		waist_nursing_left_flag=0;
		back_nursing_left_dir_flag=1;
		back_nursing_left_picture_k=19;
		body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
		Fun_Back_Nursing_Left();     //左背部护理复位
		
		//5号电机复位
		Res_Motor5(0); 
		body_left_runed_arr=0;
		body_left_flag=0;
		back_nursing_left_flag=0;
		waist_nursing_left_flag=0;
		back_nursing_left_dir_flag=0;
		waist_nursing_left_dir_flag=0;
		u2_printf("ResetBodyLeftRes");
	}
}

/***********************************************************************
 函数名      ：Res_Body_Right(void)  
 函数功能    ：右翻身复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Res_Body_Right(void)
{
	if(lock_flag==1)
	{	
		u2_printf("ResetBodyRightStart");
		//4号电机复位
		lock_flag==1;
		body_right_flag=1;
		back_nursing_right_flag==0;
		waist_nursing_right_dir_flag=1;	
		waist_nursing_right_picture_k=19;
		body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
		Fun_Waist_Nursing_Right();   //右腰部护理复位
		
		//3号电机复位
		lock_flag=1;
		body_right_flag=1;
		waist_nursing_right_flag=0;
		back_nursing_right_dir_flag=1;
		back_nursing_right_picture_k=19;
		body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
		Fun_Back_Nursing_Right();    //右背部护理复位
		
		//5号电机复位 
		Res_Motor5(1);
		body_right_runed_arr=0;
		body_right_flag=0;
		back_nursing_right_flag=0;
		waist_nursing_right_flag=0;
		back_nursing_right_dir_flag=0;
		waist_nursing_right_dir_flag=0;
		u2_printf("ResetBodyRightRes");
	}
}


/***********************************************************************
 函数名      ：back_angle_to_arr  
 函数功能    ：将角度值转换成自动重装载值
 输入        ：角度，u8类型
 输出        ：TIM3自动重装载值，u16类型
                           
************************************************************************/
u16 back_angle_to_arr(u8 angle)   //支背
{
	u16 res;
	res=560*angle;//600
	return res;
}

u16 leg_angle_to_arr(u8 angle)   //曲腿
{
	u16 res;
	res=380*angle;//400
	return res;
}

u16 body_angle_to_arr(u8 angle)  //翻身
{
	u16 res;
	res=300*angle;//280
	return res;
}

u16 desk_distance_to_arr(u8 distance)   //桌子
{
	u16 res;
	res=655*distance;//600
	return res;
}

/***********************************************************************
 函数名      ：get_newangle_usart2
 函数功能    ：通过串口2获得新的角度值
 输入        ：无
 输出        ：无 
 扫描数组     ：USART2_RX_BUF，即串口2
 其他说明        ：将“+”与“+”字符之间的数据提取出来，并存入全局的角度变量
 举例        ：angleresetall+80+40+90+45+45+100+
               支背角度（back_angle_lim）=80，上曲腿角度（leg_up_angle_lim）=40，下曲腿角度（leg_down_angle_lim）=90；
               左翻角度（body_left_angle_lim）=45，右翻角度（body_right_angle_lim）=45，小桌子移动距离（desk_distance_lim）=100；
************************************************************************/
void get_newangle_usart2(void)
{
	u8 ip1[10],ip2[10],ip3[10],ip4[10],ip5[10],ip6[10],ip7[10];
	u8 	back_angle_lim_buf[1];
	u8  leg_up_angle_lim_buf[1];
	u8  leg_down_angle_lim_buf[1];
	u8  body_left_angle_lim_buf[1];
	u8  body_right_angle_lim_buf[1];
	u8  desk_distance_lim_buf[1];
	u8  swash_dry_time_buf[1];
	
	char *presult1,*presult2,*presult3,*presult4,*presult5,*presult6,*presult7,*presult8;
	
	presult1 = strstr( (const char *)USART2_RX_BUF ,(const char *)"+");
	presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	presult3 = strstr( (const char *)presult2+1 , (const char *)"+");		
	presult4 = strstr( (const char *)presult3+1 , (const char *)"+");			
	presult5 = strstr( (const char *)presult4+1 , (const char *)"+");		
 	presult6 = strstr( (const char *)presult5+1 , (const char *)"+");
	presult7 = strstr( (const char *)presult6+1 , (const char *)"+");
	presult8 = strstr( (const char *)presult7+1 , (const char *)"+");
	
 //将presult1+1到presult2-presult1-1之间的数据复制到ip1数组中
	memcpy(ip1,presult1+1,presult2-presult1-1);   
	memcpy(ip2,presult2+1,presult3-presult2-1);
	memcpy(ip3,presult3+1,presult4-presult3-1);	
	memcpy(ip4,presult4+1,presult5-presult4-1);		
	memcpy(ip5,presult5+1,presult6-presult5-1);	
	memcpy(ip6,presult6+1,presult7-presult6-1);
	memcpy(ip7,presult7+1,presult8-presult7-1);

	back_angle_lim_buf[0]=usmart_strnum(ip1);              //支背  //将ip1数组中的数字型字符串转换成数字
	leg_up_angle_lim_buf[0]=usmart_strnum(ip2);            //上曲腿
	leg_down_angle_lim_buf[0]=usmart_strnum(ip3);          //下曲腿
	body_left_angle_lim_buf[0]=usmart_strnum(ip4);         //左翻
	body_right_angle_lim_buf[0]=usmart_strnum(ip5);        //右翻
	desk_distance_lim_buf[0]=usmart_strnum(ip6);           //桌子移动距离
	swash_dry_time_buf[0]=usmart_strnum(ip7);              //冲洗烘干时间	

	if(back_angle_lim_buf[0]<=90)             //支背
	{	back_angle_lim=back_angle_lim_buf[0];	}
	
	if(leg_up_angle_lim_buf[0]<=40)  	     //上曲腿
	{	leg_up_angle_lim=leg_up_angle_lim_buf[0]; 	} 
	
	if(leg_down_angle_lim_buf[0]<=90)	      //下曲腿
	{	leg_down_angle_lim=leg_down_angle_lim_buf[0]; 	}

	if(body_left_angle_lim_buf[0]<=90)	       //左翻
	{	body_left_angle_lim=body_left_angle_lim_buf[0];	}  
	
	if(body_left_angle_lim_buf[0]<=90)	       //右翻	
	{	body_right_angle_lim=body_right_angle_lim_buf[0]; 	} 

	if(desk_distance_lim_buf[0]<=100)          //桌子移动距离   
	{	desk_distance_lim=desk_distance_lim_buf[0];		}

	if(swash_dry_time_buf[0]<=5)             //冲洗烘干时间  
	{	swash_dry_time=swash_dry_time_buf[0];		}
 	

//	//保存最新上位机设定脉冲值
//	W25QXX_Write((u8*)back_angle_lim_buf,13,1);         //从第13地址开始，写入1个字节
//	W25QXX_Write((u8*)leg_up_angle_lim_buf,14,1);		//从第14地址开始，写入1个字节
//	W25QXX_Write((u8*)leg_down_angle_lim_buf,15,1);		//从第15地址开始，写入1个字节
//	W25QXX_Write((u8*)body_left_angle_lim_buf,16,1);	//从第16地址开始，写入1个字节
//	W25QXX_Write((u8*)body_right_angle_lim_buf,17,1);	//从第17地址开始，写入1个字节
//	W25QXX_Write((u8*)desk_distance_lim_buf,18,1);		//从第18地址开始，写入1个字节
//	W25QXX_Write((u8*)swash_dry_time_buf,19,1);		    //从第19地址开始，写入1个字节	
}

/***********************************************************************
 函数名      ：get_newangle_wifi()
 函数功能    ：通过WiFi获得新的角度值
 输入        ：无
 输出        ：无 
 扫描数组     ：UART4_RX_BUF，即串口4
 其他说明        ：将“+”与“+”字符之间的数据提取出来，并存入全局的角度变量
 举例        ：angleresetall+80+40+90+45+45+100+
               支背角度  （back_angle_lim）=80，
			   上曲腿角度（leg_up_angle_lim）=40，
			   下曲腿角度（leg_down_angle_lim）=90；
               左翻角度  （body_left_angle_lim）=45，
			   右翻角度  （body_right_angle_lim）=45，
			   小桌子移动距离（desk_distance_lim）=100；
				冲洗烘干时间 
		
************************************************************************/
void get_newangle_wifi(void)
{
	u8 ip1[10],ip2[10],ip3[10],ip4[10],ip5[10],ip6[10],ip7[10];
	
	u8 	back_angle_lim_buf[1];
	u8  leg_up_angle_lim_buf[1];
	u8  leg_down_angle_lim_buf[1];
	u8  body_left_angle_lim_buf[1];
	u8  body_right_angle_lim_buf[1];
	u8  desk_distance_lim_buf[1];
	u8  swash_dry_time_buf[1];
	
	char *presult1,*presult2,*presult3,*presult4,*presult5,*presult6,*presult7,*presult8;
	presult1 = strstr( (const char *)UART4_RX_BUF ,(const char *)"+");
	presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	presult3 = strstr( (const char *)presult2+1 , (const char *)"+");		
	presult4 = strstr( (const char *)presult3+1 , (const char *)"+");			
	presult5 = strstr( (const char *)presult4+1 , (const char *)"+");		
 	presult6 = strstr( (const char *)presult5+1 , (const char *)"+");
	presult7 = strstr( (const char *)presult6+1 , (const char *)"+");	
	presult8 = strstr( (const char *)presult7+1 , (const char *)"+");
	
	//将presult1+1到presult2-presult1-1之间的数据复制到ip1数组中
	memcpy(ip1,presult1+1,presult2-presult1-1);    
	memcpy(ip2,presult2+1,presult3-presult2-1);
	memcpy(ip3,presult3+1,presult4-presult3-1);	
	memcpy(ip4,presult4+1,presult5-presult4-1);		
	memcpy(ip5,presult5+1,presult6-presult5-1);	
	memcpy(ip6,presult6+1,presult7-presult6-1);	
	memcpy(ip7,presult7+1,presult8-presult7-1);
	
	//将ip1数组中的数字型字符串转换成数字
	back_angle_lim_buf[0]=usmart_strnum(ip1);              //支背  
	leg_up_angle_lim_buf[0]=usmart_strnum(ip2);            //上曲腿
	leg_down_angle_lim_buf[0]=usmart_strnum(ip3);          //下曲腿
	body_left_angle_lim_buf[0]=usmart_strnum(ip4);         //左翻
	body_right_angle_lim_buf[0]=usmart_strnum(ip5);        //右翻
	desk_distance_lim_buf[0]=usmart_strnum(ip6);           //桌子移动距离
	swash_dry_time_buf[0]=usmart_strnum(ip7);              //冲洗烘干时间
	
	if(back_angle_lim_buf[0]<=90)             //支背
	{	back_angle_lim=back_angle_lim_buf[0];	}
	
	if(leg_up_angle_lim_buf[0]<=40)  	     //上曲腿
	{	leg_up_angle_lim=leg_up_angle_lim_buf[0]; 	} 
	
	if(leg_down_angle_lim_buf[0]<=90)	      //下曲腿
	{	leg_down_angle_lim=leg_down_angle_lim_buf[0]; 	}

	if(body_left_angle_lim_buf[0]<=90)	       //左翻
	{	body_left_angle_lim=body_left_angle_lim_buf[0];	}  
	
	if(body_left_angle_lim_buf[0]<=90)	       //右翻	
	{	body_right_angle_lim=body_right_angle_lim_buf[0]; 	} 

	if(desk_distance_lim_buf[0]<=100)         //桌子移动距离   
	{	desk_distance_lim=desk_distance_lim_buf[0];		} 

	if(swash_dry_time_buf[0]<=5)             //冲洗烘干时间  
	{	swash_dry_time=swash_dry_time_buf[0];		}	
	
//	//保存最新上位机设定脉冲值	
//	W25QXX_Write((u8*)back_angle_lim_buf,13,1);         //从第13地址开始，写入1个字节
//	W25QXX_Write((u8*)leg_up_angle_lim_buf,14,1);	    //从第14地址开始，写入1个字节
//	W25QXX_Write((u8*)leg_down_angle_lim_buf,15,1);		//从第15地址开始，写入1个字节
//	W25QXX_Write((u8*)body_left_angle_lim_buf,16,1);	//从第16地址开始，写入1个字节
//	W25QXX_Write((u8*)body_right_angle_lim_buf,17,1);	//从第17地址开始，写入1个字节
//	W25QXX_Write((u8*)desk_distance_lim_buf,18,1);		//从第18地址开始，写入1个字节
//	W25QXX_Write((u8*)swash_dry_time_buf,19,1);		    //从第19地址开始，写入1个字节	
}

/***********************************************************************
 函数名      ：Read_Angle  
 函数功能    ：读取flash保存的上位机设定角度值
 输入        ：角度，u8类型
 输出        ：TIM3自动重装载值，u16类型
                           
************************************************************************/
void Read_Angle(void)
{
	u8 	back_angle_lim_buf[1];
	u8  leg_up_angle_lim_buf[1];
	u8  leg_down_angle_lim_buf[1];
	u8  body_left_angle_lim_buf[1];
	u8  body_right_angle_lim_buf[1];
	u8  desk_distance_lim_buf[1];
	u8  swash_dry_time_buf[1];
	
	//将保存的数据从flash中读出来
	W25QXX_Read((u8*)back_angle_lim_buf,13,1);        //从第13地址开始，读取1个字节
	W25QXX_Read((u8*)leg_up_angle_lim_buf,14,1);	  //从第14地址开始，读取1个字节
	W25QXX_Read((u8*)leg_down_angle_lim_buf,15,1);	  //从第15地址开始，读取1个字节
	W25QXX_Read((u8*)body_left_angle_lim_buf,16,1);	  //从第16地址开始，读取1个字节
	W25QXX_Read((u8*)body_right_angle_lim_buf,17,1);  //从第17地址开始，读取1个字节
	W25QXX_Read((u8*)desk_distance_lim_buf,18,1);	  //从第18地址开始，读取1个字节
	W25QXX_Read((u8*)swash_dry_time_buf,19,1);		  //从第19地址开始，读取1个字节

	back_angle_lim=back_angle_lim_buf[0];              //支背  
	leg_up_angle_lim=leg_up_angle_lim_buf[0];          //上曲腿
	leg_down_angle_lim=leg_down_angle_lim_buf[0];      //下曲腿
	body_left_angle_lim=body_left_angle_lim_buf[0];    //左翻
	body_right_angle_lim=body_right_angle_lim_buf[0];  //右翻
	desk_distance_lim=desk_distance_lim_buf[0];        //桌子移动距离
	swash_dry_time=swash_dry_time_buf[0];              //坐便冲洗烘干时间	
}

/***********************************************************************
 函数名      ：Wifi_Send  
 函数功能    ：WiFi发送函数
 输入        ：要发送的数据或指令
 输出        ：无
                           
************************************************************************/
void Wifi_Send(u8 *data)
{
	u16 i;
	u8 n;	
	i=strlen((const char*)data);                    //获取数据长度	
	if(device_num>0)
	{
		u4_printf("AT+CIPSEND=%d,%d\r\n",0,i);      //发送AT+CIPSEND指令
		delay_ms(4);                         
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	//清除串口4接收缓存区	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);                       //发送数据字符数据data
		delay_ms(200);								//延时200ms，芯片必须要求这个时间
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	//清除串口4接收缓存区	
		UART4_RX_LEN=0;  				
	}
}

/***********************************************************************
 函数名      ：Wifi_ToPC  
 函数功能    ：WiFi向PC发送函数
 输入        ：要发送的数据或指令
 输出        ：无                      
************************************************************************/
void Wifi_ToPC(u8 *data)
{
	u16 i;
	u8 n;	
	if(PC_Ready==1)
	{
		i=strlen((const char*)data);                //获取数据长度	
		u4_printf("AT+CIPSEND=%d,%d\r\n",PC,i);     //发送AT+CIPSEND指令
		delay_ms(4);
		memset(UART4_RX_BUF,0,20);	//清除串口4接收缓存区	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);       //发送数据字符数据data
		delay_ms(200);				//延时200ms，芯片必须要求这个时间
		memset(UART4_RX_BUF,0,20);	//清除串口4接收缓存区	
		UART4_RX_LEN=0;  		
	}
}

/***********************************************************************
 函数名      ：Wifi_ToPhone  
 函数功能    ：WiFi向手机发送函数
 输入        ：要发送的数据或指令
 输出        ：无
************************************************************************/
void Wifi_ToPhone(u8 *data)
{
	u16 i;
	u8 n;	
	if(Phone_Ready==1)
	{
		i=strlen((const char*)data);                //获取数据长度	
		u4_printf("AT+CIPSEND=%d,%d\r\n",Phone,i);  //发送AT+CIPSEND指令
		delay_ms(4);
		memset(UART4_RX_BUF,0,20);	//清除串口4接收缓存区	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);       //发送数据字符数据data
		delay_ms(200);				//延时200ms，芯片必须要求这个时间
		memset(UART4_RX_BUF,0,20);	//清除串口4接收缓存区	
		UART4_RX_LEN=0;  		
	}
}


/***********************************************************************
 函数名      ：Wifi_ToRemote  
 函数功能    ：WiFi向遥控器发送函数
 输入        ：要发送的数据或指令
 输出        ：无
************************************************************************/
void Wifi_ToRemote(u8 *data)
{
	u16 i;
	u8 n;	
	if(RemoteCtl_Ready==1)
	{
		i=strlen((const char*)data);                        //获取数据长度	
		u4_printf("AT+CIPSEND=%d,%d\r\n",Remote_Ctl,i);     //发送AT+CIPSEND指令
		delay_ms(4);
		memset(UART4_RX_BUF,0,20);	//清除串口4接收缓存区	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);       //发送数据字符数据data
		delay_ms(200);				//延时200ms，芯片必须要求这个时间
		memset(UART4_RX_BUF,0,20);	//清除串口4接收缓存区	
		UART4_RX_LEN=0; 		
	}
}
/***********************************************************************
 函数名      ：Wifi_ToGuard  
 函数功能    ：WiFi向护栏发送函数
 输入        ：要发送的数据或指令
 输出        ：无
************************************************************************/
void Wifi_ToGuard(u8 *data)
{
	u16 i;
	u8 n;	
	if(GuardCtl_Ready==1)
	{
		i=strlen((const char*)data);                       //获取数据长度	
		u4_printf("AT+CIPSEND=%d,%d\r\n",Guard_Ctl,i);     //发送AT+CIPSEND指令
		delay_ms(4);
		memset(UART4_RX_BUF,0,20);	 //清除串口4接收缓存区	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);        //发送数据字符数据data
		delay_ms(200);				 //延时200ms，芯片必须要求这个时间
		memset(UART4_RX_BUF,0,20);	 //清除串口4接收缓存区	
		UART4_RX_LEN=0; 		
	}
}

/***********************************************************************
 函数名      ：Uart_ToStick  
 函数功能    ：串口向PC棒发送函数
 输入        ：要发送的数据或指令
 输出        ：无
************************************************************************/
void Uart_ToStick(u8 *data)
{
	if(ComputerStick_Ready==1)
	{                             
		u2_printf("%s",data);	
		memset(USART2_RX_BUF,0,20);
		USART2_RX_LEN=0; 		
	}
}

/***********************************************************************
                    
					     串口函数(服务器)
					
************************************************************************/
/***********************************************************************
 函数名      ：Uart_Back(void)   
 函数功能    ：执行支背操作
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Back(void)
{
	u8 direct,len;
	u16 arr_now;              //当前一次运行脉冲数
	
//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;          //判断程序是否是从break跳出来 
	u16 repeat_num;
	static u8 k;             //传第k张动画
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //当前一次运行脉冲值
	static u8 back_limit_flag; //支背运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位后，才能进行支背
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		if(strstr((const char *)USART2_RX_BUF,(const char *)"BackUpPhone"))
		{
			if(back_angle_to_arr(back_angle_lim)>back_runed_arr)  //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{				
				direct=1;
				back_dir_flag=1;      
				if(back_flag==0)
				{
					back_flag=1;
					delay_ms(400);
					u2_printf("back_flag==1");
					delay_ms(400);
					u2_printf("BackStart");
					delay_ms(400);
					u2_printf("Cartoon_Back_1");
					delay_ms(400);
				}
				Motor_1_START(1);                                                          //支背上行
				TIM10_Init(back_angle_to_arr(back_angle_lim)-back_runed_arr,timer10_freq); //打开定时器
			}				
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"BackDownPhone"))
		{
			if(back_runed_arr>0)    //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				direct=0;
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
		
	 	if(((back_runed_arr!=back_angle_to_arr(back_angle_lim))&&(1==direct))||((0!=back_runed_arr)&&(0==direct)))
		{					
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环   
			{							
				for(repeat_num=0;repeat_num<700;repeat_num++)
				{			
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BackUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"BackDownPhone")))    //若接收到Stop,则跳出循环	
						{					
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;						
						}
						else 
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);      //当前一次脉冲值
				//传输动画指令
				if(direct==1)
				{
					j=(back_runed_arr+arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				else
				{
					j=abs(back_runed_arr,arr_send)/(back_angle_to_arr(back_angle_max)/19);
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
		break_flag=0;      //标志位清零
		//判断复位
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //判断是否处于复位状态，复位状态的前提是下行的定时器走完
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
		if(	direct==1)        //如果是支背上行，则用+
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{  
				back_runed_arr=back_angle_to_arr(back_angle_lim);
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
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("BackInterfere");		
		LED0=1;
		LED1=1;
	}	
}
/***********************************************************************
 函数名      ：Uart_Leg_Up(void)   
 函数功能    ：执行上曲腿操作
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Leg_Up(void)
{
	u16 arr_now;              //当前一次运行脉冲数
	u8 len;                   //WiFi串口接收字符串长度
	u8 direct;	              //运行方向标志位
//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //判断程序是否通过break跳出 
	static u8 k=0;            //传输第k张动画指令
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;               //当前一次运行脉冲值
	static u8 leg_up_limit_flag;//上曲腿运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位及下曲腿复位后才能进行上曲腿
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_down_flag==0)&&(lock_flag==1))
	{		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpUpPhone"))  //上曲腿上行	
		{				   
			if(leg_angle_to_arr(leg_up_angle_lim)>leg_up_runed_arr)             //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				direct=1; 
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
				TIM10_Init(leg_angle_to_arr(leg_up_angle_lim)-leg_up_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz					
			} 
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpDownPhone"))  //上曲腿下行
		{			
			if(leg_up_runed_arr>0)                          //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				direct=0;
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
	 	if(((leg_up_runed_arr!=leg_angle_to_arr(leg_up_angle_lim))&&(1==direct))||((0!=leg_up_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)	
				{	
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"LegUpUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpDownPhone")))    //若接收到Stop,则跳出循环	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}						
						else
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数							
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//判断传输动画指令
				if(direct==1)
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
		break_flag=0;       //标志位清零
		//判断复位
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0)) //判断是否处于复位状态，复位状态的前提是下行的定时器走完
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
		if(direct==1)    //如果是上曲腿上行，则用+
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{
				leg_up_runed_arr=leg_angle_to_arr(leg_up_angle_lim);
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
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("LegUpInterfere");		
		LED0=1;
		LED1=1;	
	}
}

/***********************************************************************
 函数名      ：Uart_Leg_Down(void)   
 函数功能    ：执行下曲腿操作
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Leg_Down(void)
{
	u16 arr_now;         //当前一次运行脉冲数   
	u8 len;              //接收的字符串长度
	u8 direct;           //代表某个动作运行的方向标志：1-正向运行；0-反向运行
	
//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;     //判断是否通过break跳出循环 
	static u8 k=0;       //发送第K张动画指令
	static u8 kj=0;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;        //当前一次运行脉冲数
	static u8 leg_down_limit_flag;//下曲腿运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位及下曲腿复位后才能进行上曲腿
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(lock_flag==1))
	{
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownDownPhone"))   //下曲腿下行
		{				
			if(leg_angle_to_arr(leg_down_angle_lim)>leg_down_runed_arr)             //下行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				direct=1;
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
				TIM10_Init(leg_angle_to_arr(leg_down_angle_lim)-leg_down_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
				leg_down_flag=1;
			}	
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownUpPhone"))  //下曲腿上行
		{
			
			if(leg_down_runed_arr>0)     //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				direct=0;
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

	   if(((leg_down_runed_arr!=leg_angle_to_arr(leg_down_angle_lim))&&(1==direct))||((0!=leg_down_runed_arr)&&(0==direct)))
	   {			
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{						
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"LegDownDownPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownUpPhone")))    //若接收到Stop,则跳出循环	
						{				
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}					
					}
				 }
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//发送动画指令
				if(direct==1)
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
			break_flag=0;	   //清除break标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
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
			if(direct==1)    //如果是下曲腿下行，则用+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim);
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
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("LegDownInterfere");		
		LED0=1;
		LED1=1;	
	}
}

/***********************************************************************
 函数名      ：Uart_Body_Left(void)  
 函数功能    ：执行左翻身操作
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Body_Left(void)
{
	u8 direct,key;
	u16 arr_now;                 //当前一次运行脉冲数,用于脉冲累计
	u8 len;                      //接收的字符串长度
	u16 arr_feed;                //计算程序中当前一次运行脉冲数，用于判断电机失步故障
	u16 pulse_num=0;             //电机理论接收到的脉冲值
	u16 num1=0,num2=0,num3=0;    //电机实际运行的脉冲值	
	static u8 motor5_run_flag;   //判断小侧翻是否已经动作，若动作该位置1 
	
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;            //判断程序是否从break跳出
	static u8 k=0,m=0;
	u8 i=0,i1=0;
	u8 j=0,n=0;
	u16 arr_send,arr_send1;     //当前一次运行脉冲数
	static u8 kj;
	static u8 M345_Start;       //345电机第一次运行
	static u8 M345_End;         //345电机运行到上极限位置
	static u8 mn;
	u8 key1;
	
	//联锁功能，只有在支背、上下曲腿、座便、桌子复位后，才能执行左翻身功能
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{			
		//小侧翻起来
		if(body_left_flag==0)   //如果复位到初始状态，才执行左翻起
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpPhone"))
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				//5号侧翻起			
				DIR5=1; 
				body_left_flag=1;
				motor5_run_flag=1;
				W25QXX_Write((u8*)&body_left_flag,33,1); 
				delay_ms(200);
				u2_printf("body_left_flag==1");
				delay_ms(200);
				u2_printf("BodyLeftStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);	 //电机启动	
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);                      //打开定时器
				
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)) )
				{	
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{					
						//光电限位
//						if((GDCheck(GD5LE))&&(1==body_left_flag))
//						{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;
//						}
						
//						if((0==GD5_Left_End)&&(1==body_left_flag))        //碰到光电开关跳出循环，电机停转 
//						{						
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;	
//							}								
//						}
						  //判断有没有收到上位机指令		
						if(USART2_RX_LEN&0x8000)
						{				
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;											
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
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送动画指令
				  arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					m=left_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);						
						if(mn<2)
						{
							m=n;   left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;												
							}
						}
					}
				}
				Motor_5_STOP();            //电机5停止
				TIM10_Stop();              //定时器关闭
				break_flag=0;              //清除标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);	//调用补偿函数
			}
		}	
		//翻身345号电机动作	
		if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpPhone"))||(1==motor5_run_flag))
		{				
			if(body_angle_to_arr(body_left_angle_lim)>body_left_runed_arr)
			{
			     motor5_run_flag=0;
				 
				//345联动左翻起			
				 DIR3=0;DIR4=0;DIR5=0;direct=1;body_left_dir_flag=1;
				 if(M345_Start==0)
				 {
					 M345_Start=1;
					 delay_ms(100);
					 u2_printf("Cartoon_Body_Left_1");
					 //delay_ms(200);
				 }				 
				 Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);				
				 TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);			
			}
		}		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftDownPhone"))
		{					
			if(body_left_runed_arr>0)
			{			   
				//345联动左翻起	
			     DIR3=1;DIR4=1;DIR5=1;direct=0;body_left_dir_flag=0;
				 if(M345_End==1)
				 {
					 M345_End=0;
					 delay_ms(200);
					 u2_printf("Cartoon_Body_Left_8");	
					 delay_ms(200);
				 }
			     Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	//电机启动			
			     TIM10_Init(body_left_runed_arr,timer10_freq);			    //打开定时器
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // 清除中断标志位
		if(((body_left_runed_arr!=body_angle_to_arr(body_left_angle_lim))&&(1==direct))||((0!=body_left_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{
					//光电限位
//					if(GDCheck(GD34S)&&(0==direct))
//					{
//						if(0==GD3_Start)
//						{u2_printf("GD3S==0\r\n");}
//						if(0==GD4_Start)
//						{u2_printf("GD4S==0\r\n");}
//							break_flag=1;
//							break;			
//					}
//					else if(GDCheck(GD34LE)&&(1==direct))
//					{			     
//						 if(0==GD3_Left_End)
//						 {
//							 u2_printf("\r\n0==GD3_Left_End\r\n");
//						 }
//						 if(0==GD4_Left_End)
//						 {
//						  	u2_printf("\r\n0==GD4_Left_End\r\n");
//						 }
//						 break_flag=1;
//					   break;
//					}
					
					
					
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("\r\n0==GD3_Start\r\n");
//							break_flag=1;
//							break;							
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("\r\n0==GD4_Start\r\n");
//							break_flag=1;
//							break;					
//						}											
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))   //如果碰到光电开关，跳出循环，停止运行
//					{ 	
//						delay_us(100);
//						if(((0==GD3_Left_End)||(0==GD4_Left_End)))
//						{
//							break_flag=1;
//							if(0==GD3_Left_End)
//							{
//								u2_printf("\r\n0==GD3_Left_End\r\n");
//							}
//							else if(0==GD4_Left_End)
//							{
//								u2_printf("\r\n0==GD4_Left_End\r\n");
//							}
//						}						
//						break;
//					}
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftDownPhone")))    //若接收到Stop,则跳出循环	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						} 
						else 
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数								
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}				
					//电机故障、故障诊断
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))       
//					{	
//						delay_us(20);		
//						if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))       
//						{	
//							if(1==Motor3_Alm)
//							{
//								body_left_overload_3=1;
//								u2_printf("\r\nBody_Left_Overload_3\r\n");
//							}
//							else if(1==Motor4_Alm)
//							{
//								body_left_overload_4=1;
//								u2_printf("\r\nBodyLeftOverload4\r\n");
//							}
//							else if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("\r\nBodyLeftOverload5\r\n");
//							}					
//							Breakdown_Treatment();
//							break_flag=1;
//							break;
//						}						
//					}						
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_left_runed_arr+arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				else
				{
					j=abs(body_left_runed_arr,arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				k=body_left_picture_k;
				if(k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{	
						k=j;  body_left_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Left_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Left_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Left_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Left_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Left_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Left_7");
									break;
						}
					}				
				}				
			}							
			Motor_3_4_5_STOP();    //电机停止
			TIM10_Stop();          //关闭定时器
			break_flag=0;		   //清除标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;
				body_left_flag=0;
//				W25QXX_Write((u8*)&body_left_flag,33,1);				
			}		
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now
				body_left_flag=1;
//				W25QXX_Write((u8*)&body_left_flag,33,1);														
			}
			//通过上下行判断脉冲累计
			if(direct==1)           //上行，脉冲+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{
					body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
					M345_End=1;  					
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_8");
					delay_ms(200);
					u2_printf("BodyLeftLim");
					delay_ms(200);
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr+arr_now;
				}		
			}	
			else     //下行，脉冲-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{
					body_left_runed_arr=0;
					M345_Start=0;  					
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_1");
					delay_ms(200);
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr-arr_now;					
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //清除中断标志位
			
			//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //光电3没到位
//				{
//					u2_printf("\r\n3号电机继续运行到光电位置\r\n");
//					DIR3=1;
//					Motor_3_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //运行时碰到光电开关，跳出循环 
//						{		
//							delay_us(100);
//							if(0==GD3_Start) 
//							{
//								body_left_flag=0;			
//								u2_printf("GD3Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_3_STOP();
//					TIM10_Stop();         //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))         //光电4没到位
//				{
//					u2_printf("\r\n4号电机继续运行到光电位置\r\n");
//					DIR4=1;
//					Motor_4_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD4_Start) 
//							{
//								body_left_flag=0;
//								u2_printf("GD4Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_4_STOP();       //电机停止
//					TIM10_Stop();         //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //光电3/4都没到位
//				{
//					u2_printf("\r\n3号及4号电机同时继续运行到光电位置\r\n");
//					DIR3=1; DIR4=1; DIR5=1;
//					Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if((0==GD3_Start)||(0==GD4_Start))
//							{
//								if(0==GD3_Start)
//								{
//									body_left_flag=0;
//									u2_printf("GD3Start");
//									break;
//								}
//								if(0==GD4_Start)
//								{
//									body_left_flag=0;
//									u2_printf("GD4Start");
//									break;
//								}
//							}								
//						}
//					}			
//					Motor_3_4_5_STOP();   //电机停止
//					TIM10_Stop();         //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
//			}
			//5号电机复位
			if(body_left_flag==0)     //345联动复位到初始状态，才复位5号电机
			{			
				//5号侧翻复位
				Motor_4_Compensate(1,bodyleft_compleate,motor_body_freq,motor_timer_freq);
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;		
				DIR5=0;
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);  //5号电机启动
				body_left_runed_arr=0;			
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);                     //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{				
						//光电限位
//						if(GDCheck(GD5S)&&(0==body_left_flag))
//						{
//						   break_flag=1;
//						   break;
//						}
						
//						if(((0==GD5_Start)&&(0==body_left_flag)) )   //碰到光电开关跳出循环，电机停转 
//						{	
//							delay_us(20);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;						
//							}		             
//						}
						  //判断有没有收到上位机指令		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
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
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送动画指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);				
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					n=9-n;
					m=left_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;  left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}
						}
					}
				}      
				Motor_5_STOP();       //电机停止
				TIM10_Stop();         //关闭定时器
				break_flag=0;         //清除标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");				
				delay_ms(200);
				u2_printf("body_left_flag==0");
				delay_ms(200);
				u2_printf("BodyLeftRes");
				delay_ms(200);
				//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=0;
//					Motor_5_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_5_STOP();
//					TIM10_Stop();
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
			}
			W25QXX_Write((u8*)&body_left_flag,33,1);			
		}
	}
	else
	{
		LED0=0;   //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("BodyLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Body_Right(void)  
 函数功能    ：执行右翻身操作
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Body_Right(void)
{
	u8 direct,key;
	u16 arr_now;        //当前一次运行脉冲数，用于脉冲累计
	u8 len;             //接收的字符串长度
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num1=0,num2=0,num3=0;
	
	static u8 motor5_run_flag;  //小侧翻已运行标志位
	
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //判断程序是否从break跳出 
	static u16 k=0,m=0;
	static u8 M345R_Start=0;  //345电机从初始位置运行
	static u8 M345R_End=0;    //345电机到达上极限位置
	u8 mn;
	u8 kj;

	u8 i=0,i1=0;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;	  //当前一次运行脉冲数
	
	//联锁功能，只有在支背、上下曲腿、座便、桌子复位后，才能执行右左翻身功能
	if((lock_flag==1)&&(back_flag==0)&&(body_left_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_right_flag==0)&&(waist_nursing_right_flag==0))
	{
		if(body_right_flag==0)   //如果复位到初始状态，才执行右翻起
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpPhone"))
			{
			 //5号侧翻起
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				DIR5=0; 
				body_right_flag=1;
				motor5_run_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);
				delay_ms(200);
				u2_printf("body_right_flag==1");
				delay_ms(200);
				u2_printf("BodyRightStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);	//电机启动	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);                     //打开定时器  
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{	
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{
						//光电限位
//						if((GDCheck(GD5RE))&&(1==body_right_flag))
//						{
//								break_flag=1;
//								u2_printf("GD5RightEnd");
//								break;
//						}
										
//						if((0==GD5_Right_End)&&(1==body_right_flag))                     //碰到光电开关跳出循环，电机停转 
//						{	
//							delay_us(100);
//							if(0==GD5_Right_End)                    //碰到光电开关跳出循环，电机停转 
//							{
//								u2_printf("GD5RightEnd");
//								break_flag=1;
//								break;						
//							}		             
//						}
						  //判断有没有收到上位机指令		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}												
						//电机故障、故障诊断
//						if(1==Motor5_Alm)        
//						{	
//							delay_us(100);
//							if(1==Motor5_Alm)        
//							{	
//								body_right_overload_5=1;
//								u2_printf("Body_Right_Overload_5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;						
//							}		             
//						}											
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送图片指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					m=right_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch(m)
							{								
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;
							}
						}
					}
				}					      
				Motor_5_STOP();       //电机停止
				TIM10_Stop();         //关闭定时器
				break_flag=0;         //清除break标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(1,bodyright_compleate,motor_body_freq,motor_timer_freq);//调用补偿函数
			}			
		}	
		//翻身345号电机动作	
		if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpPhone"))||(1==motor5_run_flag))  //右翻身上
		{		
			if(body_angle_to_arr(body_right_angle_lim)>body_right_runed_arr)
			{
				motor5_run_flag=0;
				//345联动左翻起
				DIR3=1;DIR4=1;DIR5=1;direct=1;body_right_dir_flag=1;
				if(M345R_Start==0)
				{
					M345R_Start=1;
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
					delay_ms(200);
				}			
				Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //电机启动	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim)-body_right_runed_arr,timer10_freq);	//定时器打开			
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightDownPhone"))  //右翻身下
		{			
			if(body_right_runed_arr>0)
			{
				//345联动左翻起
			   DIR3=0;DIR4=0;DIR5=0;direct=0;body_right_dir_flag=0;
			   if(M345R_End==1)
			   {
					M345R_End=0; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");	
					//delay_ms(200);
			   }			
													
			   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //电机启动		
			   TIM10_Init(body_right_runed_arr,timer10_freq);		  //打开定时器
			}	
		}				
		  memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		  USART2_RX_LEN=0;
		  __HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位	 	

		 if(((body_right_runed_arr!=body_angle_to_arr(body_right_angle_lim))&&(1==direct))||((0!=body_right_runed_arr)&&(0==direct)))
		 {
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //等待定时时间到，时间到跳出循环
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//光电限位
//					if(GDCheck(GD34S)&&(0==direct))
//					{
//							break_flag=1;
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3_Start");
//							}
//							else if(0==GD4_Start)
//							{
//								u2_printf("GD4_Start");
//							}
//							break;
//					}
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))                    //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if((0==GD3_Start)||(0==GD4_Start))                   //如果碰到光电开关，跳出循环，停止运行
//						{
//							break_flag=1;
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3_Start");
//							}
//							else if(0==GD4_Start)
//							{
//								u2_printf("GD4_Start");
//							}
//							break;					
//						}	
//					}
//					if(((0==GD3_Right_End)||(0==GD4_Right_End))&&(1==direct))           //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if((0==GD3_Right_End)||(0==GD4_Right_End))          //如果碰到光电开关，跳出循环，停止运行
//						{
//							break_flag=1;
//							if(0==GD3_Right_End)
//							{
//								u2_printf("GD3_Right_End");
//							}
//							if(0==GD4_Right_End)
//							{
//								u2_printf("GD4_Right_End");
//							}
//							break;
//						}
//					}
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpPhone"))|| (strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightDownPhone"))) //若接收到Stop,则跳出循环	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else 
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数							
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}					
					}
					//电机故障、故障诊断
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//					{	
//						delay_us(100);
//						if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//						{
//							if(1==Motor3_Alm)
//							{
//								body_right_overload_3=1;
//								u2_printf("BodyRightOverload3");
//							}
//							if(1==Motor4_Alm)
//							{
//								body_right_overload_4=1;
//								u2_printf("BodyRightOverload4");
//							}
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("BodyRightOverload5");
//							}
//							Breakdown_Treatment();	
//							break_flag=1;
//						}						
//						break;
//					}					
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_right_runed_arr+arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				else
				{
					j=abs(body_right_runed_arr,arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				k=body_right_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;  body_right_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_7");
									break;		
						}
					}				
				}				
			}				
			 
			Motor_3_4_5_STOP();    //电机停止
			TIM10_Stop();          //定时器关闭
			break_flag=0;		   //清除标志位
			//判断复位	
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;
				body_right_flag=0;
//				W25QXX_Write((u8*)&body_right_flag,34,1);				
			}					
			else
			{							
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				body_right_flag=1;
//				W25QXX_Write((u8*)&body_right_flag,34,1);					
			}
			//通过上下行判断脉冲累计
			if(direct==1)    //翻身上行，则用+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Right_End)||(0==GD4_Right_End))
				{
					body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
					M345R_End=1; 					
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
					u2_printf("BodyRightLim");
					delay_ms(200);
				}
				else
				{				
					body_right_runed_arr=body_right_runed_arr+arr_now;
				}
			}
			else		//翻身下行，则用-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))
				{
					body_right_runed_arr=0;	
					M345R_Start=0; 					
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr-arr_now;
				}			
			}			
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
			//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_right_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //光电3没到位
//				{
//					DIR3=0;
//					Motor_3_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD3_Start) 
//							{
//								body_right_flag=0;
//								u2_printf("GD3Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_3_STOP();   //电机停止
//					TIM10_Stop();     //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //光电4没到位
//				{
//					DIR4=0;
//					Motor_4_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD4_Start) 
//							{
//								body_right_flag=0;
//								u2_printf("GD4Start");
//								break;
//							}
//						}
//					}			
//					Motor_4_STOP();    //电机停止
//					TIM10_Stop();      //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //光电3/4都没到位
//				{
//					DIR3=0; DIR4=0; DIR5=0;
//					Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD3_Start)
//							{
//								body_right_flag=0;
//								u2_printf("GD3Start");
//								break;
//							}
//							if(0==GD4_Start) 
//							{
//								body_right_flag=0;
//								u2_printf("GD4Start");
//								break;
//							}
//						}
//					}			
//					Motor_3_4_5_STOP();   //电机停止
//					TIM10_Stop();         //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
//			}
			//侧翻复位
			if(0==body_right_flag)      //只有翻身复位到初始状态，小侧翻才复位
			{			
				//5号侧翻复位
				Motor_4_Compensate(0,bodyright_compleate,motor_body_freq,motor_timer_freq);//调用补偿函数
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;		
				DIR5=1;
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");	
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq); //电机启动
				body_right_runed_arr=0;
				TIM10_Init(body_angle_to_arr(body_right_angle_lim)*1,timer10_freq);                     //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位

				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{
						//光电限位
//						if(((0==GD5_Start)&&(0==body_right_flag)))  //碰到光电开关跳出循环，电机停转 
//						{	
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;					
//							}
//						}
						  //判断有没有收到上位机指令		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}										
						//电机故障、故障诊断
//						if(1==Motor5_Alm)       
//						{	
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("BodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;						
//							}

//						}										
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送动画指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					n=9-n;	
					m=right_motor5_picture_m;
					if(	m!=n)
					{
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}
						}
					}
				}	     
				Motor_5_STOP();      //电机停止
				TIM10_Stop();        //定时器关闭
				break_flag=0;   
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");				
				delay_ms(200);			
				u2_printf("body_right_flag==0");
				delay_ms(200);
				u2_printf("BodyRightRes");
				//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_right_flag))
//				{
//					DIR5=1;
//					Motor_5_START(motor_body_freq,motor_timer_freq);   //电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_5_STOP();    //电机停止
//					TIM10_Stop();      //关闭定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}			
			}	
			W25QXX_Write((u8*)&body_right_flag,34,1);
		}
	}
	else
	{
		LED0=0;   //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("BodyRightInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Desk(void)  
 函数功能    ：小桌子
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Desk(void)
{
	u8 direct,key;    //表示电机运行方向，1：小桌子前进；0：小桌子后退
	u16 arr_now;      //本次运行脉冲值
	u8 len;           //表示接收的字符串的长度
	u16 arr_feed;     //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;  //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	
//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //判断程序是否从break跳出 
	static u8 k=0;            //发送第k张图片
	static u8 kj=0;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;                 //当前一次运行脉冲数
	static u8 desk_limit_flag;    //判断小桌子是否运行到极限位置，若是发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位后，才能进行小桌子移动
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))
		{
			DIR7=0;
			direct=1;
			if(desk_flag==0)
			{
				desk_flag=1;
				delay_ms(200);
				u2_printf("desk_flag==1");
				delay_ms(200);
				u2_printf("DeskStart");
				delay_ms(200);
				u2_printf("Cartoon_Desk_1");
				delay_ms(200);
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone"))
		{
			DIR7=1;
			direct=0;
			if(1==desk_limit_flag)
			{
				desk_limit_flag=0;
				delay_ms(200);
				u2_printf("Cartoon_Desk_20");
				delay_ms(200);
			}
		}
		if(direct==1)   //如果是小桌子向前
		{		
			if(desk_distance_to_arr(desk_distance_lim)>desk_runed_arr)  //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_distance_to_arr(desk_distance_lim)-desk_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
			}				
		}
		else       //如果是小桌子后退
		{
			if(desk_runed_arr>0)    //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
		 if(((desk_runed_arr!=desk_distance_to_arr(desk_distance_lim))&&(1==direct))||((0!=desk_runed_arr)&&(0==direct)))
		 {	 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环  
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//光电限位											   						
					if((0==GD7_End)&&(1==direct))
					{
						delay_us(100);
						if(0==GD7_End)
						{
							u2_printf("GD7End");					
							break_flag=1;	
							break;					
						}								
					}
					if((0==GD7_Start)&&(0==direct))
					{
						delay_us(100);
						if(0==GD7_Start)
						{
							u2_printf("GD7Start");					
							break_flag=1;
							break;						
						}					
					}
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone")))    //若接收到Stop,则跳出循环	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数							
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}				
					//电机故障、故障诊断
//					if(PCF8574_ReadBit(1)==1)        
//					{	
//						delay_us(100);
//						if(PCF8574_ReadBit(1)==1)
//						{
//							desk_overload=1;
//							u2_printf("DeskOverload");
//							Uart_Breakdown_Treatment();
//							break_flag=1;
//							break;						
//						}								             
//					}								
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(desk_runed_arr+arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				else
				{
					j=abs(desk_runed_arr,arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				k=desk_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;  desk_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Desk_2");									
									break;
							case 2:	u2_printf("Cartoon_Desk_3");
									break;					
							case 3:	u2_printf("Cartoon_Desk_4");
									break;					
							case 4:	u2_printf("Cartoon_Desk_5");
									break;	
							case 5:	u2_printf("Cartoon_Desk_6");
									break;	
							case 6:	u2_printf("Cartoon_Desk_7");
									break;
							case 7:	u2_printf("Cartoon_Desk_8");
									break;
							case 8:	u2_printf("Cartoon_Desk_9");
									break;						
							case 9:	u2_printf("Cartoon_Desk_10");
									break;												
							case 10:u2_printf("Cartoon_Desk_11");
									break;	
							case 11:u2_printf("Cartoon_Desk_12");									
									break;
							case 12:u2_printf("Cartoon_Desk_13");
									break;					
							case 13:u2_printf("Cartoon_Desk_14");
									break;					
							case 14:u2_printf("Cartoon_Desk_15");
									break;	
							case 15:u2_printf("Cartoon_Desk_16");
									break;	
							case 16:u2_printf("Cartoon_Desk_17");
									break;
							case 17:u2_printf("Cartoon_Desk_18");
									break;
							case 18:u2_printf("Cartoon_Desk_19");
									break;						
						}
					}					
				}				
			}				    
			Motor_7_STOP();     //电机停止
			TIM10_Stop();       //定时器关闭
			break_flag=0;		//清除break标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //判断是否处于复位状态，复位状态的前提是下行的定时器走完
			{
				arr_now=0;         //此时处于复位状态，将状态值都设为0；
				desk_flag=0;
				delay_ms(200);
				u2_printf("desk_flag==0");
			}
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now				
				desk_flag=1;
			}	
			//通过上下行判断脉冲累计
			if(direct==1)        //如果是小桌子前进，则用+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_End))
				{ 					
					desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
					desk_limit_flag=1;	
					delay_ms(200);
					u2_printf("Cartoon_Desk_20");
					delay_ms(200);
					u2_printf("DeskLim");
					delay_ms(200);
				}
				else
				{  desk_runed_arr=desk_runed_arr+arr_now;	}				
			}
			else                //如果是小桌子后退，则用-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_Start))
				{						
					desk_runed_arr=0; 
					delay_ms(200);
					u2_printf("Cartoon_Desk_1");
					delay_ms(200);
					u2_printf("DeskRes");
					delay_ms(200);
				}
				else
				{	 
					desk_runed_arr=desk_runed_arr-arr_now;
				}						
			}				
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	   			
			//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段,并删除清除中断语句）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD7_Start==1)&&(direct==0)&&(desk_flag==1))
//			{   
//				u2_printf("附加脉冲运行");
//				DIR7=1;
//				Motor_7_START(motor_desk_freq,motor_timer_freq);
//			    TIM10_Init(add_arr,timer10_freq);   //打开定时器
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//				
//				u2_printf("\r\n进入附加脉冲运行，直到碰到光电开关\r\n");
//				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//				{							
//					if(0==GD7_Start)  //运行时碰到光电开关，跳出循环 
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							u2_printf("GD7Start");
//							break;
//						}							
//					}
//				}			
//				Motor_7_STOP();   //电机停止
//				TIM10_Stop();     //关闭定时器
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//			}			
		}
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("DeskInterfere");		
		LED0=1;
		LED1=1;	

	}		
}

void Uart_Desk1(void)
{
	u8 direct,key;    //表示电机运行方向，1：小桌子前进；0：小桌子后退
	u16 arr_now;      //本次运行脉冲值
	u8 len;           //表示接收的字符串的长度
	u16 arr_feed;     //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;  //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	static u8 desk_front_flag;
	
//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //判断程序是否从break跳出 
	static u8 k=0;            //发送第k张图片
	static u8 kj=0;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;                 //当前一次运行脉冲数
	static u8 desk_limit_flag;    //判断小桌子是否运行到极限位置，若是发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位后，才能进行小桌子移动
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{	
		//小桌子电机先运行一段时间
		if(desk_flag==0)
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				DIR7=0;
				desk_flag=1;
				desk_front_flag=1;
				u2_printf("desk_flag==1");
				delay_ms(200);
				u2_printf("DeskStart");
				delay_ms(200);
				u2_printf("Cartoon_Desk_Front");
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init((u16)(desk_distance_to_arr(desk_distance_lim)/2.6),timer10_freq);
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) 
				{
					
				}
				Motor_7_STOP();     //电机停止
				TIM10_Stop();       //定时器关闭
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
				u2_printf("电机7第一阶段运行完成");
			}
		}


		
		if((strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))||(desk_front_flag==1))
		{
			desk_front_flag=0;
			DIR7=0;
			direct=1;
			if(desk_flag==0)
			{
				desk_flag=1;
				delay_ms(200);
				u2_printf("desk_flag==1");
				delay_ms(200);
				u2_printf("DeskStart");
				delay_ms(200);
				u2_printf("Cartoon_Desk_1");
				delay_ms(200);
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone"))
		{
			DIR7=1;
			direct=0;
			if(1==desk_limit_flag)
			{
				desk_limit_flag=0;
				delay_ms(200);
				u2_printf("Cartoon_Desk_20");
				delay_ms(200);
			}
		}
		if(direct==1)   //如果是小桌子向前
		{		
			if(desk_distance_to_arr(desk_distance_lim)>desk_runed_arr)  //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_distance_to_arr(desk_distance_lim)-desk_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
			}				
		}
		else       //如果是小桌子后退
		{
			if(desk_runed_arr>0)    //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_runed_arr,timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz			
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
		 if(((desk_runed_arr!=desk_distance_to_arr(desk_distance_lim))&&(1==direct))||((0!=desk_runed_arr)&&(0==direct)))
		 {	 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环  
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//光电限位											   						
//					if((0==GD7_End)&&(1==direct))
//					{
//						delay_us(100);
//						if(0==GD7_End)
//						{
//							u2_printf("GD7End");					
//							break_flag=1;	
//							break;					
//						}								
//					}
//					if((0==GD7_Start)&&(0==direct))
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							u2_printf("GD7Start");					
//							break_flag=1;
//							break;						
//						}					
//					}
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone")))    //若接收到Stop,则跳出循环	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数							
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}				
					//电机故障、故障诊断
//					if(PCF8574_ReadBit(1)==1)        
//					{	
//						delay_us(100);
//						if(PCF8574_ReadBit(1)==1)
//						{
//							desk_overload=1;
//							u2_printf("DeskOverload");
//							Uart_Breakdown_Treatment();
//							break_flag=1;
//							break;						
//						}								             
//					}								
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(desk_runed_arr+arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				else
				{
					j=abs(desk_runed_arr,arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				k=desk_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;  desk_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Desk_2");									
									break;
							case 2:	u2_printf("Cartoon_Desk_3");
									break;					
							case 3:	u2_printf("Cartoon_Desk_4");
									break;					
							case 4:	u2_printf("Cartoon_Desk_5");
									break;	
							case 5:	u2_printf("Cartoon_Desk_6");
									break;	
							case 6:	u2_printf("Cartoon_Desk_7");
									break;
							case 7:	u2_printf("Cartoon_Desk_8");
									break;
							case 8:	u2_printf("Cartoon_Desk_9");
									break;						
							case 9:	u2_printf("Cartoon_Desk_10");
									break;												
							case 10:u2_printf("Cartoon_Desk_11");
									break;	
							case 11:u2_printf("Cartoon_Desk_12");									
									break;
							case 12:u2_printf("Cartoon_Desk_13");
									break;					
							case 13:u2_printf("Cartoon_Desk_14");
									break;					
							case 14:u2_printf("Cartoon_Desk_15");
									break;	
							case 15:u2_printf("Cartoon_Desk_16");
									break;	
							case 16:u2_printf("Cartoon_Desk_17");
									break;
							case 17:u2_printf("Cartoon_Desk_18");
									break;
							case 18:u2_printf("Cartoon_Desk_19");
									break;						
						}
					}					
				}				
			}				    
			Motor_7_STOP();     //电机停止
			TIM10_Stop();       //定时器关闭
			break_flag=0;		//清除break标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //判断是否处于复位状态，复位状态的前提是下行的定时器走完
			{
				arr_now=0;         //此时处于复位状态，将状态值都设为0；
				desk_flag=0;
				delay_ms(100);
				u2_printf("desk_flag==0");
			}
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //获取当前计数值arr_now				
				desk_flag=1;
			}	
			//通过上下行判断脉冲累计
			if(direct==1)        //如果是小桌子前进，则用+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_End))
				{ 					
					desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
					desk_limit_flag=1;	
					delay_ms(200);
					u2_printf("Cartoon_Desk_20");
					delay_ms(200);
					u2_printf("DeskLim");
					delay_ms(200);
				}
				else
				{  desk_runed_arr=desk_runed_arr+arr_now;	}				
			}
			else                //如果是小桌子后退，则用-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_Start))
				{						
					desk_runed_arr=0; 
					//delay_ms(200);
					u2_printf("Cartoon_Desk_1");
					delay_ms(200);
					u2_printf("DeskRes");
					//delay_ms(200);
				}
				else
				{	 
					desk_runed_arr=desk_runed_arr-arr_now;
				}						
			}				
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	   			
			//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段,并删除清除中断语句）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD7_Start==1)&&(direct==0)&&(desk_flag==1))
//			{   
//				u2_printf("附加脉冲运行");
//				DIR7=1;
//				Motor_7_START(motor_desk_freq,motor_timer_freq);
//			    TIM10_Init(add_arr,timer10_freq);   //打开定时器
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//				
//				u2_printf("\r\n进入附加脉冲运行，直到碰到光电开关\r\n");
//				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//				{							
//					if(0==GD7_Start)  //运行时碰到光电开关，跳出循环 
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							u2_printf("GD7Start");
//							break;
//						}							
//					}
//				}			
//				Motor_7_STOP();   //电机停止
//				TIM10_Stop();     //关闭定时器
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//			}			
		}
		if(desk_flag==0) 
		{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;	
				DIR7=1;
			  desk_runed_arr=0;
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_distance_to_arr(desk_distance_lim)/2.6,timer10_freq);
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) 
				{
					
				}
				Motor_7_STOP();     //电机停止
				TIM10_Stop();       //定时器关闭
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
				u2_printf("电机7第一阶段返回完成");
		}
		
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("DeskInterfere");		
		LED0=1;
		LED1=1;	

	}		
}




/***********************************************************************
 函数名      ：Uart_Back_Nursing_Left(void)  
 函数功能    ：左背部护理
 输入        ：无
 输出        ：无 

************************************************************************/
void Uart_Back_Nursing_Left(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;  
	u8 break_flag=0;    //判断程序是否从break跳出
	static u8 k=0;      //发送第k张动画指令
	u8 i=0;
	u8 j=0;	
	u16 arr_send;	    //当前一次运行脉冲数
	static u8 kj;
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	
	//联锁功能，只有在已执行左翻身功能且左腰部护理复位后，才能进行左背部护理
	if((lock_flag==1)&&(body_left_flag==1)&&(waist_nursing_left_flag==0))
	{
		back_nursing_left_flag=!back_nursing_left_flag;	         //左背部护理复位状态标志位取反
		back_nursing_left_dir_flag=!back_nursing_left_dir_flag;	 //左背部护理方向标志位		
		if(back_nursing_left_dir_flag==1)
		{ 
			DIR3=1; direct=1;
			delay_ms(200);
			u2_printf("back_nursing_left_flag==1");
			delay_ms(200);
			u2_printf("BackNursingLeftStart");
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_1");
			delay_ms(200);
		}
		else
		{ 
			DIR3=0; direct=0;
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_20");
			delay_ms(200);
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_3_START(motor_body_freq,motor_timer_freq);	       //电机启动
		TIM10_Init(body_left_runed_arr,timer10_freq);              //打开定时器body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环   
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//光电限位
//				if((0==GD3_Left_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD3_Left_End)
//					{
//						break_flag=1;
//						u2_printf("GD3LeftEnd");
//						break;				
//					}
//	
//				}
//				if((0==GD3_Start)&&(0==direct))     //落下
//				{	
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						break_flag=1;	
//						u2_printf("GD3_Start");
//					    break;					
//					}					
//				}
				  //判断有没有收到上位机指令		
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;															
				}				
				//电机故障、故障诊断
//				if(1==Motor3_Alm)        
//				{	
//					delay_us(100);
//					if(1==Motor3_Alm) 
//					{
//						back_nursing_left_overload=1;
//						u2_printf("BackNursingLeftOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;				
//					}								             
//				}							
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//发送动画指令
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				
			j=arr_send/(body_left_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=back_nursing_left_picture_k;
			if(k!=j)
			{
				kj=abs(k,j);
				if(kj<2)
				{
					k=j;   back_nursing_left_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Back_Nursing_Left_2");									
								break;
						case 2:	u2_printf("Cartoon_Back_Nursing_Left_3");
								break;					
						case 3:	u2_printf("Cartoon_Back_Nursing_Left_4");
								break;					
						case 4:	u2_printf("Cartoon_Back_Nursing_Left_5");
								break;	
						case 5:	u2_printf("Cartoon_Back_Nursing_Left_6");
								break;	
						case 6:	u2_printf("Cartoon_Back_Nursing_Left_7");
								break;
						case 7:	u2_printf("Cartoon_Back_Nursing_Left_8");									
								break;
						case 8:	u2_printf("Cartoon_Back_Nursing_Left_9");
								break;					
						case 9:	u2_printf("Cartoon_Back_Nursing_Left_10");
								break;					
						case 10:u2_printf("Cartoon_Back_Nursing_Left_11");
								break;	
						case 11:u2_printf("Cartoon_Back_Nursing_Left_12");
								break;	
						case 12:u2_printf("Cartoon_Back_Nursing_Left_13");
								break;
						case 13:u2_printf("Cartoon_Back_Nursing_Left_14");
								break;	
						case 14:u2_printf("Cartoon_Back_Nursing_Left_15");
								break;	
						case 15:u2_printf("Cartoon_Back_Nursing_Left_16");
								break;
						case 16:u2_printf("Cartoon_Back_Nursing_Left_17");
								break;	
						case 17:u2_printf("Cartoon_Back_Nursing_Left_18");
								break;
						case 18:u2_printf("Cartoon_Back_Nursing_Left_19");
								break;						
					}
				}
			}
		}				  
		Motor_3_STOP();     //电机停止
		TIM10_Stop();       //关闭定时器    
		break_flag=0;       //清除break标志位
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		if(back_nursing_left_dir_flag==1)
		{			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_20");
			delay_ms(200);
			u2_printf("BackNursingLeftLim");
		}
		else
		{			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_1");
			delay_ms(200);
			u2_printf("back_nursing_left_flag==0");
			delay_ms(200);
			u2_printf("BackNursingLeftRes");
		}	
	}
	else
	{
		LED0=0;        //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("BackNursingLeftInterfere");		
		LED0=1;	
		LED1=1;

	}	
}

/***********************************************************************
 函数名      ：Uart_Back_Nursing_Right(void)
 函数功能    ：右背部护理
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Back_Nursing_Right(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;  
	u8 break_flag=0;     //判断程序从break跳出
	static u8 k=0;       //发送第k张动画指令
	u8 i=0;
	u8 j=0;	
	u16 arr_send;	    //当前一次运行脉冲数
	static u8 kj;
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;		
	
	//联锁功能，只有在已执行右翻身功能且右腰部护理复位后，才能进行右背部护理
	if((lock_flag==1)&&(1==body_right_flag)&&(waist_nursing_right_flag==0))
	{	
		back_nursing_right_flag=!back_nursing_right_flag;         //右背部护理复位状态标志位
		back_nursing_right_dir_flag=!back_nursing_right_dir_flag; //右背部护理方向标志位 	
		if(back_nursing_right_dir_flag==1)
		{ 
			DIR3=0; direct=1;
			delay_ms(200);
			u2_printf("back_nursing_right_flag==1");
			delay_ms(200);
			u2_printf("BackNursingRightStart");
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_1");
			delay_ms(200);
		}
		else
		{ 
			DIR3=1; direct=0;
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_20");
			delay_ms(200);
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_3_START(motor_body_freq*1.4,motor_timer_freq);              //电机启动
		TIM10_Init(body_right_runed_arr,timer10_freq);                //打开定时器
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //等待定时时间到，时间到跳出循环 
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//光电限位控制
//				if((0==GD3_Right_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD3_Right_End)
//					{
//						break_flag=1;
//						u2_printf("GD3RightEnd");
//						break;				
//					}	
//				}
//				if((0==GD3_Start)&&(0==direct))     //落下
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						break_flag=1;
//						u2_printf("GD3Start");
//						break;				
//					}						
//				}
				  //判断有没有收到上位机指令		
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;												
				}
				//电机故障、故障诊断
//				if(1==Motor3_Alm)        
//				{	
//					delay_us(100);
//					if(1==Motor3_Alm) 
//					{
//						back_nursing_right_overload=1;
//						u2_printf("BackNursingRightOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;				
//					}		             
//				}										
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//发送动画指令
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_right_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=back_nursing_right_picture_k;
			if(	k!=j)
			{	
				kj=abs(k,j);				
				if(kj<2)
				{
					k=j;   back_nursing_right_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Back_Nursing_Right_2");									
								break;
						case 2:	u2_printf("Cartoon_Back_Nursing_Right_3");
								break;					
						case 3:	u2_printf("Cartoon_Back_Nursing_Right_4");
								break;					
						case 4:	u2_printf("Cartoon_Back_Nursing_Right_5");
								break;	
						case 5:	u2_printf("Cartoon_Back_Nursing_Right_6");
								break;	
						case 6:	u2_printf("Cartoon_Back_Nursing_Right_7");
								break;						
						case 7:	u2_printf("Cartoon_Back_Nursing_Right_8");									
								break;
						case 8:	u2_printf("Cartoon_Back_Nursing_Right_9");
								break;					
						case 9:	u2_printf("Cartoon_Back_Nursing_Right_10");
								break;					
						case 10:u2_printf("Cartoon_Back_Nursing_Right_11");
								break;	
						case 11:u2_printf("Cartoon_Back_Nursing_Right_12");
								break;	
						case 12:u2_printf("Cartoon_Back_Nursing_Right_13");
								break;
						case 13:u2_printf("Cartoon_Back_Nursing_Right_14");
								break;	
						case 14:u2_printf("Cartoon_Back_Nursing_Right_15");
								break;	
						case 15:u2_printf("Cartoon_Back_Nursing_Right_16");
								break;
						case 16:u2_printf("Cartoon_Back_Nursing_Right_17");
								break;	
						case 17:u2_printf("Cartoon_Back_Nursing_Right_18");
								break;
						case 18:u2_printf("Cartoon_Back_Nursing_Right_19");
								break;	
					}
				}
			}			
		}			
		Motor_3_STOP();      //电机停止
		TIM10_Stop();        //关闭定时器
		break_flag=0;        //清除break标志位
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		
		if(back_nursing_right_dir_flag==1)
		{ 		
			 delay_ms(200);
			 u2_printf("Cartoon_Back_Nursing_Right_20");
			 delay_ms(200);
			 u2_printf("BackNursingRightLim");
		}
		else
		{ 			
			 delay_ms(200);
			 u2_printf("Cartoon_Back_Nursing_Right_1");			 
			 delay_ms(200);
			 u2_printf("back_nursing_right_flag==0");
			 delay_ms(200);
			 u2_printf("BackNursingRightRes");
		}				
	}
	else
	{
		LED0=0;        //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("BackNursingRightInterfere");
		LED0=1;	
		LED1=1;		
	}
}

/***********************************************************************
 函数名      ：Uart_Waist_Nursing_Left(void)  
 函数功能    ：左腰部护理
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Waist_Nursing_Left(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;  
	u8 break_flag=0;    //判断程序从break跳出
	static u8 k=0;      //发送第k张动画指令
	u8 i=0;
	u8 j=0;	 
	u16 arr_send;	   //当前一次运行脉冲数
	static u8 kj;
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;		
	
	//联锁功能，只有在已执行左翻身功能且左背部护理复位后，才能进行左腰部护理
	if((lock_flag==1)&&(1==body_left_flag)&&(back_nursing_left_flag==0))
	{
		waist_nursing_left_flag=!waist_nursing_left_flag;         //左腰部护理复位状态标志位
		waist_nursing_left_dir_flag=!waist_nursing_left_dir_flag; //左腰部护理方向标志位		
		if(waist_nursing_left_dir_flag==1)
		{ 
			DIR4=1; direct=1;
			delay_ms(200);
			u2_printf("waist_nursing_left_flag==1");
			delay_ms(200);
			u2_printf("WaistNursingLeftStart");
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_1");	
			delay_ms(200);
		}
		else
		{ 
			DIR4=0; direct=0;
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_20");
			delay_ms(200);
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_4_START((u16)(motor_body_freq*1.2),motor_timer_freq);             //电机启动
		TIM10_Init(body_left_runed_arr,timer10_freq);                //打开定时器
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //等待定时时间到，时间到跳出循环 
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//光电限位
//				if((0==GD4_Left_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD4_Left_End)
//					{
//						break_flag=1;
//						u2_printf("GD4LeftEnd");
//						break;					
//					}
//				}
//				if((0==GD4_Start)&&(0==direct))     //落下
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						break_flag=1;
//						u2_printf("GD4Start");
//						break;				
//					}						
//				}
				  //判断有没有收到上位机指令		
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;												
				}				
				
				//电机故障、故障诊断
//				if(1==Motor4_Alm)        
//				{	
//					delay_us(100);
//					if(1==Motor4_Alm)
//					{
//						waist_nursing_left_overload=1;
//						u2_printf("WaistNursingLeftOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;					
//					}	             
//				}						
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
            //发送动画指令			
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_left_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=waist_nursing_left_picture_k;
			if(	k!=j)
			{
				kj=abs(k,j);
				if(kj<2)
				{
					k=j;   waist_nursing_left_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Waist_Nursing_Left_2");									
								break;
						case 2:	u2_printf("Cartoon_Waist_Nursing_Left_3");
								break;					
						case 3:	u2_printf("Cartoon_Waist_Nursing_Left_4");
								break;					
						case 4:	u2_printf("Cartoon_Waist_Nursing_Left_5");
								break;	
						case 5:	u2_printf("Cartoon_Waist_Nursing_Left_6");
								break;	
						case 6:	u2_printf("Cartoon_Waist_Nursing_Left_7");
								break;
						case 7:	u2_printf("Cartoon_Waist_Nursing_Left_8");									
								break;
						case 8:	u2_printf("Cartoon_Waist_Nursing_Left_9");
								break;					
						case 9:	u2_printf("Cartoon_Waist_Nursing_Left_10");
								break;					
						case 10:u2_printf("Cartoon_Waist_Nursing_Left_11");
								break;	
						case 11:u2_printf("Cartoon_Waist_Nursing_Left_12");
								break;	
						case 12:u2_printf("Cartoon_Waist_Nursing_Left_13");
								break;
						case 13:u2_printf("Cartoon_Waist_Nursing_Left_14");
								break;	
						case 14:u2_printf("Cartoon_Waist_Nursing_Left_15");
								break;	
						case 15:u2_printf("Cartoon_Waist_Nursing_Left_16");
								break;
						case 16:u2_printf("Cartoon_Waist_Nursing_Left_17");
								break;	
						case 17:u2_printf("Cartoon_Waist_Nursing_Left_18");
								break;	
						case 18:u2_printf("Cartoon_Waist_Nursing_Left_19");
								break;		
					}
				}
			}			
		}				     
		Motor_4_STOP();     //电机停止
		TIM10_Stop();       //定时器关闭
		break_flag=0;       //清除break标志位
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		
		if(waist_nursing_left_dir_flag==1)
		{ 
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_20");  
			delay_ms(200);
			u2_printf("WaistNursingLeftLim");
		}
		else
		{ 
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_1"); 			
			delay_ms(200);
			u2_printf("waist_nursing_left_flag==0");
			delay_ms(200);
			u2_printf("WaistNursingLeftRes");
		}		
		
//		//使腰部护理到达水平状态
//		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD4_Start)&&(direct==1))
//		{
//			DIR4=1;
//			Motor_4_START(motor_body_freq,motor_timer_freq);              //电机启动
//			TIM10_Init(add_arr,timer10_freq);                             //打开定时器35000
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位	 	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //等待定时时间到，时间到跳出循环
//			{
//				if(0==GD4_Start)      //如果电机过载或碰到光电开关，则跳出循环，电机停止转动 
//				{ 
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("GD4Start");			
//						break; 
//					}						
//				}				
//			}				                                 			
//			Motor_4_STOP();      //电机停止
//			TIM10_Stop();        //定时器关闭
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//		}
	}
	else
	{
		LED0=0;        //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("WaistNursingLeftInterfere");
		LED0=1;	
		LED1=1;
	}
}

/***********************************************************************
 函数名      ：Uart_Waist_Nursing_Right(void)  
 函数功能    ：右腰部护理
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Waist_Nursing_Right(void)
{
	u8 direct,key,len;
	u16 arr_feed;      //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;   //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;  
	u8 break_flag=0;      //判断程序从break跳出
	static u8 k=0;        //发送第k张动画指令
	u8 i=0;
	u8 j=0;	
	u16 arr_send;	      //当前一次运行脉冲数
	static u8 kj;
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	
	//联锁功能，只有在已执行右翻身功能且右背部护理复位后，才能进行右腰部护理
	if((lock_flag==1)&&(body_right_flag==1)&&(back_nursing_right_flag==0))
	{
		waist_nursing_right_flag=!waist_nursing_right_flag;	        //右腰部护理复位状态标志位
		waist_nursing_right_dir_flag=!waist_nursing_right_dir_flag;	//右腰部护理方向标志位	
		if(waist_nursing_right_dir_flag==1)
		{ 
			DIR4=0; direct=1;
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_1");			
			delay_ms(200);
			u2_printf("waist_nursing_right_flag==1");
			delay_ms(200);
			u2_printf("WaistNursingRightStart");	
			delay_ms(200);
		}
		else
		{ 
			DIR4=1; direct=0;
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_20");
			delay_ms(200);
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_4_START((u16)(motor_body_freq),motor_timer_freq);             //电机启动
		TIM10_Init(body_right_runed_arr,timer10_freq);               //打开定时器
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //等待定时时间到，时间到跳出循环
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//光电限位
//				if((0==GD4_Right_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD4_Right_End)
//					{
//						break_flag=1;
//						u2_printf("GD4RightEnd");
//						break;				
//					}	
//				}
//				if((0==GD4_Start)&&(0==direct))     //落下
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{	
//						break_flag=1;
//						u2_printf("GD4Start");
//						break;									
//					}
//				}
				  //判断有没有收到上位机指令		
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;												
				}
				//电机故障、故障诊断
//				if(1==Motor4_Alm)        
//				{	
//					delay_us(100);
//					if(1==Motor4_Alm) 
//					{
//						waist_nursing_right_overload=1;
//						u2_printf("WaistNursingRightOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;				
//					}		             
//				}							
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}	
			//发送动画指令
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_right_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=waist_nursing_right_picture_k;
			if(	k!=j)
			{
				kj=abs(k,j);							
				if(kj<2)	
				{
					k=j;   waist_nursing_right_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Waist_Nursing_Right_2");									
								break;
						case 2:	u2_printf("Cartoon_Waist_Nursing_Right_3");
								break;					
						case 3:	u2_printf("Cartoon_Waist_Nursing_Right_4");
								break;					
						case 4:	u2_printf("Cartoon_Waist_Nursing_Right_5");
								break;	
						case 5:	u2_printf("Cartoon_Waist_Nursing_Right_6");
								break;	
						case 6:	u2_printf("Cartoon_Waist_Nursing_Right_7");
								break;
						case 7:	u2_printf("Cartoon_Waist_Nursing_Right_8");									
								break;
						case 8:	u2_printf("Cartoon_Waist_Nursing_Right_9");
								break;					
						case 9:	u2_printf("Cartoon_Waist_Nursing_Right_10");
								break;					
						case 10:u2_printf("Cartoon_Waist_Nursing_Right_11");
								break;	
						case 11:u2_printf("Cartoon_Waist_Nursing_Right_12");
								break;	
						case 12:u2_printf("Cartoon_Waist_Nursing_Right_13");
								break;
						case 13:u2_printf("Cartoon_Waist_Nursing_Right_14");
								break;	
						case 14:u2_printf("Cartoon_Waist_Nursing_Right_15");
								break;	
						case 15:u2_printf("Cartoon_Waist_Nursing_Right_16");
								break;
						case 16:u2_printf("Cartoon_Waist_Nursing_Right_17");
								break;
						case 17:u2_printf("Cartoon_Waist_Nursing_Right_18");
								break;	
						case 18:u2_printf("Cartoon_Waist_Nursing_Right_19");
								break;							
					}
				}
			}
		}				
      
		Motor_4_STOP();    //电机停止
		TIM10_Stop();      //关闭定时器 
		break_flag=0;      //清除break标志位
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		
		if(waist_nursing_right_dir_flag==1)
		{ 
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_20");  
			delay_ms(200);
			u2_printf("WaistNursingRightLim");
		}
		else
		{ 
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_1");		
			delay_ms(200);
			u2_printf("waist_nursing_right_flag==0");
			delay_ms(200);
			u2_printf("WaistNursingRightRes");
		}		
	}
	else
	{
		LED0=0;        //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("WaistNursingRightInterfere");		
		LED0=1;	
		LED1=1;
	}
}


/***********************************************************************
 函数名      ：MOTOR111(void)   
 函数功能    ：
 输入        ：无
 输出        ：无                           
************************************************************************/
void MOTOR111(u8 dir)
{
	u8 direct,len;
	u16 arr_now;              //当前一次运行脉冲数
	
//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;          //判断程序是否是从break跳出来 
	u16 repeat_num;
	static u8 k;             //传第k张动画
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //当前一次运行脉冲值
	static u8 back_limit_flag; //支背运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位后，才能进行支背	
		if(dir==1)
		{				
			Motor_1_START(1);                                                          //支背上行
			u2_printf("Motor_1_START(1)");
		}
		else
		{
			Motor_1_START(0);                                                          //支背上行
			u2_printf("Motor_1_START(0)");
		}
		TIM10_Init(30000,timer10_freq); //打开定时器

		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;									
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
							
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环   
		{		}		
		Motor_1_STOP();    //电机停止
		TIM10_Stop();      //关闭定时器
		break_flag=0;      //标志位清零
		//判断复位
		u2_printf("Motor_1_STOP()");
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
}

/***********************************************************************
 函数名      ：MOTOR222(void)   
 函数功能    ：
 输入        ：无
 输出        ：无                           
************************************************************************/
void MOTOR222(u8 dir)
{
	u8 direct,len;
	u16 arr_now;              //当前一次运行脉冲数
	
//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;          //判断程序是否是从break跳出来 
	u16 repeat_num;
	static u8 k;             //传第k张动画
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //当前一次运行脉冲值
	static u8 back_limit_flag; //支背运行到极限位置置1，发送极限位置图片
	
	//联锁功能，只有在左右翻身功能复位后，才能进行支背	
		if(dir==1)
		{				
			Push_Rod_Start(1); 
            u2_printf("Push_Rod_Start(1)");           //支背上行
		}
		else
		{
			Push_Rod_Start(0);                                                          //支背上行
			u2_printf("Push_Rod_Start(0)");
		}
		
		TIM10_Init(30000,timer10_freq); //打开定时器		
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;									
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
							
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环   
		{		}		
		Push_Rod_Stop();   //电机停止
		TIM10_Stop();      //关闭定时器
		break_flag=0;      //标志位清零
		//判断复位
		u2_printf("Push_Rod_Stop()");
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
}


/***********************************************************************
 函数名      ：MOTOR333(void)  
 函数功能    ：
 输入        ：无
 输出        ：无 

************************************************************************/
void MOTOR333(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;	 
	u8 break_flag=0;    //判断程序是否从break跳出
					
		if(dir==1)
		{ 
			u2_printf("\r\nDIR3=1\r\n");
			DIR3=0; direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR3=0\r\n");
			DIR3=1; direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_3_START(motor_body_freq,motor_timer_freq);	       //电机启动
		TIM10_Init(8000,timer10_freq);              //打开定时器body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环   
		{		
				//光电限位
//				if((0==GD3_Left_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD3_Left_End)
//					{
//						u2_printf("\r\nGD3_Left_End\r\n");
//						break;
//					}
//				}
//				if((0==GD3_Start)&&(0==direct))     //落下
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						u2_printf("\r\nGD3_Start\r\n");
//						break;
//					}
//				}
//				if((0==GD3_Right_End)&&(1==direct))     //落下
//				{
//					u2_printf("\r\nGD3_Right_End\r\n");
//					break;						
//				}
				//电机故障、故障诊断
//				if(1==Motor3_Alm)        
//				{
//					delay_us(100);
//					if(1==Motor3_Alm) 
//					{
//						u2_printf("\r\nMotor3_Alm\r\n");
//						break;		       
//					}						
//				}															
		}				  
		Motor_3_STOP();     //电机停止
		TIM10_Stop();       //关闭定时器    
		break_flag=0;       //清除break标志位
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位		
		u2_printf("\r\nMotor_3_STOP\r\n");
}

/***********************************************************************
 函数名      ：MOTOR444(void)  
 函数功能    ：
 输入        ：无
 输出        ：无 

************************************************************************/
void MOTOR444(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	u8 break_flag=0;    //判断程序是否从break跳出
					
		if(dir==1)
		{ 
			u2_printf("\r\n0IR4=1\r\n");
			DIR4=0;   direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR4=0\r\n");
			DIR4=1; direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_4_START(motor_body_freq,motor_timer_freq);	       //电机启动
		TIM10_Init(8000,timer10_freq);              //打开定时器body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环   
		{		
				//光电限位
//				if((0==GD4_Left_End)&&(1==direct))     //起来
//				{
//					delay_us(100);
//					if(0==GD4_Left_End)
//					{
//						u2_printf("\r\nGD4_Left_End\r\n");
//						break;
//					}
//				}
//				if((0==GD4_Start)&&(0==direct))                     //落下
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("\r\nGD4_Start\r\n");
//						break;
//					}
//											
//				}
//				if((0==GD4_Right_End)&&(1==direct))     //落下
//				{
//					u2_printf("\r\nGD4_Right_End\r\n");
//					break;						
//				}
				//电机故障、故障诊断
//				if(1==Motor4_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor4_Alm) 
//					{
//						u2_printf("\r\nMotor4_Alm\r\n");
//						break;
//					}
//				}													
		}				  
		Motor_4_STOP();     //电机停止
		TIM10_Stop();       //关闭定时器    
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		u2_printf("\r\nMotor_4_STOP\r\n");
}


/************************************************************************
 函数名      ：MOTOR555(void)  
 函数功能    ：
 输入        ：无
 输出        ：无 

************************************************************************/
void MOTOR555(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	u8 break_flag=0;    //判断程序是否从break跳出
					
		if(dir==1)
		{ 
			u2_printf("\r\nDIR5=1\r\n");
			DIR5=1; direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR5=0\r\n");
			DIR5=0; direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_5_START(motor_body_freq,motor_timer_freq);	       //电机启动
		TIM10_Init(8000,timer10_freq);              //打开定时器body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环   
		{		
				//光电限位
//				if((0==GD5_Left_End))  //起来
//				{
//					u2_printf("\r\n0==GD5_Left_End\r\n");
//					break;	
//				}
						
//				if((0==GD5_Left_End)&&(1==direct))  //起来
//				{
//					delay_us(100);
//					if(0==GD5_Left_End)
//					{
//						u2_printf("\r\n0==GD5_Left_End\r\n");
//						break;	
//					}
//				}
//				if((0==GD5_Start)&&(0==direct))     //落下
//				{
//					delay_us(100);
//					if(0==GD5_Start)
//					{
//						u2_printf("\r\n0==GD5_Start\r\n");
//						break;
//					}
//				}
//				if((0==GD5_Right_End)&&(1==direct))     //落下
//				{
//					u2_printf("\r\nGD5_Right_End\r\n");
//					break;						
//				}
				//电机故障、故障诊断
//				if(1==Motor5_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor5_Alm)
//					{
//						u2_printf("\r\n1==Motor5_Alm\r\n");
//						break;
//					}
//				}														
		}				  
		Motor_5_STOP();     //电机停止
		TIM10_Stop();       //关闭定时器    
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		u2_printf("\r\nMotor_5_STOP\r\n");
}

/************************************************************************
 函数名      ：MOTOR666(void)  
 函数功能    ：
 输入        ：无
 输出        ：无 

************************************************************************/
void MOTOR666(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	u8 break_flag=0;    //判断程序是否从break跳出
					
		if(dir==1)
		{ 
			u2_printf("\r\nDIR6=0\r\n");
			DIR6=0;   direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR6=1\r\n");
			DIR6=1;   direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_6_START(motor_body_freq,motor_timer_freq);	       //电机启动
		TIM10_Init(30000,timer10_freq);              //打开定时器body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环   
		{		
				//光电限位
//				if((0==GD6_End)&&(direct==1))  //起来
//				{
//					delay_us(100);
//					if(0==GD6_End)
//					{
//						u2_printf("\r\n0==GD6_End\r\n");
//						break;
//					}					
//				}
//				if((0==GD6_Start)&&(direct==0))     //落下
//				{
//					delay_us(100);
//					if(0==GD6_Start)
//					{
//						u2_printf("\r\n0==GD6_Start\r\n");
//						break;
//					}
//				}
//			
//				//电机故障、故障诊断
//				if(1==Motor6_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor6_Alm)
//					{
//						u2_printf("\r\nMotor6_Alm\r\n");
//						break;
//					}
//				}															
		}				  
		Motor_6_STOP();     //电机停止
		TIM10_Stop();       //关闭定时器    
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		u2_printf("\r\nMotor_6_STOP\r\n");
}

/************************************************************************
 函数名      ：MOTOR777(void)  
 函数功能    ：左背部护理
 输入        ：无
 输出        ：无 

************************************************************************/
void MOTOR777(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;    //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	u8 break_flag=0;    //判断程序是否从break跳出
					
		if(dir==1)
		{ 
			u2_printf("\r\nDIR7=0\r\n");
			DIR7=0; direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR7=1\r\n");
			DIR7=1; direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_7_START(motor_desk_freq,motor_timer_freq);	       //电机启动
		TIM10_Init(64000,timer10_freq);              //打开定时器body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环   
		{		
				//光电限位
//				if((0==GD7_End)&&(1==direct))  //起来
//				{
//					u2_printf("\r\n0==GD7_End\r\n");
//					break;	
//				}
//				if((0==GD7_Start)&&(0==direct))     //落下
//				{
//					u2_printf("\r\n0==GD7_Start\r\n");
//					break;						
//				}
			
				//电机故障、故障诊断
//				if(1==Motor7_Alm)        
//				{						
//					u2_printf("\r\nMotor7_Alm\r\n");
//					break;		             
//				}															
		}				  
		Motor_7_STOP();     //电机停止
		TIM10_Stop();       //关闭定时器    
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
		u2_printf("\r\nMotor_7_STOP\r\n");
}


/***********************************************************************
 函数名      ：Liandong_Test(void)  
 函数功能    ：联动测试函数
 输入        ：无
 输出        ：无                           
************************************************************************/
void WriteInUART2(char *p)
{
	u8 len=strlen(p);
	u8 i;
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
	for(i=0;i<len;i++)
	{
	  USART2_RX_BUF[i]=p[i];
	}	
}

void LDUART2(void)
{
	//左翻
	WriteInUART2("BodyLeftUpPhone");		
	Uart_Body_Left();           //左翻身	
	delay_ms(1000);
	Uart_Back_Nursing_Left();   //左背部护理	
	delay_ms(1000);
	Uart_Back_Nursing_Left();   //左背部护理复位	
	delay_ms(1000);
	Uart_Waist_Nursing_Left();  //左腰部护理
	delay_ms(1000);
	Uart_Waist_Nursing_Left();  //左腰部护理复位	
	delay_ms(1000);
	WriteInUART2("BodyLeftDownPhone");
	Uart_Body_Left();           //左翻身复位
	delay_ms(1000);

	//右翻
//	WriteInUART2("BodyRightUpPhone");
//	Uart_Body_Right();          //右翻身
//	delay_ms(1000);
//	Uart_Back_Nursing_Right();  //右背部护理
//	delay_ms(1000);
//	Uart_Back_Nursing_Right();  //右背部护理复位
//	delay_ms(1000);
//	Uart_Waist_Nursing_Right(); //右腰部护理
//	delay_ms(1000);
//	Uart_Waist_Nursing_Right(); //右腰部护理复位
//	delay_ms(1000);
//	WriteInUART2("BodyRightDownPhone");
//	Uart_Body_Right();          //右翻身复位
//	delay_ms(1000);
	
	//上曲腿
	WriteInUART2("LegUpUpPhone");
	Uart_Leg_Up();              //上曲腿
	delay_ms(1000);
	WriteInUART2("LegUpDownPhone");
	Uart_Leg_Up();              //上曲腿复位
	delay_ms(1000);
	
	//形成坐姿-支背、下曲腿、坐便器、小桌子
	
	//支背
//	WriteInUART2("BackUpPhone");
//	Uart_Back();                //支背
//	delay_ms(1000);
//	
//	//下曲腿
//	WriteInUART2("LegDownDownPhone");
//	Uart_Leg_Down();            //下曲腿	
//	delay_ms(1000);
//	
//	//坐便器
//	Uart_Washlet(0);            //坐便器打开
//	delay_ms(1000);
//	Uart_Washlet(1);            //坐便器关闭
//	delay_ms(1000);
//	
//	//冲洗烘干
//	washlet_flag=1;	//伸
//	RELAY6=1; 
//	Uart_Push_Rod_Swash(1,5000);
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(0,5000);	//缩
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(1,5000);
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(0,5000);	//缩
//	RELAY6=0;
//	delay_ms(1000);
//	
//	//收线
//	washlet_flag=1;
//	RELAY6=1;                  //继电器得电
//	Uart_Washlet_Tig(0);
//	RELAY6=0;   
//	delay_ms(1000);

//自动坐便
	WriteInUART2("WashletAutoPhone");
	Uart_Washlet_Auto();
  delay_ms(1000);
	
	//小桌子
	WriteInUART2("DeskUpPhone");
	Uart_Desk();               //小桌子靠近
	delay_ms(1000);
	WriteInUART2("DeskDownPhone");
	Uart_Desk();               //小桌子后退
	delay_ms(1000);	
	
	
//	WriteInUART2("LegDownUpPhone");
//	Uart_Leg_Down();            //下曲腿复位
//	delay_ms(1000);
//	
//	WriteInUART2("BackDownPhone");
//	Uart_Back();                //支背复位

//	TestAll(1);
//	delay_ms(1000);
//	TestAll(0);

}


void LDUART2V2(void)
{
	u32 hangid;
	//左翻
	u2_printf("\r\n\r\n*****左翻******\r\n\r\n");
	WriteInUART2("BodyLeftUpPhone");		
	Uart_Body_Left();           //左翻身	
	delay_ms(1000);
	Uart_Back_Nursing_Left();   //左背部护理	
	delay_ms(1000);
	Uart_Back_Nursing_Left();   //左背部护理复位	
	delay_ms(1000);
	Uart_Waist_Nursing_Left();  //左腰部护理
	delay_ms(1000);
	Uart_Waist_Nursing_Left();  //左腰部护理复位	
	delay_ms(1000);
	WriteInUART2("BodyLeftDownPhone");
	Uart_Body_Left();           //左翻身复位
	delay_ms(1000);

	//右翻
		u2_printf("\r\n\r\n*****右翻******\r\n\r\n");
	WriteInUART2("BodyRightUpPhone");
	Uart_Body_Right();          //右翻身
	delay_ms(1000);
	Uart_Back_Nursing_Right();  //右背部护理
	delay_ms(1000);
	Uart_Back_Nursing_Right();  //右背部护理复位
	delay_ms(1000);
	Uart_Waist_Nursing_Right(); //右腰部护理
	delay_ms(1000);
	Uart_Waist_Nursing_Right(); //右腰部护理复位
	delay_ms(1000);
	WriteInUART2("BodyRightDownPhone");
	Uart_Body_Right();          //右翻身复位
	delay_ms(1000);
	
	//上曲腿
		u2_printf("\r\n\r\n*****上曲腿******\r\n\r\n");
	WriteInUART2("LegUpUpPhone");
	Uart_Leg_Up();              //上曲腿
	delay_ms(1000);
	WriteInUART2("LegUpDownPhone");
	Uart_Leg_Up();              //上曲腿复位
	delay_ms(1000);
	
	//形成坐姿-支背、下曲腿、坐便器、小桌子
	
	//支背
		u2_printf("\r\n\r\n*****支背******\r\n\r\n");
	WriteInUART2("BackUpPhone");
	Uart_Back();                //支背
	delay_ms(1000);
//	
//	//下曲腿
	u2_printf("\r\n\r\n*****下曲腿******\r\n\r\n");
	WriteInUART2("LegDownDownPhone");
	Uart_Leg_Down();            //下曲腿	
	delay_ms(1000);

//坐便器
	u2_printf("\r\n\r\n*****坐便器******\r\n\r\n");
	WriteInUART2("LegDownDownPhone");
	WashLet_V2(1,32950);

//	
//	//坐便器
//	Uart_Washlet(0);            //坐便器打开
//	delay_ms(1000);
//	Uart_Washlet(1);            //坐便器关闭
//	delay_ms(1000);
//	
//	//冲洗烘干
//	washlet_flag=1;	//伸
//	RELAY6=1; 
//	Uart_Push_Rod_Swash(1,5000);
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(0,5000);	//缩
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(1,5000);
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(0,5000);	//缩
//	RELAY6=0;
//	delay_ms(1000);
//	
//	//收线
//	washlet_flag=1;
//	RELAY6=1;                  //继电器得电
//	Uart_Washlet_Tig(0);
//	RELAY6=0;   
//	delay_ms(1000);

//自动坐便
//	WriteInUART2("WashletAutoPhone");
//	Uart_Washlet_Auto();
//  delay_ms(1000);
	

	
		u2_printf("\r\n\r\n*****下曲腿复位******\r\n\r\n");
	WriteInUART2("LegDownUpPhone");
	Uart_Leg_Down();            //下曲腿复位
	delay_ms(1000);
//	
	u2_printf("\r\n\r\n*****支背复位******\r\n\r\n");
	WriteInUART2("BackDownPhone");
	Uart_Back();                //支背复位
		
		Muscle_Massager();	
		delay_ms(5000);delay_ms(2000);
		Muscle_Massager();	
		
		
	WriteInUART2("HangRunUp+11+1100");
	DG_Relay=1;		//继电器得电
	hangid=usmart_strnum2(USART2_RX_BUF);			
	if(strstr((const char *)USART2_RX_BUF,(const char *)"Up"))
	{
			HangRun(1,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	else if(strstr((const char *)USART2_RX_BUF,(const char *)"Down"))
	{
		HangRun(0,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	
	
		WriteInUART2("HangRunUp+11+0011");
	DG_Relay=1;		//继电器得电
	hangid=usmart_strnum2(USART2_RX_BUF);			
	if(strstr((const char *)USART2_RX_BUF,(const char *)"Up"))
	{
			HangRun(1,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	else if(strstr((const char *)USART2_RX_BUF,(const char *)"Down"))
	{
		HangRun(0,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	
//	
	WriteInUART2("HangRunDown+11+1111");
	hangid=usmart_strnum2(USART2_RX_BUF);			
	if(strstr((const char *)USART2_RX_BUF,(const char *)"Up"))
	{
			HangRun(1,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	else if(strstr((const char *)USART2_RX_BUF,(const char *)"Down"))
	{
		HangRun(0,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}	
//	
	DG_Relay=0;		//继电器失电	
	u2_printf("\r\n\r\n*****运行结束******\r\n\r\n");	
	FlagClear();
}




/***********************************************************************
 函数名      ：Uart_GB_Back(void)  
 函数功能    ：护栏支背按键联动
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_GB_Back(void)
{
	static u8 back_limit_flag; //判断支背是否运行到极限位置，若是发送极限位置图片
	u8 direct,key;
	u16 arr_now;               //当前一次运行脉冲数，用于脉冲累计 
	u8 len;	                   //接受的字符串长度
//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //程序从break跳出标志位
	static u8 k=0;            //发送第k张图片指令
	u8 i=0;
	u8 j=0;	
	u16 arr_send;             //当前一次运行脉冲数
	static u8 kj;
	//联锁功能，只有在左右翻身功能复位后，才能进行支背
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{			
		back_dir_flag=!back_dir_flag;    //支背方向标志位
		if(back_dir_flag==1)
		{
			if(back_angle_to_arr(back_angle_lim)>back_runed_arr)  //上行时，比较值为上行极限装载值与校准后的当量装载值的差值，两个差值不能为0
			{
				direct=1;
				if(back_flag==0)
				{
					back_flag=1;
					delay_ms(200);
					u2_printf("back_flag==1");
					delay_ms(200);
					u2_printf("Guardbar_Back_Start");
					delay_ms(200);
					u2_printf("Cartoon_Back_1");
					delay_ms(200);
				}
				Motor_1_START(1);
				TIM10_Init(back_angle_to_arr(back_angle_lim)-back_runed_arr,timer10_freq);
			}
		}
		if(back_dir_flag==0)
		{
			if(back_runed_arr>0)    //下行时，比较值为校准后的当量装载值与0,且不能为0
			{
				direct=0;
				if(back_limit_flag==1)
				{
					back_limit_flag=0;
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
				}
				Motor_1_START(0);
				TIM10_Init(back_runed_arr,timer10_freq);
			}
		}				
		
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位	 	
		
		if(((back_runed_arr!=back_angle_to_arr(back_angle_lim))&&(1==direct))||((0!=back_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环 
			{			
				for(repeat_num=0;repeat_num<700;repeat_num++)
				{
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BackGB"))||(strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))  )   //若接收到Stop,则跳出循环	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else 
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数								
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}
				}	
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送图片指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(back_runed_arr+arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				else
				{
					j=abs(back_runed_arr,arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}			
				k=back_picture_k;			
				if(	k!=j)
				{
					kj=abs(k,j);												
					if(kj<2)
					{
						k=j;	back_picture_k=k;
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
			Motor_1_STOP();        //电机停止
			TIM10_Stop();          //关闭定时器
			break_flag=0;	       //清除break标志位	
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //判断是否处于复位状态，复位状态的前提是下行的定时器走完
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
			if(	direct==1)        //如果是支背上行，则用+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{  
					back_runed_arr=back_angle_to_arr(back_angle_lim); 
					back_limit_flag=1; 
					delay_ms(200);
					u2_printf("Cartoon_Back_20");		
					delay_ms(200);
					u2_printf("GuardbarBackLim");
				}
				else
				{  back_runed_arr=back_runed_arr+arr_now;	}				
			}
			else                //如果是支背下行，则用-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{	
					back_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Back_1"); 
					delay_ms(200);
					u2_printf("GuardbarBackRes");					
				}
				else
				{
					back_runed_arr=back_runed_arr-arr_now;
				}						
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 								
		}
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("BackInterfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_GB_Body_Left(void)  
 函数功能    ：护栏左翻身
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_GB_Body_Left(void)
{
	u8 direct,key;
	u16 arr_now;              //当前一次运行脉冲数，用于脉冲累计
	u8 len;                   //接受的字符串长度
	u16 arr_feed;             //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;          // 将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较 
	u16 num1=0,num2=0,num3=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //程序从brak跳出标志位      
	static u8 M345_Start;     //345电机从初始位置运动
	static u8 M345_End;       //345电机运行到上极限位置标志位  
	static u8 k=0,m=0;
	u8 i=0;
	u8 mn;
	u8 kj;
	u8 j=0,n=0;
	u16 arr_send,arr_send1;	 //当前一次运行脉冲数	
	
	//联锁功能，只有在支背、上下曲腿、座便、桌子复位后，才能执行左翻身功能
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{			
		if(body_left_flag==0)   //如果复位到初始状态，才执行小侧翻左翻起
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftGB"))
			{
			    //5号侧翻起
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				delay_ms(200);
				u2_printf("GuardbarBodyLeftStart");
				delay_ms(200);
				u2_printf("body_left_flag=1");				
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				DIR5=1; 				
				body_left_flag=1;
				W25QXX_Write((u8*)&body_left_flag,33,1);				
				Motor_5_START(motor_body_freq,motor_timer_freq);	              //电机启动	
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);  //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);               //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{	
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{				
						//光电限位
//						if((0==GD5_Left_End)&&(1==body_left_flag))  //运行时碰到光电开关，跳出循环 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;			
//							}								
//						}
						  //判断有没有收到上位机指令		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}											
						//电机故障、故障诊断
//						if(1==Motor5_Alm)       
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("GuardbarBodyLeftOverload5");
//								Uart_Breakdown_Treatment();
//								break_flag=1;
//								break;
//							}								
//						}						
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送图片指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					m=left_motor5_picture_m;
					if(m!=n)
					{
						mn=abs(m,n);					
						if(mn<2)
						{
							m=n;	left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}							
						}
					}					
				}					      
				Motor_5_STOP();       //电机停止
				TIM10_Stop();         //定时器关闭
				break_flag=0;  	      //清除break标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(0,4500,motor_body_freq,motor_timer_freq);//调用补偿函数
			}	
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		//翻身345电机动作	
		if(1==body_left_flag)
		{
			body_left_dir_flag=!body_left_dir_flag;
			if(body_left_dir_flag==1)    //左翻起
			{				
				if(body_angle_to_arr(body_left_angle_lim)>body_left_runed_arr)
				{
				   //345联动左翻起				
					 DIR3=0;DIR4=0;DIR5=0;direct=1;
					 if(M345_Start==0)
					 {
						 M345_Start=1;
						 u2_printf("Cartoon_Body_Left_1");
						 delay_ms(200);
					 }								 
					 Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	  //电机启动			
					 TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);									 
				}
			}
			else          //左翻落
			{				
				if(body_left_runed_arr>0)
				{
				   //345联动左翻落
				   DIR3=1;DIR4=1;DIR5=1;direct=0;
					if(M345_End==1)
					 {
						 M345_End=0;
						 u2_printf("Cartoon_Body_Left_8");
						 delay_ms(200);
					 }											
				   Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);   //电机启动			
				   TIM10_Init(body_left_runed_arr,timer10_freq);	           //打开定时器		
				} 
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);              //清除中断标志位
		
		if(((body_left_runed_arr!=body_angle_to_arr(body_left_angle_lim))&&(1==direct))||((0!=body_left_runed_arr)&&(0==direct)))
		{			
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //等待定时时间到，时间到跳出循环
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{
					//光电限位
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //如果碰到光电开关，跳出循环，停止运行
//					{						
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");
//							break_flag=1;
//							break;
//						}
//						
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))   //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if((0==GD3_Left_End)||(0==GD4_Left_End))
//						{
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3LeftEnd");						
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4LeftEnd");
//							}
//							break_flag=1;
//							break;
//						}
//					}
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftGB"))||(strstr((const char *)USART2_RX_BUF,(const char *)"Stop")))  //若接收到Stop,则跳出循环	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						} 
						else 
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}												
					}				
					//电机故障、故障诊断
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_left_overload_3=1;
//							u2_printf("GuardbarBodyLeftOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_left_overload_4=1;
//							u2_printf("GuardbarBodyLeftOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_left_overload_5=1;
//							u2_printf("GuardbarBodyLeftOverload5");
//						}						
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;		             
//					}									
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_left_runed_arr+arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				else
				{
					j=abs(body_left_runed_arr,arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				k=body_left_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;   body_left_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Left_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Left_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Left_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Left_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Left_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Left_7");
									break;		
						}
					}				
				}					
			}							
			Motor_3_4_5_STOP();   //电机停止
			TIM10_Stop();         //定时器关闭
			break_flag=0;         //清除break标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))  //判断是否到达复位状态
			{
				arr_now=0;                  
				body_left_flag=0;
//				W25QXX_Write((u8*)&body_left_flag,33,1);			
			}					
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);      //获取当前计数值arr_now
				body_left_flag=1;
//				W25QXX_Write((u8*)&body_left_flag,33,1);	
			}
			//通过上下行判断脉冲累计
			if(direct==1)       //上行，则用+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Left_End)||(0==GD4_Left_End))  //向上运行到极限位置
				{
					body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);					
					M345_End=1;	 										
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_8");
					delay_ms(200);
					u2_printf("GuardbarBodyLeftLim");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr+arr_now;
				}		
			}	
			else                //下行，则用-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))      //向下运行到极限位置
				{						
					body_left_runed_arr=0;
					M345_Start=0;											
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_1");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr-arr_now;	
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位
			
			//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_left_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //光电3没到位
//				{
//					DIR3=1;
//					Motor_3_START(motor_body_freq,motor_timer_freq);    //电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD3_Start) 
//							{
//								u2_printf("GD3Start");			
//								break;	
//							}								
//						}
//					}			
//					Motor_3_STOP();       //电机停止
//					TIM10_Stop();         //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //光电4没到位
//				{
//					DIR4=1;
//					Motor_4_START(motor_body_freq,motor_timer_freq);   //电机启动
//					TIM10_Init(add_arr,timer10_freq);                  //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD4_Start) 
//							{
//								u2_printf("GD4Start");
//								break;
//							}
//						}
//					}			
//					Motor_4_STOP();                                     //电机停止
//					TIM10_Stop();                                       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //光电3和4都没到位
//				{
//					DIR3=1; DIR4=1; DIR5=1;
//					Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);//电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//								break;
//							}
//							if((0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//								break;
//							}
//						}
//					}			
//					Motor_3_4_5_STOP();   //电机停止
//					TIM10_Stop();         //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
//			}
			//侧翻复位
			if((body_left_flag==0)&&(0==direct))     //345联动复位到初始状态，才复位5号电机
			{			
				//5号侧翻复位
//				Motor_4_Compensate(1,4500,motor_body_freq,motor_timer_freq);//调用补偿函数
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				DIR5=0;
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);                 //电机启动
				body_left_runed_arr=0;
				
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq); //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);              //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{
						//光电限位
//						if((0==GD5_Start)&&(0==body_left_flag))    //碰到光电开关跳出循环，电机停转 
//						{								
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;			
//							}								
//						}
						  //判断有没有收到上位机指令		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}									
						//电机故障、故障诊断
//						if(1==Motor5_Alm)         
//						{
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("GuardbarBodyLeftOverload5");
//								Uart_Breakdown_Treatment();
//								break_flag=1;
//								break;
//							}								
//						}						
					}	
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//发送动画指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					n=9-n;	
					m=left_motor5_picture_m;
					if(	m!=n)
					{			
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;    left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}	
						}				
					}	
				}      
				Motor_5_STOP();     //电机停止
				TIM10_Stop();       //定时器关闭
				break_flag=0;       //清除break标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				u2_printf("body_left_flag==0");				
				delay_ms(200);
				u2_printf("GuardbarBodyLeftRes");
				//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=0;
//					Motor_5_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break;		
//							}								
//						}
//					}			
//					Motor_5_STOP();
//					TIM10_Stop();       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}			
			}				
		}
	}
	else
	{
		LED0=0;   //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("BodyLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_GB_Body_Right(void)  
 函数功能    ：护栏右翻身
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_GB_Body_Right(void)
{
	u8 direct,key;
	u16 arr_now;              //当前一次运行脉冲数，用于脉冲累计
	u8 len;                   //接收的字符串长度
	u16 arr_feed;             //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;          //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num1=0,num2=0,num3=0;
	//实现上位机实时显示护理床当前运动状态
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //程序从break跳出标志位 
	static u8 k=0,m=0;
	static u8 M345R_Start;    //345电机从初始位置启动
	static u8 M345R_End;      //345电机运行到上极限位置
	u8 mn;
	u8 kj;
	u8 i=0;
	u8 j=0,n=0;
	u16 arr_send,arr_send1;	  //当前一次运行脉冲数	
	
	//联锁功能，只有在支背、上下曲腿、座便、桌子复位后，才能执行右翻身功能
	if((lock_flag==1)&&(back_flag==0)&&(body_left_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_right_flag==0)&&(waist_nursing_right_flag==0))
	{			
		if(body_right_flag==0)   //如果复位到初始状态，才执行左翻起
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightGB"))
			{
			  //5号侧翻起
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				delay_ms(200);
				u2_printf("GuardbarBodyRightStart");
				delay_ms(200);
				u2_printf("body_right_flag==1");
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);
				DIR5=0; 
				body_right_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);				
				Motor_5_START(motor_body_freq,motor_timer_freq);	              //电机启动	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq); //打开定时器                        //打开定时器
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);               //清除中断标志位
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{	
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{				
						//光电限位
//						if((0==GD5_Right_End)&&(1==body_right_flag))                      //运行时碰到光电开关，跳出循环 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Right_End)
//							{
//								u2_printf("GD5RightEnd");
//								break_flag=1;
//								break;		
//							}								
//						}
						  //判断有没有收到上位机指令		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}											
						//电机故障、故障诊断
//						if(1==Motor5_Alm)         
//						{		
//							delay_us(100);
//							if(1==Motor5_Alm)  
//							{
//								body_right_overload_5=1;
//								u2_printf("GuardbarBodyRightOverload5");
//								Uart_Breakdown_Treatment();
//								break_flag=1;
//								break;	
//							}								
//						}											
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					} 
					//发送图片指令
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					m=right_motor5_picture_m;
					if(m!=n)
					{
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}	
						}					
					}		
				}						      
				Motor_5_STOP();        //电机停止
				TIM10_Stop();          //定时器关闭
			    break_flag=0;          //清除break标志位
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(1,4500,motor_body_freq,motor_timer_freq);//调用补偿函数
			}	
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		//翻身345电机动作	
		if(1==body_right_flag)
		{
			body_right_dir_flag=!body_right_dir_flag;
			if(body_right_dir_flag==1)
			{				
				if(body_angle_to_arr(body_right_angle_lim)>body_right_runed_arr)
				{
				   //345联动左翻起		
				   DIR3=1;DIR4=1;DIR5=1;direct=1;
				   if(M345R_Start==0)
				   {
					   M345R_Start=1;	
					   delay_ms(200);
					   u2_printf("Cartoon_Body_Right_1");
					   delay_ms(200);
				   }									 		
				   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq); //电机启动				
				   TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);//打开定时器			
				}
			}
			else
			{				
				if(body_right_runed_arr>0)
				{
				   //345联动左翻起
				   DIR3=0;DIR4=0;DIR5=0;direct=0;
				   if(M345R_End==1)
				   {
						M345R_End=0;
						delay_ms(200);
					    u2_printf("Cartoon_Body_Right_8");	
					    delay_ms(200);
					}							 	
				   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //电机启动			
				   TIM10_Init(body_right_runed_arr,timer10_freq);	      //打开定时器		
				}
			}
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //清除中断标志位		
		if(((body_right_runed_arr!=body_angle_to_arr(body_right_angle_lim))&&(1==direct))||((0!=body_right_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到，时间到跳出循环
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//光电限位
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //如果碰到光电开关，跳出循环，停止运行
//					{						
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");
//							break_flag=1;
//							break;
//						}
//						if((0==GD4_Start))
//						{
//							u2_printf("GD4Start");
//							break_flag=1;
//							break;
//						}
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))   //如果碰到光电开关，跳出循环，停止运行
//					{
//						delay_us(100);
//						if(0==GD3_Left_End)
//						{
//							u2_printf("GD3LeftEnd");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Left_End)
//						{
//							u2_printf("GD4LeftEnd");
//							break_flag=1;
//							break;
//						}
//					}
					//停止指令
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightGB"))||(strstr((const char *)USART2_RX_BUF,(const char *)"Stop")) )    //若接收到Stop,则跳出循环	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						} 
						else 
						{
							u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}
					}			
					//电机故障、故障诊断
//					if((1==Motor3_Alm) || (1==Motor4_Alm) || (1==Motor5_Alm))       
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_right_overload_3=1;
//							u2_printf("GuardbarBodyRightOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_right_overload_4=1;
//							u2_printf("GuardbarBodyRightOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_right_overload_5=1;
//							u2_printf("GuardbarBodyRightOverload5");
//						}					
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;		             
//					}							
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_right_runed_arr+arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				else
				{
					j=abs(body_right_runed_arr,arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				k=body_right_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);			
					if(kj<2)
					{
						k=j;   body_right_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_7");
									break;	
						}
					}				
				}							
			}							
			Motor_3_4_5_STOP();   //电机停止
			TIM10_Stop();         //定时器关闭
			break_flag=0;	      //清除break标志位
			//判断复位
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))  //判断是否到达复位状态
			{
				arr_now=0;        //此时arr_now为0
				body_right_flag=0;	
//				W25QXX_Write((u8*)&body_right_flag,34,1);			
			}			
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);         //获取当前计数值arr_now
				body_right_flag=1;
//				W25QXX_Write((u8*)&body_right_flag,34,1);	
			}
			//通过上下行判断脉冲累计
			if(direct==1)      //上行，则用+
			{ 
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Right_End)||(0==GD4_Right_End)) //向上运行到极限位置
				{
					body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
					M345R_End=1;    								
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
					u2_printf("GuardbarBodyRightLim");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr+arr_now;
				}		
			}	
			else              //下行，则用-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))   //向下运行到极限位置
				{
					body_right_runed_arr=0;	
					M345R_Start=0;  								
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr-arr_now;	
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);       //清除中断标志位
//		//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_right_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //光电3没到位
//				{
//					DIR3=0;
//					Motor_3_START(motor_body_freq,motor_timer_freq);    //电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD3_Start) 
//							{
//								u2_printf("GD3Start");
//								break;	
//							}							
//						}
//					}			
//					Motor_3_STOP();     //电机停止
//					TIM10_Stop();       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //光电4没到位
//				{
//					DIR4=0;
//					Motor_4_START(motor_body_freq,motor_timer_freq);    //电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//								break;	
//							}							
//						}
//					}			
//					Motor_4_STOP();     //电机停止
//					TIM10_Stop();       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //光电3/4都没到位
//				{
//					DIR3=0; DIR4=0; DIR5=0;
//					Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);//电机启动
//					TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //运行时碰到光电开关，跳出循环 
//						{
//							delay_us(100);
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//								break;	
//							}
//							if((0==GD4_Start) 
//							{
//								u2_printf("GD4Start");
//								break;	
//							}
//						}
//					}			
//					Motor_3_4_5_STOP(); //电机停止
//					TIM10_Stop();       //定时器关闭
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//				}
//			}
		//侧翻复位
		if((body_right_flag==0)&&(0==direct))     //345联动复位到初始状态，才复位5号电机
		{			
			//5号侧翻复位
			Motor_4_Compensate(0,4500,motor_body_freq,motor_timer_freq);//调用补偿函数
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;
			DIR5=1;
			delay_ms(200);
			u2_printf("Cartoon_Body_Right_Motor5_10");
			delay_ms(200);
			Motor_5_START(motor_body_freq,motor_timer_freq);  //电机启动
			body_right_runed_arr=0;
		
			TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);                              //打开定时器
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //等待定时时间到，
			{				
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//光电限位
//					if((1==GD5_Start)&&(0==body_right_flag))       //运行时碰到光电开关，跳出循环 
//					{			   										
//						delay_us(100);
//						if(1==GD5_Start)
//						{	
//							u2_printf("GD5Start");	
//							break_flag=1;
//							break;		
//						}							
//					}
					  //判断有没有收到上位机指令		
					if(USART2_RX_LEN&0x8000)
					{
						u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
						memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
						USART2_RX_LEN=0;												
					}					
					//电机故障、故障诊断
//					if(1==Motor5_Alm)
//					{
//						delay_us(100);
//						if(1==Motor5_Alm)
//						{
//							body_right_overload_5=1;
//							u2_printf("GuardbarBodyRightOverload5");
//							Uart_Breakdown_Treatment();
//							break_flag=1;
//							break;		
//						}							
//					}									
				}				
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//发送动画指令
				 arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);			
				 n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
				 n=9-n;
				 m=right_motor5_picture_m;
				if(	m!=n)
				{	
					mn=abs(m,n);			
					if(mn<2)
					{
						m=n;   right_motor5_picture_m=m;
						switch (m)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
									break;
							case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
									break;
							case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
									break;							
						}
					}				
				}	
			}      
			Motor_5_STOP();        //电机停止
			TIM10_Stop();		   //定时器关闭
			break_flag=0;          //清除break标志位
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
			delay_ms(200);
			u2_printf("Cartoon_Body_Right_Motor5_1");
		    delay_ms(200);
			u2_printf("body_right_flag==0");			
		    delay_ms(200);
		    u2_printf("GuardbarBodyRightRes");
			//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段，并将上面的清除中断删除）
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//			{
//				DIR5=1;
//				Motor_5_START(motor_body_freq,motor_timer_freq);    //电机启动
//				TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//				{			
//					if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//					{
//						delay_us(100);
//						if(0==GD5_Start) 
//						{
//							u2_printf("GD5Start");
//							break;		
//						}							
//					}
//				}			
//				Motor_5_STOP();        //电机停止
//				TIM10_Stop();		   //定时器关闭
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//			}			
		}			
	 }
  }

	else
	{
		LED0=0;   //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("BodyRightInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_GB_Back_Nursing(void)  
 函数功能    ：护栏背部护理
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_GB_Back_Nursing(void)
{
	if((lock_flag==1)&&(1==body_left_flag))  //如果此时处于左翻身，则进行左背部护理
	{
		Uart_Back_Nursing_Left(); 
	}
	if((lock_flag==1)&&(1==body_right_flag))//如果此时处于右翻身，则进行右背部护理
	{
		Uart_Back_Nursing_Right();
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

/***********************************************************************
 函数名      ：Uart_GB_Back_Nursing(void)  
 函数功能    ：护栏腰部护理
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_GB_Waist_Nursing(void)
{
	if((lock_flag==1)&&(1==body_left_flag))  //如果此时处于左翻身，则进行左腰部护理
	{
		Uart_Waist_Nursing_Left(); 
	}
	if((lock_flag==1)&&(1==body_right_flag))//如果此时处于右翻身，则进行右腰部护理
	{
		Uart_Waist_Nursing_Right();
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;	
		
	}	
}

/***********************************************************************
 函数名      ：Uart_GB_Lock(void)  
 函数功能    ：一键锁定程序
 输入        ：无
 输出        ：无
                           
************************************************************************/
u8 Uart_GB_Lock(void)
{	
	lock_dir_flag=!lock_dir_flag;
	if(lock_dir_flag==1)
	{
		lock_flag=0;	//护栏锁定
		u2_printf("GBLock");
	}
	else
	{
		lock_flag=1;   //护栏解锁
		u2_printf("GBUnLock");
	}
    return 	lock_flag;
}

/***********************************************************************
 函数名      ：Uart_Washlet_Auto(void)   
 函数功能    ：按键执行自动坐便器功能
 输入        ：无
 输出        ：无 
                          
************************************************************************/
void Uart_Washlet_Auto(void) 
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	
	//联锁功能，只有在上曲腿、左右翻身复位后，才能执行座便器功能	
	if((lock_flag==1)&&(0==body_left_flag)&&(0==body_right_flag)) 
	{
		if(0==washlet_auto_flag)
		{		
			delay_ms(200);		
			u2_printf("Uart_Washlet_Auto_Start");
			delay_ms(200);
		}                         
		washlet_auto_dir_flag=!washlet_auto_dir_flag; 
		
		if((washlet_auto_dir_flag==1)&&(0==washlet_auto_flag))     //自动坐便正行程
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
		else if((washlet_auto_dir_flag==0)&&(1==washlet_auto_flag))         //自动坐便复位-支背、下曲腿复位
		{
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
		if(washlet_auto_dir_flag==1)
		{
			Uart_Washlet(0);	            //坐便打开			
			delay_ms(1000);
//			Uart_Washlet_Weight();          //重物检测
//			delay_ms(1000);
			Uart_Swash_Dry();               //冲洗烘干			
			Uart_Washlet(1);	            //坐便关闭
			if(washlet_flag==0)             //判断坐便是否处于复位状态，再进行坐便袋收紧
			{									
				Uart_Washlet_Tig(1);        //坐便袋收紧
				delay_ms(100);
				Uart_Washlet_Auto();        //再次调用该函数，使标志位取反，复位
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


/***********************************************************************
 函数名      ：Uart_Washlet(void)  
 函数功能    ：按键执行坐便器功能
 输入        ：dir: 0(打开坐便)；1（关闭坐便）
 输出        ：无
                           
************************************************************************/
void Uart_Washlet(u8 dir)
{
	u8 direct;       //代表某个动作运行的方向标志：1-正向运行；0-反向运行
	u8 key;          //按键扫描函数返回值,用于判断电机失步故障
	u16 num,len;
	u16 arr_feed;    //计算程序中当前一次运行脉冲数
	u16 pulse_num;   //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	
//实现上位机实时显示护理床当前运动状态
	u16 repeat_num;
	u8 break_flag=0;          //程序从break跳出标志位
	static u16 k=0;           //发送第k张图片指令
	u8 i=0;
	u16 j=0;	
	u16 arr_send;             //当前一次运行脉冲数
	static u8 kj;
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	washlet_flag=1;
	//联锁功能，只有在上曲腿、左右翻身复位后，才能执行座便器功能
	if((leg_up_flag==0)&&(body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{
		DIR6=dir;
		direct=!dir;		

		Motor_6_START(motor_washlet_freq,motor_timer_freq);           //电机启动
		TIM10_Init(washlet_arr_lim,timer10_freq);                     //打开定时器35000
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位
		if(1==direct)
		{
			washlet_flag=1;	
			u2_printf("Cartoon_Washlet_1");
			delay_ms(200);    
		}
		else
		{
			u2_printf("Cartoon_Washlet_25");
			delay_ms(200);
		}
		
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //等待定时时间到，时间到跳出循环
		{						
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{	
				//光电限位
//				if((0==GD6_End)&&(1==direct))
//				{
//					delay_us(100);
//					if(0==GD6_End)
//					{
//						break_flag=1;
//						u2_printf("GD6End"); 
//						break ;
//					}						
//				}
//				if((0==GD6_Start)&&(0==direct))
//				{
//					delay_us(100);
//					if(0==GD6_Start)
//					{
//						break_flag=1;
//						u2_printf("GD6Start"); 
//						break ; 
//					}
//				}
				  //判断有没有收到上位机指令		
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
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//发送动画指令
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(washlet_arr_lim/24);
			if(0==direct)
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
		break_flag=0;      //清除break标志位
		//判断复位
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))//判断是否处于复位状态，复位状态的前提是下行的定时器走完
		{		
			washlet_flag=0;	 
			delay_ms(200);
			u2_printf("Cartoon_Washlet_1");		
		}
		 else if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==direct))
		 {
			washlet_flag=1;    
			delay_ms(200);
			u2_printf("Cartoon_Washlet_25");
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位
		//使电机复位到初始状态（光电安装后直接打开此段）
//		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(0==GD6_Start)&&(direct==0))  //定时时间到，光电没到
//		{
//			DIR6=1;
//			Motor_6_START(motor_washlet_freq,motor_timer_freq);           //电机启动
//			TIM10_Init(add_arr,timer10_freq);                             //打开定时器35000
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //清除中断标志位	 	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )  //等待定时时间到，时间到跳出循环
//			{
//				if(0==GD6_Start)  //如果电机过载或碰到光电开关，则跳出循环，电机停止转动 
//				{  
//					delay_us(100);
//					if(0==GD6_Start) 
//					{
//						u2_printf("GD6Start");
//						break; 
//					}						
//				}				
//			}				                                 			
//			Motor_6_STOP();      //电机停止
//			TIM10_Stop();        //关闭定时器
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位
//		}
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

/***********************************************************************
 函数名      ：Uart_Washlet_Weight()   
 函数功能    ：称重函数
 输入        ：无
 输出        ：1：重物在变化；0：重物未发生变化 
                          
************************************************************************/
u8 Uart_Weight(void)  
{
    //开始检测重物				
	if(num==1)
	{				         			
		num=2;
		
		u1=filter();
				u2_printf("\r\nu1=%d\r\n",u1);
		delay_ms(500);
		if(num==2)
		{	
			num=3;		
			u2=filter();
					u2_printf("\r\nu2=%d\r\n",u2);		
			delay_ms(500);
			if(num==3)
			{	 
				num=1;
				u3=filter();
					u2_printf("\r\nu3=%d\r\n",u3);
				delay_ms(500);
			}
		}
	}
	//检测重物质量是否发生变化
	 if((abs(u1,u2)<3999)&&(abs(u1,u3)<3999)&&(abs(u2,u3)<3999))//0x300=768
	 {
		 return 0;
	 }
	 else
	 {
		 return 1;
	 }	 	
}		

/***********************************************************************
 函数名      ：Uart_Washlet_Tig()   
 函数功能    ：坐便袋收紧
 输入        ：dir:电机运行方向标志；1-正转；0-反转
 输出        ：无
                          
************************************************************************/	
void Uart_Washlet_Tig(u8 dir)
{	
//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;          //判断程序是否是从break跳出来 
	u16 repeat_num;
	u16 k=0;                  //传第k张动画
	u8 kj;
	u16 j=0;	
	u16 arr_send;                   //当前一次运行脉冲值	
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	u8 len;
	RELAY6=1;                       //继电器得电，常开触点闭合，坐便袋扎紧电机得电
	delay_ms(1000);		
	Uart_Motor_6_2_START(1,17000);   //收线推杆伸出
	u2_printf("Cartoon_Washlet_Tig_1");	
	DIR6_1=dir;
	Motor_6_1_START(3600-1,motor_timer_freq);                  //将坐便袋扎紧
//	Motor_6_1_START(7200-1,motor_timer_freq); 
	TIM2_Init(25000,timer10_freq);                              //打开定时器3500
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);         //清除中断标志位
	while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler, TIM_SR_CC1IF)))  //等待定时时间到
	{
		for(repeat_num=0;repeat_num<700;repeat_num++)
		{ 			
			//判断有没有收到上位机指令		
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
		}
		 arr_send=__HAL_TIM_GET_COUNTER(&TIM2_Handler);      //当前一次脉冲值
		//传输动画指令
		if(dir==1)
		{
			j=arr_send/(10500/6);
		}
		else
		{
			j=arr_send/(10500/6);
			j=6-j;
		}
		if(	k!=j)
		{	
			kj=abs(k,j);
			if(kj<2)
			{
				k=j;
				switch (k)
				{						
					case 1:	u2_printf("Cartoon_Washlet_Tig_2");									
							break;
					case 2:	u2_printf("Cartoon_Washlet_Tig_3");
							break;					
					case 3:	u2_printf("Cartoon_Washlet_Tig_4");
							break;					
					case 4:	u2_printf("Cartoon_Washlet_Tig_5");
							break;	
					case 5:	u2_printf("Cartoon_Washlet_Tig_6");
							break;																				
				}
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);          //清除中断标志位
	Motor_6_1_STOP();                                           //电机停止
	TIM2_Stop();                                                //关闭定时器
	delay_ms(200);
	u2_printf("Cartoon_Washlet_Tig_7");
	delay_ms(1000);
	Uart_Motor_6_2_START(0,17000);            //收线推杆缩回
    delay_ms(1000);	
	RELAY6=0;                                //继电器复位，坐便袋扎紧电机断电
}	

void UartWashletTig(u8 dir,u32 TGArr,u32 SXArr)
{	
//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;          //判断程序是否是从break跳出来 
	u16 repeat_num;
	u16 k=0;                  //传第k张动画
	u8 kj;
	u16 j=0;	
	u16 arr_send;                   //当前一次运行脉冲值	
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	u8 len;
	RELAY6=1;                       //继电器得电，常开触点闭合，坐便袋扎紧电机得电
	delay_ms(1000);		
	Uart_Motor_6_2_START(1,TGArr);   //收线推杆伸出
	u2_printf("Cartoon_Washlet_Tig_1");	
	DIR6_1=dir;
	Motor_6_1_START(3600-1,motor_timer_freq);                  //将坐便袋扎紧
//	Motor_6_1_START(7200-1,motor_timer_freq); 
	TIM2_Init(SXArr,timer10_freq);                              //打开定时器3500
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);         //清除中断标志位
	while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler, TIM_SR_CC1IF)))  //等待定时时间到
	{
		for(repeat_num=0;repeat_num<700;repeat_num++)
		{ 			
			//判断有没有收到上位机指令		
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
		}
		 arr_send=__HAL_TIM_GET_COUNTER(&TIM2_Handler);      //当前一次脉冲值
		//传输动画指令
		if(dir==1)
		{
			j=arr_send/(10500/6);
		}
		else
		{
			j=arr_send/(10500/6);
			j=6-j;
		}
		if(	k!=j)
		{	
			kj=abs(k,j);
			if(kj<2)
			{
				k=j;
				switch (k)
				{						
					case 1:	u2_printf("Cartoon_Washlet_Tig_2");									
							break;
					case 2:	u2_printf("Cartoon_Washlet_Tig_3");
							break;					
					case 3:	u2_printf("Cartoon_Washlet_Tig_4");
							break;					
					case 4:	u2_printf("Cartoon_Washlet_Tig_5");
							break;	
					case 5:	u2_printf("Cartoon_Washlet_Tig_6");
							break;																				
				}
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);          //清除中断标志位
	Motor_6_1_STOP();                                           //电机停止
	TIM2_Stop();                                                //关闭定时器
	delay_ms(200);
	u2_printf("Cartoon_Washlet_Tig_7");
	delay_ms(1000);
	Uart_Motor_6_2_START(0,TGArr+4200);            //收线推杆缩回
    delay_ms(1000);	
	RELAY6=0;                                //继电器复位，坐便袋扎紧电机断电
}	


void Uart_WashletTigOnly(u8 dir)
{	
//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;          //判断程序是否是从break跳出来 
	u16 repeat_num;
	u16 k=0;                  //传第k张动画
	u8 kj;
	u16 j=0;	
	u16 arr_send;                   //当前一次运行脉冲值	
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	u8 len;
	RELAY6=1;                       //继电器得电，常开触点闭合，坐便袋扎紧电机得电
	delay_ms(1000);		
//	Uart_Motor_6_2_START(1,16000);   //收线推杆伸出
	u2_printf("Cartoon_Washlet_Tig_1");	
	DIR6_1=dir;
	Motor_6_1_START(3600-1,motor_timer_freq);                  //将坐便袋扎紧
//	Motor_6_1_START(7200-1,motor_timer_freq); 
	TIM2_Init(17000,timer10_freq);                              //打开定时器3500
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);         //清除中断标志位
	while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler, TIM_SR_CC1IF)))  //等待定时时间到
	{
		for(repeat_num=0;repeat_num<700;repeat_num++)
		{ 			
			//判断有没有收到上位机指令		
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
		}
		 arr_send=__HAL_TIM_GET_COUNTER(&TIM2_Handler);      //当前一次脉冲值
		//传输动画指令
		if(dir==1)
		{
			j=arr_send/(10500/6);
		}
		else
		{
			j=arr_send/(10500/6);
			j=6-j;
		}
		if(	k!=j)
		{	
			kj=abs(k,j);
			if(kj<2)
			{
				k=j;
				switch (k)
				{						
					case 1:	u2_printf("Cartoon_Washlet_Tig_2");									
							break;
					case 2:	u2_printf("Cartoon_Washlet_Tig_3");
							break;					
					case 3:	u2_printf("Cartoon_Washlet_Tig_4");
							break;					
					case 4:	u2_printf("Cartoon_Washlet_Tig_5");
							break;	
					case 5:	u2_printf("Cartoon_Washlet_Tig_6");
							break;																				
				}
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);          //清除中断标志位
	Motor_6_1_STOP();                                           //电机停止
	TIM2_Stop();                                                //关闭定时器
	delay_ms(200);
	u2_printf("Cartoon_Washlet_Tig_7");
	delay_ms(1000);
//	Uart_Motor_6_2_START(0,16000);            //收线推杆缩回
    delay_ms(1000);	
	RELAY6=0;                                //继电器复位，坐便袋扎紧电机断电
}	



/***********************************************************************
 函数名      ：Back_Leg()   
 函数功能    ：支背、下曲腿同时运行函数
 输入        ：无
 输出        ：无 
                          
************************************************************************/
void Uart_Back_Leg(void)
{
	u8 len;
	u16 arr_now;               //当前一次运行脉冲数
	
//实现上位机实时显示护理床当前运动状态
	u8 break_flag=0;           //判断程序是否是从break跳出来 
	u16 repeat_num;
	static u8 k;               //传第k张动画
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //当前一次运行脉冲值
		
	//联锁功能，只有在左右翻身功能复位后，才能进行支背、下曲腿
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;									
		//运行支背或下曲腿或支背和下曲腿同时运行
		TIM10_Init(leg_angle_to_arr(leg_down_angle_lim),timer10_freq);  //定时器周期=(freq1*freq_time1_1)/90mhz
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )    //定时时间到
		{
			 //判断有没有收到别的指令
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
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			 //只发送支背动画指令
			if((1==back_state_flag)&&(0==leg_down_state_flag))           //只执行支背
			{
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);				
				j=arr_send/(back_angle_to_arr(back_angle_max)/19);
				if(0==washlet_auto_dir_flag)
				{
					j=19-j;
				}
				k=back_picture_k;
				if(k!=j)
				{
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;   back_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Back_2");							
									break;
							case 2:	u2_printf("Cartoon_Washlet_Back_3");											
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Back_4");											
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Back_5");											
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Back_6");											
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Back_7");											
									break;
							case 7:	u2_printf("Cartoon_Washlet_Back_8");										
									break;
							case 8:	u2_printf("Cartoon_Washlet_Back_9");											
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Back_10");											
									break;												
							case 10:u2_printf("Cartoon_Washlet_Back_11");											
									break;
							case 11:u2_printf("Cartoon_Washlet_Back_12");																		
									break;
							case 12:u2_printf("Cartoon_Washlet_Back_13");											
									break;					
							case 13:u2_printf("Cartoon_Washlet_Back_14");											
									break;					
							case 14:u2_printf("Cartoon_Washlet_Back_15");										
									break;	
							case 15:u2_printf("Cartoon_Washlet_Back_16");																		
									break;
							case 16:u2_printf("Cartoon_Washlet_Back_17");											
									break;					
							case 17:u2_printf("Cartoon_Washlet_Back_18");											
									break;					
							case 18:u2_printf("Cartoon_Washlet_Back_19");										
									break;							
						}
					}
				}
			}
			//只发送曲腿动画指令
			else if((0==back_state_flag)&&(1==leg_down_state_flag))      //只执行下曲腿
			{
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				j=arr_send/(leg_angle_to_arr(leg_down_angle_max)/19);
				if(0==washlet_auto_dir_flag)
				{
					j=19-j;
				}
				k=leg_down_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;  leg_down_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Leg_Down_2");	
									break;
							case 2:	u2_printf("Cartoon_Washlet_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Washlet_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Washlet_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Washlet_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Washlet_Leg_Down_12");	
									break;
							case 12:u2_printf("Cartoon_Washlet_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Washlet_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Washlet_Leg_Down_15");
									break;	
							case 15:u2_printf("Cartoon_Washlet_Leg_Down_16");
									break;	
							case 16:u2_printf("Cartoon_Washlet_Leg_Down_17");	
									break;
							case 17:u2_printf("Cartoon_Washlet_Leg_Down_18");
									break;					
							case 18:u2_printf("Cartoon_Washlet_Leg_Down_19");
									break;					
							case 19:u2_printf("Cartoon_Washlet_Leg_Down_20");
									break;	
						}
					}				
				}			
			}			
			
			//发送支背和下曲腿同时运行动画指令
			else if((1==back_state_flag)&&(1==leg_down_state_flag))      //支背和下曲腿同时运行
			{				
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				j=arr_send/(back_angle_to_arr(back_angle_max)/14);
				if(0==washlet_auto_dir_flag)
				{
					j=14-j;
				}
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Back_Leg_Down_2");	
									break;
							case 2:	u2_printf("Cartoon_Washlet_Back_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Back_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Back_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Back_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Back_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Washlet_Back_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Washlet_Back_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Back_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Washlet_Back_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Washlet_Back_Leg_Down_12");	
									break;
							case 12:u2_printf("Cartoon_Washlet_Back_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Washlet_Back_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
									break;																											
						}
					}				
				}								
			}										
		}
		Push_Rod_Stop();            //下曲腿停止
		TIM10_Stop();		        //关闭定时器	
		//判断下曲腿复位
		if((washlet_auto_dir_flag==0)&&(1==leg_down_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
		{
			leg_down_flag=0; 
			leg_down_runed_arr=0; 
//			W25QXX_Write((u8*)&leg_down_flag,32,1);
			if(0==back_state_flag)                   //只有下曲腿运行，支背不运行
			{
				k=0;   leg_down_picture_k=0;
				delay_ms(200);
				u2_printf("Cartoon_Washlet_Leg_Down_1");
			}
		}
		if((washlet_auto_dir_flag==1)&&(1==leg_down_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
		{
			leg_down_flag=1; 
			leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim); 
//			W25QXX_Write((u8*)&leg_down_flag,32,1);
			if(0==back_state_flag)                   //只有下曲腿运行，支背不运行
			{
				k=19;   leg_down_picture_k=19;
				delay_ms(200);
				u2_printf("Cartoon_Washlet_Leg_Down_20");
			}
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //清除中断标志位
		
		//继续运行支背或支背及下曲腿同时继续运行
		if(back_state_flag==1)              //继续运行支背或支背及下曲腿同时继续运行
		{				
			TIM10_Init(back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim),timer10_freq);
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //清除中断标志位
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) ) //支背定时时间到
			{	
				 //判断有没有收到别的指令
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
				 if(break_flag==1)
				 {
					u2_printf("break_flag==1");
					break;
				 }
				 //继续发送支背运行动画指令
				 if(0==leg_down_state_flag)        //只有支背运行
				 {
					 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					 if(washlet_auto_dir_flag==1)
					 {
						 j=(leg_angle_to_arr(leg_down_angle_lim)+arr_send)/(back_angle_to_arr(back_angle_max)/19);
					 }
					 else
					 {
						 arr_send=back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim)-arr_send;
						 j=arr_send/(back_angle_to_arr(back_angle_max)/19);
					 }
					 k=back_picture_k;
					 if(k!=j)
					 {				
						kj=abs(k,j);
						if(kj<2)
						{
							k=j;   back_picture_k=k;
							switch (k)
							{						
								case 1:	u2_printf("Cartoon_Washlet_Back_2");							
										break;
								case 2:	u2_printf("Cartoon_Washlet_Back_3");											
										break;					
								case 3:	u2_printf("Cartoon_Washlet_Back_4");											
										break;					
								case 4:	u2_printf("Cartoon_Washlet_Back_5");											
										break;	
								case 5:	u2_printf("Cartoon_Washlet_Back_6");											
										break;	
								case 6:	u2_printf("Cartoon_Washlet_Back_7");											
										break;
								case 7:	u2_printf("Cartoon_Washlet_Back_8");										
										break;
								case 8:	u2_printf("Cartoon_Washlet_Back_9");											
										break;						
								case 9:	u2_printf("Cartoon_Washlet_Back_10");											
										break;												
								case 10:u2_printf("Cartoon_Washlet_Back_11");											
										break;
								case 11:u2_printf("Cartoon_Washlet_Back_12");																		
										break;
								case 12:u2_printf("Cartoon_Washlet_Back_13");											
										break;					
								case 13:u2_printf("Cartoon_Washlet_Back_14");											
										break;					
								case 14:u2_printf("Cartoon_Washlet_Back_15");										
										break;	
								case 15:u2_printf("Cartoon_Washlet_Back_16");																		
										break;
								case 16:u2_printf("Cartoon_Washlet_Back_17");											
										break;					
								case 17:u2_printf("Cartoon_Washlet_Back_18");											
										break;					
								case 18:u2_printf("Cartoon_Washlet_Back_19");										
										break;																																			
							}
						}				
					 }						
				 }
				 //继续发送下曲腿、支背同时运行的动画指令
				 if(1==leg_down_state_flag)       //支背、曲腿都运行
				 { 
					 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					 if(washlet_auto_dir_flag==1)
					 {
						 j=(leg_angle_to_arr(leg_down_angle_lim)+arr_send)/(back_angle_to_arr(back_angle_max)/14);
					 }
					 else
					 {
						 arr_send=back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim)-arr_send;
						 j=arr_send/(back_angle_to_arr(back_angle_max)/14);
					 }
					 if(k!=j)
					 {				
						kj=abs(k,j);
						if(kj<2)
						{
							k=j;
							switch (k)
							{						
								case 1:	u2_printf("Cartoon_Washlet_Back_Leg_Down_2");	
										break;
								case 2:	u2_printf("Cartoon_Washlet_Back_Leg_Down_3");
										break;					
								case 3:	u2_printf("Cartoon_Washlet_Back_Leg_Down_4");
										break;					
								case 4:	u2_printf("Cartoon_Washlet_Back_Leg_Down_5");
										break;	
								case 5:	u2_printf("Cartoon_Washlet_Back_Leg_Down_6");
										break;	
								case 6:	u2_printf("Cartoon_Washlet_Back_Leg_Down_7");
										break;
								case 7:	u2_printf("Cartoon_Washlet_Back_Leg_Down_8");
										break;
								case 8:	u2_printf("Cartoon_Washlet_Back_Leg_Down_9");
										break;						
								case 9:	u2_printf("Cartoon_Washlet_Back_Leg_Down_10");
										break;												
								case 10:u2_printf("Cartoon_Washlet_Back_Leg_Down_11");
										break;	
								case 11:u2_printf("Cartoon_Washlet_Back_Leg_Down_12");	
										break;
								case 12:u2_printf("Cartoon_Washlet_Back_Leg_Down_13");
										break;					
								case 13:u2_printf("Cartoon_Washlet_Back_Leg_Down_14");
										break;																																									
							}
						}				
					}	
				}						 
			}
			Motor_1_STOP();                                             //支背停止
			TIM10_Stop();                                               //关闭定时器
			//判断支背复位
			if((washlet_auto_dir_flag==0)&&(1==back_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				back_flag=0; 
				back_runed_arr=0;
//				W25QXX_Write((u8*)&back_flag,30,1);
				if(0==leg_down_state_flag)         //只有支背运行
				{
					k=0;   back_picture_k=0;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_1");
				}
				if(1==leg_down_state_flag)         //支背、曲腿同时运行
				{
					k=0;   
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_Leg_Down_1");
				}
			}
			if((washlet_auto_dir_flag==1)&&(1==back_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				back_flag=1; 
				back_runed_arr=back_angle_to_arr(back_angle_lim);
				if(0==leg_down_state_flag)         //只有支背运行
				{
					k=19;   back_picture_k=19;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_20");
				}
				if(1==leg_down_state_flag)         //支背、曲腿同时运行
				{
					k=14;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
				}
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //清除中断标志位
		}				
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

/***********************************************************************
 函数名      ：Washlet_Weight(void)   
 函数功能    ：按键执行自动坐便器功能
 输入        ：无
 输出        ：无 
                          
************************************************************************/
u8  Uart_Washlet_Weight(void)
{
	u8 m=0,i=0;
	while(1)
	{
		TIM10_Init(5000-1,timer10_freq);      //打开定时器30S
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //清除中断标志位
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //等待定时时间到
		{
			if(Uart_Weight())                //若重物在变化跳出循环，重新开始计时
			{  
				m++;
				switch(m)
				{
					case 1:u2_printf("Cartoon_Washlet_Weight_1");									
							break;
					case 2:u2_printf("Cartoon_Washlet_Weight_2");									
							break;
					case 3:u2_printf("Cartoon_Washlet_Weight_3");									
							break;
					case 4:u2_printf("Cartoon_Washlet_Weight_4");									
							break;
					case 5:u2_printf("Cartoon_Washlet_Weight_5");									
							break;
					case 6:u2_printf("Cartoon_Washlet_Weight_6");									
							break;
					case 7:u2_printf("Cartoon_Washlet_Weight_7");									
							break;
					case 8:u2_printf("Cartoon_Washlet_Weight_8");									
							break;
					case 9:u2_printf("Cartoon_Washlet_Weight_9");									
							break;
					case 10:u2_printf("Cartoon_Washlet_Weight_10");									
							break;
					case 11:u2_printf("Cartoon_Washlet_Weight_11");									
							break;
					case 12:u2_printf("Cartoon_Washlet_Weight_12");									
							break;
					case 13:u2_printf("Cartoon_Washlet_Weight_13");									
							break;
					case 14:u2_printf("Cartoon_Washlet_Weight_14");									
							break;
					case 15:u2_printf("Cartoon_Washlet_Weight_15");									
							break;
				}				
			}
		}
		//如果30S定时间到且重物没有发生变化
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(0==Uart_Weight())) 
		{ 				
			i++; 
			u2_printf("称重i=%d",i);
		}
		else { i=0; u2_printf("称重i=%d\r\n",i);}                    //否则重新计时
		if(i==2)                         //重物两分钟没有发生变化，开始冲洗烘干
		{
			u2_printf("Washlet_Over");   //排便结束
			u1=0;
			u2=0;
			u3=0;
			break;
		}
	}	   	 		
}

/***********************************************************************
 函数名      ：Uart_Swash_Dry(void)   
 函数功能    ：按键执行冲洗烘干功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Uart_Swash_Dry()
{
	u8 num,len;    //接收字符串长度
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
	USART2_RX_LEN=0;
	
    //连锁功能，只有在坐便打开时才能进行喷气烘干
	if((lock_flag==1)&&(1==washlet_flag))		
	{
		//Uart_Push_Rod_Swash_Dry(1,29000);      //冲洗烘干推杆伸出  
		u2_printf("推杆伸出\r\n");	
		RELAY6=1;
		Uart_Push_Rod_Swash(1,30000);		
		RELAY6=0;
		delay_ms(50);
		
/********************开始冲洗**********************/
		u2_printf("开始冲洗\r\n");	
    Uart_Swash_Auto();                               //自动冲洗烘干
		u2_printf("结束冲洗\r\n");	
		//自动冲洗结束，等待30S若再次按下冲洗按键，进行手动调节冲洗
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);     //清空接收寄存器
		USART2_RX_LEN=0;
		TIM9_Init(4000,timer10_freq);                   //打开定时器，定时30S60000
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);         //清除中断标志位
		while(!(__HAL_TIM_GET_FLAG(&TIM9_Handler, TIM_SR_CC1IF)))  //等待定时时间到
		{							
			if(USART2_RX_LEN&0x8000)
			{
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;	
				if(strstr((const char *)USART2_RX_BUF,(const char *)"SwashPhone"))   //若接收到Stop,则跳出循环	
				{					
					if(1==Liq_Sensor)           //足够一次冲洗
					{						
						  Uart_Swash_Hand();
						  memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					    USART2_RX_LEN=0;
					}
					else                         //不足一次冲洗
					{
						break;
					}
				}				
			}
		}
		TIM9_Stop();                                         //关闭定时器
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);   //清除中断标志位
	
		//delay_ms(500);
		
/**********************************开始烘干***********************************/
		u2_printf("开始烘干\r\n");	
		Uart_Dry_Auto();       //自动烘干2分钟
		u2_printf("结束烘干\r\n");	
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
		USART2_RX_LEN=0;
		
		//自动烘干结束，等待30S若再次按下烘干按键，进行手动调节烘干		
		TIM9_Init(7000,timer10_freq);                             //打开定时器，定时30S
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);         //清除中断标志位
		while(!(__HAL_TIM_GET_FLAG(&TIM9_Handler, TIM_SR_CC1IF)))  //等待定时时间到
		{
			if(USART2_RX_LEN&0x8000)
			{
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;
				if(strstr((const char *)USART2_RX_BUF,(const char *)"DryPhone"))    //若接收到Stop,则跳出循环	
				{
					Uart_Dry_Hand();
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
				}				
			}
		}
		
		TIM9_Stop();                                        //关闭定时器
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);  //清除中断标志位		
		delay_ms(500);
		//Uart_Push_Rod_Swash_Dry(0,2000+swash_dry_runed_pulse);                    //冲洗烘干推杆缩回		
				RELAY6=1;
		Uart_Push_Rod_Swash(0,30000);
		RELAY6=0;		
		u2_printf("推杆缩回\r\n");	
	}
	else
	{
		LED0=0;          //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;
	}		
}
			
/***********************************************************************
 函数名      ：Uart_Swash_Hand(void)   
 函数功能    ：按键执行冲洗功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Uart_Swash_Hand(void)
{
	u8 len;              //WiFi接收字符串长度
	u8 direct;           //方向标志位
	u8 i;
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
	USART2_RX_LEN=0;
	
    //连锁功能，只有在坐便打开时才能进行喷水冲洗
	if((lock_flag==1)&&(1==washlet_flag))		
	{								
		//喷水冲洗
		RELAY6=1;             //继电器得电
		DIR_SB=1;             //水泵开启PB12	
		swash_hand_flag=1;
		if(0==swash_dry_runed_pulse)
		{
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		for(i=0;i<2*swash_dry_time;i++)                                //冲洗2*swash_dry_time分钟
		{
			TIM10_Init(60000-1,timer10_freq_1);                        //打开定时器,定时器周期为30S					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位 
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环
			{ 					
				if(0==Liq_Sensor)	                                   //水位在低水位下，不足一次冲洗，则直接跳出
				{ 
					u2_printf("LiquidLevellow");                       //发送给上位机指令信号，表示此时水位偏低
					break;
				}				
				if(USART2_RX_LEN&0x8000)
				{
					len=USART2_RX_LEN&0x3fff;				
					USART2_RX_BUF[len]=0;
					if(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))    //推杆伸出	
					{
						direct=1;
						Uart_Push_Rod_Swash(1,swash_dry_pulse_lim); 
					}
					if(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone"))  //推杆缩回	
					{ 
						direct=0;
						Uart_Push_Rod_Swash(0,swash_dry_pulse_lim); 					
					}
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
					USART2_RX_LEN=0;	
				}	
			}			
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位   		
			TIM10_Stop();		                                //关闭定时器				
		}
		swash_hand_flag=0;
		DIR_SB=0;             //水泵关闭PB12
		if(swash_dry_runed_pulse>0)
		{
			direct=0;
			Uart_Push_Rod_Swash(0,swash_dry_runed_pulse); 
		}
		if(0==swash_dry_runed_pulse)
		{
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		RELAY6=0;        //继电器断电
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

/***********************************************************************
 函数名      ：Uart_Dry_Hand(void)   
 函数功能    ：按键执行烘干功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Uart_Dry_Hand(void)
{
	u8 len,i;
	u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行         
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
	USART2_RX_LEN=0;	
    //连锁功能，只有在坐便打开时才能进行喷气烘干
	if((lock_flag==1)&&(1==washlet_flag))		
	{	
		//喷气烘干
		DIR_HG=1;             //烘干喷气阀门打开PB10
		delay_ms(500);       //等待阀门打开1S
		RELAY6=1;             //继电器得电
		DIR_QB=1;             //气泵启动PH2
		dry_hand_flag=1;
		if(0==swash_dry_runed_pulse)
		{
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}		
		for(i=0;i<2*swash_dry_time;i++)      //烘干swash_dry_time分钟
		{
			TIM10_Init(60000-1,timer10_freq_1);                        //打开定时器,定时器周期为30S					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //清除中断标志位 
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //等待定时时间到，时间到跳出循环
			{ 					
				if(USART2_RX_LEN&0x8000)
				{
					len=USART2_RX_LEN&0x3fff;				
					USART2_RX_BUF[len]=0;
					//判断上下行
					if(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))    //推杆伸出	
					{
						direct=1;
						Uart_Push_Rod_Dry(1,swash_dry_pulse_lim); 
					}
					if(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone"))  //推杆缩回	
					{ 
						direct=0;
						Uart_Push_Rod_Dry(0,swash_dry_pulse_lim); 					
					}
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
					USART2_RX_LEN=0;
				}				
			}              		
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位   		
			TIM10_Stop();   //关闭定时器		 	
		}
		dry_hand_flag=0;		
		if(swash_dry_runed_pulse>0)
		{
			direct=0;
			Uart_Push_Rod_Dry(0,swash_dry_runed_pulse); 
		}		
		if(0==swash_dry_runed_pulse)
		{
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}
		RELAY6=0;       //继电器断电
		DIR_QB=0;       //气泵关闭PH2
		DIR_HG=0;       //烘干喷气阀关闭PB10		
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

/***********************************************************************
 函数名      ：Uart_Swash_Auto(void)   
 函数功能    ：按键执行冲洗功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Uart_Swash_Auto(void)
{
	u8 direct;   //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	u8 flag=1;   //控制推杆方向切换
	u8 i;
	u8 num,len;
	
    //连锁功能，只有在坐便打开时才能进行喷水冲洗
	if((lock_flag==1)&&(1==washlet_flag))		
	{						
//		//冲洗之前检测水位若在正常水位以下，发出报警声，并等待水箱注水
		if(0==Liq_Sensor)  
		{
			delay_ms(100);
			u2_printf("LiquidLevellow");  //发送给上位机指令信号，表示此时水位偏低
		}
		while(0==Liq_Sensor)                //水箱注水后，才能继续往下执行
		{
			PCF8574_WriteBit(BEEP_IO,0);	//控制蜂鸣器报警	
			if(USART2_RX_LEN&0x8000)
			{
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;
				if(strstr((const char *)USART2_RX_BUF,(const char *)"BeepOff"))
				{
					PCF8574_WriteBit(BEEP_IO,1);                     //蜂鸣器停止报警
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);     //清空接收寄存器
					USART2_RX_LEN=0;
					break;
				}
			}		
		}
		while(0==Liq_Sensor);                //等待水箱注满
		PCF8574_WriteBit(BEEP_IO,1);
		RELAY6=1;                            //继电器得电
		//喷水冲洗
		DIR_SB=1;                            //水泵开启PB12	
//		if(0==swash_dry_runed_pulse)
//		{
//			u2_printf("Cartoon_Push_Rod_Swash_1");
//		}
//		for(i=0;i<2*swash_dry_time;i++)     //冲洗烘干推杆自动循环冲洗swash_dry_time分钟
//		{
//			flag=!flag;
//			Uart_Push_Rod_Swash(flag,30000);      //每次伸出缩回5S钟
//			delay_ms(10);			
//		}
//		if(0==swash_dry_runed_pulse)
//		{
//			u2_printf("Cartoon_Push_Rod_Swash_1");
//		}
		delay_ms(5000);
		delay_ms(4000);
		RELAY6=0;             //继电器断电
		DIR_SB=0;             //水泵关闭PB12		
	}
	else
	{
		LED0=0;               //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Dry_Auto(void)   
 函数功能    ：按键执行烘干功能
 输入        ：无
 输出        ：无  
                          
************************************************************************/
void Uart_Dry_Auto(void)
{
	u8 flag=0;        //控制推杆方向切换
	u8 i;
	
    //连锁功能，只有在坐便打开时才能进行喷气烘干
	if((lock_flag==1)&&(1==washlet_flag))		
	{
		//喷气烘干
		DIR_HG=1;             //烘干喷气阀门打开PB10
		delay_ms(500);       //等待阀门打开1S
		RELAY6=1;             //继电器得电
		DIR_QB=1;             //气泵启动PH2
//		if(0==swash_dry_runed_pulse)
//		{
//			u2_printf("Cartoon_Push_Rod_Dry_1");
//		}		
//		for(i=0;i<2*swash_dry_time;i++)     //冲洗烘干推杆自动循环烘干swash_dry_time 分钟
//		{
//			flag=!flag;
//			Uart_Push_Rod_Dry(flag,swash_dry_pulse_lim);        //每次伸出缩回5S钟
//			delay_ms(50);
//		}
		
		
//		if(0==swash_dry_runed_pulse)
//		{
//			u2_printf("Cartoon_Push_Rod_Dry_1");
//		}

		delay_ms(6000);
		delay_ms(6000);
		RELAY6=0;       //继电器断电
		DIR_QB=0;       //气泵关闭PH2
		DIR_HG=0;       //烘干喷气阀关闭PB10
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

/***********************************************************************
 函数名      ：Uart_Exp_Back(void)  
 函数功能    ：专家系统推理-支背
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Exp_Back(void) 
{
	//支背起
	WriteInUART2("BackUpPhone");
	Uart_Back();                //支背
	delay_ms(1000);	
	//支背复位	
	WriteInUART2("BackDownPhone");
	Uart_Back();                //支背复位	
	
}

/***********************************************************************
 函数名      ：Uart_Exp_Body(void)  
 函数功能    ：专家系统推理-翻身
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Exp_Body(void) 
{
	//左翻身
	WriteInUART2("BodyLeftUpPhone");		
	Uart_Body_Left();           //左翻身	
	delay_ms(1000);
	WriteInUART2("BodyLeftDownPhone");		
	Uart_Body_Left();           //左翻身	复位
	delay_ms(1000);
			
	//右翻身
	WriteInUART2("BodyRightUpPhone");
	Uart_Body_Right();          //右翻身
	delay_ms(1000);
	WriteInUART2("BodyRightDownPhone");
	Uart_Body_Right();          //右翻身复位
		
}

/***********************************************************************
 函数名      ：Uart_Exp_Leg(void)  
 函数功能    ：专家系统推理-曲腿
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Exp_Leg(void) 
{
	//上曲腿
	WriteInUART2("LegUpUpPhone");
	Uart_Leg_Up();              //上曲腿
	delay_ms(1000);
	WriteInUART2("LegUpDownPhone");
	Uart_Leg_Up();              //上曲腿复位
	delay_ms(1000);	
	
	//下曲腿
	WriteInUART2("LegDownDownPhone");
	Uart_Leg_Down();            //下曲腿	
	delay_ms(1000);
	WriteInUART2("LegDownUpPhone");
	Uart_Leg_Down();            //下曲腿复位
	
}

/***********************************************************************
 函数名      ：Uart_Exp_Washlet_Auto(void)  
 函数功能    ：专家系统推理-自动坐便
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Exp_Washlet_Auto(void) 
{
	Uart_Washlet_Auto();	  //自动坐便
}



/***********************************************************************
 函数名      ：Uart_Auto_Arm_Leg_Left(void)  
 函数功能    ：执行左肢康复锻炼
 输入        ：t-康复锻炼次数
 输出        ：无
                           
************************************************************************/
void Uart_Auto_Arm_Leg_Left(int t)
{
   u32 pulse;          //吊挂运行脉冲数
   int j;	
	//病人只有平躺在床上才能进行吊挂康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   { 
		
		DG_Relay=1;		//继电器得电
		 if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftAuto"))    //左胳膊
		{
			pulse=1000000;
			if(leg_fore_left_flag==0)       //防止按键误触发，导致标志位置位
			{	
				arm_fore_left_flag=1;
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmLeftAutoStart");				
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftAuto"))    //左腿
		{
			pulse=1000000;
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
		
		Uart_Auto_Hang_1_2(1,pulse);       //将左肢抬高
		delay_ms(1000);	     
		   
		for(j=0;j<t;j++)                   //进行t此左肢康复训练
		{
			Uart_Auto_Hang_1(1,75000);     //向上运动
			delay_ms(1000);
			Uart_Auto_Hang_1(0,75000);     //向下运动
			delay_ms(1000);	
		}	
		Uart_Auto_Hang_1_2(0,pulse);       //将左肢放平到原来的位置
		//将标志位置位
		if((0==arm_left_runed)&&(1==arm_fore_left_flag))         //左胳膊已运行脉冲为零，复位到初始状态
		{
			arm_fore_left_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("ArmLeftAutoRes");
			delay_ms(200);
		}
		if((0==leg_left_runed)&&(1==leg_fore_left_flag))         //左腿已运行脉冲为零，复位到初始状态
		{
			leg_fore_left_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("LegLeftAutoRes");
			delay_ms(200);
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}		
}

/***********************************************************************
 函数名      ：Uart_Auto_Arm_Leg_Right(void)   
 函数功能    ：执行右肢吊挂康复训练
 输入        ：t吊挂次数
 输出        ：无 
                          
************************************************************************/
void Uart_Auto_Arm_Leg_Right(int t)
{
	u32 pulse;     //吊挂运行脉冲数
	//病人只有平躺在床上才能进行吊挂康复训练
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{
		DG_Relay=1;		//继电器得电
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmRightAuto"))    //右胳膊
		{
			pulse=1000000;
			if(leg_fore_right_flag==0)          //防止按键误触发，导致标志位置位
			{	
				arm_fore_right_flag=1;
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmRightAutoStart");
				delay_ms(200);
			}			
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegRightAuto"))    //右腿
		{
			pulse=1000000;
			if(arm_fore_right_flag==0)          //防止按键误触发，导致标志位置位
			{	
				leg_fore_right_flag=1;
				delay_ms(200);				
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegRightAutoStart");	
				delay_ms(200);
			}			
		}
	
		int j;
		Uart_Auto_Hang_3_4(1,pulse);       //将右肢抬高
		delay_ms(1000);
		
		for(j=0;j<t;j++)                   //进行t次右肢康复训练
		{		
			Uart_Auto_Hang_3(1,75000);     //向上运动		
			delay_ms(1000);		
			Uart_Auto_Hang_3(0,75000);     //向下运动
			delay_ms(1000);		
		}	
		Uart_Auto_Hang_3_4(0,pulse);       //将右肢放平到原来的位置
		//将标志位置位
		if((0==arm_right_runed)&&(1==arm_fore_right_flag))   //左胳膊已运行脉冲为零，复位到初始状态
		{
			arm_fore_right_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);			
			u2_printf("ArmRightAutoRes");
			delay_ms(200);
		}
		if((0==leg_right_runed)&&(1==leg_fore_right_flag))   //左腿已运行脉冲为零，复位到初始状态
		{
			leg_fore_right_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("LegRightAutoRes");
			delay_ms(200);
		}	
			DG_Relay=0;		//继电器失电
	}
	else
	 {
		LED0=0;                 //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		 
		LED0=1;
		LED1=1;
	 }	
}

/***********************************************************************
 函数名      ：Uart_Auto_Arm_Leg_Left_Right(void)   
 函数功能    ：执行左右肢吊挂康复训练
 输入        ：t吊挂次数
 输出        ：无  
                          
***********************************************************************/
void Uart_Auto_Arm_Leg_Left_Right(int t)
{
	u32 pulse;	         //吊挂运行脉冲数
	int j;
	//病人只有平躺在床上才能进行吊挂康复训练
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{
		DG_Relay=1;		//继电器得电
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftRightAuto"))    //左右胳膊
		{
			pulse=1000000;
			if(leg_fore_left_right_flag==0)       //防止按键误触发，导致标志位置位
			{	
				arm_fore_left_right_flag=1;	
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmLeftRightAutoStart");	
				delay_ms(200);				
			}
			else
			{
				arm_fore_left_right_flag=0;	
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftRightAuto"))    //左右腿
		{
			pulse=1000000;
			if(arm_fore_left_right_flag==0)      //防止按键误触发，导致标志位置位
			{	
				leg_fore_left_right_flag=1;
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegLeftRightAutoStart");	
				delay_ms(200);				
			}
			else
			{
				leg_fore_left_right_flag=0;	
			}
		}	
		Uart_Auto_Hang_1_2_3_4(1,pulse);     //将左右肢抬高，小臂抬得比大臂高
		delay_ms(1000);	     
			   
		for(j=0;j<t;j++)                     //进行t次康复训练
		{
			LED1=0;                          //在康复训练过程中LED0闪烁
			Uart_Auto_Hang_1_3(1,75000);     //小臂上升一定高度
			LED1=1;
			delay_ms(1000);
			LED1=0;
			Uart_Auto_Hang_1_3(0,75000);     //小臂下降一定高度
			delay_ms(1000);
			LED1=1;	
		}
			Uart_Auto_Hang_1_2_3_4(0,pulse); //将左右肢放平到原来的位置
		//将标志位置位
		if((0==arm_left_right_runed)&&(1==arm_fore_left_right_flag))    //左胳膊已运行脉冲为零，复位到初始状态
		{
			arm_fore_left_right_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("ArmLeftRightAutoRes");
			delay_ms(200);
		}
		if((0==leg_left_right_runed)&&(1==leg_fore_left_right_flag))    //左腿已运行脉冲为零，复位到初始状态
		{
			leg_fore_left_right_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("LegLeftRightAutoRes");
			delay_ms(200);
		}	
			DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;               //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Arm_Fore_Left(void)  
 函数功能    ：手动执行左小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Arm_Fore_Left(void)
{
	 u8 direct;   //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag=0;
	//连锁功能只有在键锁打开、支背、坐便、翻身复位的情况下才能进行小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 if(0==arm_fore_left_flag)                    //先将左肢抬高到一定高度
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmForeLeftHandStart");
				delay_ms(200);
				arm_fore_left_flag=1;
				Uart_Hand_Hang_1_2(1,arm_left_lim);  //将左肢抬高到一定高度，开始小臂康复训练
				runed_flag=1;
			}
		}				
		//左小臂向上运行
		if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftUpHand"))||(1==runed_flag))
		{	
			runed_flag=0;
			if(arm_fore_left_lim>arm_fore_left_runed)
			{
				Uart_Hand_Hang_1(1,arm_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==arm_fore_left_lim)
				{
					arm_fore_left_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForeLeftHandLim");
					delay_ms(200);
				}
				else
				{
					arm_fore_left_flag=1;		
				}								
			}	
		}
		//左小臂向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftDownHand"))
		{	
			if(arm_fore_left_runed>0)
			{
				Uart_Hand_Hang_1(0,arm_fore_left_runed);					
			}
			if(0==arm_fore_left_runed)               //若小臂复位，则将胳膊复位
			{
				Uart_Hand_Hang_1_2(0,arm_left_runed);     //将左肢复位到水平状态 
			}
			if((0==arm_fore_left_runed)&&(0==arm_left_runed))
			{
				arm_fore_left_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForeLeftHandRes");
				delay_ms(200);
			}
		}	
			DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Leg_Fore_Left(void)  
 函数功能    ：手动执行左小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Leg_Fore_Left(void)
{
	 u8 direct;   //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag=0;
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 if(0==leg_fore_left_flag)                    //先将左肢抬高到一定高度
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegForeLeftHandStart");
				delay_ms(200);
				leg_fore_left_flag=1;
				Uart_Hand_Hang_1_2(1,leg_left_lim);      //将左肢抬高到一定高度，开始小臂康复训练
				runed_flag=1;
			}
		}
		if((strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftUpHand"))||(1==runed_flag))
		{	
			runed_flag=0;
			if(leg_fore_left_lim>leg_fore_left_runed)
			{
				Uart_Hand_Hang_1(1,leg_fore_left_lim-leg_fore_left_runed);	
				if(leg_fore_left_runed==leg_fore_left_lim)
				{
					leg_fore_left_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForeLeftHandLim");
					delay_ms(200);
				}
				else
				{
					leg_fore_left_flag=1;		
				}								
			}	
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftDownHand"))
		{	
			if(leg_fore_left_runed>0)
			{
				Uart_Hand_Hang_1(0,leg_fore_left_runed);					
			}
			if(0==leg_fore_left_runed)             //若小臂已运行脉冲为0，则将胳膊复位
			{
				Uart_Hand_Hang_1_2(0,leg_left_runed);   //将左肢复位到水平状态 
			}
			if((0==leg_fore_left_runed)&&(0==leg_left_runed))
			{
				leg_fore_left_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForeLeftHandRes");
				delay_ms(200);
			}			
		}	
			DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Arm_Fore_Right(void)  
 函数功能    ：手动执行右小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Arm_Fore_Right(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag=0;
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 if(0==arm_fore_right_flag)                    //先将右肢抬高到一定高度
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmForeRightHandStart");
				delay_ms(200);
				arm_fore_right_flag=1;
				Uart_Hand_Hang_3_4(1,arm_right_lim);       //将右肢抬高到一定高度，开始小臂康复训练
				runed_flag=1;
			}
		}
		//右小臂向上运行
		if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightUpHand"))||(1==runed_flag))		
		{
			runed_flag=0;
			if(arm_fore_right_lim>arm_fore_right_runed)
			{
				Uart_Hand_Hang_3(1,arm_fore_right_lim-arm_fore_right_runed);	
				if(arm_fore_right_runed==arm_fore_right_lim)
				{
					arm_fore_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForeRightHandLim");
					delay_ms(200);
				}
				else
				{
					arm_fore_right_flag=1;						
				}								
			}	
		}			
		//右小臂向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightDownHand"))
		{
			if(arm_fore_right_runed>0)    //右小臂向下运动
			{
				Uart_Hand_Hang_3(0,arm_fore_right_runed);					
			}
			if(arm_fore_right_runed==0)    //若右小臂复位，则将右胳膊复位到水平位置
			{
				Uart_Hand_Hang_3_4(0,arm_right_runed); 
			}
			if((arm_fore_right_runed==0)&&(0==arm_right_runed))   //标志位置位
			{
				arm_fore_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForeRightHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Leg_Fore_Right(void)  
 函数功能    ：手动执行右小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Leg_Fore_Right(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag;
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		 DG_Relay=1;		//继电器得电
		 if(0==leg_fore_right_flag)                    //先将左肢抬高到一定高度
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegForeRightHandStart");
				delay_ms(200);
				leg_fore_right_flag=1;
				Uart_Hand_Hang_3_4(1,leg_right_lim);       //将左肢抬高到一定高度，开始小臂康复训练
				runed_flag=1;
			}
		}
		//右小腿上行
	   if((strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightUpHand"))||(runed_flag==1))
		{
			runed_flag=0;
			if(leg_fore_right_lim>leg_fore_right_runed)
			{
				Uart_Hand_Hang_3(1,leg_fore_right_lim-leg_fore_right_runed);	
				if(leg_fore_right_runed==leg_fore_right_lim)
				{
					leg_fore_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForeRightHandLim");
					delay_ms(200);
				}
				else
				{
					leg_fore_right_flag=1;						
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightDownHand"))
		{
			if(leg_fore_right_runed>0)     //右小腿向下运行
			{
				Uart_Hand_Hang_3(0,leg_fore_right_runed);					
			}
			if(0==leg_fore_right_runed)    //若右小腿复位，则将左肢放平到水平位置
			{
				Uart_Hand_Hang_3_4(0,leg_right_runed);
			}
			if((0==leg_fore_right_runed)&&(0==leg_right_runed))  //标志位复位
			{
				leg_fore_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForeRightHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Arm_Post_Left(void)  
 函数功能    ：手动执行左大臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Arm_Post_Left(void)
{
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //向上运行		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftUpHand"))
		{
			if(arm_post_left_lim>arm_post_left_runed)
			{
			    if(0==arm_post_left_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmPostLeftHandStart");
					delay_ms(200);
				}
				arm_post_left_flag=1;
				Uart_Hand_Hang_1_2(1,arm_post_left_lim-arm_post_left_runed);	
				if(arm_post_left_runed==arm_post_left_lim)
				{
					arm_post_left_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmPostLeftHandLim");
					delay_ms(200);
				}
				else
				{
					arm_post_left_flag=1;						
				}								
			}	
		}
         //向下运行		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftDownHand"))
		{
			if(arm_post_left_runed>0)
			{
				Uart_Hand_Hang_1_2(0,arm_post_left_runed);
				if(0==arm_post_left_runed)
				{					
					arm_post_left_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("ArmPostLeftHandRes");
					delay_ms(200);
				}
				else
				{
					arm_post_left_flag=1;			
				}					
			}				
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_ArmLeg_Post_Left(void)  
 函数功能    ：手动执行左大腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Leg_Post_Left(void)
{
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //向上运行		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftUpHand"))
		{
			if(leg_post_left_lim>leg_post_left_runed)
			{
				if(0==leg_post_left_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegPostLeftHandStart");
					delay_ms(200);
				}
				leg_post_left_flag=1;
				Uart_Hand_Hang_1_2(1,leg_post_left_lim-leg_post_left_runed);	
				if(leg_post_left_runed==leg_post_left_lim)
				{
					leg_post_left_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegPostLeftHandLim");
					delay_ms(200);
				}
				else
				{
					leg_post_left_flag=1;						
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftDownHand"))
		{
			if(leg_post_left_runed>0)
			{
				Uart_Hand_Hang_1_2(0,leg_post_left_runed);
				if(0==leg_post_left_runed)
				{					
					leg_post_left_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("LegPostLeftHandRes");
					delay_ms(200);
				}
				else
				{
					leg_post_left_flag=1;			
				}					
			}				
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Arm_Post_Right(void)  
 函数功能    ：手动执行右大臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Arm_Post_Right(void)
{
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //向上运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightUpHand"))
		{
			if(arm_post_right_lim>arm_post_right_runed)
			{
				if(0==arm_post_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmPostRightHandStart");
					delay_ms(200);
				}
				arm_post_right_flag=1;
				Uart_Hand_Hang_3_4(1,arm_post_right_lim-arm_post_right_runed);	
				if(arm_post_right_runed==arm_post_right_lim)
				{
					arm_post_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmPostRightHandLim");
					delay_ms(200);
				}
				else
				{
					arm_post_right_flag=1;						
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightDownHand"))
		{
			if(arm_post_right_runed>0)
			{
				Uart_Hand_Hang_3_4(0,arm_post_right_runed);
				if(0==arm_post_right_runed)
				{
					arm_post_right_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("ArmPostRightHandRes");
					delay_ms(200);
				}
				else
				{
					arm_post_right_flag=1;			
				}					
			}				
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Leg_Post_Right(void)  
 函数功能    ：手动执行右大腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Leg_Post_Right(void)
{
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //同过上下行判断脉冲累计
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightUpHand"))
		{
			if(leg_post_right_lim>leg_post_right_runed)
			{
				if(0==leg_post_right_flag)
				{
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("LegPostRightHandStart");
					delay_ms(200);
				}
				leg_post_right_flag=1;
				Uart_Hand_Hang_3_4(1,leg_post_right_lim-leg_post_right_runed);	
				if(leg_post_right_runed==leg_post_right_lim)
				{
					leg_post_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegPostRightHandLim");
					delay_ms(200);
				}
				else
				{
					leg_post_right_flag=1;					
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightDownHand"))
		{
			if(leg_post_right_runed>0)
			{
				Uart_Hand_Hang_3_4(0,leg_post_right_runed);
				if(0==leg_post_right_runed)
				{
					leg_post_right_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("LegPostRightHandRes");
					delay_ms(200);
				}
				else
				{
					leg_post_right_flag=1;
				}					
			}			
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Arm_Fore_Post_Left(void)  
 函数功能    ：手动执行左大小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Arm_Fore_Post_Left(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
     DG_Relay=1;		//继电器得电
		 //向上运行		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftUpHand"))
		{
			 //先运行左大臂
			if(arm_post_left_lim>arm_post_left_runed)
			{
				if(0==arm_fore_post_left_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmForePostLeftHandStart");
					delay_ms(200);
				}
				arm_fore_post_left_flag=1;
				Uart_Hand_Hang_1_2(1,arm_post_left_lim-arm_post_left_runed);	
				if(arm_post_left_runed==arm_post_left_lim)
				{
					arm_fore_post_left_flag=1;					
				}
				else
				{
					arm_fore_post_left_flag=1;						
				}								
			}
			//再运行左小臂
			if((arm_fore_left_lim>arm_fore_left_runed)&&(arm_post_left_runed==arm_post_left_lim))  
			{
				Uart_Hand_Hang_1(1,arm_fore_left_lim-arm_fore_left_runed);	
				if(arm_fore_left_runed==arm_fore_left_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForePostLeftHandLim");
					delay_ms(200);
				}
			}			
		}	
		//向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftDownHand"))
		{
			//先运行左小臂
			if(arm_fore_left_runed>0)   
			{
				Uart_Hand_Hang_1(0,arm_fore_left_runed);					
			}
			//再运行左大臂
			if((arm_post_left_runed>0)&&(0==arm_fore_left_runed))
			{
				Uart_Hand_Hang_1_2(0,arm_post_left_runed);	
			}
			if((0==arm_post_left_runed)&&(0==arm_fore_left_runed))
			{
				arm_fore_post_left_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForePostLeftHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_ArmLeg_Fore_Post_Left(void)  
 函数功能    ：手动执行左大小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Leg_Fore_Post_Left(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	//连锁功能只有在键锁打开、左肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //向上运行		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftUpHand"))
		{
			 //先运行左大腿
			if(leg_post_left_lim>leg_post_left_runed)
			{
				if(0==leg_fore_post_left_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegForePostLeftHandStart");
					delay_ms(200);
				}
				leg_fore_post_left_flag=1;
				Uart_Hand_Hang_1_2(1,leg_post_left_lim-leg_post_left_runed);	
				if(leg_post_left_runed==leg_post_left_lim)
				{
					leg_fore_post_left_flag=1;
				}
				else
				{
					leg_fore_post_left_flag=1;						
				}								
			}
			//再运行左小腿
			if((leg_fore_left_lim>leg_fore_left_runed)&&(leg_post_left_runed==leg_post_left_lim))  
			{
				Uart_Hand_Hang_1(1,leg_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==leg_fore_left_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForePostLeftHandLim");
					delay_ms(200);
				}
			}							
		}
		//向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftDownHand"))
		{
			//先运行左小腿
			if(leg_fore_left_runed>0)   
			{
				Uart_Hand_Hang_1(0,leg_fore_left_runed);					
			}
			//再运行左大腿
			if((leg_post_left_runed>0)&&(0==leg_fore_left_runed))
			{
				Uart_Hand_Hang_1_2(0,leg_post_left_runed);	
			}			
			if((0==leg_post_left_runed)&&(0==leg_fore_left_runed))
			{
				leg_fore_post_left_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForePostLeftHandRes");
				delay_ms(200);
			}				
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Arm_Fore_Post_Right(void)  
 函数功能    ：手动执行右大小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Arm_Fore_Post_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //同过上下行判断脉冲累计
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightUpHand"))
		{
			 //先运行右大臂
			if(arm_post_right_lim>arm_post_right_runed)
			{
				if(0==arm_fore_post_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmForePostRightHandStart");
					delay_ms(200);
				}
				arm_fore_post_right_flag=1;
				Uart_Hand_Hang_3_4(1,arm_post_right_lim-arm_post_right_runed);	
				if(arm_post_right_runed==arm_post_right_lim)
				{
					arm_fore_post_right_flag=1;
				}
				else
				{
					arm_fore_post_right_flag=1;						
				}								
			}
			//再运行右小臂
			if((arm_fore_right_lim>arm_fore_right_runed)&&(arm_post_right_runed==arm_post_right_lim))  
			{
				Uart_Hand_Hang_3(1,arm_fore_right_lim-arm_fore_right_runed);
				if(arm_fore_right_runed==arm_fore_right_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForePostRightHandLim");
					delay_ms(200);
				}
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightDownHand"))
		{
			//先运行右小臂
			if(arm_fore_right_runed>0)   
			{
				Uart_Hand_Hang_3(0,arm_fore_right_runed);					
			}
			//再运行右大臂
			if((arm_post_right_runed>0)&&(0==arm_fore_right_runed))
			{
				Uart_Hand_Hang_3_4(0,arm_post_right_runed);	
			}
			if((0==arm_post_right_runed)&&(0==arm_fore_right_runed))
			{
				arm_fore_post_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForePostRightHandRes");
				delay_ms(200);
			}			
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Leg_Fore_Post_Right(void)  
 函数功能    ：手动执行右大小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Leg_Fore_Post_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	//连锁功能只有在键锁打开、右肢或左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //同过上下行判断脉冲累计
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightUpHand"))
		{
			 //先运行右大腿
			if(leg_post_right_lim>leg_post_right_runed)
			{
				if(0==leg_fore_post_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegForePostRightHandStart");
					delay_ms(200);
				}
				leg_fore_post_right_flag=1;
				Uart_Hand_Hang_3_4(1,leg_post_right_lim-leg_post_right_runed);	
				if(leg_post_right_runed==leg_post_right_lim)
				{
					leg_fore_post_right_flag=1;
				}
				else
				{
					leg_fore_post_right_flag=1;						
				}								
			}
			//再运行右小腿
			if((leg_fore_right_lim>leg_fore_right_runed)&&(leg_post_right_runed==leg_post_right_lim))  
			{
				Uart_Hand_Hang_3(1,leg_fore_right_lim-leg_fore_right_runed);
				if(leg_fore_right_runed==leg_fore_right_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForePostRightHandLim");
					delay_ms(200);
				}
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightDownHand"))
		{
			//先运行右小腿
			if(leg_fore_right_runed>0)   
			{
				Uart_Hand_Hang_3(0,leg_fore_right_runed);					
			}
			//再运行右大腿
			if((leg_post_right_runed>0)&&(0==leg_fore_right_runed))
			{
				Uart_Hand_Hang_3_4(0,leg_post_right_runed);	
			}
			if((0==leg_post_right_runed)&&(0==leg_fore_right_runed))
			{
				leg_fore_post_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForePostRightHandRes");
				delay_ms(200);
			}				
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Arm_Fore_Left_Right(void)  
 函数功能    ：手动执行左右小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Arm_Fore_Left_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag;
	//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行左右小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 if(0==arm_fore_left_right_flag)    //先将左右肢抬到一定高度
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmForeLeftRightHandStart");
				delay_ms(200);
				arm_fore_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,arm_left_right_lim);
				runed_flag=1;
			}
		}
		//向上运行
		if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))||(1==runed_flag))
		{	
			if(arm_fore_left_right_lim>arm_fore_left_right_runed)
			{
				runed_flag=0;
				Uart_Hand_Hang_1_3(1,arm_fore_left_right_lim-arm_fore_left_right_runed);	
				if(arm_fore_left_right_runed==arm_fore_left_right_lim)
				{
					arm_fore_left_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForeLeftRightHandLim");
					delay_ms(200);
				}
				else
				{
					arm_fore_left_right_flag=1;						
				}								
			}	
		}	
		//向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))
		{
			if(arm_fore_left_right_runed>0)      //左右小臂向下运行
			{
				Uart_Hand_Hang_1_3(0,arm_fore_left_right_runed);					
			}
			if(arm_fore_left_right_runed==0)     //若左右小臂复位，则将左右肢复位到水平位置
			{
				Uart_Hand_Hang_1_2_3_4(0,arm_left_right_runed);
			}
			if((arm_fore_left_right_runed==0)&&(0==arm_left_right_runed))  //标志位置位
			{
				arm_fore_left_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForeLeftRightHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Leg_Fore_Left_Right(void)  
 函数功能    ：手动执行左右小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Leg_Fore_Left_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
	 static u8 runed_flag;
	//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行左右小臂/小腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
	   DG_Relay=1;		//继电器得电
		 if(0==leg_fore_left_right_flag)
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegForeLeftRightHandStart");
				delay_ms(200);
				leg_fore_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,leg_left_right_lim);
				runed_flag=1;
			}				
		}
		//同过上下行判断脉冲累计
		if((strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightUpHand"))||(1==runed_flag))
		{
			if(leg_fore_left_right_lim>leg_fore_left_right_runed)
			{
				runed_flag=0;
				Uart_Hand_Hang_1_3(1,leg_fore_left_right_lim-leg_fore_left_right_runed);	
				if(leg_fore_left_right_runed==leg_fore_left_right_lim)
				{
					leg_fore_left_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForeLeftRightHandLim");
					delay_ms(200);
				}
				else
				{
					leg_fore_left_right_flag=1;						
				}								
			}	
		}
		//向下运行
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightDownHand"))
		{
			if(leg_fore_left_right_runed>0)    //左右小腿向下运行
			{
				Uart_Hand_Hang_1_3(0,leg_fore_left_right_runed);					
			}
			if(leg_fore_left_right_runed==0)    //若左右小腿复位，则将左右腿复位到水平位置
			{
				Uart_Hand_Hang_1_2_3_4(0,leg_left_right_runed);
			}
			if((leg_fore_left_right_runed==0)&&(0==leg_left_right_runed))   //标志位置位
			{
				leg_fore_left_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForeLeftRightHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Arm_Post_Left_Right(void)  
 函数功能    ：手动执行左右大臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Arm_Post_Left_Right(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行左右大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //同过上下行判断脉冲累计		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))
		{
			if(arm_post_left_right_lim>arm_post_left_right_runed)
			{
				if(0==arm_post_left_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmPostLeftRightHandStart");
					delay_ms(200);
				}
				arm_post_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,arm_post_left_right_lim-arm_post_left_right_runed);	
				if(arm_post_left_right_runed==arm_post_left_right_lim)
				{
					arm_post_left_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmPostLeftRightHandLim");
					delay_ms(200);
				}
				else
				{
					arm_post_left_right_flag=1;						
				}								
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))
		{
			if(arm_post_left_right_runed>0)
			{
				Uart_Hand_Hang_1_2_3_4(0,arm_post_left_right_runed);
				if(0==arm_post_left_right_runed)
				{
					arm_post_left_right_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("ArmPostLeftRightHandRes");
					delay_ms(200);
				}
				else
				{
					arm_post_left_right_flag=1;			
				}					
			}				
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Leg_Post_Left_Right(void)  
 函数功能    ：手动执行左右大腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Leg_Post_Left_Right(void)
{
	 u8 direct;    //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行左右大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //同过上下行判断脉冲累计		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightUpHand"))
		{
			if(leg_post_left_right_lim>leg_post_left_right_runed)
			{
				if(0==leg_post_left_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegPostLeftRightHandStart");
					delay_ms(200);
				}
				leg_post_left_right_flag=1;	
				Uart_Hand_Hang_1_2_3_4(1,leg_post_left_right_lim-leg_post_left_right_runed);	
				if(leg_post_left_right_runed==leg_post_left_right_lim)
				{
					leg_post_left_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegPostLeftRightHandLim");
					delay_ms(200);
				}
				else
				{
					leg_post_left_right_flag=1;							
				}								
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightDownHand"))
		{
			if(leg_post_left_right_runed>0)
			{
				Uart_Hand_Hang_1_2_3_4(0,leg_post_left_right_runed);
				if(0==leg_post_left_right_runed)
				{
					leg_post_left_right_flag=0;	
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("LegPostLeftRightHandRes");
					delay_ms(200);
				}
				else
				{
					leg_post_left_right_flag=1;			
				}					
			}			
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Arm_Fore_Post_Left_Right(void)  
 函数功能    ：手动执行左右大小臂
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Arm_Fore_Post_Left_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //向上运动
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))
		{
			 //先运行左右大臂
			if(arm_post_left_right_lim>arm_post_left_right_runed)
			{
				if(0==arm_fore_post_left_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmForePostLeftRightHandStart");
					delay_ms(200);
				}
				arm_fore_post_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,arm_post_left_right_lim-arm_post_left_right_runed);	
				if(arm_post_left_right_runed==arm_post_left_right_lim)
				{
					arm_fore_post_left_right_flag=1;
				}
				else
				{
					arm_fore_post_left_right_flag=1;						
				}								
			}
			//再运行左右小臂
			if((arm_fore_left_right_lim>arm_fore_left_right_runed)&&(arm_post_left_right_runed==arm_post_left_right_lim))  
			{
				Uart_Hand_Hang_1_3(1,arm_fore_left_right_lim-arm_fore_left_right_runed);
				if(arm_fore_left_right_runed==arm_fore_left_right_lim)
				{
					
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForePostLeftRightHandLim");
					delay_ms(200);
				}
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))
		{
			//先运行左右小臂
			if(arm_fore_left_right_runed>0)   
			{
				Uart_Hand_Hang_1_3(0,arm_fore_left_right_runed);					
			}
			//再运行左右大臂
			if((arm_post_left_right_runed>0)&&(0==arm_fore_left_right_runed))
			{
				Uart_Hand_Hang_1_2_3_4(0,arm_post_left_right_runed);	
			}
			if((0==arm_post_left_right_runed)&&(0==arm_fore_left_right_runed))
			{
				arm_fore_post_left_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForePostLeftRightHandRes");
				delay_ms(200);
			}				
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;	
	}	
}

/***********************************************************************
 函数名      ：Uart_Hand_Leg_Fore_Post_Left_Right(void)  
 函数功能    ：手动执行左右大小腿
 输入        ：无
 输出        ：无                           
************************************************************************/
void Uart_Hand_Leg_Fore_Post_Left_Right(void)
{
	 u8 direct;     //代表没某个动作运行的方向标志：1-正向运行；0-反向运行
//连锁功能只有在键锁打开、左右肢上去、支背、坐便、翻身复位的情况下才能进行大臂/大腿康复训练
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//继电器得电
		 //同过上下行判断脉冲累计
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))
		{
			 //先运行左右大腿
			if(leg_post_left_right_lim>leg_post_left_right_runed)
			{
				if(0==leg_fore_post_left_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegForePostLeftRightHandStart");
					delay_ms(200);
				}
				leg_fore_post_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,leg_post_left_right_lim-leg_post_left_right_runed);	
				if(leg_post_left_right_runed==leg_post_left_right_lim)
				{
					leg_fore_post_left_right_flag=1;
				}
				else
				{
					leg_fore_post_left_right_flag=1;						
				}								
			}
			//再运行左右小腿
			if((leg_fore_left_right_lim>leg_fore_left_right_runed)&&(leg_post_left_right_runed==leg_post_left_right_lim))  
			{
				Uart_Hand_Hang_1_3(1,leg_fore_left_right_lim-leg_fore_left_right_runed);
				if(leg_fore_left_right_runed==leg_fore_left_right_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForePostLeftRightHandLim");
					delay_ms(200);
				}
			}
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightDownHand"))
		{
			//先运行左右小腿
			if(leg_fore_left_right_runed>0)   
			{
				Uart_Hand_Hang_1_3(0,leg_fore_left_right_runed);					
			}
			//再运行左右大腿
			if((leg_post_left_right_runed>0)&&(0==leg_fore_left_right_runed))
			{
				Uart_Hand_Hang_1_2_3_4(0,leg_post_left_right_runed);	
			}
			if((0==leg_post_left_right_runed)&&(0==leg_fore_left_right_runed))
			{
				leg_fore_post_left_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForePostLeftRightHandRes");
				delay_ms(200);
			}			
		}
		DG_Relay=0;		//继电器失电
	}
	else
	{
		LED0=0;             //若不满足条件，LED0/LED1闪一下
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;	
	}	
}

/***********************************************************************
 函数名      ：Uart_Res_Power_Down(void)  
 函数功能    ：掉电复位,每个功能函数停止条件靠光电开关
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Res_Power_Down(void)
{	
	if(lock_flag==1)
	{
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		//读取翻身复位状态标志位
		u8 	body_left_flag_buf[1];
		u8 	body_right_flag_buf[1];
		W25QXX_Read((u8*)body_left_flag_buf,33,1);        //从第33地址开始，读取1个字节
		W25QXX_Read((u8*)body_right_flag_buf,34,1);    //从第34地址开始，读取1个字节
		body_left_flag=body_left_flag_buf[0];
		body_right_flag=body_right_flag_buf[0];
		
		if(1==body_left_flag)     //左翻复位
		{			
			Uart_Res_Body_Left();    		
		}		
		if(1==body_right_flag)    //右翻复位
		{
			Uart_Res_Body_Right();   			
		}	
		delay_ms(1000);
		
		washlet_picture_k=24;
		Uart_Washlet(1);          //坐便器复位       
		delay_ms(1000);
		
		Uart_Res_Leg();           //曲腿复位
		delay_ms(1000);
		
		Uart_Res_Back();          //支背复位     
		delay_ms(1000);

		Uart_Res_Desk();          //办公娱乐一体桌复位   
	}
}




void Uart_Res_Power_Down1(void)
{	
	if(lock_flag==1)
	{
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		//读取翻身复位状态标志位
		u8 	body_left_flag_buf[1];
		u8 	body_right_flag_buf[1];
		W25QXX_Read((u8*)body_left_flag_buf,33,1);        //从第33地址开始，读取1个字节
		W25QXX_Read((u8*)body_right_flag_buf,34,1);    //从第34地址开始，读取1个字节
		body_left_flag=body_left_flag_buf[0];
		body_right_flag=body_right_flag_buf[0];
		
		if(1==body_left_flag)     //左翻复位
		{			
			Uart_Res_Body_Left();    		
		}		
		else if(1==body_right_flag)    //右翻复位
		{
			Uart_Res_Body_Right();   			
		}	
		delay_ms(1000);

	
//		washlet_picture_k=24;
//		Uart_Washlet(1);          //坐便器复位       
//		delay_ms(1000);
		
		Uart_Res_Leg();           //曲腿复位
		delay_ms(1000);
		
		Uart_Res_Back();          //支背复位     
		delay_ms(1000);


		if(GD7S==1)
		{
				Uart_Res_Desk();          //办公娱乐一体桌复位 
		}		

		if(GD6S==1)
		{
			washlet_flag=1;
			RELAY6=1; 
			Uart_Push_Rod_Swash(0,25000);  //冲洗烘干推杆缩回
			RELAY6=0;	 
			washlet_flag=0;	
			WashLet_V1(0,32950);	
		}
		
		
		
		
	}
}


void Uart_Res_Power_Down2(void)
{
	if(lock_flag==1)
	{
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		//读取翻身复位状态标志位
		u8 	body_left_flag_buf[1];
		u8 	body_right_flag_buf[1];
		W25QXX_Read((u8*)body_left_flag_buf,33,1);        //从第33地址开始，读取1个字节
		W25QXX_Read((u8*)body_right_flag_buf,34,1);    //从第34地址开始，读取1个字节
		body_left_flag=body_left_flag_buf[0];
		body_right_flag=body_right_flag_buf[0];
		
		if(1==body_left_flag)     //左翻复位
		{			
			Uart_Res_Body_Left();    		
		}		
		else if(1==body_right_flag)    //右翻复位
		{
			Uart_Res_Body_Right();   			
		}	
		delay_ms(1000);

	
//		washlet_picture_k=24;
//		Uart_Washlet(1);          //坐便器复位       
//		delay_ms(1000);
		
		Uart_Res_Leg();           //曲腿复位
		delay_ms(1000);
		
		Uart_Res_Back();          //支背复位     
		delay_ms(1000);


		if(GD7S==1)
		{
				Uart_Res_Desk();          //办公娱乐一体桌复位 
		}		

		if(GD6S==1)
		{
			washlet_flag=1;
			RELAY6=1; 
			Uart_Push_Rod_Swash(0,25000);  //冲洗烘干推杆缩回
			RELAY6=0;	 
			washlet_flag=0;			
			WashLet_V1(0,32950);	
		}
	}
}



/***********************************************************************
 函数名      ：Uart_Res_Back(void)  
 函数功能    ：支背复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Res_Back(void)
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	back_runed_arr=back_angle_to_arr(back_angle_lim);
	back_picture_k=19;
	WriteInUART2("BackDownPhone");
	Uart_Back();                //支背复位		
}

/***********************************************************************
 函数名      ：Uart_Res_Leg(void)  
 函数功能    ：曲腿复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Res_Leg(void)
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim);  //下曲腿上行
	leg_down_picture_k=19;
	WriteInUART2("LegDownUpPhone");
	Uart_Leg_Down();           
	delay_ms(1000);	

	WriteInUART2("LegUpUpPhone");
	Uart_Leg_Up();              //上曲腿
	delay_ms(1000);
	WriteInUART2("LegUpDownPhone");
	Uart_Leg_Up();              //上曲腿复位
	delay_ms(1000);		
}

/***********************************************************************
 函数名      ：Uart_Res_Desk(void)  
 函数功能    ：就餐娱乐一体桌复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Res_Desk(void)
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
	desk_picture_k=19;
	WriteInUART2("DeskDownPhone");
	Uart_Desk();             //小桌子复位
	delay_ms(1000);	
}

	
/***********************************************************************
 函数名      ：Uart_Reset_Motor5_0(void)  
 函数功能    ：5号电机复位:
 输入        ：反向复位0;正向复位1
 输出        ：无
                           
************************************************************************/
void Uart_Res_Motor5(u8 dir) 
{
	u8 key;
	u16 arr_feed;      //计算程序中当前一次运行脉冲数,用于判断电机失步故障
	u16 pulse_num=0;   //将程序中当前一次运行脉冲数进行转化，与电机反馈的当前运行脉冲数相比较
	u16 num=0;
	DIR5=dir;  
	u8 direct=0;       
	Motor_5_START(motor_body_freq,motor_timer_freq);	//电机启动	
	TIM10_Init(20000,timer10_freq);                     //打开定时器
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位
	while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
	{				
		//光电限位
		if((0==GD5_Start)&&(0==direct))   //碰到光电开关跳出循环，电机停转 
		{
			delay_us(100);
			if(0==GD5_Start)
			{
				u2_printf("GD5Start");
				break;	
			}				
		}
		//电机过载
		if(1==Motor5_Alm)      
		{						
			delay_us(100);
			if(1==Motor5_Alm)
			{
				if(body_left_flag==1)
				{
					body_left_overload_5=1;
					u2_printf("Reset_Body_Left_Overload_5");
				}
				if(body_right_flag==1)
				{
					body_right_overload_5=1;
					u2_printf("Reset_Body_Right_Overload_5");
				}
				Uart_Breakdown_Treatment();
				break;
			}			
		}
		  //判断有没有收到上位机指令		
		if(USART2_RX_LEN&0x8000)
		{
			u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数	
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;												
		}		
		//电机失步或异常停止
//		key=KEY_Scan(0);              //按键扫描
//		if(key==Motor5_Tim_PRES)
//		{
//			num++;
//		}	
//		arr_feed=__HAL_TIM_GET_COUNTER(&TIM10_Handler); 
//		pulse_num=arr_feed/7.44*timer10_freq*0.36/(motor_body_freq*motor_timer_freq);						

//		if((abs(pulse_num,num)>150)&&pulse_num>0)
//		{
//			if(body_left_flag==1)
//			{
//				body_left_losepulse=1;
//				u2_printf("Reset_Body_Left_Losepulse_5");
//			}
//			if(body_right_flag==1)
//			{
//				body_right_losepulse=1;
//				u2_printf("Reset_Body_Right_Losepulse_5");
//			}
//			Uart_Breakdown_Treatment();
//			break;
//		}		
	}		      
	Motor_5_STOP();     //电机停止
	TIM10_Stop();       //关闭定时器
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	
	//若定时时间到但是光电没到，则电机再运行一段距离，到达光电开关位置（光电开关安装后打开此段）
//	if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD5_Start==1)&&(direct==0))
//	{  
//		DIR5=dir;
//		Motor_5_START(motor_body_freq,motor_timer_freq);    //电机启动
//		TIM10_Init(add_arr,timer10_freq);                   //打开定时器
//		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //清除中断标志位	 	
//		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//		{			
//			if(0==GD5_Start)  //运行时碰到光电开关，跳出循环 
//			{
//				delay_us(100);
//				if(0==GD5_Start) 
//				{
//					u2_printf("GD5Start");
//					break;	
//				}					
//			}
//		}			
//		Motor_5_STOP();     //电机停止
//	    TIM10_Stop();       //关闭定时器
//		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	
//	}	
}


/***********************************************************************
 函数名      ：Uart_Res_Body_Left(void)  
 函数功能    ：左翻身复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Res_Body_Left(void)
{		
	//4号电机复位
	lock_flag==1;
	body_left_flag=1;
	back_nursing_left_flag==0;
	waist_nursing_left_dir_flag=0;
	waist_nursing_left_picture_k=19;
	body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
	Uart_Waist_Nursing_Left();    //左腰部护理复位
	
	//3号电机复位
	lock_flag=1;
	body_left_flag=1;
	waist_nursing_left_flag=0;
	back_nursing_left_dir_flag=0;
	back_nursing_left_picture_k=19;
	body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
	Uart_Back_Nursing_Left();     //左背部护理复位
	
	//5号电机复位
//	Res_Motor5(0); 
//	body_left_runed_arr=0;
//	body_left_flag=0;
//	back_nursing_left_flag=0;
//	waist_nursing_left_flag=0;
//	back_nursing_left_dir_flag=0;
//	waist_nursing_left_dir_flag=0;		
}

/***********************************************************************
 函数名      ：Uart_Res_Body_Right(void)  
 函数功能    ：右翻身复位，用于掉电复位
 输入        ：无
 输出        ：无
                           
************************************************************************/
void Uart_Res_Body_Right(void)
{
	//4号电机复位
	lock_flag==1;
	body_right_flag=1;
	back_nursing_right_flag==0;
	waist_nursing_right_dir_flag=0;
	waist_nursing_right_picture_k=19;
	body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
	Uart_Waist_Nursing_Right();   //右腰部护理复位
	
	//3号电机复位
	lock_flag=1;
	body_right_flag=1;
	waist_nursing_right_flag=0;
	back_nursing_right_dir_flag=0;
	back_nursing_right_picture_k=19;
	body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
	Uart_Back_Nursing_Right();    //右背部护理复位 
	
	//5号电机复位 
//	Res_Motor5(1);
//	body_right_runed_arr=0;
//	body_right_flag=0;
//	back_nursing_right_flag=0;
//	waist_nursing_right_flag=0;
//	back_nursing_right_dir_flag=0;
//	waist_nursing_right_dir_flag=0;	
}


/***********************************************************************
 函数名      ： 
 函数功能    ：吊挂测试函数
 输入        ：无
 输出        ：无
***********************************************************************/                           

void  TestAll(u8 dir)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;
	HANG_DIR1=dir;       //3号吊挂电机转动方向控制，高电平正转，低电平反转 
	HANG_DIR2=!dir;
	HANG_DIR3=!dir;
	HANG_DIR4=dir;
	
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
	
    for(i=0;i<1500000;i++)
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM1=flag;               //3号吊挂电机脉冲输出口高电平				 
		HANG_PWM2=flag;
		HANG_PWM3=flag;
		HANG_PWM4=flag;
	}
	HANG_PWM1=0;              //3号吊挂电机对应脉冲输出口拉低 
	HANG_PWM2=0;
	HANG_PWM2=0;
	HANG_PWM2=0;
	TIM10_Stop();             //关闭定时器	
}	


/***********************************************************************
 函数名      ：Hang1Test(u8 dir)
 函数功能    ：吊挂测试函数
 输入        ：dir，代表方向
 输出        ：无
***********************************************************************/   
void Hang1Test(u8 dir)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;
	HANG_DIR1=dir;       //3号吊挂电机转动方向控制，高电平正转，低电平反转 
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(50-1,motor_timer_freq);                       //打开定时器3
	
    for(i=0;i<1500000;i++)   //最终确定为3000000
    { 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
			flag = !flag ;
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
			HANG_PWM1=flag;               //3号吊挂电机脉冲输出口高电平				 
	  }
	HANG_PWM1=0;              //3号吊挂电机对应脉冲输出口拉低 
	TIM10_Stop();             //关闭定时器
}

void Hang2Test(u8 dir)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;
	HANG_DIR2=dir;       //3号吊挂电机转动方向控制，高电平正转，低电平反转 
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
	
    for(i=0;i<1500000;i++)  //最终确定为3000000
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM2=flag;               //3号吊挂电机脉冲输出口高电平				 
	}
	HANG_PWM2=0;              //3号吊挂电机对应脉冲输出口拉低 
	TIM10_Stop();             //关闭定时器
}

void Hang3Test(u8 dir)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;
	HANG_DIR3=dir;       //3号吊挂电机转动方向控制，高电平正转，低电平反转 
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
	
    for(i=0;i<1500000;i++)   //最终确定为3000000
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM3=flag;               //3号吊挂电机脉冲输出口高电平				 
	}
	HANG_PWM3=0;              //3号吊挂电机对应脉冲输出口拉低 
	TIM10_Stop();             //关闭定时器
}

void Hang4Test(u8 dir)
{
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;
	HANG_DIR4=dir;       //3号吊挂电机转动方向控制，高电平正转，低电平反转 
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
	
    for(i=0;i<1500000;i++)    //最终确定为3000000
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM4=flag;               //3号吊挂电机脉冲输出口高电平
		
	}
	HANG_PWM4=0;              //3号吊挂电机对应脉冲输出口拉低 
	TIM10_Stop();             //关闭定时器
}


/***********************************************************************
 函数名      ：Hang1Test(u8 dir)
 函数功能    ：吊挂测试函数
 输入        ：dir，代表方向
 输出        ：无
***********************************************************************/ 


void test(u8 dir)
{  
//电机1
	Hang_Init();         //电机方向口初始化函数
	int i,flag=1;        //flag为脉冲高低标志位
	int j=0,k=0;
	HANG_DIR1=dir;       //3号吊挂电机转动方向控制，高电平正转，低电平反转 
	HANG_DIR2=dir;
	HANG_DIR3=dir;
	HANG_DIR4=dir;
	
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //打开定时器3
	
    for(i=0;i<30000;i++)
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //等待定时时间到，时间到跳出循环
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //清除中断标志位
		HANG_PWM1=flag;               //3号吊挂电机脉冲输出口高电平				 
		HANG_PWM2=flag;
		HANG_PWM3=flag;
		HANG_PWM4=flag;
	}
	HANG_PWM1=0;              //3号吊挂电机对应脉冲输出口拉低 
	HANG_PWM2=0;
	HANG_PWM2=0;
	HANG_PWM2=0;
	TIM10_Stop();             //关闭定时器
			
}


void FlagClear(void)
{
	back_flag=0;               //支背
	leg_up_flag=0;             //上曲腿
	leg_down_flag=0;           //下曲腿
	body_left_flag=0;          //左翻
	body_right_flag=0;         //右翻
	back_nursing_left_flag=0;  //左背部护理
	back_nursing_right_flag=0; //右背部护理
	waist_nursing_left_flag=0; //左腰部护理
	waist_nursing_right_flag=0;//右腰部护理
	washlet_flag=0;            //坐便器
	washlet_auto_flag=0;       //自动坐便器
	desk_flag=0;               //就餐娱乐一体桌
	jram_flag=0;               //肌肉按摩
	swash_dry_flag=0;          //冲洗烘干
	lock_flag=1;               //一键锁定程序
	fault_flag=0;              //电机故障标志位

	swash_hand_flag=0;         //手动冲洗标志位
	dry_hand_flag=0;           //手动烘干标志位

	//吊挂
	armleg_left_flag=0;        //手动左肢
	armleg_right_flag=0;       //手动右肢
	armleg_left_right_flag=0;  //手动左右肢

	arm_fore_left_flag=0;      //左小臂
	leg_fore_left_flag=0;      //左小腿

	arm_fore_right_flag=0;     //右小臂
	leg_fore_right_flag=0;     //右小腿

	arm_post_left_flag=0;      //左大臂
	leg_post_left_flag=0;      //左大腿

	arm_post_right_flag=0;     //右大臂
	leg_post_right_flag=0;     //右大腿

	arm_fore_post_left_flag=0;       //左大小臂
	leg_fore_post_left_flag=0;       //左大小腿

	arm_fore_post_right_flag=0;      //右大小臂
	leg_fore_post_right_flag=0;      //右大小腿

	arm_fore_left_right_flag=0;      //左右小臂
	leg_fore_left_right_flag=0;      //左右小腿

	arm_post_left_right_flag=0;      //左右大臂
	leg_post_left_right_flag=0;      //左右大腿

	arm_fore_post_left_right_flag=0; //左右大小臂
	leg_fore_post_left_right_flag=0; //左右大小腿
}

