#include "check.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "pump.h"
#include "function.h"
#include "common.h"
#include "motor.h"
#include "w25qxx.h"
#include "key.h"
#include "pcf8574.h"

/***********************************************************************
 函数名      ：Breakdown_Judge(void)  
 函数功能    ：故障诊断函数，判断电机是否出现运行干涉错误
 输入        ：无
 输出        ：1：出现错误；0：未出现错误
                           
************************************************************************/
u8 Breakdown_Judge(void)
{		
	//支背未复位，翻身不能运行
	if((1==back_flag)&&((1==body_left_flag)||(1==body_right_flag))) 
	{
		return 1;
	}
	//翻身未复位，曲腿、桌子、坐便器不能运行
	if(((1==body_left_flag)||(1==body_right_flag))&&((1==leg_up_flag)||(1==leg_down_flag)||(1==washlet_flag)||(1==desk_flag))) 
	{
		return 1;
	}
	//左/右翻身未复位，右/左翻身不能运行
	if((1==body_left_flag)&&(1==body_right_flag))
	{
		return 1;
	}
	//左背部护理未复位，左腰部护理及右翻不能运行
	if((back_nursing_left_flag==1)&&((waist_nursing_left_flag==1)||(1==body_right_flag)))
	{
		return 1;
	}
	//左腰部护理未复位，左背部护理及右翻不能运行
	if((waist_nursing_left_flag==1)&&((back_nursing_left_flag==1)||(1==body_right_flag)))
	{
		return 1;
	}
	//右背部护理未复位，右腰部护理及左翻不能运行
	if((back_nursing_right_flag==1)&&((waist_nursing_right_flag==1)||(1==body_left_flag)))
	{
		return 1;
	}
	//右腰部护理未复位，右背部护理及左翻不能运行
	if((waist_nursing_right_flag==1)&&((back_nursing_right_flag==1)||(1==body_left_flag)))
	{
		return 1;
	}
	else 
	{
		return 0;
	}
}

/***********************************************************************
 函数名      ：Breakdown_Treatment(void)  
 函数功能    ：电机故障处理函数，可用于电机故障后，发指令复位各动作
 输入        ：无
 输出        ：1：故障排除后，返回1                        
************************************************************************/
u8 Breakdown_Treatment(void)
{
	u8 len;
	Motor_All_Stop();	            //所有电机停止
	delay_ms(100);
	//Wifi_Send("Fault_Warning");     //向上位机发出故障报警信号
	u2_printf("Fault_Warning");
	PCF8574_WriteBit(BEEP_IO,0);	//控制蜂鸣器发出报警声
	
/*************判断电机过载故障**********************************/		
	//左翻身
	if(1==body_left_overload_3)     
	{	
		Wifi_Send("BodyLeftOverload3");	
	}
	if(1==body_left_overload_4)     
	{	
		Wifi_Send("BodyLeftOverload4");	
	}
	if(1==body_left_overload_5)     
	{	
		Wifi_Send("BodyLeftOverload5");	
	}	
	//左背部护理
	if(1==back_nursing_left_overload)      
	{	
		Wifi_Send("BackNursingLeftOverload");	
	}
	//左腰部护理
	if(1==waist_nursing_left_overload)     
	{	
		Wifi_Send("WaistNursingLeftOverload");		
	}	
	//右翻身
	if(1==body_right_overload_3)     
	{	
		Wifi_Send("BodyRightOverload3");		
	}
	if(1==body_right_overload_4)     
	{	
		Wifi_Send("BodyRightOverload4");		
	}
	if(1==body_right_overload_5)     
	{	
		Wifi_Send("BodyRightOverload5");		
	}	
	//右背部护理
	if(1==back_nursing_right_overload)      
	{	
		Wifi_Send("BackNursingRightOverload");		
	}
	//右腰部护理
	if(1==waist_nursing_right_overload)      
	{	
		Wifi_Send("WaistNursingRightOverload");		
	}		
	 //自动坐便
	if(1==washlet_auto_overload)   
	{	
		Wifi_Send("WashletAutoOverload");	
	}
	//小桌子
	if(1==desk_overload)            
	{	
		Wifi_Send("DeskOverload");	
	}
    
/*************判断电机失步故障**********************************/	
		
	//左翻身
	if(1==body_left_losepulse)     
	{	
		Wifi_Send("BodyLeftLosepulse");	
	}
	
	//左背部护理
	if(1==back_nursing_left_losepulse)      
	{	
		Wifi_Send("BackNursingLeftLosepulse");	
	}
	//左腰部护理
	if(1==waist_nursing_left_losepulse)     
	{	
		Wifi_Send("WaistNursingLeftLosepulse");		
	}	
	//右翻身
	if(1==body_right_losepulse)     
	{	
		Wifi_Send("BodyRightLosepulse");		
	}
	
	//右背部护理
	if(1==back_nursing_right_losepulse)      
	{	
		Wifi_Send("BackNursingRightLosepulse");		
	}
	//右腰部护理
	if(1==waist_nursing_right_losepulse)      
	{	
		Wifi_Send("WaistNursingRightLosepulse");		
	}		
	 //自动坐便
	if(1==washlet_auto_losepulse)   
	{	
		Wifi_Send("WashletAutoLosepulse");	
	}
	//小桌子
	if(1==desk_losepulse)            
	{	
		Wifi_Send("DeskLosepulse");	
	}
	memset(UART4_RX_BUF,0,len);   //清除接收
	UART4_RX_LEN=0;               //清除标志位 
	//可通过指令手动复位动作（不掉电的情况下，故障排除后可返回到原函数）
	while(1)
	{
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BeepOff"))
			{
				PCF8574_WriteBit(BEEP_IO,1); //蜂鸣器停止报警
			}
			
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BackFaultReset"))
			{
				Res_Back();     		    //支背复位
			}

			if(strstr((const char *)UART4_RX_BUF,(const char *)"LegFaultReset")) 
			{			
				Res_Leg();   		        //曲腿复位
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"WashLetFaultReset"))
			{			
				Washlet(1);	               //坐便器复位
			}	
			
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftFaultReset")) 
			{			
				Res_Body_Left();         //左翻身复位					
			}

			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightFaultReset"))
			{			
				Res_Body_Right();        //右翻身复位	
			}
									
			if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskFaultReset")) 
			{			
				Res_Desk();              //就餐娱乐一体桌复位
			}
			
			if(strstr((const char *)UART4_RX_BUF,(const char *)"All_Fault_Reset")) 
			{			
				Res_Power_Down();        //所有功能一起复位
			}			
		
			if(strstr((const char *)UART4_RX_BUF,(const char *)"FaultSolved"))   //故障已排除，返回原函数
			{			
				Wifi_Send("FaultSolved");
/*********************将标志位重新置为0***********************/
				
				//电机过载标志位			
				body_left_overload_3=0;          //左翻身
				body_left_overload_4=0;          
				body_left_overload_5=0;          
				body_right_overload_3=0;         //右翻身
				body_right_overload_4=0;         
				body_right_overload_5=0;        
				washlet_auto_overload=0;         //自动坐便
				desk_overload=0;                 //就餐娱乐一体桌
				back_nursing_left_overload=0;    //左背部护理
				back_nursing_right_overload=0;   //右背部护理
				waist_nursing_left_overload=0;   //左腰部护理
				waist_nursing_right_overload=0;  //右腰部护理
				
				//电机失步标志位	
				body_left_losepulse=0;            //左翻身
				body_right_losepulse=0;           //右翻身 
				washlet_auto_losepulse=0;         //自动坐便
				desk_losepulse=0;                 //就餐娱乐一体桌
				back_nursing_left_losepulse=0;    //左背部护理
				back_nursing_right_losepulse=0;   //右背部护理
				waist_nursing_left_losepulse=0;   //左腰部护理
				waist_nursing_right_losepulse=0;  //右腰部护理
				
				break;
			}
			memset(UART4_RX_BUF,0,len);   //清除接收
			UART4_RX_LEN=0;               //清除标志位 
		}	
	}
	return 1;
}

/***********************************************************************
 函数名      ：Uart_Breakdown_Treatment(void)  
 函数功能    ：电机故障处理函数，可用于电机故障后，发指令复位各动作
 输入        ：无
 输出        ：1：故障排除后，返回1
                           
************************************************************************/
u8 Uart_Breakdown_Treatment(void)
{
	u8 len;
	Motor_All_Stop();	            //所有电机停止
	u2_printf("Fault_Warning");     //向上位机发出故障报警信号	
	PCF8574_WriteBit(BEEP_IO,0);	//控制蜂鸣器发出报警声
								
	
/*************判断电机过载故障**********************************/	
		
	//左翻身
	if(1==body_left_overload_3)     
	{	
		u2_printf("BodyLeftOverload3");	
	}
	if(1==body_left_overload_4)     
	{	
		u2_printf("BodyLeftOverload4");		
	}
	if(1==body_left_overload_5)     
	{	
		u2_printf("BodyLeftOverload5");		
	}	
	//左背部护理
	if(1==back_nursing_left_overload)      
	{	
		u2_printf("BackNursingLeftOverload");	
	}
	//左腰部护理
	if(1==waist_nursing_left_overload)     
	{	
		u2_printf("WaistNursingLeftOverload");		
	}	
	//右翻身
	if(1==body_right_overload_3)     
	{	
		u2_printf("BodyRightOverload3");		
	}
	if(1==body_right_overload_4)     
	{	
		u2_printf("BodyRightOverload4");		
	}
	if(1==body_right_overload_5)     
	{	
		u2_printf("BodyRightOverload5");		
	}	
	//右背部护理
	if(1==back_nursing_left_overload)      
	{	
		u2_printf("BackNursingRightOverload");		
	}
	//右腰部护理
	if(1==waist_nursing_right_overload)      
	{	
		u2_printf("WaistNursingRightOverload");		
	}		
	 //自动坐便
	if(1==washlet_auto_overload)   
	{	
		u2_printf("WashletAutoOverload");	
	}
	//小桌子
	if(1==desk_overload)            
	{	
		u2_printf("DeskOverload");	
	}
    
/*************判断电机失步故障**********************************/	
		
	//左翻身
	if(1==body_left_losepulse)     
	{	
		u2_printf("BodyLeftLosepulse");	
	}
	
	//左背部护理
	if(1==back_nursing_left_losepulse)      
	{	
		u2_printf("BackNursingLeftLosepulse");	
	}
	//左腰部护理
	if(1==waist_nursing_left_losepulse)     
	{	
		u2_printf("WaistNursingLeftLosepulse");		
	}	
	//右翻身
	if(1==body_right_losepulse)     
	{	
		u2_printf("BodyRightLosepulse");		
	}
	
	//右背部护理
	if(1==back_nursing_right_losepulse)      
	{	
		u2_printf("BackNursingRightLosepulse");		
	}
	//右腰部护理
	if(1==waist_nursing_right_losepulse)      
	{	
		u2_printf("WaistNursingRightLosepulse");		
	}		
	 //自动坐便
	if(1==washlet_auto_losepulse)   
	{	
		u2_printf("WashletAutoLosepulse");	
	}
	//小桌子
	if(1==desk_losepulse)            
	{	
		u2_printf("DeskLosepulse");	
	}
	memset(USART2_RX_BUF,0,len);   //清除接收
	USART2_RX_LEN=0;               //清除标志位 
	//可通过指令手动复位动作（不掉电的情况下，故障排除后可返回到原函数）
	while(1)
	{
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BeepOff"))
			{
				PCF8574_WriteBit(BEEP_IO,1); //蜂鸣器停止报警
			}
			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BackFaultReset"))
			{
				Res_Back();     		    //支背复位
			}

			if(strstr((const char *)USART2_RX_BUF,(const char *)"LegFaultReset")) 
			{			
				Res_Leg();   		        //曲腿复位
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetFaultReset"))
			{			
				Washlet(1);	               //坐便器复位
			}	
			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftFaultReset")) 
			{			
				Res_Body_Left();         //左翻身复位					
			}

			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightFaultReset"))
			{			
				Res_Body_Right();        //右翻身复位	
			}						
			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskFaultReset")) 
			{			
				Res_Desk();              //就餐娱乐一体桌复位
			}
			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"All_Fault_Reset")) 
			{			
				Res_Power_Down();        //所有功能一起复位
			}			
		
			if(strstr((const char *)USART2_RX_BUF,(const char *)"FaultSolved"))   //故障已排除，返回原函数
			{			
				u2_printf("FaultSolved");				
			
/*********************将标志位重新置为0***********************/
				
				//电机过载标志位			
				body_left_overload_3=0;          //左翻身
				body_left_overload_4=0;          
				body_left_overload_5=0;          
				body_right_overload_3=0;         //右翻身
				body_right_overload_4=0;         
				body_right_overload_5=0; 
				washlet_auto_overload=0;         //自动坐便
				desk_overload=0;                 //就餐娱乐一体桌
				back_nursing_left_overload=0;    //左背部护理
				back_nursing_right_overload=0;   //右背部护理
				waist_nursing_left_overload=0;   //左腰部护理
				waist_nursing_right_overload=0;  //右腰部护理
				
				//电机失步标志位	
				body_left_losepulse=0;           //左翻身 
				body_right_losepulse=0;          //右翻身
				washlet_auto_losepulse=0;        //自动坐便
				desk_losepulse=0;                //就餐娱乐一体桌
				back_nursing_left_losepulse=0;   //左背部护理
				back_nursing_right_losepulse=0;  //右背部护理
				waist_nursing_left_losepulse=0;  //左腰部护理
				waist_nursing_right_losepulse=0; //右腰部护理
				break;
			}
			memset(USART2_RX_BUF,0,len);        //清除接收
			USART2_RX_LEN=0;                    //清除标志位 
		}	
	}
	return 1;    
}


/***********************************************************************
 函数名      ：GDCheck(u8 num)
 函数功能    ：光电检测函数
 输入        ：光电序号
 输出        ：1：触发了，0：误触发                      
************************************************************************/
u8 GDCheck(u8 num)
{
	u32 i=0,j=0,n;
	n=285000;
	switch(num)
	{
		case GD3S:
			for(i=0;i<n;i++)
			{
				if(GD3_Start==0)
				{
					j++;
				}
			}
			break;
		case GD3LE:
			for(i=0;i<n;i++)
			{
				if(GD3_Left_End==0)
				{
					j++;
				}
			}
			break;
		case GD3RE:
			for(i=0;i<n;i++)
			{
				if(GD3_Right_End==0)
				{
					j++;
				}
			}
			break;	
		case GD4S:
			for(i=0;i<n;i++)
			{
				if(GD4_Start==0)
				{
					j++;
				}
			}			
			break;
		case GD4LE:
			for(i=0;i<n;i++)
			{
				if(GD4_Left_End==0)
				{
					j++;
				}
			}				
			break;
		case GD4RE:
			for(i=0;i<n;i++)
			{
				if(GD4_Right_End==0)
				{
					j++;
				}
			}	
			break;		
		case GD5S:
			for(i=0;i<n;i++)
			{
				if(GD5_Start==0)
				{
					j++;
				}
			}	
			break;
		case GD5LE:
			for(i=0;i<n;i++)
			{
				if(GD5_Left_End==0)
				{
					j++;
				}
			}				
			break;
		case GD5RE:
			for(i=0;i<n;i++)
			{
				if(GD5_Right_End==0)
				{
					j++;
				}
			}	
			break;	
		case GD6S:
			for(i=0;i<n;i++)
			{
				if(GD6_Start==0)
				{
					j++;
				}
			}
			break;
		case GD6E:
			for(i=0;i<n;i++)
			{
				if(GD6_End==0)
				{
					j++;
				}
			}
			break;
		case GD7S:
			for(i=0;i<n;i++)
			{
				if(GD7_Start==0)
				{
					j++;
				}
			}
			break;
		case GD7E:
			for(i=0;i<n;i++)
			{
				if(GD7_End==0)
				{
					j++;
				}
			}		
			break;
		case GD34S:
			for(i=0;i<n;i++)
			{
				if((GD3_Start==0)||((GD4_Start==0)))
				{
					j++;
				}
			}		
			break;	
		case GD34LE:
			for(i=0;i<n;i++)
			{
				if((GD3_Left_End==0)||((GD4_Left_End==0)))
				{
					j++;
				}
			}		
			break;
		case GD34RE:
			for(i=0;i<n;i++)
			{
				if((GD3_Right_End==0)||((GD4_Right_End==0)))
				{
					j++;
				}
			}		
			break;

		
		default:
			break;
	}
		if(j<284000)
		{
			j=0;
			return 0;
		}
		else
		{
			j=0;
			return 1;
		}
}

void GDCheckAll()
{
	if(GDCheck(GD3S))
	{u2_printf("GD3S\r\n");}
	if(GDCheck(GD3LE))
	{u2_printf("GD3LE\r\n");}
	if(GDCheck(GD3RE))
	{u2_printf("GD3RE\r\n");}
	if(GDCheck(GD4S))
	{u2_printf("GD4S\r\n");}
	if(GDCheck(GD4LE))
	{u2_printf("GD4LE\r\n");}
	if(GDCheck(GD4RE))
	{u2_printf("GD4RE\r\n");}
	if(GDCheck(GD5S))
	{u2_printf("GD5S\r\n");}	
	if(GDCheck(GD5RE))
	{u2_printf("GD5RE\r\n");}	
	if(GDCheck(GD5RE))
	{u2_printf("GD5RE\r\n");}
	if(GDCheck(GD6S))
	{u2_printf("GD6S\r\n");}
	if(GDCheck(GD6E))
	{u2_printf("GD6E\r\n");}	
	if(GDCheck(GD7S))
	{u2_printf("GD7S\r\n");}	
	if(GDCheck(GD7E))
	{u2_printf("GD7E\r\n");}
}


//光电跳出检测函数
u8 GDCheckDealy(u8 GDID,u16 timedelay)
{
	switch(GDID)
	{
		case GD3S:
			if(1==GD3_Start)
			{
				delay_us(100);
				{
					if(1==GD3_Start)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD3LE:
			if(1==GD3_Left_End)
			{
				delay_us(100);
				{
					if(1==GD3_Left_End)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD3RE:
			if(1==GD3_Right_End)
			{
				delay_us(100);
				{
					if(1==GD3_Right_End)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;	
		case GD4S:
			if(1==GD4_Start)
			{
				delay_us(100);
				{
					if(1==GD4_Start)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD4LE:
			if(1==GD4_Left_End)
			{
				delay_us(100);
				{
					if(1==GD4_Left_End)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD4RE:
			if(1==GD4_Right_End)
			{
				delay_us(100);
				{
					if(1==GD4_Right_End)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;		
		case GD5S:
			if(1==GD5_Start)
			{
				delay_us(100);
				{
					if(1==GD5_Start)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD5LE:
			if(1==GD5_Left_End)
			{
				delay_us(100);
				{
					if(1==GD5_Left_End)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD5RE:
			if(1==GD5_Right_End)
			{
				delay_us(100);
				{
					if(1==GD5_Right_End)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;	
		case GD6S:
			if(1==GD6_Start)
			{
				delay_us(100);
				{
					if(1==GD6_Start)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD6E:
			if(1==GD6_End)
			{
				delay_us(100);
				{
					if(1==GD6_End)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD7S:
			if(1==GD7_Start)
			{
				delay_us(100);
				{
					if(1==GD7_Start)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD7E:
			if(1==GD7_End)
			{
				delay_us(100);
				{
					if(1==GD7_End)
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD34S:	
			if((1==GD3_Start)||(1==GD4_Start))
			{
				delay_us(100);
				{
					if((1==GD3_Start)||(1==GD4_Start))
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
		
			break;	
		case GD34LE:
			if((1==GD3_Left_End)||(1==GD4_Left_End))
			{
				delay_us(100);
				{
					if((1==GD3_Left_End)||(1==GD4_Left_End))
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;
		case GD34RE:
			if((1==GD3_Right_End)||(1==GD4_Right_End))
			{
				delay_us(100);
				{
					if((1==GD3_Right_End)||(1==GD4_Right_End))
					{
						delay_ms(timedelay);	//延时timedelay ms
					}
				}
			}
			break;

		default:
			break;
	}

}

/***********************************************************************
 函数名      ：UsartCheck(u8 num)
 函数功能    ：检查接收到的串口是不是str;
 输入        ：
 输出        ：                    
************************************************************************/
u8 UsartCheck(u8* str)
{
		u8 len;
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;
			if(strstr((const char *)USART2_RX_BUF,(const char *)str))    //若接收到Stop,则跳出循环	
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				return 1;
			} 
			else 
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数								
				return 0;
			}						
		}	
		else
		{
			return 0;
		}
}


/***********************************************************************
 函数名      ：UsartCheck2
 函数功能    ：检查接收到的串口是不是str;
 输入        ：
 输出        ：                    
************************************************************************/
u8 UsartCheck2(u8* str1,u8* str2)
{
		u8 len;
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;
			if((strstr((const char *)USART2_RX_BUF,(const char *)str1))||(strstr((const char *)USART2_RX_BUF,(const char *)str2)))   //若接收到Stop,则跳出循环	
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				return 1;
			} 
			else 
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				u2_printf("NotRun");    //收到指令，但正在执行当前函数，无法去执行去新收到的指令函数								
				return 0;
			}						
		}	
		else
		{
			return 0;
		}
}

u8 GDCheckAlm(u8 GDID,u8 num)
{
	switch(GDID)
	{
		case GD3S:
			if(1==GD3_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD3LE:
			if(1==GD3_Left_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD3RE:
			if(1==GD3_Right_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;	
		case GD4S:
			if(1==GD4_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD4LE:
			if(1==GD4_Left_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD4RE:
			if(1==GD4_Right_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;		
		case GD5S:
			if(1==GD5_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD5LE:
			if(1==GD5_Left_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD5RE:
			if(1==GD5_Right_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;	
		case GD6S:
			if(1==GD6_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD6E:
			if(1==GD6_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD7S:
			if(1==GD7_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD7E:
			if(1==GD7_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD34S:	
			if((1==GD3_Start)||(1==GD4_Start))
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
		
			break;	
		case GD34LE:
			if((1==GD3_Left_End)||(1==GD4_Left_End))
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD34RE:
			if((1==GD3_Right_End)||(1==GD4_Right_End))
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;

		default:
			break;
	}
}

u8 GDSCheckAllAlm(u8 num)
{
if(GDCheckAlm(GD3S,num))
{u2_printf("GD3S\r\n");}
if(GDCheckAlm(GD4S,num))
{u2_printf("GD4S\r\n");}
if(GDCheckAlm(GD5S,num))
{u2_printf("GD5S\r\n");}
if(GDCheckAlm(GD6S,num))
{u2_printf("GD6S\r\n");}
if(GDCheckAlm(GD7S,num))
{u2_printf("GD7S\r\n");}
}

u8 GDCheckPrint(u8 GDID,u16 usdelay,u8* str)
{
	switch(GDID)
	{
		case GD3S:
			if(0==GD3_Start)
			{
				delay_us(usdelay);
				if(0==GD3_Start)
				{
					
				}
			}
			break;
		case GD3LE:
			if(1==GD3_Left_End)
			{
				delay_us(usdelay);
				if(0==GD3_Left_End)
				{
					
				}
			}
			break;
		case GD3RE:
			if(1==GD3_Right_End)
			{
				delay_us(usdelay);
				if(0==GD3_Right_End)
				{
					
				}
			}
			break;	
		case GD4S:
			if(1==GD4_Start)
			{
				delay_us(usdelay);
				if(0==GD4_Start)
				{
					
				}
			}
			break;
		case GD4LE:
			if(1==GD4_Left_End)
			{
				delay_us(usdelay);
				if(0==GD4_Left_End)
				{
					
				}
			}
			break;
		case GD4RE:
			if(1==GD4_Right_End)
			{
				delay_us(usdelay);
				if(0==GD4_Right_End)
				{
					
				}
			}
			break;		
		case GD5S:
			if(1==GD5_Start)
			{
				delay_us(usdelay);
				if(0==GD5_Start)
				{
					
				}
			}
			break;
		case GD5LE:
			if(1==GD5_Left_End)
			{
				delay_us(usdelay);
				if(0==GD5_Left_End)
				{
					
				}
			}
			break;
		case GD5RE:
			if(1==GD5_Right_End)
			{
				delay_us(usdelay);
				if(0==GD5_Right_End)
				{
					
				}
			}
			break;	
		case GD6S:
			if(1==GD6_Start)
			{
				delay_us(usdelay);
				if(0==GD6_Start)
				{
					
				}
			}
			break;
		case GD6E:
			if(1==GD6_End)
			{
				delay_us(usdelay);
				if(0==GD6_End)
				{
					
				}
			}
			break;
		case GD7S:
			if(1==GD7_Start)
			{
				delay_us(usdelay);
				if(0==GD7_Start)
				{
					
				}
			}
			break;
		case GD7E:
			if(1==GD7_End)
			{
				delay_us(usdelay);
				if(0==GD7_End)
				{
					
				}
			}
			break;
		case GD34S:	
			if((1==GD3_Start)||(1==GD4_Start))
			{
				delay_us(usdelay);
				if(0==GD3_Start)
				{
					
				}
				if(0==GD4_Start)
				{
					
				}
			}
		
			break;	
		case GD34LE:
			if((1==GD3_Left_End)||(1==GD4_Left_End))
			{
				delay_us(usdelay);
				if(0==GD3_Left_End)
				{
					
				}
				if(0==GD4_Left_End)
				{
					
				}
			}
			break;
		case GD34RE:
			if((1==GD3_Right_End)||(1==GD4_Right_End))
			{
				delay_us(usdelay);
				if(0==GD3_Right_End)
				{
					
				}
				if(0==GD4_Right_End)
				{
					
				}
			}
			break;

		default:
			break;
	}
}






