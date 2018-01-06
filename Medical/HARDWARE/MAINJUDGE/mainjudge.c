#include "mainjudge.h"
#include "sys.h"
#include "usart.h"
#include "pcf8574.h"
#include "function.h"
#include "check.h"
#include "Hang.h"
#include "washlet.h"
#include "reset.h"
#include "delay.h"
#include "common.h"
#include "pump.h"
#include "w25qxx.h"
#include "led.h"
#include "spi.h"
#include "nand.h"
#include "key.h"
#include "backleg.h"

void AllInit(void)
{
	HAL_Init();                            //初始化HAL库   
	Stm32_Clock_Init(360,25,2,8);          //设置时钟,180Mhz
	delay_init(180);                       //初始化延时函数
	uart_init(115200);                     //初始化USART
	usart2_init(115200);                   //初始化串口2
	usart3_init(115200);                   //初始化串口3
	uart4_init(115200);	                   //初始化串口4
	LED_Init();                            //初始化LED 	
	SPI5_Init();					       //初始化SPI口
	Motor_Dir_Init();                      //初始化电机方向口
	Push_Rod_Init();                       //初始化曲腿电机方向口脉冲口
	Hang_Init();                           //初始化吊挂电机方向口脉冲口
	Push_Rod_Swash_Dry_Init();             //冲洗烘干电动杆
	Sensor_Init();                         //初始化光电开关
	KEY_Init();                            //初始化按键
	Pump_Init();                           //初始化水箱   
	PCF8574_Init();                        //初始化PCF8574/串口扩展芯片
	NAND_Init();                           //初始化NAND FLASH			
	W25QXX_Init();					       //初始化w25q256/SPI FLASH
}


void WifiReceiveJudge(void)
{
	u8 len;
	u8 *p;
	
	if(UART4_RX_LEN&0x8000)
	{
		len=UART4_RX_LEN&0x3fff;   
		UART4_RX_BUF[len]=0;       
		//计算当前连接上WiFi的设备数，并判断连接上的设备类型
		if(strstr((const char *)UART4_RX_BUF,(const char *)"CONNECT")) 
		{	
			device_num++;                        //连接的设备数累加
			PCF8574_WriteBit(BEEP_IO,0);	     //控制蜂鸣器
			delay_ms(200);
			PCF8574_WriteBit(BEEP_IO,1);	     //控制蜂鸣器		
		}
		//设备断开，设备数自动减1，并判断断开的设备类型
		if(strstr((const char *)UART4_RX_BUF,(const char *)"CLOSED")) 
		{
			device_num--;                        //连接的设备数累加
			PCF8574_WriteBit(BEEP_IO,0);	     //控制蜂鸣器
			delay_ms(200);
			PCF8574_WriteBit(BEEP_IO,1);	     //控制蜂鸣器
//			ESP8266_Close_Controler_Type();      //获取当前断开的设备类型
		}
		
		  
/*********************************************************************************			
		
			             接收上位机参数设定界面设定的参数值
			
**********************************************************************************/		
				
		//上位机重设角度
		if(strstr((const char *)UART4_RX_BUF,(const char *)"angleresetall")) 
		  {			 
				get_newangle_wifi();
		  }
		  
		//支背角度		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"backrunangle"))  
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"backrunangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=90) )  //将上位机发送的字符串中的数字转换为十进制
			{
				back_angle_lim=usmart_strnum(p+1);                  //接收上位机设定的角度值		
//				W25QXX_Write((u8*)back_angle_lim,13,1);             //保存最新上位机设定角度值
			}
			 else
			{
				u2_printf("\r\n支背参数设置有误\r\n");
			}
		}
		
		//左翻身角度		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"bodyturnleftangle")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"bodyturnleftangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=60))   //将上位机发送的字符串中的数字转换为十进制
			{				
				body_left_angle_lim=usmart_strnum(p+1);             //接收上位机设定的角度值
//				W25QXX_Write((u8*)body_left_angle_lim,14,1);        //保存最新上位机设定脉冲值
			}
			 else
			{
				u2_printf("\r\n左翻身参数设置有误\r\n");
			}
		}
		
		//右翻身角度		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"bodyturnrightangle")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"bodyturnrightangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=60))    //将上位机发送的字符串中的数字转换为十进制
			{
				body_right_angle_lim=usmart_strnum(p+1);             //接收上位机设定的角度值	
//				W25QXX_Write((u8*)body_right_angle_lim_buf,15,1);    //保存最新上位机设定脉冲值
			}
			 else
			{
				u2_printf("\r\n右翻身参数设置有误\r\n");
			}
		}
		
		//上曲腿角度			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"legrunupangle")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"legrunupangle");
			if((usmart_strnum(p+1)!=0) &&(usmart_strnum(p+1)<=40))   //将上位机发送的字符串中的数字转换为十进制
			{
				leg_up_angle_lim=usmart_strnum(p+1);                 //接收上位机设定的角度值	
//				W25QXX_Write((u8*)leg_up_angle_lim_buf,16,1);        //保存最新上位机设定脉冲值
			}
		    else
			{
				u2_printf("\r\n上曲腿参数设置有误\r\n");
			}
		}
		//下曲腿角度		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"legrundownangle")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"legrundownangle");
			if((usmart_strnum(p+1)!=0) &&(usmart_strnum(p+1)<=90))   //将上位机发送的字符串中的数字转换为十进制
			{				
				leg_down_angle_lim=usmart_strnum(p+1);               //接收上位机设定的角度值	
//				W25QXX_Write((u8*)leg_down_angle_lim_buf,17,1);	     //保存最新上位机设定脉冲值
			}
			 else
			{
				u2_printf("\r\n下曲腿参数设置有误\r\n");
			}
		}
		
		//小桌子距离		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"deskdistance")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"deskdistance");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=100))    //将上位机发送的字符串中的数字转换为十进制
			{
				desk_distance_lim=usmart_strnum(p+1);                 //接收上位机设定的角度值										
//				W25QXX_Write((u8*)desk_distance_lim_buf,18,1);		  //保存最新上位机设定脉冲值
			}
		    else
			{
				u2_printf("\r\n小桌子参数设置有误\r\n");
			}
		}	

		//冲洗烘干时间		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"swashdry")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"swashdry");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=5))   //将上位机发送的字符串中的数字转换为十进制
			{
				swash_dry_time=usmart_strnum(p+1);                 //接收上位机设定的冲洗烘干时间										
//				W25QXX_Write((u8*)swash_dry_time_buf,19,1);	       //保存最新上位机设定脉冲值	
			}
		    else
			{
				u2_printf("\r\n冲洗烘干时间设置有误\r\n");
			}
		}
		
/**********************************************************************************
		
		            WiFi接收上位机指令并执行功能函数
		
**********************************************************************************/		
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"wifireset"))  //wifi重设 
		{
			delay_ms(100);
			get_wifiname_wifipassword(wifi_name,wifi_password);	          //获取新WiFi的名称和密码			
//              esp_8266_apsta_Init(4);		                              //WiFi初始化		
		}			
						
/***********************************************************************************	
                          
			               功能函数
			
***********************************************************************************/		
	
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackUpPhone"))             //支背上
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Back();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackDownPhone"))           //支背下
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Back();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpUpPhone"))            //上曲腿上
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Leg_Up();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpDownPhone"))          //上曲腿下
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Leg_Up();					
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownDownPhone"))        //下曲腿下
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Leg_Down();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownUpPhone"))          //下曲腿上
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Leg_Down();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftUpPhone"))         //左翻身上
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Body_Left();						
		}			

		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftDownPhone"))       //左翻身下
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Body_Left();					
		}	

		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightUpPhone"))        //右翻身上
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Body_Right();						
		}	
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightDownPhone"))      //右翻身下
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Body_Right();						
		}						
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskUpPhone"))             //餐饮娱乐一体桌向前
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Desk();
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskDownPhone"))           //餐饮娱乐一体桌后退
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Desk();
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackNursingLeftPhone"))    //左背部护理
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Back_Nursing_Left();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackNursingRightPhone"))   //右背部护理
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Back_Nursing_Right();					
		}				
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WaistNursingLeftPhone"))   //左腰部护理
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Waist_Nursing_Left();						
		}				

		if(strstr((const char *)UART4_RX_BUF,(const char *)"WaistNursingRightPhone"))  //右腰部护理
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Fun_Waist_Nursing_Right();
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WashletAutoPhone"))        //自动坐便
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Washlet_Auto();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftMuscleMassager"))   //左臂肌肉按摩
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Muscle_Massager();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmRightMuscleMassager"))   //右臂肌肉按摩
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Muscle_Massager();						
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftMuscleMassager"))   //左腿肌肉按摩
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Muscle_Massager();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegRightMuscleMassager"))  //右腿肌肉按摩
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Muscle_Massager();						
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLegMuscleMassager"))    //上下肢肌肉按摩
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Muscle_Massager();						
		}		
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackMuscleMassager"))      //背部肌肉按摩
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Muscle_Massager();						
		}		
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"Heat"))                    //加热
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Heat();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ResetPhone"))              //掉电复位
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Res_Power_Down();						
		}
	
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LDUART4"))                 //功能函数联动         
		{
			LDUART4();	  //联动调试					
		}		
		
/***********************************************************************************	
                          
			                    护栏功能
			
***********************************************************************************/
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackGB"))           //支背
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			GB_Back();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftGB"))      //左翻身
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			GB_Body_Left();						
		}				
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightGB"))     //右翻身
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			GB_Body_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackNursingGB"))   //背部护理
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			GB_Back_Nursing();					
		}							
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WaistNursingGB"))  //腰部护理
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			GB_Waist_Nursing();						
		}				
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WashletAutoGB"))   //自动坐便
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Washlet_Auto();					
		}						
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LockGB"))           //键锁
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			GB_Lock();
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ResetGB"))          //掉电复位
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Res_Power_Down();					
		}						
			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"MuscleMassagerPhone"))     //肌肉按摩
		{
			Muscle_Massager();						
		}

		
/***********************************************************************************	
                          
			               吊挂训练自动模式
			
***********************************************************************************/
					
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftAuto"))          //左胳膊        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Auto_Arm_Leg_Left(1); 					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmRightAuto"))         //右胳膊        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Auto_Arm_Leg_Right(1);					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftRightAuto"))    //左右胳膊        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Auto_Arm_Leg_Left_Right(1);					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftAuto"))          //左腿        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Auto_Arm_Leg_Left(1); 					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegRightAuto"))         //右腿       
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Auto_Arm_Leg_Right(1);						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftRightAuto"))    //左右腿        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Auto_Arm_Leg_Left_Right(1);						
		}
			
/***********************************************************************************	
                          
			               吊挂训练手动模式
			
***********************************************************************************/			
		
		//左胳膊						
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftUpHand"))            //左胳膊小臂上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftDownHand"))          //左胳膊小臂下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Left();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftUpHand"))            //左胳膊大臂上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Post_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftDownHand"))          //左胳膊大臂下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Post_Left();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftUpHand"))       //左胳膊大小臂上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Post_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftDownHand"))     //左胳膊大小臂下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Post_Left();						
		}			

		//右胳膊			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightUpHand"))           //右胳膊小臂上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightDownHand"))          //右胳膊小臂下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightUpHand"))            //右胳膊大臂上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Post_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightDownHand"))          //右胳膊大臂下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Post_Right();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightUpHand"))       //右胳膊大小臂上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Post_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightDownHand"))     //右胳膊大小臂下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Post_Right();						
		}
			
		//左右胳膊			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))       //左右胳膊小臂上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))     //左右胳膊小臂下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Left_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))       //左右胳膊大臂上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Post_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))     //左右胳膊大臂下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Post_Left_Right();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))  //左右胳膊大小臂上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Post_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightDownHand")) //左右胳膊大小臂下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Arm_Fore_Post_Left_Right();						
		}			
			
		//左腿			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftUpHand"))              //左腿小腿上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftDownHand"))            //左腿小腿下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Left();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftUpHand"))              //左腿大腿上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Post_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftDownHand"))            //左腿大腿下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Post_Left();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftUpHand"))         //左腿大小腿上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Post_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftDownHand"))       //左腿大小腿下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Post_Left();						
		}			

		//右腿
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightUpHand"))             //右腿小腿上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightDownHand"))           //右腿小腿下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightUpHand"))             //右腿大腿上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Post_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightDownHand"))           //右腿大腿下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Post_Right();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightUpHand"))        //右腿大小腿上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Post_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightDownHand"))      //右腿大小腿下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Post_Right();						
		}
			
		//左右腿
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightUpHand"))        //左右腿小腿上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightDownHand"))      //左右腿小腿下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Left_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightUpHand"))        //左右腿大腿上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Post_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightDownHand"))      //左右腿大腿下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Post_Left_Right();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))   //左右腿大小腿上        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Post_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightDownHand")) //左右腿大小腿下        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Hand_Leg_Fore_Post_Left_Right();						
		}

/***********************************************************************************	
                          
			               WiFi接收专家系统推理指令
			
***********************************************************************************/
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ExpertBack"))           //专家系统支背     
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Exp_Back();						
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ExpertBody"))          //专家系统翻身      
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Exp_Body();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ExpertLeg"))           //专家系统曲腿        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Exp_Leg();						
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ExpertWashletAuto"))  //专家系统自动坐便        
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Exp_Washlet_Auto();						
		}
		
/***********************************************************************************	
					  
					         WiFi接收故障处理界面指令
		
***********************************************************************************/			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackFaultReset"))	            //支背复位
		{
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Res_Back();     		    
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegFaultReset"))                //曲腿复位
		{			
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Res_Leg();   		       
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WashLetFaultReset"))            //坐便器复位
		{			
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			washlet_picture_k=24;
			Washlet(1);	              
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskFaultReset"))               //就餐娱乐一体桌复位
		{			
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Res_Desk();              
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftFaultReset"))          //左翻身复位
		{			
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Res_Body_Left();       					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightFaultReset"))         //右翻身复位	
		{			
			if(ESP8266_Get_ID())	//接收到当前设备发送的指令后，立马给其返回一个Received指令
			Res_Body_Right();        
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"FaultSolved"))                   //故障已排除，故障处理界面最后一条必按指令
		{
			Wifi_Send("Received");
			Wifi_Send("FaultSolved");
			
/*********************将标志位重新置为0***********************/
			
			//电机过载标志位			
			body_left_overload_3=0;          //左翻身
			body_left_overload_4=0;          //左翻身
			body_left_overload_5=0;          //左翻身
			body_right_overload_3=0;         //右翻身
			body_right_overload_4=0;         //右翻身
			body_right_overload_5=0;         //右翻身
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

		}					
		
/*****************************************************************************/
		
		memset(UART4_RX_BUF,0,len);			
		UART4_RX_LEN=0;				
	}		
}

void Usart2ReceiveJudge(void)
{
	if(USART2_RX_LEN&0x8000)
	{
		u8 *p;
		u8 len;
		u32 HangId;
		len=USART2_RX_LEN&0x3fff;				
		USART2_RX_BUF[len]=0;

/*****************************************************************************			
			             WiFi设置函数		
******************************************************************************/		
		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LEDChange"))        //WiFi设置为AP模式
		{			 
			LED0=!LED0;
			LED1=!LED1;
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"mode"))          //上位机重设WiFi模式
		{			 
			if(strstr((const char *)USART2_RX_BUF,(const char *)"AP"))        //WiFi设置为AP模式
			{			 
				W25QXX_Write((u8*)modeap_buf,10,2);
				u2_printf("\r\n设置成AP模式\r\n");
			}
			
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"STA"))       //WiFi设置为STA模式
			{			 
				W25QXX_Write((u8*)modesta_buf,10,2);	
				u2_printf("\r\n设置成STA模式\r\n");					
			}					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"staset"))        //wifi重设-STA模式
		{ 
			get_wifiname_wifipassword(wifi_station,wifi_station_password);    //获取WiFi名称和密码
		}		

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"apset"))         //wifi重设-AP模式
		{ 
			get_apname_wifipassword(wifi_ssid,wifi_ssid_password);            //获取WiFi的名称和密码
		}	

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ipportset"))     //wifi重设
		{ 
			get_ip_port(wifi_ap_ip,wifi_ap_port);                             //获取WiFi的ip地址和端口号
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifireset"))     //wifi重设
		{ 
			delay_ms(100);
			get_wifiname_wifipassword(wifi_name,wifi_password);	              //获取WiFi的名称和密码
		}	

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"stastart"))      //wifi模式初始化
		{ 
			ESP8266_apsta_Init(4);	
		}			

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"apstart"))       //ap模式初始化
		{ 
			ESP8266_AP_Init(4);	
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"newangle"))      //获取上位机参数设定值
		{ 
			Read_Angle();
		}	
/*******************************************************************************

					     判断连接设备名称
		
********************************************************************************/
		


		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifiphone"))      
		{			 
			ESP8266_send_data(Phone,"来自手机");
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifipad"))       
		{			 
			ESP8266_send_data(Pad,"来自平板");
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifiremote"))      
		{			 
			ESP8266_send_data(Remote_Ctl,"来自遥控器");
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifiguard"))       
		{			 
			ESP8266_send_data(Guard_Ctl,"来自护栏");
		}		

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifipc"))          
		{			 
			ESP8266_send_data(PC,"来自电脑");
		}	

/*******************************************************************************

						接收上位机参数设定值
		
********************************************************************************/
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"angleresetall"))  //上位机重设角度
		{			 
			get_newangle_usart2();  //获得新的角度值
		}		
		
		//支背		  		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"backrunangle"))   //支背
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"backrunangle");
			if((usmart_strnum(p+1)!=0) &&(usmart_strnum(p+1)<=90))             //将上位机发送的字符串中的数字转换为十进制
			{
				back_angle_lim=usmart_strnum(p+1);                             //接收上位机设定的角度值																			
//				W25QXX_Write((u8*)back_angle_lim,13,1);                        //保存最新上位机设定角度值
			}
		}

		//左翻身	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"bodyturnleftangle"))//左翻身
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"bodyturnleftangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=60))                //将上位机发送的字符串中的数字转换为十进制
			{				
				body_left_angle_lim=usmart_strnum(p+1);                          //接收上位机设定的角度值										 								
//				W25QXX_Write((u8*)body_left_angle_lim,14,1);                     //保存最新上位机设定脉冲值
			}
		}

		//右翻身	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"bodyturnrightangle"))//右翻身
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"bodyturnrightangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=60))                 //将上位机发送的字符串中的数字转换为十进制
			{
				body_right_angle_lim=usmart_strnum(p+1);                          //接收上位机设定的角度值															
//				W25QXX_Write((u8*)body_right_angle_lim_buf,15,1);                 //保存最新上位机设定脉冲值
			}
		}

		//上曲腿		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"legrunupangle"))     //上曲腿
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"legrunupangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=40))                 //将上位机发送的字符串中的数字转换为十进制
			{
				leg_up_angle_lim=usmart_strnum(p+1);                              //接收上位机设定的角度值													
//				W25QXX_Write((u8*)leg_up_angle_lim_buf,16,1);                     //保存最新上位机设定脉冲值				
			}
		}

		//下曲腿	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"legrundownangle"))   //下曲腿
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"legrundownangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=90))                 //将上位机发送的字符串中的数字转换为十进制
			{
				leg_down_angle_lim=usmart_strnum(p+1);                            //接收上位机设定的角度值													
//				W25QXX_Write((u8*)leg_down_angle_lim_buf,17,1);	                  //保存最新上位机设定脉冲值
			}
		}

		//餐饮娱乐一体桌	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"deskdistance"))      //餐饮娱乐一体桌
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"deskdistance");
			if((usmart_strnum(p+1)!=0) &&(usmart_strnum(p+1)<=100))               //将上位机发送的字符串中的数字转换为十进制
			{
				desk_distance_lim=usmart_strnum(p+1);                             //接收上位机设定的角度值											
//				W25QXX_Write((u8*)desk_distance_lim_buf,18,1);		              //保存最新上位机设定脉冲值
			}
		}
								
		//冲洗烘干时间		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"swashdry")) 
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"swashdry");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=5))    //将上位机发送的字符串中的数字转换为十进制
			{
				swash_dry_time=usmart_strnum(p+1);                  //接收上位机设定的冲洗烘干时间														
//				W25QXX_Write((u8*)swash_dry_time_buf,19,1);	        //保存最新上位机设定脉冲值	
			}
		    else
			{
				u2_printf("\r\n冲洗烘干时间设置有误\r\n");
			}
		}

/***********************************************************************************	                       
			                 串口接收移动设备指令	
***********************************************************************************/		
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackUpPhone"))            //支背上
		{
			Uart_Back();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackDownPhone"))           //支背下
		{
			Uart_Back();
		}

		
	/***新的支背调用函数***/
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackUpNew"))    //支背上,例如BackUpNew+85
		{
			u32 BackUpParam;
			BackUpParam=usmart_strnum2(USART2_RX_BUF);
			BackRun(1,BackUpParam%100);		
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackDownNew"))           //支背下,例如BackUpNew+85
		{
			u32 BackDownParam;
			BackDownParam=usmart_strnum2(USART2_RX_BUF);
			BackRun(0,BackDownParam%100);	
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpUpPhone"))            //上曲腿上
		{
			Uart_Leg_Up();
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpDownPhone"))          //上曲腿下
		{
			Uart_Leg_Up();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpUpNew"))           //支背下,例如BackUpNew+85
		{
			u32 LegUpParam;
			LegUpParam=usmart_strnum2(USART2_RX_BUF);
			LegUpRun(1,LegUpParam%100);	
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpDownNew"))           //支背下,例如BackUpNew+85
		{
			u32 LegUpParam;
			LegUpParam=usmart_strnum2(USART2_RX_BUF);
			LegUpRun(0,LegUpParam%100);	
		}
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownDownPhone"))        //下曲腿下
		{
			Uart_Leg_Down();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownUpPhone"))          //下曲腿上
		{
			Uart_Leg_Down();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownDownNew"))           //支背下,例如BackUpNew+85
		{
			u32 LegDownParam;
			LegDownParam=usmart_strnum2(USART2_RX_BUF);
			LegDownRun(1,LegDownParam%100);	
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownUpNew"))           //支背下,例如BackUpNew+85
		{
			u32 LegDownParam;
			LegDownParam=usmart_strnum2(USART2_RX_BUF);
			LegDownRun(0,LegDownParam%100);	
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpPhone"))         //左翻身上
		{
			Uart_Body_Left();						
		}			

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftDownPhone"))       //左翻身下
		{
			Uart_Body_Left();					
		}	

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpPhone"))        //右翻身上
		{
			Uart_Body_Right();						
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightDownPhone"))     //右翻身下
		{
			Uart_Body_Right();						
		}						
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpNew")) 	//BodyLeftUpNew+45     
		{
			u32 BdyLftUpPara;
			BdyLftUpPara=usmart_strnum2(USART2_RX_BUF);		
			BodyLeftRun(1,motor_body_freq,motor_body_freq,BdyLftUpPara%100);
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftDownNew")) //BodyLeftDownNew+45        
		{
			u32 BdyLftDwnPara;
			BdyLftDwnPara=usmart_strnum2(USART2_RX_BUF);		
			BodyLeftRun(0,motor_body_freq,motor_body_freq,BdyLftDwnPara%100);
		}		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpNew")) 	//BodyRightUpNew+45     
		{
			u32 BdyRhtUpPara;
			BdyRhtUpPara=usmart_strnum2(USART2_RX_BUF);		
			BodyRightRun(1,motor_body_freq,motor_body_freq,BdyRhtUpPara%100);
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightDownNew")) //BodyightDownNew+45        
		{
			u32 BdyRhtDwnPara;
			BdyRhtDwnPara=usmart_strnum2(USART2_RX_BUF);		
			BodyRightRun(0,motor_body_freq,motor_body_freq,BdyRhtDwnPara%100);
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))            //餐饮娱乐一体桌向前
		{
			Uart_Desk1();
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone"))          //餐饮娱乐一体桌后退
		{
			Uart_Desk1();
		}
		

//		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))            //餐饮娱乐一体桌向前
//		{
//			Uart_Desk();
//		}
//		
//		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone"))          //餐饮娱乐一体桌后退
//		{
//			Uart_Desk();
//		}


		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackNursingLeftPhone"))    //左背部护理
		{
			Uart_Back_Nursing_Left();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackNursingRightPhone"))   //右背部护理
		{
			Uart_Back_Nursing_Right();					
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WaistNursingLeftPhone"))   //左腰部护理
		{
			Uart_Waist_Nursing_Left();						
		}				

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WaistNursingRightPhone"))  //右腰部护理
		{
			Uart_Waist_Nursing_Right();
		}
		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MuscleMassagerPhone"))     //肌肉按摩
		{
			Muscle_Massager();						
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))   
		{
			Motor_6_1_STOP();			
		}		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"JD0"))     //肌肉按摩
		{
			DIR_QB=0;           //气泵停止
			//DIR_JRAM=0;         //按摩喷气阀门关闭
			delay_ms(1000);     //等待阀门关闭
			//Wifi_Send("MuscleMassagerStop");
			u2_printf("MuscleMassagerStop");						
		}
		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashletAutoPhone"))        //自动坐便
		{
			//Uart_Washlet_Auto();
			WashLet_V2(1,32950);
		}
		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DIR_HG1"))        //自动坐便
		{
			DIR_HG=1;						
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DIR_HG0"))        //自动坐便
		{
			DIR_HG=0;						
		}
	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"SB1"))                    //水泵打开
		{
			DIR_SB=1;
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"SB0"))                    //水泵关闭
		{
			DIR_SB=0;
		}
				
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Heat"))                    //加热
		{
			Heat();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ResetPhone"))             //掉电复位
		{
			Uart_Res_Power_Down();						
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LDUART2"))                //功能函数联动         
		{
			LDUART2();	  //联动调试					
		}
		
//************************调试函数*********************************
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart22222_Dry_Auto"))   //左腰部护理
		{
			washlet_flag=1;
			Uart_Dry_Auto();		//自动烘干				
		}
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart22222_Swash_Auto"))   //左腰部护理
		{
			washlet_flag=1;
			Uart_Swash_Auto();	//自动冲洗					
		}
	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Push_Rod_SwashUp"))        //冲洗烘干
		{
			RELAY6=1; 
			Uart_Push_Rod_Swash(1,30000);   //冲洗烘干推杆伸出
			RELAY6=0; 
		}

		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Push_Rod_SwashDown"))          //冲洗烘干       
		{
			RELAY6=1; 
			Uart_Push_Rod_Swash(0,30000);  //冲洗烘干推杆缩回
			RELAY6=0;	      					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"weight"))          //冲洗烘干       
		{	 
			Uart_Washlet_Weight(); 	
			PCF8574_WriteBit(BEEP_IO,0);	//控制蜂鸣器响
			delay_ms(300);
			PCF8574_WriteBit(BEEP_IO,1);	//控制蜂鸣器停			
		}		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart2_Push_Rod_Swash1"))        //冲洗烘干
		{
			washlet_flag=1;
			RELAY6=1; 
			Uart_Push_Rod_Swash(1,30000);   //冲洗烘干推杆伸出
			RELAY6=0; 
		}

		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart2_Push_Rod_Swash0"))          //冲洗烘干       
		{
//			washlet_flag=1;
			RELAY6=1; 
			Uart_Push_Rod_Swash(0,30000);  //冲洗烘干推杆缩回
			RELAY6=0;	      					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_Washlet_Tig0"))      //坐便袋收紧          
		{
//			washlet_flag=1;
			Uart_Washlet_Tig(0);	  	//用这个放线	
		}		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_Washlet_Tig1"))      //坐便袋收紧        
		{
//			washlet_flag=1;
			Uart_Washlet_Tig(1);	  		 //坐便袋收线电机收线
		}
		
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_WashletTigOnly"))      //坐便袋收紧        
		{
			//washlet_flag=1;
			Uart_WashletTigOnly(1);
		}


		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_Motor_6_2_START1"))      //坐便袋推杆       
		{
			washlet_flag=1;
			RELAY6=1;                       //继电器得电
			Uart_Motor_6_2_START(0,16500);   //收线推杆缩回	
			RELAY6=0;             
		}		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_Motor_6_2_START0"))      //坐便袋推杆          
		{
			washlet_flag=1;
			RELAY6=1;             //继电器得电
			Uart_Motor_6_2_START(1,16500);            //收线推杆伸出	
			RELAY6=0;           
		}		

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"SxTgRepeat"))      //坐便袋推杆          
		{
			u8 RepeatNum;
			u8 repeat;
			u32 SxtgParam;
			SxtgParam==usmart_strnum2(USART2_RX_BUF);
			for(RepeatNum=0;RepeatNum<6;RepeatNum++)
			{
				MotorStart(9,1,SxtgParam%100000);
				delay_ms(800);
				MotorStart(9,0,SxtgParam%100000);
				delay_ms(800);
			}  
			BeepRun(1,200);
		}		
		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetMotor"))      //坐便袋推杆          
		{
			u32 WashLetMotorParam;
			WashLetMotorParam=usmart_strnum2(USART2_RX_BUF);
			u2_printf("dir=%d,Arr=%d",WashLetMotorParam/100000%10,WashLetMotorParam%100000);
			if(strstr((const char *)USART2_RX_BUF,(const char *)"CXHG"))
			{
				u2_printf("CXHG\r\n");
				MotorStart(8,WashLetMotorParam/100000%10,WashLetMotorParam%100000);
			}
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"SXTG"))
			{
				u2_printf("SXTG\r\n");
				MotorStart(9,WashLetMotorParam/100000%10,WashLetMotorParam%100000);
			}
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"SXDJ"))
			{
				u2_printf("SXDJ\r\n");
				MotorStart(10,WashLetMotorParam/100000%10,WashLetMotorParam%100000);		
			}				
		}


		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart222_Washlet1"))      //坐便打开       
		{
			u2_printf("坐便器打开");
			Uart_Washlet(0);	  				
		}		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart222_Washlet0"))      //坐便关闭          
		{
			u2_printf("坐便器关闭");
			washlet_picture_k=24;
			Uart_Washlet(1);	  				
		}		
	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"testhang1"))      //电机         
		{
			TestAll(1);	  				
		}	

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"testhang0"))      //电机         
		{
			TestAll(0);	  				
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"testhangAll_1"))      //电机         
		{
			test(1);	  				
		}


		else if(strstr((const char *)USART2_RX_BUF,(const char *)"pcf1"))      //电机         
		{
			PCF8574_WriteBit(EXIO1,1);	       //继电器
			PCF8574_WriteBit(EXIO2,1);	       //继电器
			PCF8574_WriteBit(EXIO3,1);	       //继电器
			PCF8574_WriteBit(EXIO4,1);	       //继电器
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"pcf0"))      //电机         
		{
			PCF8574_WriteBit(EXIO1,0);	       //继电器
			PCF8574_WriteBit(EXIO2,0);	       //继电器
			PCF8574_WriteBit(EXIO3,0);	       //继电器
			PCF8574_WriteBit(EXIO4,0);	       //继电器
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang1_1"))      //电机         
		{
			DG_Relay=1;		//继电器得电
			Hang1Test(1);	  	
			DG_Relay=0;		//继电器失电
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang1_0"))      //电机         
		{
			DG_Relay=1;		//继电器得电
			Hang1Test(0);	
			DG_Relay=0;		//继电器失电
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang2_1"))      //电机         
		{
			DG_Relay=1;		//继电器得电
			Hang2Test(1);	  
			DG_Relay=0;		//继电器失电
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang2_0"))      //电机         
		{
			DG_Relay=1;		//继电器得电
			Hang2Test(0);	
			DG_Relay=0;		//继电器失电
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang3_1"))      //电机         
		{
			DG_Relay=1;		//继电器得电
			Hang3Test(1);	 
			DG_Relay=0;		//继电器失电
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang3_0"))      //电机         
		{
			DG_Relay=1;		//继电器得电
			Hang3Test(0);	
			DG_Relay=0;		//继电器失电
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang4_1"))      //电机         
		{
			DG_Relay=1;		//继电器得电
			Hang4Test(1);	 
			DG_Relay=0;		//继电器失电
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang4_0"))      //电机         
		{
			DG_Relay=1;		//继电器得电
			Hang4Test(0);
			DG_Relay=0;		//继电器失电
		}
		
		//坐便器打开或者闭合，例：WashLetNew+1+33000
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetNew"))      //坐便器打开，默认33000这个参数     
		{
			u32 WashLetPara;
			WashLetPara=usmart_strnum2(USART2_RX_BUF);
			WashLet_V1(WashLetPara/100000%10,WashLetPara%100000);		//1：打开，0，下降
		}
	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetRunNew"))  //WashLetRunNew+1+33000
		{
			u32 WashLetRunPara;
			WashLetRunPara=usmart_strnum2(USART2_RX_BUF);
			WashletRun(WashLetRunPara/100000%10,WashLetRunPara%100000,timer10_freq);
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskRunUpNew"))      //DeskRunUpNew+1+099         
		{
			u32 DeskRunPara;
			DeskRunPara=usmart_strnum2(USART2_RX_BUF);
			DeskRun1(DeskRunPara/1000,DeskRunPara%1000);
			
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskRunDownNew"))      //DeskRunDownNew+1+099         
		{
			u32 DeskRunPara;
			DeskRunPara=usmart_strnum2(USART2_RX_BUF);
			DeskRun1(DeskRunPara/1000,DeskRunPara%1000);
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLet_V2")) 
		{WashLet_V2(1,32950);}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"RunAllAuto"))      //电机        
		{
			LDUART2V2();
			u2_printf("over");
		}

		
		//吊挂，例如：HangRunUp+6+1111
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"HangRun"))
		{
			int temp;
			DG_Relay=1;		//继电器得电
//			HangId=usmart_strnum(USART2_RX_BUF);	
				HangId=usmart_strnum2(USART2_RX_BUF);			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"Up"))
			{
					HangRun(1,HangId/10000,HangId/1000%10,HangId/100%10,HangId/10%10,HangId%10);
			}
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"Down"))
			{
				HangRun(0,HangId/10000,HangId/1000%10,HangId/100%10,HangId/10%10,HangId%10);
			}
			DG_Relay=0;		//继电器失电			
		}	
		
		//举例：HangLR+0+1+0+2+15(上下肢+方向+左右+倍数+高度),(0+1+0+2+15)=(上肢+上+左吊挂+2倍+高度)
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"HangLR"))   
		{
			u32 HangLRPara;
			DG_Relay=0;		//继电器失电
			HangLRPara=usmart_strnum2(USART2_RX_BUF);
			if(HangLRPara)
			{
				HangRunAuto(HangLRPara/100000,HangLRPara/10000%10,HangLRPara/1000%10,HangLRPara/100%10,HangLRPara%100);
			}
			DG_Relay=0;		//继电器失电
		}

		
 //电机1  
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR1111"))      //电机1         
		{
			MOTOR111(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR1110"))      //电机1         
		{
			MOTOR111(0);	  				
		} 
 
 //电机2  
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR2221"))      //电机2         
		{
			MOTOR222(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR2220"))      //电机2         
		{
			MOTOR222(0);	  				
		} 
 
 //电机3  
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR3331"))      //电机3         
		{
			MOTOR333(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR3330"))      //电机3         
		{
			MOTOR333(0);	  				
		}
 //电机4 
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR4441"))      //电机4         
		{
			MOTOR444(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR4440"))      //电机4         
		{
			MOTOR444(0);	  				
		}
//电机5 		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR5551"))      //电机5         
		{
			MOTOR555(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR5550"))      //电机5         
		{
			MOTOR555(0);	  				
		}
//电机6 		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR6661"))      //电机6         
		{
			MOTOR666(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR6660"))      //电机6         
		{
			MOTOR666(0);	  				
		}
//电机7		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR7771"))      //电机7         
		{
			MOTOR777(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR7770"))      //电机7         
		{
			MOTOR777(0);	  				
		}

	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Nippon"))      //电机7         
		{
			if(UsartCheck2("Japan","Nippon"))
			{u2_printf("true\r\n");}
		}
		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ResetAllAuto"))      //电机7         
		{
			ResetAll();
		}		

/***********************************************************************************	
                          
			               串口接收护栏指令
			
***********************************************************************************/
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackGB"))           //支背
		{
			Uart_GB_Back();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftGB"))      //左翻身
		{
			Uart_GB_Body_Left();						
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightGB"))     //右翻身
		{
			Uart_GB_Body_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackNursingGB"))   //背部护理
		{
			Uart_GB_Back_Nursing();					
		}							
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WaistNursingGB"))  //腰部护理
		{
			Uart_GB_Waist_Nursing();						
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashletAutoGB"))   //自动坐便
		{
			Uart_Washlet_Auto();					
		}						
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LockGB"))          //键锁
		{
			Uart_GB_Lock();
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ResetGB"))        //掉电复位
		{
			Uart_Res_Power_Down();					
		}	
	
			
/***********************************************************************************	
                          
			               康复训练自动模式
			
***********************************************************************************/
							
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftAuto"))        //左胳膊        
		{
			Uart_Auto_Arm_Leg_Left(1); 					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmRightAuto"))       //右胳膊        
		{
			Uart_Auto_Arm_Leg_Right(1);					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftRightAuto"))  //左右胳膊        
		{
			Uart_Auto_Arm_Leg_Left_Right(1);					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftAuto"))        //左腿        
		{
			Uart_Auto_Arm_Leg_Left(1); 					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegRightAuto"))       //右腿       
		{
			Uart_Auto_Arm_Leg_Right(1);						
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftRightAuto"))  //左右腿        
		{
			Uart_Auto_Arm_Leg_Left_Right(1);						
		}
		
/*****************************************************************************************
									
							    吊挂训练手动模式
		
/****************************************************************************************/
			
		//左胳膊						
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftUpHand"))      //左胳膊小臂上        
		{
			Uart_Hand_Arm_Fore_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftDownHand"))    //左胳膊小臂下        
		{
			Uart_Hand_Arm_Fore_Left();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftUpHand"))      //左胳膊大臂上        
		{
			Uart_Hand_Arm_Post_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftDownHand"))    //左胳膊大臂下        
		{
			Uart_Hand_Arm_Post_Left();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftUpHand"))  //左胳膊大小臂上        
		{
			Uart_Hand_Arm_Fore_Post_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftDownHand"))//左胳膊大小臂下        
		{
			Uart_Hand_Arm_Fore_Post_Left();						
		}			

		//右胳膊			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightUpHand"))     //右胳膊小臂上        
		{
			Uart_Hand_Arm_Fore_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightDownHand"))   //右胳膊小臂下        
		{
			Uart_Hand_Arm_Fore_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightUpHand"))     //右胳膊大臂上        
		{
			Uart_Hand_Arm_Post_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightDownHand"))   //右胳膊大臂下        
		{
			Uart_Hand_Arm_Post_Right();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightUpHand")) //右胳膊大小臂上        
		{
			Uart_Hand_Arm_Fore_Post_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightDownHand"))//右胳膊大小臂下        
		{
			Uart_Hand_Arm_Fore_Post_Right();						
		}
		
		//左右胳膊			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))  //左右胳膊小臂上        
		{
			Uart_Hand_Arm_Fore_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))//左右胳膊小臂下        
		{
			Uart_Hand_Arm_Fore_Left_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))  //左右胳膊大臂上        
		{
			Uart_Hand_Arm_Post_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))//左右胳膊大臂下        
		{
			Uart_Hand_Arm_Post_Left_Right();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))//左右胳膊大小臂上        
		{
			Uart_Hand_Arm_Fore_Post_Left_Right();					
		}

		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))//左右胳膊大小臂下        
		{
			Uart_Hand_Arm_Fore_Post_Left_Right();						
		}			
		
		//左腿			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftUpHand"))           //左腿小腿上        
		{
			Uart_Hand_Leg_Fore_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftDownHand"))         //左腿小腿下        
		{
			Uart_Hand_Leg_Fore_Left();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftUpHand"))           //左腿大腿上        
		{
			Uart_Hand_Leg_Post_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftDownHand"))         //左腿大腿下        
		{
			Uart_Hand_Leg_Post_Left();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftUpHand"))       //左腿大小腿上        
		{
			Uart_Hand_Leg_Fore_Post_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftDownHand"))     //左腿大小腿下        
		{
			Uart_Hand_Leg_Fore_Post_Left();						
		}			

		//右腿
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightUpHand"))          //右腿小腿上        
		{
			Uart_Hand_Leg_Fore_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightDownHand"))        //右腿小腿下        
		{
			Uart_Hand_Leg_Fore_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightUpHand"))          //右腿大腿上        
		{
			Uart_Hand_Leg_Post_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightDownHand"))        //右腿大腿下        
		{
			Uart_Hand_Leg_Post_Right();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightUpHand"))      //右腿大小腿上        
		{
			Uart_Hand_Leg_Fore_Post_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightDownHand"))   //右腿大小腿下        
		{ 
			Uart_Hand_Leg_Fore_Post_Right();						
		}
		
		//左右腿
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightUpHand"))    //左右腿小腿上        
		{
			Uart_Hand_Leg_Fore_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightDownHand"))  //左右腿小腿下        
		{
			Uart_Hand_Leg_Fore_Left_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightUpHand"))    //左右腿大腿上        
		{
			Uart_Hand_Leg_Post_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightDownHand"))  //左右腿大腿下        
		{
			Uart_Hand_Leg_Post_Left_Right();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightUpHand")) //左右腿大小腿上        
		{
			Uart_Hand_Leg_Fore_Post_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightDownHand"))//左右腿大小腿下        
		{
			Uart_Hand_Leg_Fore_Post_Left_Right();						
		}
		

/***********************************************************************************	
                          
			               串口接收专家系统推理指令
			
***********************************************************************************/
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ExpertBack"))         //专家系统支背     
		{
			Uart_Exp_Back();						
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ExpertBody"))         //专家系统翻身      
		{
			Uart_Exp_Body();						
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ExpertLeg"))          //专家系统曲腿        
		{
			Uart_Exp_Leg();						
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ExpertWashletAuto"))  //专家系统自动坐便        
		{
			Uart_Exp_Washlet_Auto();						
		}
	
		
/***********************************************************************************	
                          
			               串口接收故障处理界面指令
			
***********************************************************************************/			
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackFaultReset"))	      //支背复位
		{
			Uart_Res_Back();     		    
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegFaultReset"))         //曲腿复位
		{			
			Uart_Res_Leg();   		       
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetFaultReset"))     //坐便器复位
		{
			washlet_picture_k=24;
			Uart_Washlet(1);	              
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskFaultReset"))        //就餐娱乐一体桌复位
		{			
			Uart_Res_Desk();              
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftFaultReset"))    //左翻身复位
		{			
			Uart_Res_Body_Left();       					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightFaultReset"))   //右翻身复位	
		{			
			Uart_Res_Body_Right();        
		}					

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"FaultSolved"))           //故障已排除，故障处理界面最后一条必按指令
		{
			u2_printf("Received");
			u2_printf("FaultSolved");
			
/*********************将标志位重新置为0***********************/
			
			//电机过载标志位			
			body_left_overload_3=0;          //左翻身
			body_left_overload_4=0;          //左翻身
			body_left_overload_5=0;          //左翻身
			body_right_overload_3=0;         //右翻身
			body_right_overload_4=0;         //右翻身
			body_right_overload_5=0;         //右翻身
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
		}					
			
/********************************************************************************************************/	
				
		memset(USART2_RX_BUF,0,len);   //清除接收
		USART2_RX_LEN=0;               //清除标志位           	
	}	
}




