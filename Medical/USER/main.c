#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pwm.h"
#include "nand.h"
#include "key.h"
#include "function.h"
#include "common.h"
#include "pcf8574.h"
#include "ds18b20.h"
#include "check_cmd.h"
#include "motor.h"
#include "pump.h"
#include "hx711.h"
#include "modbus_master.h"
#include "bsp_user_lib.h"
#include "spi.h"
#include "washlet.h"
#include "w25qxx.h"
#include "check.h"
#include "Hang.h"
#include "washlet.h"
#include "reset.h"
#include "mainjudge.h"

//上位机设定角度极限值-默认值
u8  back_angle_lim=90;                      //上位机设定支背运行角度
u8  leg_up_angle_lim=30;                    //上位机设定上曲腿运行角度
u8  leg_down_angle_lim=80;                  //上位机设定下曲腿运行角度
u8  body_left_angle_lim=50;                 //上位机设定左翻运行角度
u8  body_right_angle_lim=55;                //上位机设定右翻运行角度
u8  desk_distance_lim=100;                   //上位机设定娱乐小桌子运动距离
u8  swash_dry_time=1;                       //上位机设定娱乐坐便冲洗烘干时间/分钟


//WiFi配置-热点-默认名称和密码
u8 wifi_ssid[20]="Medical_Bed";	    //热点名称
u8 wifi_ssid_password[20]="12345678";	    //热点密码    

//WiFi配置-路由-默认值
u8 wifi_station[20]="tmqs";					//路由器名称
u8 wifi_station_password[20]="daizhiwen";	//路由器密码
u8 wifi_ap_ip[15]="192.168.1.115";          //IP地址
u16 wifi_ap_port=8086;                      //端口号

//WiFi配置数组
u8 wifi_ip_address[20]={0};                 //ip地址
u8 wifi_name[20]={0};                       //WiFi名称
u8 wifi_password[20]={0};                   //密码

const	u8 modeap_buf[2]={"AP"};            //ap模式写入数组，此模式写入AP
const	u8 modesta_buf[2]={"ST"};           //STA模式写入数组,此模式写入ST

int main(void)
{
	u8 num;                                //记录连接的设备数
	u8 k;
	u8 len;                                //串口接收的字符串长度
	u8 key;                                //按键扫描返回值
	u8 mode_buf[2];                        //存储WiFi模式，AP还是STA模式
	u8 angle_buf[14];                      //存储上位设定角度值

	AllInit();
//	Read_Angle();                          //读取上位机设定角度值
	
//判断上电时的WiFi模式-默认为AP模式
//	W25QXX_Read(mode_buf,10,2);            //从第10个地址开始读出2个字节，判断是AP还是STA模式
//	if(strstr((const char *)mode_buf,(const char *)"ST"))	  //若有路由器，则配置为路由器模式	
//	{
//		ESP8266_apsta_Init(4);
//	}
//	else      //默认为AP模式
//	{
//		ESP8266_AP_Init(4);		
//	}
//PCF8574_WriteBit(EXIO1,0);
	ESP8266_AP_Init(4);	

//清空串口2、串口WiFi接收寄存器
	UART4_RX_LEN=0;      
	USART2_RX_LEN=0;			

	PCF8574_WriteBit(BEEP_IO,0);	//控制蜂鸣器响
	delay_ms(300);
	PCF8574_WriteBit(BEEP_IO,1);	//控制蜂鸣器停
	LED0=0;
	LED1=0;	
//	IO_TEST();
//			washlet_flag=1;
//			RELAY6=1; 
//			Uart_Push_Rod_Swash(0,25000);  //冲洗烘干推杆缩回
//			RELAY6=0;	 
//			washlet_flag=0;

//		GDSCheckAllAlm(2);
    while(1)
    {
			WifiReceiveJudge();		//遥控器接收判断函数
			Usart2ReceiveJudge();	//串口2接收函数
		}	
}
