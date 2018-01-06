#include "common.h"
#include "usart.h"
#include "timer.h"
#include "w25qxx.h"

//默认设备号
u8  Phone=6;       //手机app
u8  Remote_Ctl=6;  //遥控器
u8  Guard_Ctl=6;   //护栏
u8  PC=6;          //PC机
u8  Pad=6;         //Pad

//设备连接状态 0：未连接；1：连接上
u8 Phone_Ready=0;          //手机app
u8 PC_Ready=0;             //PC
u8 RemoteCtl_Ready=0;      //遥控器
u8 GuardCtl_Ready=0;       //护栏
u8 ComputerStick_Ready=0;  //电脑

u8 ID_NUM;                 //智能终端对应的设备号

/***********************************************************************
 函数名      ：atk_8266_at_response(u8 mode)  
 函数功能    ：用串口2打印WiFi接收到的数据。
 输入        ：0：UART4_RX_LEN不清零，1：清零
 输出        ：无  
 
************************************************************************/
void atk_8266_at_response(u8 mode)
{
	if(UART4_RX_LEN&0X8000)		
	{ 
		UART4_RX_BUF[UART4_RX_LEN&0X7FFF]=0;
		u2_printf("%s",UART4_RX_BUF);	
		if(mode)UART4_RX_LEN=0;
	} 
}


/***********************************************************************
 函数名      ：atk_8266_check_cmd(u8 *str)  
 函数功能    ：判断UART4_RX_BUF数组中有无指定字符串
 输入        ：0：UART4_RX_LEN不清零，     1：清零
 输出        ：指定字符串第一个字符的地址

************************************************************************/
u8* atk_8266_check_cmd(u8 *str)
{	
	char *strx=0;
	if(UART4_RX_LEN&0X8000)	
	{ 
		UART4_RX_BUF[UART4_RX_LEN&0X7FFF]=0;
		strx=strstr((const char*)UART4_RX_BUF,(const char*)str); 		
	} 
	return (u8*)strx;
}

/***********************************************************************
 函数名      ：atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
 函数功能    ：发送WiFi指令
 输入        ：cmd:指令,ack:返回值,waittime:超时时间
 输出        ：1: 指令发送成功，返回值正确  0：失败  

************************************************************************/
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u16 time=0;
	u8 res=0; 
	UART4_RX_LEN=0;
	u4_printf("%s\r\n",cmd);                  //发送CMD指令
	time = 0;
	while(time<waittime)	
	{
		delay_ms(10);		
		if(atk_8266_check_cmd(ack))       //判断返回值
		{
//				printf("ack:%s\r\n",(u8*)ack); 
			break;
		}	
		time++;				
	}
	if(time<waittime)res=1;                   //如果在超时时间内break出来，则res=1
//	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	
	return res;
} 

/***********************************************************************
 函数名      ：atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
 函数功能    ：发送数据
 输入        ：
 输出        ：

************************************************************************/
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
{
	u8 res=0; 
	UART4_RX_LEN=0;
	u4_printf("%s",data);	
	if(ack&&waittime)	
	{
		while(--waittime)	
		{
			delay_ms(5);
			if(UART4_RX_LEN&0X8000)
			{
				if(atk_8266_check_cmd(ack))break;
				UART4_RX_LEN=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}

/***********************************************************************
 函数名      ：atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
 函数功能    ：发送数据给设备0
 输入        ：字符串
 输出        ：无 

************************************************************************/
void esp8266_send_data(u8 *data)
{
	u16 i;
	i=strlen((const char*)data);          //获取数据长度
	u4_printf("AT+CIPSEND=0,%d\r\n",i);   //发送AT+CIPSEND指令
	delay_ms(5);                         
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);		//清除串口4接收缓存区	
	UART4_RX_LEN=0;                               
	u4_printf("%s",data);                         //发送数据字符数据data
}

/***********************************************************************
 函数名      ：void ESP8266_send_data(u8 id,u8 *data)
 函数功能    ：发送数据给设备号为id的设备
 输入        ：id：设备号；data：要发送的数据
 输出        ：无 

************************************************************************/
void ESP8266_send_data(u8 id,u8 *data)
{
	u16 i;
	i=strlen((const char*)data);                //获取数据长度
	u4_printf("AT+CIPSEND=%d,%d\r\n",id,i);     //发送AT+CIPSEND指令
	delay_ms(5);                         
	memset(UART4_RX_BUF,0,50);	                //清除串口4接收缓存区	
	UART4_RX_LEN=0;                               
	u4_printf("%s",data);                       //发送数据字符数据data	
	delay_ms(200);
	memset(UART4_RX_BUF,0,50);	                //清除串口4接收缓存区	
	UART4_RX_LEN=0; 	
}

/***********************************************************************
 函数名      ：atk_8266_quit_trans(void)
 函数功能    ：退出透传
 输入        ：无
 输出        ：1:成功 0:失败

************************************************************************/
u8 atk_8266_quit_trans(void)
{
	while((UART4->SR&0X40)==0);   //循环发送"+",3次
	UART4->DR='+';      
	delay_ms(15);				
	while((UART4->SR&0X40)==0);	
	UART4->DR='+';      
	delay_ms(15);	
	while((UART4->SR&0X40)==0);	
	UART4->DR='+';      
	delay_ms(500);	
	return atk_8266_send_cmd("AT","OK",20);
}


/***********************************************************************
 函数名      ：atk_8266_get_wanip(u8* ipbuf)
 函数功能    ：获取ipbuf中，介于"\"和"\"中的数据
 输入        ：无
 输出        ：无

************************************************************************/
void atk_8266_get_wanip(u8* ipbuf)
{
	  u8 *p,*p1;
		if(atk_8266_send_cmd("AT+CIFSR","OK",50))
		{
			ipbuf[0]=0;
			return;
		}		
		p=atk_8266_check_cmd("\"");
		p1=(u8*)strstr((const char*)(p+1),"\"");
		*p1=0;
		sprintf((char*)ipbuf,"%s",p+1);	
}

/***********************************************************************
 函数名      ：esp8266_get_ip(u8* ipbuf)
 函数功能    ：从ipbuf中获取路由器分配给STM的IP地址，介于"\"和"\"中的数据
 输入        ：ipbuf：获取IP的数组
 输出        ：无

************************************************************************/
void esp8266_get_ip(u8* ipbuf)
{
	  u8 *p,*p1;	
		if(atk_8266_send_cmd("AT+CIFSR","OK",50))
		{
			ipbuf[0]=0;
			return;
		}		
		p=atk_8266_check_cmd("\"");
		p1=(u8*)strstr((const char*)(p+1),"\"");
		*p1=0;
		sprintf((char*)ipbuf,"%s",p+1);
		u2_printf("\r\n%s\r\n",ipbuf);		
}

/***********************************************************************
 函数名      ：atk_8266_SET_AP(u8 *ssid,u8 *password,u8 mode,u16 timeout)
 函数功能    ：esp8266设置AP模式
 输入        ：ssid,password,模式，超时时间
 输出        ：0：失败  1：成功

************************************************************************/
u8 atk_8266_SET_AP(u8 *ssid,u8 *password,u8 mode,u16 timeout)
{
	u8 res=0;
	UART4_RX_LEN=0;
	u16 time =0;
	u4_printf("AT+CWSAP=\"%s\",\"%s\",4,%d\r\n",ssid,password,mode);  //发送指令
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
		break;
		delay_ms(10);
		time++;
	}
	if(time<timeout) res =1;
	return res;
}

/***********************************************************************
 函数名      ：atk_8266_SET_STA(u8 *ssid,u8 *password,u16 timeout)
 函数功能    ：esp8266设置STA模式
 输入        ：ssid,password,模式，超时时间
 输出        ：0：失败  1：成功

************************************************************************/
u8 atk_8266_SET_STA(u8 *ssid,u8 *password,u16 timeout)
{	
	 u8 res=0;
	 UART4_RX_LEN=0;
	 u16 time =0;

	u4_printf("AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,password);  //发送指令
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
	    break;
	    delay_ms(40);
		time++;
	}
	if(time<timeout) res =1;
	return res;	 
}

/***********************************************************************
 函数名      ：atk_8266_SET_IPPORT(u16 timeout)
 函数功能    ：esp8266设置多路连接，并设置IP端口为192.168.1.115
 输入        ：超时时间
 输出        ：0：失败  1：成功

************************************************************************/
u8 atk_8266_SET_IPPORT(u16 timeout)
{
	u8 res = 0;
	UART4_RX_LEN=0;
	u16 time =0;
	if(atk_8266_send_cmd("AT+CIPMUX=1","OK",50))
	{
		u2_printf("\r\n开启多路连接成功\r\n");		
	}
	else
	{
		u2_printf("\r\n开启多路连接失败\r\n");			
	}
		
	if(atk_8266_send_cmd("AT+CIPAP=\"192.168.1.115\"","OK",100))   //配置IP地址：192.168.1.115
	{
		u2_printf("设置IP成功");	
	}
	else
	{
		u2_printf("设置IP失败");	
	}
	
	u4_printf("AT+CIPSERVER=1,8086\r\n");    //配置端口号为8086
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
		break;
		delay_ms(10);
		time++;
	}
	if(time<timeout) res =1;
	return res;
}

/***********************************************************************
 函数名      ：atk_8266_SET_IPPORT(u16 timeout)
 函数功能    ：设置端口号
 输入        ：超时时间
 输出        ：0：失败  1：成功

************************************************************************/
u8 atk_8266_SET_PORT(u16 timeout)
{
	u8 res = 0;
	UART4_RX_LEN=0;
	u16 time =0;
//  if(atk_8266_send_cmd("AT+CIPMUX=1","OK",450))
//	{
//		 u2_printf("\r\n开启多路连接成功\r\n");
//	}
//	  else
//	{
//	  u2_printf("\r\n开启多路连接失败\r\n");		
//	}
		
	if(atk_8266_send_cmd("AT+CIPSERVER=1,8080","OK",50))
	{
		delay_ms(1000); u2_printf("\r\n端口设置成功\r\n");res=1;
	}
	else
	{
		u2_printf("\r\n端口设置失败\r\n");	res=0;
	}
//	time=0;
//   while(time<timeout)
//	 {
//	    if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
//	    break;
//	    delay_ms(20);
//			time++;
//	 }
//	 if(time<timeout) res =1;
	return res;
}


/***********************************************************************
 函数名      ：ESP8266_SET_IP(u8 *Ip,u16 timeout)
 函数功能    ：设置端口号
 输入        ：超时时间
 输出        ：0：失败  1：成功

************************************************************************/
u8 ESP8266_SET_IP(u8 *Ip,u16 timeout)
{
	u8 res = 0;
	UART4_RX_LEN=0;
	u16 time =0;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	//清除串口4接收缓存区	
	
  u4_printf("AT+CIPAP=\"%s\"\r\n",Ip);  //发送指令	
	delay_ms(100);
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
	    break;
	    delay_ms(40);
		time++;
	}
	if(time<timeout) res =1;
	return res;	 
}

/***********************************************************************
 函数名      ：ESP8266_SET_PORT(u8 *Ip,u16 timeout)
 函数功能    ：设置端口号
 输入        ：超时时间
 输出        ：0：失败  1：成功

************************************************************************/
u8 ESP8266_SET_PORT(u16 Port,u16 timeout)
{
	u8 res = 0;
	UART4_RX_LEN=0;
	u16 time =0;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	//清除串口4接收缓存区	
	
  u4_printf("AT+CIPSERVER=1,%d\r\n",Port);  //发送指令	
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
	    break;
	    delay_ms(40);
		time++;
	}
	if(time<timeout) res =1;
	return res;	 
}




/***********************************************************************
 函数名      ：esp8266_Get_IP(u16 timeout)
 函数功能    ：
 输入        ：无
 输出        ：无

************************************************************************/
void esp8266_Get_IP(void)
{
	if(atk_8266_send_cmd("AT+CIFSR","OK",500))
		{u2_printf("请确定返回IP地址");}
	else
		{u2_printf("获取返回值失败");}
	atk_8266_at_response(1);
}

/***********************************************************************
 函数名：esp8266_AP_Init
 功能：  初始化esp8266的AP模式
 参数：  无
 返回值：无
 其他: 热点名称ATK_ESP8266;密码：12345678;端口号：8086

***********************************************************************/
void esp8266_AP_Init(void)
{
  u8 i;
	for(i=0;i<1;i++)
	{
//		delay_ms(50);		
		if(atk_8266_send_cmd("AT+CWMODE=2","OK",60))
		  {u2_printf("\r\nAP模式设置成功\r\n");}
		else
		  {u2_printf("\r\nAP模式设置失败\r\n");}		
		if(atk_8266_send_cmd("AT+CIPMUX=1","OK",60))
		  {u2_printf("\r\n多连接开启\r\n");}
		else
		  {u2_printf("\r\n多连接开启失败\r\n");}		
		if(atk_8266_SET_AP(SSID,PASS,WPA_WPA2_PSK,500))
		  { u2_printf("\r\n网络设置完成\r\n");}
		else
		  {u2_printf("\r\n网络设置失败r\n");}	
		if (atk_8266_SET_IPPORT(50))
		  {  u2_printf("\r\nIP和端口设置完成\r\n");  }		   		
		else
		  {u2_printf("\r\nIP和端口设置失败\r\n");}	
	}
}

/***********************************************************************
 函数名：ESP8266_AP_Init
 功能：  初始化esp8266的AP模式
 参数：  无
 返回值：无
 其他: 热点名称ATK_ESP8266;密码：12345678;端口号：8086

***********************************************************************/
void ESP8266_AP_Init(u8 n)
{
	 u8 i;
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+CWMODE=2","OK",60))
		 {
			u2_printf("\r\n模式设置成功\r\n");
			break;
		 }
		 else
		 {
			u2_printf("\r\n第%d次模式设置尝试失败，准备继续尝试设置\r\n",(i+1));
		 }
		delay_ms(100);			 
	 }

	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+CIPMUX=1","OK",60))
		 {
			u2_printf("\r\n多连接开启\r\n");
			break;
		 }
		 else
		 {	u2_printf("\r\n第%d次多连接开启失败，准备继续尝试设置\r\n",(i+1)); }
		 delay_ms(100);			 
	 }	 

	 for(i=0;i<n;i++)
	 {
			if(atk_8266_SET_AP(wifi_ssid,wifi_ssid_password,WPA_WPA2_PSK,500))
			{
				delay_ms(200);	
				u2_printf("\r\nAP设置成功\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n第%d次AP设置尝试失败，准备继续尝试设置\r\n",i+1);
			}
			delay_ms(100);	
	 }	 

	 for(i=0;i<n;i++)
	 {
			if(ESP8266_SET_IP(wifi_ap_ip,60))
			{
				u2_printf("\r\nIP设置成功\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n第%d次IP设置尝试失败，准备继续尝试设置\r\n",i+1);
			}		
			delay_ms(500);			
	 }

	 for(i=0;i<n;i++)
	 {		 
			if(ESP8266_SET_PORT(wifi_ap_port,50))
			{
				u2_printf("\r\n端口设置成功\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n第%d次端口设置尝试失败，准备继续尝试设置\r\n",i+1);
			}
			delay_ms(1000);					
	 }
}




/***********************************************************************
 函数名    ：esp8266_STA_SERVER_Init
 功能      ：初始化esp8266的STA模式
 参数      ：无
 返回值    ：无
 其他: 

***********************************************************************/
void esp8266_STA_SERVER_Init(void)
{
  u8 i;
	for(i=0;i<2;i++)
	{
		delay_ms(100);		
		if(atk_8266_send_cmd("AT+CWMODE=1","OK",500))
			{u2_printf("\r\n模式设置成功\r\n");}
		else
			{u2_printf("\r\n模式设置失败\r\n");}	
			
		if(atk_8266_send_cmd("AT+RST","OK",3000))
			{u2_printf("\r\n重置成功\r\n");}
		else
			{u2_printf("\r\n重置失败\r\n");}
			
		delay_ms(5000);
					
		if(atk_8266_SET_STA(WIFI_STATION,PASSWORD,600))
			{delay_ms(5000);u2_printf("\r\nWiFi连接成功\r\n");}
		else
			{u2_printf("\r\nWiFi连接失败\r\n");}
			
		delay_ms(3000);

		if(atk_8266_SET_PORT(600))
			{ u2_printf("\r\n 端口设置成功 \r\n"); }		  		
		else	
			{ u2_printf("\r\n 端口设置成功 \r\n"); }		
	}
}

/***********************************************************************
 函数名    ：esp_8266_apsta_Init
 功能      ：初始化esp8266的AP服务器-TCP服务器模式
 参数      ：n,失败重连次数
 返回值    ：无
 其他: 

***********************************************************************/
void esp_8266_apsta_Init(u8 n)
{
   u8 i;
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+CWMODE=3","OK",100))
		 {
			u2_printf("\r\n模式设置成功\r\n");
			break;
		 }
		 else
		 {
			u2_printf("\r\n第%d次模式设置尝试失败，准备继续尝试设置\r\n",(i+1));
		 }
	 }
	for(i=0;i<n;i++)
	{
		if(atk_8266_send_cmd("AT+RST","OK",100))
		{
			u2_printf("\r\n重启设置成功\r\n");
			delay_ms(2000);	
			break;
		}
		else
		{
			u2_printf("\r\n第%d次重启尝试失败，准备继续重启设置\r\n",i+1);
		}
//			 delay_ms(1000);		 
	}
	for(i=0;i<n;i++)
	{
		if(atk_8266_SET_AP(wifi_ssid,wifi_ssid_password,WPA_WPA2_PSK,500))
		{
			u2_printf("\r\nAP设置成功\r\n");
			break;
		}
		else
		{
			u2_printf("\r\n第%d次AP设置尝试失败，准备继续尝试设置\r\n",i+1);
		}
			delay_ms(100);
	 }

	 for(i=0;i<n;i++)
	 {
			if(ESP8266_SET_IP(wifi_ap_ip,60))
			{
				u2_printf("\r\nIP设置成功\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n第%d次IP设置尝试失败，准备继续尝试设置\r\n",i+1);
			}		
			delay_ms(500);			
	 }

	 for(i=0;i<n;i++)
	 {		 
			if(ESP8266_SET_PORT(wifi_ap_port,50))
			{
				u2_printf("\r\n端口设置成功\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n第%d次端口设置尝试失败，准备继续尝试设置\r\n",i+1);
			}
			delay_ms(1000);					
	 }

	
	for(i=0;i<n;i++)
	{
		if(atk_8266_SET_STA(wifi_station,wifi_station_password,700))
		{
			delay_ms(4000);
			u2_printf("\r\nWIFI-STATION设置成功\r\n");
			break;
		}
		else
		{
			u2_printf("\r\n第%d次WIFI-STATION设置尝试失败，准备继续尝试设置\r\n",i+1);
		}
	 }
	
	 for(i=0;i<n;i++)
	 {
//	   delay_ms(50);
		if(atk_8266_SET_PORT(200))
		{
			u2_printf("\r\n服务器端口设置成功\r\n");
			break;
		}
		else
		{
			u2_printf("\r\n第%d次服务器端口设置失败，准备继续尝试设置\r\n",i+1);
		}
	 }	 
	 for(i=0;i<n;i++)
	 {
		delay_ms(20);
		if(atk_8266_send_cmd("AT+CIPSTO=2200","OK",500))
		{
			u2_printf("\r\n超时时间设置成功\r\n");
			break;
		}
		else
		{
			u2_printf("\r\n第%d次超时时间设置尝试失败，准备继续尝试设置\r\n",i+1);
		}
	 } 
//	 esp8266_get_ip(wifi_ip_address);
	 delay_ms(500);
	 esp8266_Get_IP();
	 u2_printf("\r\nready\r\n");
}


/***********************************************************************
 函数名   ：get_wifiname_wifipassword(u8 *ip1,u8 *ip2)
 功能     ：获取字符串中的WiFi名称和密码
 输入     ：ip1：存入WiFi名称；ip2：存入WiFi密码
 输出     ： 

***********************************************************************/
void get_wifiname_wifipassword(u8 *ip1,u8 *ip2)
{
	u8 receive_data[20];
	char *presult1,*presult2,*presult3;
	USART2_RX_LEN=0;
	memset(ip1,0,20);
	memset(ip2,0,20);
	presult1 = strstr( (const char *)USART2_RX_BUF ,(const char *)"+");
	if( presult1 != NULL )
	{
	  presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	  if( presult2 != NULL )
		{
			memcpy(ip1,presult1+1,presult2-presult1-1);
			W25QXX_Write((u8 *)ip1,60,presult2-presult1-1);
		  u2_printf("\r\nWIFI热点名称:%s\r\n",ip1);
			memset(receive_data,0,20);			
			W25QXX_Read((u8*)receive_data,60,presult2-presult1-1); 
		  u2_printf("\r\n存入WIFI热点名称:%s\r\n",receive_data);
			memset(receive_data,0,20);			
			presult3 = strstr( (const char *)presult2+1 ,(const char *)"+");
			if( presult3 != NULL )
			{
				memcpy(ip2,presult2+1,presult3-presult2-1);
				W25QXX_Write((u8 *)ip2,90,presult3-presult2-1);
				u2_printf("\r\nWIFI密码:%s\r\n",ip2);
				W25QXX_Read((u8*)receive_data,90,presult3-presult2-1); 
				u2_printf("\r\n存入WIFI密码名称:%s\r\n",receive_data);					
			}				
		}		
	}
}

/***********************************************************************
 函数名   ：get_apname_wifipassword(u8 *ip1,u8 *ip2)
 功能     ：获取字符串中的WiFi名称和密码
 输入     ：ip1：存入WiFi名称；ip2：存入WiFi密码
 输出     ： 
***********************************************************************/
void get_apname_wifipassword(u8 *ip1,u8 *ip2)
{
	u8 receive_data[20];
	char *presult1,*presult2,*presult3;
	USART2_RX_LEN=0;
	memset(ip1,0,20);
	memset(ip2,0,20);
	presult1 = strstr( (const char *)USART2_RX_BUF ,(const char *)"+");
	if( presult1 != NULL )
	{
	  presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	  if( presult2 != NULL )
		{
			memcpy(ip1,presult1+1,presult2-presult1-1);
			W25QXX_Write((u8 *)ip1,120,presult2-presult1-1);
		  u2_printf("\r\n设定AP-WIFI热点名称:%s\r\n",ip1);	
			memset(receive_data,0,20);			
			W25QXX_Read((u8*)receive_data,120,presult2-presult1-1); 
		  u2_printf("\r\n存入WIFI热点名称:%s\r\n",receive_data);	
			memset(receive_data,0,20);
			presult3 = strstr( (const char *)presult2+1 ,(const char *)"+");
			if( presult3 != NULL )
			{
					memcpy(ip2,presult2+1,presult3-presult2-1);
					W25QXX_Write((u8 *)ip2,150,presult3-presult2-1);
					u2_printf("\r\n设定AP-WIFI密码:%s\r\n",ip2);
					W25QXX_Read((u8*)receive_data,150,presult3-presult2-1); 
					u2_printf("\r\n存入WIFI密码名称:%s\r\n",receive_data);					
			}				
		}		
	}
}

/***********************************************************************
 函数名   ：get_ip_port(u8 *ip1,u16 data)
 功能     ：获取字符串中的WiFi名称和密码
 输入     ：ip1：存入WiFi名称；ip2：存入WiFi密码
 输出     ： 
***********************************************************************/
void get_ip_port(u8 *ip1,u16 data)
{
	u8 receive_data[15];
	u8 tem[5];
	u8 portdata[2];
	u16 test;
	char *presult1,*presult2,*presult3;
	USART2_RX_LEN=0;
	memset(ip1,0,20);
	
	presult1 = strstr( (const char *)USART2_RX_BUF ,(const char *)"+");
	if( presult1 != NULL )
	{
	  presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	  if( presult2 != NULL )
		{
			memcpy(ip1,presult1+1,presult2-presult1-1);
			W25QXX_Write((u8 *)ip1,180,presult2-presult1-1);
		  u2_printf("\r\n设定IP地址为:%s\r\n",ip1);	
			memset(receive_data,0,15);
			
			W25QXX_Read((u8*)receive_data,180,presult2-presult1-1); 
		  u2_printf("\r\n存入IP地址为:%s\r\n",receive_data);	
			memset(receive_data,0,15);
			
			presult3 = strstr( (const char *)presult2+1 ,(const char *)"+");
			if( presult3 != NULL )
			{
					memcpy(tem,presult2+1,presult3-presult2-1);
				  data=usmart_strnum(tem);
				  wifi_ap_port=data;
					portdata[0]=data>>8;   //高8位
					portdata[1]=data;   	 //低8位				
					W25QXX_Write((u8 *)portdata,195,2);
					u2_printf("\r\n设定端口与号为:%d\r\n",data);
					memset(portdata,0,2);			
					W25QXX_Read((u8*)portdata,195,2);
					test=	portdata[0]<<8;
					test=test|portdata[1];			
					u2_printf("\r\n存入的端口为%d\r\n",test);					
			}				
		}		
	}
}


/***********************************************************************
 函数名    ：ESP8266_apsta_Init
 功能      ：初始化esp8266的AP服务器-TCP服务器模式
 参数      ：n,失败重连次数
 返回值    ：无
 其他      ：
***********************************************************************/
void ESP8266_apsta_Init(u8 n)
{
   u8 i;
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+CWMODE=3","OK",100))
		 {
		   u2_printf("\r\n模式设置成功\r\n");
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n第%d次模式设置尝试失败，准备继续尝试设置\r\n",(i+1));
		 }
	 }
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+RST","OK",100))
		 {
		   u2_printf("\r\n重启设置成功\r\n");
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n第%d次重启尝试失败，准备继续重启设置\r\n",i+1);
		 }	 
	 }
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_SET_AP(wifi_ssid,wifi_ssid_password,WPA_WPA2_PSK,500))
		 {
		   u2_printf("\r\nAP设置成功\r\n");
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n第%d次AP设置尝试失败，准备继续尝试设置\r\n",i+1);
		 }
		 delay_ms(100);
	 }
	 
	 for(i=0;i<n;i++)
	 {
			if(ESP8266_SET_IP(wifi_ap_ip,60))
			{
				u2_printf("\r\nIP设置成功\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n第%d次IP设置尝试失败，准备继续尝试设置\r\n",i+1);
			}		
			delay_ms(1000);			
	 }
	 
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_SET_STA(wifi_station,wifi_station_password,3000))
		 {
		   u2_printf("\r\nWIFI-STATION设置成功\r\n");
			 delay_ms(9000);
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n第%d次WIFI-STATION设置尝试失败，准备继续尝试设置\r\n",i+1);
		 }
		 memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		 delay_ms(8000);
	 }

	 for(i=0;i<n;i++)
	 {
			if(atk_8266_send_cmd("AT+CIPMUX=1","OK",450))
			{
				 u2_printf("\r\n开启多路连接成功\r\n");
				 break;
			}
				else
			{
				u2_printf("\r\n第%d次多路连接失败，准备继续尝试设置\r\n",i+1);		
			}
		 memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		 delay_ms(1000);
	 }	

	 for(i=0;i<n;i++)
	 {		 
			if(ESP8266_SET_PORT(wifi_ap_port,50))
			{
				u2_printf("\r\n端口设置成功\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n第%d次端口设置尝试失败，准备继续尝试设置\r\n",i+1);
			}
			delay_ms(1000);					
	 }	 

	 
	 for(i=0;i<n;i++)
	 {
	   delay_ms(20);
		 if(atk_8266_send_cmd("AT+CIPSTO=2200","OK",500))
		 {
		   u2_printf("\r\n超时时间设置成功\r\n");
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n第%d次超时时间设置尝试失败，准备继续尝试设置\r\n",i+1);
		 }
		 memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		 delay_ms(1000);
	 } 
	 esp8266_Get_IP();
	 u2_printf("\r\nready\r\n");
}

/***********************************************************************
 函数名    ：ESP8266_Get_Controler_Type()
 功能      ：有WiFi设备连入时，分配相应的设备号
 输入      ：超时时间
 输出      ：无
 其他      ：

***********************************************************************/
void ESP8266_Get_Controler_Type(u16 timeout)
{
	u8 ID,num,len;
	u8 time=0;
	num=1;
	ID=UART4_RX_BUF[0]-48;   //进制的转换
	USART2->DR =ID;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	
	u2_printf("\r\nID=%x,num=%d\r\n",ID,num);
	UART4_RX_LEN=0;
	ESP8266_send_data(ID,"who_are_you");
	delay_ms(300);
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	UART4_RX_LEN=0;	
	TIM10_Init(30000,21000);
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
	while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
	{
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;		
			if(strstr((const char *)UART4_RX_BUF,(const char *)"Phone"))  //手机
			{
				 Phone=ID;
				Phone_Ready=1;
				u2_printf("\r\nPhone=%d\r\n",Phone);				
				break;
			}		
			
			if(strstr((const char *)UART4_RX_BUF,(const char *)"Remote"))  //遥控器
			{			
				Remote_Ctl =ID;
				RemoteCtl_Ready=1;
				u2_printf("\r\nRemote=%d\r\n",Remote_Ctl);					
				break;
			}

			if(strstr((const char *)UART4_RX_BUF,(const char *)"Guard"))  //护栏
			{			
				Guard_Ctl= ID;
				GuardCtl_Ready=1;				
				u2_printf("\r\nGuard=%d\r\n",Guard_Ctl);					
				break;
			}

			if(strstr((const char *)UART4_RX_BUF,(const char *)"PC"))  //电脑
			{		
				 PC =ID;
				PC_Ready=1;				
				u2_printf("\r\nPC=%d\r\n",PC);					
				break;
			}
		 }	
	 }
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // 清除中断标志位	 	
	u2_printf("\r\nfinish\r\n");
}

/***********************************************************************
 函数名    ：ESP8266_Close_Controler_Type()
 功能      ：wifi设备退出时，释放分配的设备号
 输入      ：无
 输出      ：无
 其他      ：
***********************************************************************/
void ESP8266_Close_Controler_Type(void)
{
	u8 ID,len;
	u8 time=0;
	ID=UART4_RX_BUF[0]-48;   //进制的转换
	USART2->DR =ID;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	UART4_RX_LEN=0;	
	u2_printf("\r\nID=%x\r\n",ID);

	if(ID==Phone)
	{Phone=6;Phone_Ready=0;u2_printf("恢复Phone\r\n");}
	if(ID==Remote_Ctl)
	{Remote_Ctl=6;RemoteCtl_Ready=0;u2_printf("恢复Remote_Ctl\r\n");}
	if(ID==Guard_Ctl)
	{Guard_Ctl=6;	GuardCtl_Ready=0;	u2_printf("恢复Guard_Ctl\r\n");}
	if(ID==PC)
	{PC=6;	PC_Ready=0;	u2_printf("恢复PC\r\n");}	
	u2_printf("\r\nPhone=%d,Remote_Ctl=%d,Guard_Ctl=%d,PC=%d\r\n",Phone,Remote_Ctl,Guard_Ctl,Guard_Ctl,PC);
}

/***********************************************************************
 函数名    ：ESP8266_Get_ID()
 功能      ：接收到当前设备发送的指令后，立马给其返回一个Received指令
 输入      ：无
 输出      ：0：失败，1：成功
 其他      ：
***********************************************************************/
u8 ESP8266_Get_ID(void)
{
	u8 i,len;
	u8 res=0;                     //函数返回值
	u8 temp_buf[50];              //用来保存WiFi接收的功能指令
	ID_NUM=UART4_RX_BUF[7]-48;    //读取发送指令的设备号
	len=UART4_RX_LEN&0x3fff;      //WiFi接收的字符串长度
	for(i=0;i<len;i++)            //保存UART4_RX_BUF中的数据
	{
		temp_buf[i]=UART4_RX_BUF[i+2];             
	}
	if((ID_NUM==0)||(ID_NUM==1)||(ID_NUM==2)||(ID_NUM==3)||(ID_NUM==4))
	{
		res=1;
		ESP8266_send_data(ID_NUM,"Received");   //向发送指令的设备发送Received反馈
	}
	else
	{	res=0;   }
	
	for(i=0;i<len;i++)
	{
		UART4_RX_BUF[i]=temp_buf[i];           //将WiFi接受的功能指令重新写入UART4_RX_BUF中
	}	
	return res;
}

/***********************************************************************
 函数名    ：Flash_Read_STA()
 功能      ：在Flash内读取保存的路由器名称和密码
 输入      ：无
 输出      ：无
 其他      ：
***********************************************************************/
void Flash_Read_STA(void)
{
	u8 receive_data[20];
	memset(receive_data,0,20);
	W25QXX_Read(receive_data,60,20);				//从spi flash中第60位地址开始读取20个字节数据
	u2_printf("\r\nflash内的sta_wifi名称:%s\r\n",receive_data);
	W25QXX_Read(receive_data,90,20);				//从spi flash中第90位地址开始读取20个字节数据
	u2_printf("\r\nflash内的sta_wifi密码:%s\r\n",receive_data);
}


/***********************************************************************
 函数名    ：Flash_Read_AP()
 功能      ：在Flash内读取保存的热点名称和对应密码
 输入      ：无
 输出      ：无
 其他      ：
***********************************************************************/
void Flash_Read_AP(void)
{
	u8 receive_data[20];
	memset(receive_data,0,20);
	W25QXX_Read(receive_data,120,20);  //从spi flash中第120位地址开始读取20个字节数据
	u2_printf("\r\nflash内的sta_wifi名称:%s\r\n",receive_data);
	memset(receive_data,0,20);	
	W25QXX_Read(receive_data,150,20);		//从spi flash中第150位地址开始读取20个字节数据
	u2_printf("\r\nflash内的sta_wifi密码:%s\r\n",receive_data);
}


/***********************************************************************
 函数名    ：Flash_Read_AP_IPPORT()
 功能      ：在Flash内读取保存的IP地址和端口号
 输入      ：无
 输出      ：无
 其他      ：
***********************************************************************/
void Flash_Read_AP_IPPORT(void)
{
	u8 receive_data[15];
	u8 receive_tem[2];
	u16 receive_port;
	memset(receive_data,0,15);
	W25QXX_Read(receive_data,180,20);		   //从spi flash中第180位地址开始读取20个字节数据
	u2_printf("\r\nflash内的AP模式IP地址为:%s\r\n",receive_data);
	memset(receive_tem,0,2);
	W25QXX_Read(receive_tem,195,2);	      //从spi flash中第195位地址开始读取2个字节数据
	receive_port=	receive_tem[0]<<8;           //获取高8位
	receive_port=receive_port|receive_tem[1];			//高8位与低8位组成16位数据
	u2_printf("\r\nflash内的端口为%d\r\n",receive_port);					
}

 

/***********************************************************************
 函数名    ：stick_send_cmd()
 功能      ：通过串口，向PC棒发送消息并等待PC棒的应答
 输入      ：无
 输出      ：0：应答失败  1：应答成功
 其他      ：
***********************************************************************/
u8 stick_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u16 time=0;
	u8 res=0; 
	USART2_RX_LEN=0;
	u2_printf("%s\r\n",cmd); 
	time = 0;
	while(time<waittime)	
	{
			delay_us(10);		
			if(stick_check_cmd(ack))       //判断返回值
			{
				break;
			}	
			time++;				
	}
	if(time<waittime)res=1;  //如果在超时时间内break出来，则res=1	
	return res;
} 


/***********************************************************************
 函数名    ：stick_check_cmd()
 功能      ：判断串口返回值与设定值是否一致
 输入      ：无
 输出      ：0：不一致   1：一致
 其他      ：
***********************************************************************/
u8* stick_check_cmd(u8 *str)
{	
	char *strx=0;
	if(USART2_RX_LEN&0X8000)	
	{ 
		USART2_RX_BUF[USART2_RX_LEN&0X7FFF]=0;
		strx=strstr((const char*)USART2_RX_BUF,(const char*)str); 		
	} 
	return (u8*)strx;
}


