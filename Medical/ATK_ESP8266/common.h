#ifndef __COMMON_H__
#define __COMMON_H__	 
#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"   	 
#include "key.h"	 	 	 	 	    
#include "string.h"   

#define SSID "Medical_Nursing_Bed"    //AP模式下的热点名称
#define PASS "12345678"               //热点的密码

//WiFi热点名称
#define WIFI_STATION "mitsubishi"     //要加入的WiFi站点名称
#define PASSWORD "884869japan"        //相应的密码

#define ECN WPA_WPA2_PSK               
#define port 8086                     //端口号
#define WPA_WPA2_PSK 4                //加密模式

//智能终端的名称
extern u8  Phone;                     //手机app
extern u8  Remote_Ctl;                //遥控器
extern u8  Guard_Ctl;                 //护栏
extern u8  PC;                        //PC机
extern u8  Pad;                       //Pad

//智能终端是否准备好发送/接收数据
extern u8 Phone_Ready;               //手机app
extern u8 PC_Ready;                  //PC机
extern u8 RemoteCtl_Ready;           //遥控器
extern u8 GuardCtl_Ready;            //护栏
extern u8 ComputerStick_Ready;       //PC棒

//存储WiFi参数
extern u8 wifi_ssid[];                      
extern u8 wifi_ssid_password[];
extern u8 wifi_station[];
extern u8 wifi_station_password[];
extern u8 wifi_ip_address[];
extern u8 wifi_ap_ip[];
extern u16 wifi_ap_port;


void atk_8266_at_response(u8 mode);                           //用串口2打印WiFi接收到的数据。
u8* atk_8266_check_cmd(u8 *str);                              //判断UART4_RX_BUF数组中有无指定字符串 
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime);           //发送WiFi指令
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime);         //发送数据
u8 atk_8266_quit_trans(void);                                 //退出透传
u8 atk_8266_SET_AP(u8 *ssid,u8 *password,u8 mode,u16 timeout);//esp8266设置AP模式
u8 atk_8266_SET_IPPORT(u16 timeout);                          //esp8266设置多路连接
u8 atk_8266_SET_STA(u8 *ssid,u8 *password,u16 timeout);       //esp8266设置STA模式
u8 atk_8266_SET_PORT(u16 timeout);                            //设置端口号

u8 stick_send_cmd(u8 *cmd,u8 *ack,u16 waittime);              //通过串口，向PC棒发送消息并等待PC棒的应答
u8* stick_check_cmd(u8 *str);                                 //判断串口返回值与设定值是否一致

u8 ESP8266_SET_IP(u8 *Ip,u16 timeout);                        //设置IP地址
u8 ESP8266_SET_PORT(u16 Port,u16 timeout);                    //设置端口号

void esp_8266_apsta_Init(u8 n);                 //初始化esp8266的AP服务器-TCP服务器模式

void atk_8266_get_wanip(u8* ipbuf);             //获取ipbuf中，介于"\"和"\"中的数据
void esp8266_get_ip(u8* ipbuf);                 //从ipbuf中获取路由器分配给STM的IP地址
void esp8266_AP_Init(void);                     //初始化esp8266的AP模式
void esp8266_STA_SERVER_Init(void);             //初始化esp8266的STA模式
void esp8266_Get_IP(void);                      //获取IP地址
void esp8266_send_data(u8 *data);               //发送数据给设备0

void ESP8266_apsta_Init(u8 n);                  //初始化esp8266的AP服务器-TCP服务器模式
void ESP8266_AP_Init(u8 n);		                //初始化esp8266的AP模式
void ESP8266_Get_Controler_Type(u16 timeout);   //有WiFi设备连入时，分配相应的设备号
void ESP8266_Close_Controler_Type(void);        //wifi设备退出时，释放分配的设备号
void ESP8266_send_data(u8 id,u8 *data);         //发送数据给设备号为id的设备
u8   ESP8266_Get_ID(void);                      //接收到当前设备发送的指令后，立马给其返回一个Received指令

void get_wifiname_wifipassword(u8 *ip1,u8 *ip2);//获取字符串中的WiFi名称和密码
void get_apname_wifipassword(u8 *ip1,u8 *ip2);  //获取字符串中的WiFi名称和密码
void get_ip_port(u8 *ip1,u16 data);             //获取字符串中的WiFi的IP地址和端口号

void Flash_Read_STA(void);                      //在Flash内读取保存的路由器名称和密码
void Flash_Read_AP(void);                       //在Flash内读取保存的热点名称和对应密码
void Flash_Read_AP_IPPORT(void);                //在Flash内读取保存的IP地址和端口号

#endif

