#ifndef __MAINJUDGE_
#define __MAINJUDGE_
#include "sys.h"
#include "usart.h"


//WiFi配置-热点-默认名称和密码
extern u8 wifi_ssid[20];	    //热点名称
extern u8 wifi_ssid_password[20];	    //热点密码    

//WiFi配置-路由-默认值
extern u8 wifi_station[20];					//路由器名称
extern u8 wifi_station_password[20];	//路由器密码
extern u8 wifi_ap_ip[15];          //IP地址
extern u16 wifi_ap_port;                      //端口号

//WiFi配置数组
extern u8 wifi_ip_address[20];                 //ip地址
extern u8 wifi_name[20];                       //WiFi名称
extern u8 wifi_password[20];                   //密码

extern const	u8 modeap_buf[2];            //ap模式写入数组，此模式写入AP
extern const	u8 modesta_buf[2];           //STA模式写入数组,此模式写入ST



void WifiReceiveJudge(void);
void Usart2ReceiveJudge(void);
void AllInit(void);



#endif


