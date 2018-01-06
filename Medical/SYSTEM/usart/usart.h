#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	
#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define RXBUFFERSIZE            1       //缓存大小


extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
extern UART_HandleTypeDef UART1_Handler; //UART1句柄
extern UART_HandleTypeDef UART2_Handler; //UART2句柄
extern u8 aRxBuffer[RXBUFFERSIZE];
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);

//串口2相关宏定义
#define USART2_MAX_RECV_LEN		800					//最大接收缓存字节数
#define USART2_MAX_SEND_LEN		800					//最大发送缓存字节数
#define USART2_RX_EN 			1					//0,不接收;1,接收.


//串口3相关宏定义
#define USART3_MAX_RECV_LEN		800					//最大接收缓存字节数
#define USART3_MAX_SEND_LEN		800					//最大发送缓存字节数
#define USART3_RX_EN 			1					//0,不接收;1,接收.

//串口4相关宏定义
#define UART4_MAX_RECV_LEN		2048				//最大接收缓存字节数
#define UART4_MAX_SEND_LEN		2048				//最大发送缓存字节数
#define UART4_RX_EN 			1					//0,不接收;1,接收.

//串口6相关宏定义
#define USART6_MAX_RECV_LEN		600					//最大接收缓存字节数
#define USART6_MAX_SEND_LEN		600					//最大发送缓存字节数
#define USART6_RX_EN 			1					//0,不接收;1,接收.


extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//接收缓冲,最大USART3_MAX_RECV_LEN字节
extern u8  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		//发送缓冲,最大USART3_MAX_SEND_LEN字节
extern u16 USART3_RX_LEN;   						//接收数据状态

extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 		//接收缓冲,最大USART2_MAX_RECV_LEN字节
extern u8  USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		//发送缓冲,最大USART2_MAX_SEND_LEN字节
extern u16 USART2_RX_LEN;   						//接收数据状态

extern u8  UART4_RX_BUF[UART4_MAX_RECV_LEN]; 		//接收缓冲,最大UART4_MAX_RECV_LEN字节
extern u8  UART4_TX_BUF[UART4_MAX_SEND_LEN]; 		//发送缓冲,最大UART4_MAX_SEND_LEN字节
extern u16 UART4_RX_LEN;   							//接收数据状态

extern u8  USART6_RX_BUF[USART6_MAX_RECV_LEN]; 		//接收缓冲,最大USART3_MAX_RECV_LEN字节
extern u8  USART6_TX_BUF[USART6_MAX_SEND_LEN]; 		//发送缓冲,最大USART3_MAX_SEND_LEN字节
extern u16 USART6_RX_LEN;   						//接收数据状态


void usart6_init(u32 bound);				//串口6初始化
void uart4_init(u32 bound);					//串口4初始化
void usart3_init(u32 bound);				//串口3初始化 
void usart2_init(u32 bound);				//串口2初始化 
void TIM7_Init(u16 arr,u16 psc);            //定时器7初始化函数

void u6_printf(char* fmt, ...);             //串口6发送函数
void u4_printf(char* fmt, ...);             //串口4发送函数
void u3_printf(char* fmt, ...);             //串口3发送函数
void u2_printf(char* fmt, ...);             //串口2发送函数


int long usmart_pow1(u8 m,u8 n);             //求m^n次方函数
int long usmart_strnum(u8*str);              //把字符串转为数字函数

u32 usmart_strnum2(u8*str);

u16 abs(u16 data1,u16 data2);                //求绝对值函数
#endif

