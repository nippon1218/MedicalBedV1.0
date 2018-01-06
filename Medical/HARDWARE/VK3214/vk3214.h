#ifndef _VK3224_H
#define _VK3224_H
#include "sys.h"

#define UART_RECEIVE_LEN 	200

/********************全局寄存器列表*********************/

#define GCR			((uint8_t)0x01)	//全局控制寄存器
#define GMUCR		((uint8_t)0x02)	//全局主串口控制寄存器
#define GIR		    ((uint8_t)0x03)	//全局中断寄存器

/********************子串口寄存器列表*********************/

#define SCTLR		((uint8_t)0x06)	//子串口控制寄存器 
#define SCONR		((uint8_t)0x07)	//子串口配置寄存器 
#define SFWCR       ((uint8_t)0x08) //子串口流量控制寄存器
#define SFOCR		((uint8_t)0x09)	//子串口FIFO控制寄存器 
#define	SIER		((uint8_t)0x0B)	//子串口中断使能寄存器 
#define	SIFR		((uint8_t)0x0C)	//子串口中断标志寄存器 
#define	SSR			((uint8_t)0x0D)	//子串口状态寄存器 
#define	SFSR		((uint8_t)0x0E)	//子串口FIFO状态寄存器 
#define	SFDR		((uint8_t)0x0F)	//子串口FIFO数据寄存器

extern u16 UART_RECEIVE;
extern u8 UART_SAVE_BUFFER[UART_RECEIVE_LEN];

//子串口用2表示，主串口用22表示
/********************子串口UART基本配置*********************/

typedef struct
{
  uint8_t UART2_BaudRate;                                       
  uint8_t UART2_WordLength;         
  uint8_t UART2_StopBits;	
  uint8_t UART2_Parity;   			 
  uint8_t UART2_Mode;
} UART2_Base_InitTypeDef;   


typedef struct
{
  uint8_t UART2_TFTL;  //发送FIFO触点控制                                              
  uint8_t UART2_RFTL;  //接收FIFO触点控制                                             
  uint8_t UART2_TFEN;  //发送FIFO使能控制位                                                
  uint8_t UART2_RFEN;  //接收FIFO使能			
  uint8_t UART2_TFCL;  //清除发送FIFO
  uint8_t UART2_RFCL;  // 清除接收FIFO 
} UART2_FIFO_InitTypeDef;

/********************主串口UART基本配置*********************/

typedef struct
{
  uint8_t UART22_BaudRate;                                             
  uint8_t UART22_WordLength;          
  uint8_t UART22_StopBits;	
  uint8_t UART22_Parity;   
} UART22_Base_InitTypeDef;   

#define SET 					 ((uint8_t)0x01)
#define RESET 					 ((uint8_t)0x00)
#define IS_FUNCTION_STATE(STATE) (((STATE) == SET) || ((STATE) == RESET))
                                
/********************定义子串口*********************/

#define COM1 								((uint8_t)0x00)
#define COM2								((uint8_t)0x10)
#define COM3 								((uint8_t)0x20)
#define COM4 								((uint8_t)0x30)
#define IS_UART2_ALL_PERIPH(COM)			(((COM) == COM1) || ((COM) == COM2) || ((COM) == COM3) ||((COM) == COM4)) 
 
//子串口控制寄存器SCTLR                                			 								 			                
/********************定义子串口波特率*********************/

#define UART2_BaudRate_0              		 ((uint8_t)0x00)
#define UART2_BaudRate_1                 	 ((uint8_t)0x10)
#define UART2_BaudRate_2                     ((uint8_t)0x20)
#define UART2_BaudRate_3                  	 ((uint8_t)0x30)
#define UART2_BaudRate_4                	 ((uint8_t)0x40)
#define UART2_BaudRate_5                 	 ((uint8_t)0x50)
#define UART2_BaudRate_6                	 ((uint8_t)0x60)
#define UART2_BaudRate_7                     ((uint8_t)0x70)
#define UART2_BaudRate_8               		 ((uint8_t)0x80)
#define UART2_BaudRate_9                 	 ((uint8_t)0x90)
#define UART2_BaudRate_10                    ((uint8_t)0xa0)
#define UART2_BaudRate_11                	 ((uint8_t)0xb0)
#define UART2_BaudRate_12                	 ((uint8_t)0xc0)
#define UART2_BaudRate_13                	 ((uint8_t)0xd0)
#define UART2_BaudRate_14                	 ((uint8_t)0xe0)
#define UART2_BaudRate_15                	 ((uint8_t)0xf0)
#define IS_UART2_BAUDRATE(BAUDRATE)			 (((BAUDRATE)== UART2_BaudRate_0 ) || \
                                 			 ((BAUDRATE) == UART2_BaudRate_1 ) || \
                                 			 ((BAUDRATE) == UART2_BaudRate_2 ) || \
								 			 ((BAUDRATE) == UART2_BaudRate_3 ) || \
								 			 ((BAUDRATE) == UART2_BaudRate_4 ) || \
											 ((BAUDRATE) == UART2_BaudRate_5 ) || \
											 ((BAUDRATE) == UART2_BaudRate_6 ) || \
											 ((BAUDRATE) == UART2_BaudRate_7 ) || \
											 ((BAUDRATE) == UART2_BaudRate_8 ) || \
											 ((BAUDRATE) == UART2_BaudRate_9 ) || \
											 ((BAUDRATE) == UART2_BaudRate_10) || \
											 ((BAUDRATE) == UART2_BaudRate_11) || \
											 ((BAUDRATE) == UART2_BaudRate_12) || \
											 ((BAUDRATE) == UART2_BaudRate_13) || \
											 ((BAUDRATE) == UART2_BaudRate_14) || \
											 ((BAUDRATE) == UART2_BaudRate_15))

//子串口配置寄存器SCONR
/********************定义子串口数据位*********************/

#define UART2_WordLength_8b                  ((uint8_t)0x00)
#define UART2_WordLength_9b                  ((uint8_t)0x40)                                  
#define IS_UART2_WORD_LENGTH(LENGTH)         (((LENGTH) == UART2_WordLength_8b) || \
                                             ((LENGTH) == UART2_WordLength_9b))
											 
/********************定义子串口停止位*********************/

#define UART2_StopBits_1                     ((uint8_t)0x00)
#define UART2_StopBits_2                     ((uint8_t)0x80)
#define IS_UART2_STOPBITS(STOPBITS)          (((STOPBITS) == UART2_StopBits_1) || \
                                             ((STOPBITS) == UART2_StopBits_2) )
											 
/********************定义子串口奇偶校验位*********************/

#define UART2_Parity_0                       ((uint8_t)0x00)
#define UART2_Parity_Even                    ((uint8_t)0x10)//偶校验
#define UART2_Parity_Odd                     ((uint8_t)0x08)//奇校验
#define UART2_Parity_1                       ((uint8_t)0x18)
#define IS_UART2_PARITY(PARITY)              (((PARITY) == UART2_Parity_0) || \
                                             ((PARITY) == UART2_Parity_Even) || \
                                             ((PARITY) == UART2_Parity_Odd)	 || \
								             ((PARITY) == UART2_Parity_1)	)

/********************定义主串口波特率*********************/

#define UART22_BaudRate_0              		 ((uint8_t)0x00)
#define UART22_BaudRate_1                 	 ((uint8_t)0x10)
#define UART22_BaudRate_2                    ((uint8_t)0x20)
#define UART22_BaudRate_3                  	 ((uint8_t)0x30)
#define UART22_BaudRate_4                	 ((uint8_t)0x40)
#define UART22_BaudRate_5                 	 ((uint8_t)0x50)
#define UART22_BaudRate_6                	 ((uint8_t)0x60)
#define UART22_BaudRate_7                    ((uint8_t)0x70)
#define UART22_BaudRate_8               	 ((uint8_t)0x80)
#define UART22_BaudRate_9                 	 ((uint8_t)0x90)
#define UART22_BaudRate_10                   ((uint8_t)0xa0)
#define UART22_BaudRate_11                	 ((uint8_t)0xb0)
#define UART22_BaudRate_12                	 ((uint8_t)0xc0)
#define UART22_BaudRate_13                	 ((uint8_t)0xd0)
#define UART22_BaudRate_14                	 ((uint8_t)0xe0)
#define UART22_BaudRate_15                	 ((uint8_t)0xf0)
#define IS_UART22_BAUDRATE(BAUDRATE)		 (((BAUDRATE) == UART22_BaudRate_0  ) || \
                                 			 ((BAUDRATE) == UART22_BaudRate_1 ) || \
                                 			 ((BAUDRATE) == UART22_BaudRate_2 )    || \
								 			 ((BAUDRATE) == UART22_BaudRate_3	)   || \
								 			 ((BAUDRATE) == UART22_BaudRate_4	)   || \
											 ((BAUDRATE) == UART22_BaudRate_5	)   || \
											 ((BAUDRATE) == UART22_BaudRate_6  )	|| \
											 ((BAUDRATE) == UART22_BaudRate_7  )	|| \
											 ((BAUDRATE) == UART22_BaudRate_8	)   || \
											 ((BAUDRATE) == UART22_BaudRate_9	)	|| \
											 ((BAUDRATE) == UART22_BaudRate_10	)	|| \
											 ((BAUDRATE) == UART22_BaudRate_11	)   || \
											 ((BAUDRATE) == UART22_BaudRate_12	)	|| \
											 ((BAUDRATE) == UART22_BaudRate_13	)	|| \
											 ((BAUDRATE) == UART22_BaudRate_14 )   || \
											 ((BAUDRATE) == UART22_BaudRate_15))

/********************定义主串口数据位*********************/

#define UART22_WordLength_8b                  ((uint8_t)0x00)
#define UART22_WordLength_9b                  ((uint8_t)0x08)                                  
#define IS_UART22_WORD_LENGTH(LENGTH)         (((LENGTH) == UART_WordLength22_8b) || \
                                              ((LENGTH) == UART_WordLength22_9b))

/********************定义主串口停止位*********************/

#define UART22_StopBits_1                     ((uint8_t)0x00)
#define UART22_StopBits_2                     ((uint8_t)0x04)
#define IS_UART22_STOPBITS(STOPBITS)          (((STOPBITS) == UART22_StopBits_1) || \
                                              ((STOPBITS) == UART22_StopBits_2) )
											  
/********************定义主串口奇偶校验位*********************/

#define UART22_Parity_0                     ((uint8_t)0x00)
#define UART22_Parity_Even                  ((uint8_t)0x02)//偶校验
#define UART22_Parity_Odd                   ((uint8_t)0x01)//奇校验
#define UART22_Parity_1                     ((uint8_t)0x03)
#define IS_UART22_PARITY(PARITY)            (((PARITY) == UART22_Parity_0) || \
                                            ((PARITY) == UART22_Parity_Even) || \
                                            ((PARITY) == UART22_Parity_Odd)	 || \
								            ((PARITY) == UART22_Parity_1))
											
//子串口控制寄存器SCTLR								 
/********************定义子串口模式*********************/								 

#define UART2_Mode_RS232                    ((uint8_t)0x00)
#define UART2_Mode_NONE                     ((uint8_t)0x04)
#define IS_UART2_MODE(MODE) 			    (((MODE) == UART2_Mode_RS232) || \
                                            ((MODE) == UART2_Mode_NONE) ||)
//子串口FIFO控制寄存器
/********************发送FIFO触点控制*********************/	

#define UART2_TFTL_0BYTE					((uint8_t)0x00)
#define UART2_TFTL_4BYTE					((uint8_t)0x40)
#define UART2_TFTL_8BYTE					((uint8_t)0x80)
#define UART2_TFTL_12BYTE				    ((uint8_t)0xC0)
#define IS_UART2_TFTL(TFTL) 		        (((TFTL) == UART2_TFTL_0BYTE	) || \
                                            ((TFTL) == UART2_TFTL_4BYTE	) || \
                                            ((TFTL) == UART2_TFTL_8BYTE	)	 || \
								            ((TFTL) == UART2_TFTL_12BYTE)	)

/********************接收FIFO触点控制*********************/	

#define UART2_RFTL_1BYTE					((uint8_t)0x00)
#define UART2_RFTL_4BYTE					((uint8_t)0x10)
#define UART2_RFTL_8BYTE					((uint8_t)0x20)
#define UART2_RFTL_14BYTE					((uint8_t)0x30)
#define IS_UART2_RFTL(RFTL) 		 		(((RFTL) == UART2_RFTL_0BYTE	) || \
											((RFTL) == UART2_RFTL_4BYTE	) || \
											((RFTL) == UART2_RFTL_8BYTE	)	 || \
								            ((RFTL) == UART2_RFTL_12BYTE)	)

//子串口中断使能寄存器SIER
/********************中断使能寄存器*********************/	

#define UART2_IT_FOEIEN                     ((uint8_t)0x40)		//FIFO数据错误中断使能位
#define UART2_IT_TRIEN                      ((uint8_t)0x02)		//发送FIFO触点中断使能位
#define UART2_IT_RFIEN                      ((uint8_t)0x01)		//使能接收FIFO触点中断 
#define IS_UART2_CONFIG_IT(IT) 				(((IT) == UART2_IT_FOEIEN ) || \
								            ((IT) == UART2_IT_TRIEN	) || \
								            ((IT) == UART2_IT_RFIEN	) )

/********************判断中断状态*********************/

#define IS_UART2_GET_IT(IT)		            (((IT) == UART2_IT_FOEIEN ) || \
								            ((IT) == UART2_IT_TRIEN	) || \
								            ((IT) == UART2_IT_RFIEN	) )

/********************清除中断*************************/

#define IS_UART2_CLEAR_IT(IT)	            (((IT) == UART2_IT_FOEIEN ) || \
								            ((IT) == UART2_IT_TRIEN	) || \
								            ((IT) == UART2_IT_RFIEN	) )

/********************子串口状态寄存器SSR*************************/

#define UART2_FLAG_OE                       ((uint8_t)0x80)		//子串口接收 FIFO 中当前数据(最早写入)的溢出错误标志位
#define UART2_FLAG_FE                       ((uint8_t)0x40)		//子串口接收 FIFO 中当前数据(最早写入)的帧错误标志位
#define UART2_FLAG_PE                       ((uint8_t)0x20)		//子串口接收 FIFO 中当前数据(最早写入)的校验错误标志位
#define UART2_FLAG_TFFL                     ((uint8_t)0x08)	   	//子串口发送 FIFO 满标志
#define UART2_FLAG_TFEM                     ((uint8_t)0x04)		//子串口发送 FIFO 空标志
#define UART2_FLAG_TXBY                     ((uint8_t)0x02)		//子串口发送 TX 忙标志
#define UART2_FLAG_RFEM                     ((uint8_t)0x01)		//子串口接收 FIFO 空标志
#define IS_UART2_FLAG(FLAG)  (((FLAG) == UART2_FLAG_OE)   || ((FLAG) == UART2_FLAG_FE) || \
                             ((FLAG) == UART2_FLAG_PE)    || ((FLAG) == UART2_FLAG_TFFL) || \
                             ((FLAG) == UART2_FLAG_TFEM ) || ((FLAG) == UART2_FLAG_TXBY) || \
                             ((FLAG) == UART2_FLAG_RFEM) )

void Uart22_Send_Data( uint8_t _cmd,uint8_t data );                                  //主串口发送函数
uint8_t Uart22_Receive_Data( uint8_t value);                                         //主串口接收函数
void Init_vk3214_UART( void );                                                       //测试STM32与vk3214的UART通信
void UART22_Base_Init( UART22_Base_InitTypeDef* UART22_InitStruct );		         //初始化VK3224主串口的基本工作状态
void Uart_Master_Config(void);                                                       //配置主串口参数
void UART22_StructInit( UART22_Base_InitTypeDef* UART22_InitStruct );                //初始化vk3214主串口的结构体变量
void UART2_Base_Init( uint8_t UARTx, UART2_Base_InitTypeDef* UART2_InitStruct );     //初始化VK3224子串口的UARTx的基本工作状态
void UART2_StructInit( UART2_Base_InitTypeDef* UART2_InitStruct );                   //初始化vk3214的UARTx的结构体变量
void UART2_FIFO_Init( uint8_t UARTx, UART2_FIFO_InitTypeDef* UART2_FIFO_InitStruct );//初始化VK3224的UARTx关于FIFO基本工作状态
void UART2_FIFO_StructInit( UART2_FIFO_InitTypeDef* UART2_FIFO_InitStruct );         //初始化vk3214的UARTx的FIFO结构体变量
void UART2_ITConfig( uint8_t UARTx, uint8_t UART2_IT, uint8_t FunctionalState );	 //配置VK3224的UARTx中断
void UART2_Cmd( uint8_t UARTx, uint8_t FunctionalState );		                     //使能子串口
void UART2_SendData( uint8_t UARTx, uint8_t data );			                         //子串口发送数据
uint8_t UART2_ReceiveData( uint8_t UARTx );			                                 //子串口接收数据
uint8_t UART2__FIFO_ReceiveData( uint8_t UARTx );                                    //读UARTx的FIFO寄存器的数值
uint8_t Get_IT_UARTx( void );						                                 //获取子串口中断号
uint8_t UART2_GetFlagStatus( uint8_t UARTx, uint8_t UART2_FLAG );		             //查询子串口的各种状态
uint8_t UART2_GetITStatus( uint8_t UARTx, uint8_t UART2_IT);			             //查询子串口的中断状态
void UART2_ClearITPendingBit( uint8_t UARTx, uint8_t UART2_IT );		             //清楚子串口的中断标志位
uint8_t UART2_Get_Num_TXFIFO( uint8_t UARTx );							             //查询发送FIFO的字节数
uint8_t UART2_Get_Num_RXFIFO( uint8_t UARTx );							             //查询接收FIFO的字节数
void UART2_Clear_TXFIFO( uint8_t UARTx );								             //清空发送FIFO
void UART2_Clear_RXFIFO( uint8_t UARTx );								             //清空接收FIFO
void Save_Uart_Data(u8 data);                                                        //保存串口数据
void UART_Slave1_Config(void);                                                       //配置子串口1参数
void UART_Slave2_Config(void);                                                       //配置子串口2参数
void UART_Slave3_Config(void);                                                       //配置子串口3参数
void UART_Slave4_Config(void);                                                       //配置子串口4参数
void UART_EX_Config(void);                                                           //配置所有串口参数
 
#endif
