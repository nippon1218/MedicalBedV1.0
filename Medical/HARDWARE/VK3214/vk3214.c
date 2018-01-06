#include "vk3214.h"
#include "usart.h"
#include "sys.h"
#include "led.h"
#include "delay.h"

u16 UART_RECEIVE=0;
u8 UART_SAVE_BUFFER[UART_RECEIVE_LEN];

/***************************************************************
 * 函数名：Uart22_Send_Data();
 * 功能  ：发送数据到vk3214
 * 输入  : uint8_t value
 * 输出  ：uint8_t rxd
 * 举例  ：无
 * 注意  ：无
 
*****************************************************************/

void Uart22_Send_Data( uint8_t _cmd,uint8_t data )//第一个参数代表命令字节，第二个参数代表要发送的数据
{
  u8 i; 
	for(i=0;i<1;i++)							  //循环发送数据
	{
		while((USART6->SR&0X40)==0);			  //循环发送,直到发送完毕   
		USART6->DR=_cmd;
		USART2->DR=_cmd; 		
	} 
//	delay_us (50);				
	for(i=0;i<1;i++)							  //循环发送数据
	{
		while((USART6->SR&0X40)==0);			  //循环发送,直到发送完毕   
		USART6->DR=data;
		USART2->DR=data; 		
	} 	
}

/***************************************************************
 * 函数名：UART22_Receive_Data();
 * 功能  ：从vk3214读取数据
 * 输入  : uint8_t value
 * 输出  ：uint8_t  rbuf
 * 举例  ：无
 * 注意  ：无

*****************************************************************/
uint8_t Uart22_Receive_Data( uint8_t data )
{
	u16 i; 
	for(i=0;i<1;i++)							//循环发送数据
	{
		while((USART6->SR&0X40)==0);			//循环发送,直到发送完毕   
		USART6->DR=data;
		USART2->DR=data;
	}   
		delay_us (250);
//		for(i=0;i<1;i++)					    //循环发送数据
//	{
//		 while((USART2->SR&0X40)==0);			//循环发送,直到发送完毕  
////     delay_ms(1);		
//		 USART2->DR=UART_SAVE_BUFFER[0]; 
//	} 	
   return UART_SAVE_BUFFER[0];
}


/***************************************************************
 * 函数名：Init_vk3214_UART( );
 * 功能  ：测试STM32与vk3214的UART通信
 * 输入  : 无
 * 输出  ：无
 * 举例  ：无
 * 注意  ：无

*****************************************************************/

void Init_vk3214_UART( void )
{
	uint8_t flag ,i;
	uint8_t ch, addr, data1, data2, data;
	ch = COM1;
	addr = SCTLR ;
	data1 = 0X80 | ch | addr;
    data2	= UART22_BaudRate_1;
	Uart22_Send_Data( data1, data2 );    //发送数据到vk3214	
	delay_ms(10);		
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 ); //从vk3214读取数据，8位
	  
	if( data != UART22_BaudRate_1 ) 
	{
		flag=0;			
		for(i=0; i<4; i++)
		{
			LED1=!LED1;                 //若通信不成功，则LED1闪2下
			LED0=!LED0;
			delay_ms(10);
		}
			u2_printf("\r\n通信不成功，请重新尝试\r\n");			
	}
	else
	{
		flag=1;			
		for(i=0; i<4; i++)
		{
			LED0=!LED0;                 //若通信成功，则LED0闪2下
			delay_ms(10);
		}
			u2_printf("\r\nVK3214通信成功\r\n");			
	}
}

/***************************************************************
 * 函数名：UART_Base_Init;
 * 功能  ：初始化VK3214主串口的基本工作状态
 * 输入  : uint8_t UARTx；UART_Base_InitTypeDef* UART_InitStruct
 * 输出  ：无
 * 举例  ：UART_Base_Init( COM1,  &UART1_InitStruct )
 * 注意  ：无
	
*****************************************************************/
                   
void UART22_Base_Init( UART22_Base_InitTypeDef* UART22_InitStruct )
{
	uint8_t data, addr, data1, data2;
	 /* Check the parameters */
  	assert_param(IS_UART22_BAUDRATE(UART22_InitStruct->UART22_BaudRate));  
  	assert_param(IS_UART22_WORD_LENGTH(UART22_InitStruct->UART22_WordLength));
  	assert_param(IS_UART22_STOPBITS(UART22_InitStruct->UART22_StopBits));
  	assert_param(IS_UART22_PARITY(UART22_InitStruct->UART22_Parity));                         
	addr  = GMUCR ;
	data1 = 0x80 | addr;
	data2 = (UART22_InitStruct->UART22_BaudRate) |  (UART22_InitStruct->UART22_WordLength) | ( UART22_InitStruct->UART22_StopBits ) | ( UART22_InitStruct->UART22_Parity );    
	Uart22_Send_Data( data1 ,data2 );      //配置GMUCR
	delay_ms(10);		
	data1 = addr;
	data = Uart22_Receive_Data( data1 );   //读GMUCR寄存器中的数据   
    if(data==data2)
	{
		u2_printf("\r\n主串口配置成功\r\n");
	}
	else
	{
		u2_printf("\r\n主串口配置失败\r\n");
	}			
}

/***************************************************************
 * 函数名：UART_StructInit;
 * 功能  ：初始化vk3214主串口的结构体变量
 * 输入  : UART_Base_InitTypeDef* UART_InitStruc
 * 输出  ：无
 * 举例  ：UART_StructInit( &UART1_InitStruct )
 * 注意  ：无

*****************************************************************/
void UART22_StructInit( UART22_Base_InitTypeDef* UART22_InitStruct )
{
/* UART_InitStruct members default value */
	UART22_InitStruct->UART22_BaudRate = UART22_BaudRate_1;
	UART22_InitStruct->UART22_WordLength = UART22_WordLength_8b;
	UART22_InitStruct->UART22_StopBits = UART22_StopBits_1;
	UART22_InitStruct->UART22_Parity = UART22_Parity_0;
}

/***************************************************************
 * 函数名：UART_Base_Init;
 * 功能  ：初始化vk3214的UARTx的基本工作状态--4个子串口
 * 输入  : uint8_t UARTx；UART_Base_InitTypeDef* UART_InitStruct
 * 输出  ：无
 * 举例  ：UART_Base_Init( COM1,  &UART1_InitStruct )
 * 注意  ：无

*****************************************************************/
                   
void UART2_Base_Init( uint8_t UARTx, UART2_Base_InitTypeDef* UART2_InitStruct )
{
	uint8_t ch, data, addr, data1, data2;
	 /* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(USARTx));//判断是哪个子串口
  	assert_param(IS_UART2_BAUDRATE(UART2_InitStruct->UART2_BaudRate));  
  	assert_param(IS_UART2_WORD_LENGTH(UART2_InitStruct->UART2_WordLength));
  	assert_param(IS_UART2_STOPBITS(UART2_InitStruct->UART2_StopBits));
  	assert_param(IS_UART2_PARITY(UART2_InitStruct->UART2_Parity));
  	assert_param(IS_UART2_MODE(UART2_InitStruct->UART2_Mode));

	ch = UARTx;                          //SCTLR配置
	addr = SCTLR ;
	data1 = ch | 0x80 | addr ;	
    data2 =(UART2_InitStruct->UART2_BaudRate) | (UART2_InitStruct->UART2_Mode);
    data2=data2 | 0x08;                  //使能子串口进行数据收发
	Uart22_Send_Data( data1 ,data2 );    //配置SCTLR	
    delay_ms(15);
	
	data1 = ch | addr;	
	data = Uart22_Receive_Data( data1 ); //读取SCTLR寄存器中的数据
	delay_ms(15);
    if(data==data2)
	{
	  u2_printf("\r\n子串口波特率配置成功\r\n"); delay_ms(10);
	}
	else
	{
		 u2_printf("\r\n子串口波特率配置失败\r\n"); delay_ms(10);
	}

	addr = SCONR ;
	data1 = ch | 0x80 | addr ;		
	data2 =(UART2_InitStruct->UART2_WordLength) | ( UART2_InitStruct->UART2_StopBits ) | ( UART2_InitStruct->UART2_Parity );
	delay_ms(5);
	Uart22_Send_Data( data1, data2 );     //配置SCONR
    delay_ms(15);
		
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
    delay_ms(15);		
    if(data==data2)
	{
		 u2_printf("\r\n子串口数据长度校验模式配置成功\r\n");    delay_ms(10);
	}
	else
	{
		u2_printf("\r\n子串口数据长度校验模式配置失败\r\n");    delay_ms(10);
	}
}


/***************************************************************
 * 函数名：UART_StructInit;
 * 功能  ：初始化vk3214的UARTx的结构体变量
 * 输入  : UART_Base_InitTypeDef* UART_InitStruc
 * 输出  ：无
 * 举例  ：UART_StructInit( &UART1_InitStruct )
 * 注意  ：无

*****************************************************************/
void UART2_StructInit( UART2_Base_InitTypeDef* UART2_InitStruct )
{
/* UART_InitStruct members default value */
	UART2_InitStruct->UART2_BaudRate = UART2_BaudRate_1;  //子串口波特率设置为115200
	UART2_InitStruct->UART2_WordLength = UART2_WordLength_8b;
	UART2_InitStruct->UART2_StopBits = UART2_StopBits_1;
	UART2_InitStruct->UART2_Parity = UART2_Parity_0;
	UART2_InitStruct->UART2_Mode = UART2_Mode_RS232;
}


/***************************************************************
 * 函数名：UART_FIFO_Init;
 * 功能  ：初始化vk3214的UARTx关于FIFO基本工作状态
 * 输入  : uint8_t UARTx; UART_FIFO_InitTypeDef* UART_FIFO_InitStruct
 * 输出  ：无
 * 举例  ：UART_FIFO_Init( COM1,  &UART1_FIFO_InitStruct )
 * 注意  ：无

*****************************************************************/

void UART2_FIFO_Init( uint8_t UARTx, UART2_FIFO_InitTypeDef* UART2_FIFO_InitStruct )
{
	uint8_t ch, data, addr, data1, data2;
	
	/* Check the parameters */
	assert_param(IS_UART2_ALL_PERIPH(USARTx));
  	assert_param(IS_UART2_TFTL(UART2_FIFO_InitStruct->UART2_TFTL));  
  	assert_param(IS_UART2_RFTL(UART2_FIFO_InitStruct->UART2_RFTL));
  	assert_param(IS_FUNCTION_STATE(UART2_FIFO_InitStruct->UART2_TFEN));
  	assert_param(IS_FUNCTION_STATE(UART2_FIFO_InitStruct->UART2_RFEN));
  	assert_param(IS_FUNCTION_STATE(UART2_FIFO_InitStruct->UART2_TFCL));
	assert_param(IS_FUNCTION_STATE(UART2_FIFO_InitStruct->UART2_RFCL));		

	ch = UARTx;
	addr = SFOCR ;
	data2 = (UART2_FIFO_InitStruct->UART2_TFTL) | (UART2_FIFO_InitStruct->UART2_RFTL) | (UART2_FIFO_InitStruct->UART2_RFCL);
	data = ((UART2_FIFO_InitStruct->UART2_TFEN) << 3); //发送FIFO使能位
	data2 = data | data2;
	data = (UART2_FIFO_InitStruct->UART2_RFEN) << 2;   //接收FIFO使能位
	data2 = data | data2;
	data = (UART2_FIFO_InitStruct->UART2_TFCL) << 1;   //清除发送FIFO位
	data2 = data | data2;	 
	data1 = ch | 0x80 | addr ;                         //配置SFOCR 子串口FIFO控制寄存器
	Uart22_Send_Data( data1, data2 );
	delay_ms(5);
	
	data1 = ch | addr;		
	data = Uart22_Receive_Data( data1 );               //读取SFOCR寄存器中的数据
	delay_ms(5);
	if(data==data2)
	{
		u2_printf("\r\n\r\nSFOCR寄存器配置成功\r\n");
	}
	else
	{
		u2_printf("\r\n\r\nSFOCR寄存器配置失败\r\n"); 
	}	
}

/***************************************************************
 * 函数名：UART_FIFO_StructInit;
 * 功能  ：初始化vk3214的UARTx的FIFO结构体变量
 * 输入  : UART_FIFO_InitTypeDef* UART_FIFO_InitStruct
 * 输出  ：无
 * 举例  ：UART_FIFO_StructInit( &UART1_FIFO_InitStruct )
 * 注意  ：无

*****************************************************************/

void UART2_FIFO_StructInit( UART2_FIFO_InitTypeDef* UART2_FIFO_InitStruct )
{
/* UART_FIFO_InitStruct members default value */

	UART2_FIFO_InitStruct->UART2_TFTL = UART2_TFTL_0BYTE	;    //具体配置SFOCR
	UART2_FIFO_InitStruct->UART2_RFTL = UART2_RFTL_1BYTE	;    //禁止发送，禁止接收FIFO,不清除发送、接收FIFO中的所有数据
	UART2_FIFO_InitStruct->UART2_TFEN = SET;
	UART2_FIFO_InitStruct->UART2_RFEN = SET;
	UART2_FIFO_InitStruct->UART2_TFCL = RESET;
	UART2_FIFO_InitStruct->UART2_RFCL = RESET;
}

/***************************************************************
 * 函数名：UART_ITConfig;
 * 功能  ：配置vk3214的UARTx中断
 * 输入  :uint8_t UARTx, uint8_t UART_IT, uint8_t FunctionalState
 * 输出  ：无
 * 举例  ：UART_ITConfig( UART1, UART_IT_FOEIEN , SET )
 * 注意  ：无

*****************************************************************/

void UART2_ITConfig( uint8_t UARTx, uint8_t UART2_IT, uint8_t FunctionalState )//参数UART_IT代表要配置的中断类型
{
	uint8_t ch, addr;
	uint8_t data1, data2, data ;
	uint8_t IR;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  	assert_param(IS_UART2_CONFIG_IT(UART2_IT));      //使能数据错误中断、发送触点中断，接收触点中断
  	assert_param(IS_FUNCTION_STATE(FunctionalState));//配置为0或1

	ch = UARTx;
	addr = SIER ;
	data1 =  ch | addr ;
	data = Uart22_Receive_Data( data1 );             //读取子串口中断使能寄存器里面的数据	
	IR = Uart22_Receive_Data( 0x03 );			     //读GIR寄存器的值

/*	关闭UARTx的全局中断		*/                
	if( UARTx == COM1 )                              //全局中断寄存器的配置GIR,先把所有子串口中断关闭
	{
		data1 = 0x83;
		data2 = (IR&0XEF);
		Uart22_Send_Data( data1, data2 );            //关闭子串口1中断
	}
	else if( UARTx == COM2 ) 
	{		
		data1 = 0x83;
		data2 = (IR&0XdF);
		Uart22_Send_Data( data1, data2 );            //关闭子串口2中断
	}
	else if( UARTx == COM3 ) 
	{			
		data1 = 0x83;
		data2 = (IR&0XbF);
		Uart22_Send_Data( data1, data2 );            //关闭子串口3中断
	}
	else
	{
		data1 = 0x83;
		data2 = (IR&0X7F);
		Uart22_Send_Data( data1, data2 );            //关闭子串口4中断
	}
				
	if( FunctionalState == SET )//打开GIR中对应的子中断口，并对子串口中断寄存器特定中断位进行配置
	{		
		data1 = 0x80 | ch | addr ;
		data2 = data | UART2_IT;
		Uart22_Send_Data( data1, data2 );            //配置SIER

	/*	使能UARTx的全局中断		*/
		if( UARTx == COM1 ) 
		{			
			data1 = 0x83;
			data2 = (IR | 0X10);
			Uart22_Send_Data( data1, data2 );       //使能子串口1中断
		}
		else if( UARTx == COM2 ) 
		{			
			data1 = 0x83;
			data2 = (IR | 0X20);
			Uart22_Send_Data( data1, data2 );       //使能子串口2中断
		}
		else if( UARTx == COM3 ) 
		{			
			data1 = 0x83;
			data2 = (IR | 0X40);
			Uart22_Send_Data( data1, data2 );       //使能子串口3中断
		}
		else           
		{	
			data1 = 0x83;
			data2 = (IR | 0X80);
			Uart22_Send_Data( data1, data2 );        //使能子串口4中断
		}
	}
	else            //不打开子串口中断，只对子串口相应中断位进行配置
	{
		if( UART2_IT == UART2_IT_FOEIEN ) 		data2 = data & 0xbf;
		else if( UART2_IT == UART2_IT_TRIEN )	data2 = data & 0xfd;
		else 									data2 = data & 0xfe;

		data1 =  0x80 | ch | addr ;
		Uart22_Send_Data( data1, data2 );
	}
}

/***************************************************************
 * 函数名：UART_Cmd;
 * 功能  ：使能子串口
 * 输入  : uint8_t UARTx, uint8_t UART_IT, uint8_t FunctionalState
 * 输出  ：无
 * 举例  ：UART_Cmd( COM1, SET )
 * 注意  ：无
*****************************************************************/

void UART2_Cmd( uint8_t UARTx, uint8_t FunctionalState )//自己设置使能或不使能某个子串口，FunctionalState=1使能；FunctionalState=0不使能
{
	uint8_t ch, data, data1, data2, addr;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  	assert_param(IS_FUNCTION_STATE(FunctionalState));

  	ch = UARTx;
  	addr = SCTLR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );

	if(  FunctionalState == SET )
	{
		data1 = 0x80 | ch | addr  ;
		data2 = data | 0x08 ;
		Uart22_Send_Data( data1, data2 );//使能子串口，可以进行正常数据收发
	}
	else
	{
		data1 = 0x80 | ch | addr  ;	
		data2 = ( data & 0xF7);	
		Uart22_Send_Data( data1, data2 );//不使能子串口，此时该子串口不能进行数据收发
	}
}

/***************************************************************
 * 函数名：UART_SendData;
 * 功能  ：子串口发送数据
 * 输入  : uint8_t UARTx,  uint8_t data
 * 输出  ：无
 * 举例  ：UART_SendData( com1, 0xffff )
 * 注意  ：

*****************************************************************/

void UART2_SendData( uint8_t UARTx, uint8_t data )
{
	uint8_t ch, data1,data2, addr;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
	
	ch = UARTx;
	addr = SFDR ;
	data1 = 0x80 | ch | addr ;
	data2 = data ;

	Uart22_Send_Data( data1, data2 );
}

/***************************************************************
 * 函数名：UART_ReceiveData;
 * 功能  ：子串口接收数据
 * 输入  : uint8_t UARTx
 * 输出  ：读UARTx的FIFO寄存器的数值
 * 举例  ：UART_ReceiveData( com1 )
 * 注意  ：

*****************************************************************/

uint8_t UART2_ReceiveData( uint8_t UARTx )
{
	uint8_t ch, addr, data1 ;
	uint8_t data,data2;
	 
  /* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  
  /* Receive Data */
	ch =  UARTx;
	addr = SFDR ;
	data1 = ch | addr;

	data = Uart22_Receive_Data( data1 );
	data2=data;
//	delay_us(100);	
	UART2_SendData( ch, data2 );
	return data ;
}

/***************************************************************
 * 函数名：UART2__FIFO_ReceiveData();
 * 功能  ：通过串口UARTx通道接收最近一次的数据
 * 输入  : UARTx：子串口号
 * 输出  ：读UARTx的FIFO寄存器的数值
 * 举例  ：UART_ReceiveData( com1 )
 * 注意  ：

*****************************************************************/
uint8_t UART2__FIFO_ReceiveData( uint8_t UARTx )
{
	uint8_t ch, addr, data1 ;
	uint8_t data,data2;
	 
  /* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  
  /* Receive Data */
	ch =  UARTx;
//	addr = SFDR ;
	data1 = ch | 0x40;

	data = Uart22_Receive_Data( data1 );
	data2=data;
//	delay_us(100);
	
	UART2_SendData( ch, data2 );
	return data ;
}

/***************************************************************
 * 函数名：Get_IT_UARTx;
 * 功能  :获取子串口中断号
 * 输入  :无
 * 输出  ：返回子串口号
 * 举例  ：
 * 注意  ：

*****************************************************************/

uint8_t Get_IT_UARTx( void )
{
	uint8_t addr, data1 ;
	uint8_t data;

	addr = GIR ;
	data1 = addr;
	data = Uart22_Receive_Data( data1 );	
	data = data & 0x0f;

	if( (data & 0x08) == 0x08 )				  return COM4;
    else if( (data & 0x04) == 0x04 )		  return COM3;
	else if( (data & 0x02) == 0x02 )		  return COM2;
	else									  return COM1;
}	

/***************************************************************
 * 函数名：UART_GetFlagStatus;
 * 功能  :查询子串口的各种状态
 * 输入  :uint8_t UARTx, uint8_t UART_FLAG
 * 输出  ：
 * 举例  ：UART_GetFlagStatus( COM1, UART_FLAG_OE);
 * 注意  ：

*****************************************************************/

uint8_t UART2_GetFlagStatus( uint8_t UARTx, uint8_t UART2_FLAG )//UART_FLAG为要查询的某位状态，
{
	uint8_t ch, addr, data1;
	uint8_t data;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  	assert_param(IS_UART2_FLAG(UART2_FLAG));

  	ch = UARTx;
  	addr = SSR ;
  	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );     //读取SSR寄存器的数据

	if( (data & UART2_FLAG) == UART2_FLAG ) 	return SET;
	else 									    return RESET;
}

/***************************************************************
 * 函数名：UART_GetITStatus;
 * 功能  :查询子串口的中断状态
 * 输入  :uint8_t UARTx, uint8_t UART_IT
 * 输出  ：
 * 举例  ：UART_GetITStatus( COM1, UART_IT_FOEIEN );
 * 注意  ：

*****************************************************************/

uint8_t UART2_GetITStatus( uint8_t UARTx, uint8_t UART2_IT)
{
	uint8_t ch, addr, data1;
	uint8_t data;
	
	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
 	assert_param(IS_UART2_GET_IT(UART_IT));

	ch = UARTx;
	addr = SIFR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );    //读取子串口SIFR寄存器的数值，以判断发生了何种中断

	if( (data & UART2_IT)  == UART2_IT ) 	return 	SET;
	else							        return  RESET;
}
	
/***************************************************************
 * 函数名：UART_ClearITPendingBit;
 * 功能  :清楚子串口的中断标志位
 * 输入  :uint8_t UARTx, uint8_t UART_IT
 * 输出  ：
 * 举例  ：UART_ClearITPendingBit( COM1, UART_IT_FOEIEN );
 * 注意  ：

*****************************************************************/

void UART2_ClearITPendingBit( uint8_t UARTx, uint8_t UART2_IT )
{	
	uint8_t ch, addr, data1, data2;
	uint8_t data, IFR;
	
	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  	assert_param(IS_UART2_CLEAR_IT(UART2_IT));
	
	ch = UARTx;
	addr = SIFR ;
	data1 = ch | addr;
	IFR = Uart22_Receive_Data( data1 );
	if(  UART2_IT == UART2_IT_FOEIEN  )  	data2 = IFR & 0XBF;   //清除子串口FIFO数据错误中断标志
	else if( UART2_IT == UART2_IT_TRIEN )  	data2 = IFR & 0Xfd;   //清除子串口发送FIFO触点中断标志位
	else									data2 = IFR & 0Xfe;   //清除子串口接收FIFO触点中断标志位	
	data1 = data1 | 0x80 ;
	Uart22_Send_Data( data1, data2 );
}

/***************************************************************
 * 函数名： UART_Get_Num_TXFIFO;
 * 功能  : 查询发送FIFO的字节数
 * 输入  : uint8_t UARTx
 * 输出  ：uint8_t num;
 * 举例  ：UART_Get_Num_TXFIFO( COM1 )
 * 注意  ：

*****************************************************************/

uint8_t UART2_Get_Num_TXFIFO( uint8_t UARTx )
{
	uint8_t ch, addr, data1, data;
	uint8_t num;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFSR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data >> 4;
	num = ( (uint8_t) data );
	return num;
}

/***************************************************************
 * 函数名： UART_Get_Num_RXFIFO;
 * 功能  : 查询接收FIFO的字节数
 * 输入  : uint8_t UARTx
 * 输出  ：uint8_t num;
 * 举例  ：UART_Get_Num_RXFIFO( COM1 )
 * 注意  ：

*****************************************************************/

uint8_t UART2_Get_Num_RXFIFO( uint8_t UARTx )
{
	uint8_t ch, addr, data1, data;
	uint8_t num;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFSR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data & 0X0F;
	num = ( (uint8_t) data );
	return num;
}

/***************************************************************
 * 函数名： UART_Clear_TXFIFO
 * 功能  : 清空发送FIFO
 * 输入  : uint8_t UARTx
 * 输出  ：无
 * 举例  ：UART_Clear_TXFIFO( COM1 )
 * 注意  ：

*****************************************************************/

void UART2_Clear_TXFIFO( uint8_t UARTx )
{
	uint8_t ch, addr, data1, data, data2 ;

	/* Check the parameters */
	assert_param(IS_UART2_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFOCR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data | 0x02;
	data1 = 0x80 | ch | addr ;
	data2 = data;
	Uart22_Send_Data( data1, data2 );                                //清空发送FIFO中的所有数据
	while( UART2_GetFlagStatus(  UARTx, UART2_FLAG_TFEM ) == RESET );//判断子串口发送FIFO是否为空，即是否清除完毕	
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data & 0xfd;                                              //不清除发送FIFO中的寄存器
	data1 = 0x80 | ch | addr ; 
	data2 = data; 
	Uart22_Send_Data( data1, data2 );
}

/***************************************************************
 * 函数名： UART_Clear_RXFIFO
 * 功能  : 清空接收FIFO
 * 输入  : uint8_t UARTx
 * 输出  ：无
 * 举例  ：UART_Clear_RXFIFO( COM1 )
 * 注意  ：

*****************************************************************/

void UART2_Clear_RXFIFO( uint8_t UARTx )
{
	uint8_t ch, addr, data1, data2, data;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
	ch = UARTx;
	addr = SFOCR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data | 0x01;
	data1 = 0x80 | ch | addr ;                                       //清除接收FIFO寄存器的数据
	data2 = data;
	Uart22_Send_Data( data1, data2 );
	while( UART2_GetFlagStatus(  UARTx, UART2_FLAG_RFEM ) == RESET );//子串口接收FIFO空
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data & 0xfE;
	data1 = 0x80 | ch | addr ;                                       //不清除接收FIFO中的数据
	data2 = data;
	Uart22_Send_Data( data1, data2);
}

/***************************************************************
 * 函数名： Save_Uart_Data
 * 功能  : 保存串口数据
 * 输入  : u8 data
 * 输出  ：无

*****************************************************************/
void Save_Uart_Data(u8 data)
{ 
	UART_SAVE_BUFFER[0]=data;
}

/***************************************************************
 * 函数名：Uart_Master_Config;
 * 功能  ：配置主串口参数
 * 输入  : 无
 * 输出  ：无
 * 举例  ：
 * 注意  ：无
*****************************************************************/

void Uart_Master_Config(void)
{
	UART22_Base_InitTypeDef UART22_InitStruct;	
	UART22_InitStruct.UART22_BaudRate = UART22_BaudRate_1;
  	UART22_InitStruct.UART22_WordLength = UART22_WordLength_8b;
	UART22_InitStruct.UART22_StopBits = UART22_StopBits_1;
  	UART22_InitStruct.UART22_Parity = UART22_Parity_0;
	UART22_Base_Init(  &UART22_InitStruct );
//	  delay_ms(10);
//	  u2_printf( "\r\n配置主串口,波特率115200,数据位：8位,无奇偶校验\r\n" );
    usart6_init(115200);
}

/***************************************************************
 * 函数名：UART_Slave1_Config;
 * 功能  ：配置子串口1参数
 * 输入  : 无
 * 输出  ：无
 * 举例  ：
 * 注意  ：无

*****************************************************************/
void UART_Slave1_Config(void)
{
	UART2_Base_InitTypeDef UART2_InitStruct;
	UART2_FIFO_InitTypeDef UART2_FIFO_InitStruct;
    UART2_StructInit(&UART2_InitStruct);	
	
	UART2_Base_Init( COM1, &UART2_InitStruct );	
    UART2_FIFO_StructInit(&UART2_FIFO_InitStruct);
	UART2_FIFO_Init( COM1, &UART2_FIFO_InitStruct );
	
	UART2_Clear_TXFIFO( COM1 );
	UART2_Clear_RXFIFO( COM1 );
	UART2_ITConfig( COM1, UART2_IT_RFIEN, SET );
		
	UART2_Cmd( COM1, SET );
		
//		delay_ms(100);
//		u2_printf( "\r\n配置串口1;波特率：14400(不知为何，实际值为230400)；数据位：8位；无奇偶校验\r\n" );
//		delay_ms(100);
}


/***************************************************************
 * 函数名：UART_Slave2_Config;
 * 功能  ：配置子串口2参数
 * 输入  : 无
 * 输出  ：无
 * 举例  ：
 * 注意  ：无

*****************************************************************/
void UART_Slave2_Config(void)
{
	UART2_Base_InitTypeDef UART2_InitStruct;
	UART2_FIFO_InitTypeDef UART2_FIFO_InitStruct;
    UART2_StructInit(&UART2_InitStruct);
	UART2_Base_Init( COM2, &UART2_InitStruct );
	
    UART2_FIFO_StructInit(&UART2_FIFO_InitStruct);	
	UART2_FIFO_Init( COM2, &UART2_FIFO_InitStruct );
	UART2_Clear_TXFIFO( COM2 );
	UART2_Clear_RXFIFO( COM2 );
	UART2_ITConfig( COM2, UART2_IT_RFIEN, SET );
	UART2_Cmd( COM2, SET );
//		delay_ms(100);
//		u2_printf( "\r\n 配置串口2;波特率：14400；数据位：8位；无奇偶校验\r\n" );
//		delay_ms(100);
}

/***************************************************************
 * 函数名：UART_Slave3_Config;
 * 功能  ：配置子串口3参数
 * 输入  : 无
 * 输出  ：无
 * 举例  ：
 * 注意  ：无

*****************************************************************/
void UART_Slave3_Config(void)
{
	UART2_Base_InitTypeDef UART2_InitStruct;
	UART2_FIFO_InitTypeDef UART2_FIFO_InitStruct;
    UART2_StructInit(&UART2_InitStruct);	
	UART2_Base_Init( COM3, &UART2_InitStruct );
    UART2_FIFO_StructInit(&UART2_FIFO_InitStruct);
	UART2_FIFO_Init( COM3, &UART2_FIFO_InitStruct );
	UART2_Clear_TXFIFO( COM3 );
	UART2_Clear_RXFIFO( COM3 );
	UART2_ITConfig( COM3, UART2_IT_RFIEN, SET );
	UART2_Cmd( COM3, SET );
//		delay_ms(100);
//		u2_printf( "\r\n 配置串口3;波特率：14400；数据位：8位；偶校验\r\n" );
//		delay_ms(100);
}

/***************************************************************
 * 函数名：UART_Slave4_Config;
 * 功能  ：配置子串口4参数
 * 输入  : 无
 * 输出  ：无
 * 举例  ：
 * 注意  ：无

*****************************************************************/
void UART_Slave4_Config(void)
{
	UART2_Base_InitTypeDef UART2_InitStruct;
	UART2_FIFO_InitTypeDef UART2_FIFO_InitStruct;
    UART2_StructInit(&UART2_InitStruct);
	UART2_Base_Init( COM4, &UART2_InitStruct );	
    UART2_FIFO_StructInit(&UART2_FIFO_InitStruct);
	UART2_FIFO_Init( COM4, &UART2_FIFO_InitStruct );

	UART2_Clear_TXFIFO( COM4 );

	UART2_Clear_RXFIFO( COM4 );

	UART2_ITConfig( COM4, UART2_IT_RFIEN, SET );
	UART2_Cmd( COM4, SET );
//		delay_ms(100);
//		u2_printf( "\r\n 配置串口4;波特率：14400；数据位：8位；偶校验\r\n" );
//		delay_ms(100);
}

/***************************************************************
 * 函数名：UART_EX_Config;
 * 功能  ：配置串口参数
 * 输入  : 无
 * 输出  ：无
 * 举例  ：
 * 注意  ：无

*****************************************************************/
void UART_EX_Config(void)
{
	Uart_Master_Config();
	delay_ms(10);
	UART_Slave1_Config();
	delay_ms(10);
	UART_Slave2_Config();
	delay_ms(10);
	UART_Slave3_Config();
	delay_ms(10);
	UART_Slave4_Config();	
}