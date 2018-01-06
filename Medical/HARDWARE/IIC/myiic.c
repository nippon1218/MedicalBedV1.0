#include "myiic.h"
#include "delay.h"
	
/***********************************************************************
 函数名      ：IIC_Init(void) 
 函数功能    ：IIC初始化
 输入        ：无
 输出        ：无
                           
************************************************************************/
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;    
    __HAL_RCC_GPIOH_CLK_ENABLE();            //使能GPIOH时钟
    
    GPIO_Initure.Pin=GPIO_PIN_4|GPIO_PIN_5;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;   //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;           //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;      //快速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
    
    IIC_SDA=1;                               //IIC数据线
    IIC_SCL=1;                               //IIC时钟线  
}

/***********************************************************************
 函数名      ：IIC_Start(void) 
 函数功能    ：产生IIC起始信号
 输入        ：无
 输出        ：无
                           
************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();           //SDA线输出
	IIC_SDA=1;	         //数据线高电平  	  
	IIC_SCL=1;	         //时钟线高电平 
	delay_us(4);
 	IIC_SDA=0;           //START信号:当时钟线为高电平时，数据线从高到低变化
	delay_us(4);
	IIC_SCL=0;           //钳住I2C总线，准备发送或接收数据 
}	  

/***********************************************************************
 函数名      ：IIC_Stop(void) 
 函数功能    ：产生IIC停止信号
 输入        ：无
 输出        ：无
                           
************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();           //SDA线输出
	IIC_SCL=0;	         //时钟线高电平
	IIC_SDA=0;	         //数据线低电平 
 	delay_us(4);
	IIC_SCL=1;           //STOP信号:当时钟线为高电平，数据线从低到高变化
	IIC_SDA=1;           //发送I2C总线结束信号
	delay_us(4);							   	
}

/***********************************************************************
 函数名      ：IIC_Wait_Ack(void) 
 函数功能    ：等待应答信号到来
 输入        ：无
 输出        ：1，接收应答失败
               0，接收应答成功
                           
************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();                 //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)           //如果在规定时间内未收到应答信号，则接收应答失败
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;                //时钟输出0 	   
	return 0;  
} 

/***********************************************************************
 函数名      ：IIC_Ack(void) 
 函数功能    ：产生ACK应答
 输入        ：无
 输出        ：无
                           
************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;	         //时钟线低电平
	SDA_OUT();
	IIC_SDA=0;	         //数据线低电平
	delay_us(2);
	IIC_SCL=1;	         //时钟线高电平
	delay_us(2);
	IIC_SCL=0;	         //时钟线低电平
}

/***********************************************************************
 函数名      ：IIC_NAck(void) 
 函数功能    ：不产生ACK应答
 输入        ：无
 输出        ：无
                           
************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL=0;	         //时钟线低电平
	SDA_OUT();           //SDA设置为输出
	IIC_SDA=1;	         //数据线高电平
	delay_us(2);
	IIC_SCL=1;	         //时钟线高电平
	delay_us(2);
	IIC_SCL=0;	         //时钟线低电平
}					 				     
	
/***********************************************************************
 函数名      ：IIC_Send_Byte(u8 txd) 
 函数功能    ：IIC发送一个字节
 输入        ：txd：要发送的值
 输出        ：无
                           
************************************************************************/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	           //SDA设置为输出    
    IIC_SCL=0;             //拉低时钟线，开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);       //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
 
/***********************************************************************
 函数名      ：IIC_Read_Byte(unsigned char ack)
 函数功能    ：IIC读取一个字节
 输入        ：ack=1时，发送ACK，ack=0，发送nACK 
 输出        ：receive
                           
************************************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();                   //SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();            //发送nACK
    else
        IIC_Ack();             //发送ACK   
    return receive;
}


