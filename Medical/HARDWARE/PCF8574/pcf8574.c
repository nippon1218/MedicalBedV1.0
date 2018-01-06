#include "pcf8574.h"
#include "delay.h"

/***********************************************************************
 函数名      ：PCF8574_Init(void) 
 函数功能    ：PCF8574初始化函数
 输入        ：无
 输出        ：temp：1，接收应答失败；0，接收应答成功
                           
************************************************************************/
u8 PCF8574_Init(void)
{
    u8 temp=0;
    GPIO_InitTypeDef GPIO_Initure;          //定义结构体变量GPIO_Initure
    __HAL_RCC_GPIOB_CLK_ENABLE();           //使能GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_12;           //PB12
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //初始化
    IIC_Init();					            //IIC初始化 	
	//检查PCF8574是否在位
    IIC_Start();                            //产生IIC起始信号	 	   
		IIC_Send_Byte(PCF8574_ADDR);            //写地址			   
		temp=IIC_Wait_Ack();		            //等待应答,通过判断是否有ACK应答,来判断PCF8574的状态
    IIC_Stop();					            //产生一个停止条件
    PCF8574_WriteOneByte(0XFF);	            //默认情况下所有IO输出高电平
	return temp;                            //返回应答信号                         
}

/***********************************************************************
 函数名      ：PCF8574_ReadOneByte(void) 
 函数功能    ：读取PCF8574的8位IO口值
 输入        ：无
 输出        ：读到的数据
                           
************************************************************************/
u8 PCF8574_ReadOneByte(void)
{				  
	u8 temp=0;		  	    																 
    IIC_Start();                        //产生IIC起始信号  	 	   
	IIC_Send_Byte(PCF8574_ADDR|0X01);   //进入接收模式			   
	IIC_Wait_Ack();	                    //等待应答信号到来
    temp=IIC_Read_Byte(0);	            //读1个字节	   
    IIC_Stop();							//产生一个停止条件	    
	return temp;                        //返回读取到的值
}

/***********************************************************************
 函数名      ：PCF8574_WriteOneByte(u8 DataToWrite) 
 函数功能    ：向PCF8574写入8位IO值 
 输入        ：DataToWrite:要写入的数据
 输出        ：无
                           
************************************************************************/
void PCF8574_WriteOneByte(u8 DataToWrite)
{				   	  	    																 
    IIC_Start();                        //产生IIC起始信号	
    IIC_Send_Byte(PCF8574_ADDR|0X00);   //发送器件地址0X40,写数据 	 
	IIC_Wait_Ack();	                    //等待应答信号到来    										  		   
	IIC_Send_Byte(DataToWrite);    	 	//发送字节							   
	IIC_Wait_Ack();                     //等待应答信号到来     
    IIC_Stop();							//产生一个停止条件 	 
}

/***********************************************************************
 函数名      ：PCF8574_WriteBit(u8 bit,u8 sta) 
 函数功能    ：设置PCF8574某个IO的高低电平 
 输入        ：bit:要设置的IO编号,0~7
               sta:IO要置位的状态;0：低电平；1：高电平
 输出        ：无
                           
************************************************************************/
void PCF8574_WriteBit(u8 bit,u8 sta)
{
    u8 data;
    data=PCF8574_ReadOneByte(); //先读出原来的设置
    if(sta==0) data&=~(1<<bit); //将IO置0  
    else data|=1<<bit;          //将IO置1
    PCF8574_WriteOneByte(data); //写入新的数据
	
}

/***********************************************************************
 函数名      ：PCF8574_ReadBit(u8 bit)
 函数功能    ：读取PCF8574的某个IO的值 
 输入        ：bit:要设置的IO编号,0~7
 输出        ：此IO口的值;0：低电平；1：高电平
                           
************************************************************************/
u8 PCF8574_ReadBit(u8 bit)
{
    u8 data;
    data=PCF8574_ReadOneByte(); //先读取这个8位IO的值 
    if(data&(1<<bit))return 1;
    else return 0;   
}  
    

void BeepRun(u8 num,u16 time)
{
	u8 i;
	for(i=0;i<num;i++)
	{
		PCF8574_WriteBit(BEEP_IO,0);delay_ms(time);	//控制蜂鸣器响
		PCF8574_WriteBit(BEEP_IO,1);delay_ms(time);	//控制蜂鸣器停			
	}
}



