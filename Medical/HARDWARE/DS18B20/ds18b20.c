#include "ds18b20.h"
#include "delay.h"

/***********************************************************************
 函数名      ：DS18B20_Rst(void)   
 函数功能    ：复位DS18B20
 输入        ：无
 输出        ：无  
                          
***********************************************************************/
void DS18B20_Rst(void)	   
{                 
	DS18B20_IO_OUT();   //设置为输出
	DS18B20_DQ_OUT=0;   //拉低DQ
	delay_us(750);      //拉低750us
	DS18B20_DQ_OUT=1;   //DQ=1 
	delay_us(15);       //15US
}


/***********************************************************************
 函数名      ：DS18B20_Check(void)   
 函数功能    ：等待DS18B20的回应
 输入        ：无
 输出        ：返回1:未检测到DS18B20的存在 
			  返回0:存在  
                          
***********************************************************************/
u8 DS18B20_Check(void) 	   
{   
	u8 retry=0;
	DS18B20_IO_IN();    //设置为输入
    while (DS18B20_DQ_IN&&retry<200)
	{
		retry++;
		delay_us(1);
	};	 
	if(retry>=200)return 1;
	else retry=0;
    while (!DS18B20_DQ_IN&&retry<240)
	{
		retry++;
		delay_us(1);
	};
	if(retry>=240)return 1;	    
	return 0;
}

/***********************************************************************
 函数名      ：DS18B20_Read_Bit(void)   
 函数功能    ：从DS18B20读取一个位
 输入        ：无
 输出        ：返回值：1/0
                          
***********************************************************************/
u8 DS18B20_Read_Bit(void) 
{
	u8 data;
	DS18B20_IO_OUT();   //设置为输出
	DS18B20_DQ_OUT=0; 
	delay_us(2);
	DS18B20_DQ_OUT=1; 
	DS18B20_IO_IN();    //设置为输入
	delay_us(12);
	if(DS18B20_DQ_IN)data=1;
	else data=0;	 
	delay_us(50);           
	return data;
}


/***********************************************************************
 函数名      ：DS18B20_Read_Byte(void)   
 函数功能    ：从DS18B20读取一个字节
 输入        ：无
 输出        ：读到的数据
                          
***********************************************************************/
u8 DS18B20_Read_Byte(void)   
{        
	u8 i,j,dat;
	dat=0;
	for (i=1;i<=8;i++) 
	{
        j=DS18B20_Read_Bit();
        dat=(j<<7)|(dat>>1);
    }						    
	return dat;
}

/***********************************************************************
 函数名      ：DS18B20_Write_Byte(u8 dat)   
 函数功能    ：写一个字节到DS18B20
 输入        ：dat：要写入的字节
 输出        ：无
                          
***********************************************************************/
void DS18B20_Write_Byte(u8 dat)     
 {             
    u8 j;
    u8 testb;
    DS18B20_IO_OUT();         //设置为输出
    for (j=1;j<=8;j++) 
	{
        testb=dat&0x01;
        dat=dat>>1;
        if(testb)            // 写1
        {
            DS18B20_DQ_OUT=0;
            delay_us(2);                            
            DS18B20_DQ_OUT=1;
            delay_us(60);             
        }
        else                 //写0
        {
            DS18B20_DQ_OUT=0;
            delay_us(60);             
            DS18B20_DQ_OUT=1;
            delay_us(2);                          
        }
    }
}
 
/***********************************************************************
 函数名      ：DS18B20_Start(void)  
 函数功能    ：开始温度转换
 输入        ：无
 输出        ：无
                          
***********************************************************************/
void DS18B20_Start(void)
{   						               
    DS18B20_Rst();	              //复位DS18B20
    DS18B20_Check();	          //等待DS18B20的回应 
    DS18B20_Write_Byte(0xcc);     //转换
    DS18B20_Write_Byte(0x44);     
}

/***********************************************************************
 函数名      ：DS18B20_Init(void)  
 函数功能    ：初始化DS18B20的IO口 DQ 同时检测DS的存在
 输入        ：无
 输出        ：返回1:不存在
			   返回0:存在 
                          
***********************************************************************/
u8 DS18B20_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_8;            //PH8
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //初始化
 
	DS18B20_Rst();	                        //复位DS18B20
	return DS18B20_Check();
}

/***********************************************************************
 函数名      ：DS18B20_Get_Temp(void)  
 函数功能    ：从ds18b20得到温度值
 输入        ：无
 输出        ：精度：0.1C
               返回值：温度值 （-550~1250） 
                          
***********************************************************************/
short DS18B20_Get_Temp(void)
{
    u8 temp;
    u8 TL,TH;
    short tem;
    DS18B20_Start ();           //开始转换
    DS18B20_Rst();	            //复位DS18B20
    DS18B20_Check();	 
    DS18B20_Write_Byte(0xcc);   // 转换
    DS18B20_Write_Byte(0xbe);  	    
    TL=DS18B20_Read_Byte();     // 低8位   
    TH=DS18B20_Read_Byte();     // 高8位 
    if(TH>7)
    {
        TH=~TH;
        TL=~TL; 
        temp=0;                 //温度为负  
    }else temp=1;               //温度为正	  	  
    tem=TH;                     //获得高八位
    tem<<=8;    
    tem+=TL;                    //获得低八位
    tem=(double)tem*0.625;      //转换     
	if(temp)return tem;         //返回温度值
	else return -tem;    
}

