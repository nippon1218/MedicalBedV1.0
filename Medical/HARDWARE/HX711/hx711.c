#include "hx711.h"
#include "usart.h"
#include "delay.h"
#include "sys.h"

#define WeightN 10    //定义滤波时存放采样值对的队列长度

//u8 WeightN=10;
unsigned long Weight_Shiwu;  //测得的实物的重量
//int WeightN;

/***********************************************************************
 函数名      ：HX711_Init() 
 函数功能    ：初始化称重传感器HX711的IO口
 输入        ：无
 输出        ：无
                          
************************************************************************/
void HX711_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();		    //开启GPIOB时钟
	__HAL_RCC_GPIOH_CLK_ENABLE();		    //开启GPIOH时钟
	
    GPIO_Initure.Pin=GPIO_PIN_5;            //PB5
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出GPIO_MODE_OUTPUT_PP
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //初始化
	
	GPIO_Initure.Pin=GPIO_PIN_10;           //PH10
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_NOPULL;          //浮空
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //初始化
}


/***********************************************************************
 函数名      ：HX711_Read()   
 函数功能    ：称重函数
 输入        ：无
 输出        ：无 
 说明        ：4个量程为5kg的称重传感器，最大量程为20kg，
               AD采样最大输出值为8589933
                          
************************************************************************/
unsigned long HX711_Read(void)	       //增益128，可将采集的信号放大128倍
{
	unsigned long  val;                //存放采样数据
	unsigned char i; 
	
	HX711_DOUT_OUTPUT();               //DOUT设置为输出模式
  	HX711_DOUT_OUT=1;                  //DOUT置高	
  	delay_us(1);
    HX711_SCK_OUTPUT();                //SCK设置为输出模式
  	HX711_SCK_OUT=0;                   //使能AD(SCK置低)	
  	val=0;
  	HX711_DOUT_INPUT();                //DOUT设置为输入模式
	while(HX711_DOUT_IN);              //等待AD转换结束，否则开始读取,即等待HX711_DOUT_IN=0
    delay_us(1);
	for(i=0;i<24;i++)
	{ 
	  	HX711_SCK_OUT=1;               //SCK 置高(发送脉冲)
	  	val=val<<1; 
		delay_us(1);
	    HX711_SCK_OUT=0;               //SCK 置低
		HX711_DOUT_INPUT();            //设置为输入模式，检测引脚高低电平
	    if(HX711_DOUT_IN)              //DOUT=1
	    {
	        val++;
			delay_us(1);
	    }				
	} 
	HX711_SCK_OUT=1;
	val=val^0x800000;                  //第25个脉冲下降沿来时，转换数据
  	delay_us(2);
	HX711_SCK_OUT=0;
	delay_us(1);
	return(val);
}

/***********************************************************************
 函数名      ：Get_Weight()  
 函数功能    ：采集数据函数
 输入        ：无
 输出        ：无 
                          
************************************************************************/
unsigned long Get_Weight(void)
{   
	unsigned long HX711_Buffer = 0;
     
    Weight_Shiwu = 0;
	HX711_Buffer = HX711_Read();
	HX711_Buffer = HX711_Buffer/100;                                //便于后续计算
		 
	Weight_Shiwu = HX711_Buffer;                                    //获取实物的AD采样数据	 
	Weight_Shiwu = (unsigned int)((float)Weight_Shiwu /4.22+0.05); 	//计算实物的实际重量，4.22矫正值，可调，
																	//如果测出来的数据偏小，减小该数值，一般在4.0-5.0之间
//	Weight_Shiwu =(unsigned int)((18128-Weight_Shiwu )/0.6012);																//0.05为了四舍五入百分位
//	u2_printf("\r\nWeight_Shiwu=%d\r\n",Weight_Shiwu);	
    return(Weight_Shiwu);
} 


/***********************************************************************
 函数名      ：filter() 
 函数功能    ：数据处理函数：将传感器采集的数据进行处理
 输入        ：无
 输出        ：sum/(N - 2) 
                          
************************************************************************/
unsigned long filter(void)
{
	unsigned long temp =0;
	unsigned long t;
	unsigned long r;
	unsigned long s;
	unsigned long value_buf[WeightN];    //存放采集的N个数
	long sum=0;                    //存放采集数据的和
	
	for(t = 0;t<WeightN;t++)             //采集十个数据	
	{
		value_buf[t] =Get_Weight();
		delay_us(1);
	}
	for(r = 0;r < WeightN - 1;r++)      //将N个数据从小到大排序
	{
	 	for (s=0;s<WeightN-r;s++)            
	 	{
	 		if(value_buf[s] > value_buf[s+1])
	 		{
	 			temp = value_buf[s];
	 			value_buf[s]   = value_buf[s+1];
	 			value_buf[s+1] = temp;	
	  	}
	 	}
	}
	for(t = 3;t <WeightN -3 ;t++)     //提取中间四个数求本次采样平均值
	{
		sum += value_buf[t];
	}
	sum=(long)(sum/(WeightN- 6));
	sum =(long)((18158-sum )/0.6012);
	if(sum < 0)
		return -sum;
	else
		return sum;
}



