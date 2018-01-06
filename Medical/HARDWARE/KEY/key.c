#include "key.h"
#include "delay.h"

/***********************************************************************
 函数名      ：KEY_Init(void)  
 函数功能    ：按键初始化函数
 输入        ：无
 输出        ：无
                           
************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;          //定义结构体变量GPIO_Initure
    __HAL_RCC_GPIOA_CLK_ENABLE();           //使能GPIOA时钟 
    __HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();           //开启GPIOE时钟
    __HAL_RCC_GPIOH_CLK_ENABLE();           //开启GPIOH时钟
    __HAL_RCC_GPIOG_CLK_ENABLE();           //开启GPIOG时钟
    __HAL_RCC_GPIOI_CLK_ENABLE();           //开启GPIOI时钟
		
    GPIO_Initure.Pin=GPIO_PIN_0;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);	

//    GPIO_Initure.Pin=GPIO_PIN_12;            //PA12
//    GPIO_Initure.Mode=GPIO_MODE_INPUT;       //输入
//    GPIO_Initure.Pull=GPIO_PULLUP;           //上拉
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;      //高速
//    HAL_GPIO_Init(GPIOA,&GPIO_Initure);		
	
    GPIO_Initure.Pin=GPIO_PIN_5; 
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP; 
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_1|GPIO_PIN_4; //PI4
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);	
}

/***********************************************************************
 函数名      ：KEY_Scan()  
 函数功能    ：按键处理函数
 输入        ：mode:0,不支持连续按;1,支持连续按;
 输出        ：返回按键值:0，没有任何按键按下;2，WKUP按下 WK_UP
                           
************************************************************************/
u8 KEY_Scan(u8 mode)
{
    static u8 key_up=1;     //按键松开标志
    if(mode==1)key_up=1;    //mode=1支持连按
    if(key_up&&(WK_UP==1||Motor3_Tim==1||Motor3_Alm==1||Motor4_Tim==1||Motor4_Alm==1||Motor5_Tim==1||Motor5_Alm==1||Motor6_Tim==1||Motor6_Alm==1||Motor7_Tim==1))  
    {
        delay_us(5);       //按键消抖
        key_up=0;  
		if(WK_UP==1) return WKUP_PRES;                     //如果WK_UP按键按下，返回WKUP_PRES
		else if(Motor3_Tim==1) return Motor3_Tim_PRES;     //3号电机反馈报警
		else if(Motor3_Alm==1) return Motor3_Alm_PRES;     //3号电机故障报警
		else if(Motor4_Tim==1) return Motor4_Tim_PRES;     //4号电机反馈报警
		else if(Motor4_Alm==1) return Motor4_Alm_PRES;     //4号电机故障报警
		else if(Motor5_Tim==1) return Motor5_Tim_PRES;     //5号电机反馈报警
		else if(Motor5_Alm==1) return Motor5_Alm_PRES;     //5号电机故障报警
		else if(Motor6_Tim==1) return Motor6_Tim_PRES;     //6号电机反馈报警
		else if(Motor6_Alm==1) return Motor6_Alm_PRES;     //6号电机故障报警
		else if(Motor7_Tim==1) return Motor7_Tim_PRES;     //7号电机反馈报警
    }
	else if(WK_UP==0&&Motor3_Tim==0&&Motor3_Alm==0&&Motor4_Tim==0&&Motor4_Alm==0&&Motor5_Tim==0&&Motor5_Alm==0&&Motor6_Tim==0&&Motor6_Alm==0&&Motor7_Tim==0)key_up=1; 
    return 0;   //无按键按下
}

u8 KeyCheck(u8 num)
{
	u32 i=0,j=0,n;
	n=285000;
	switch(num)
	{
		case M3TIM:
			for(i=0;i<n;i++)
			{
				if(Motor3_Tim==1)
				{	j++;	}
			}
			break;	
		case M3ALM:
			for(i=0;i<n;i++)
			{
				if(Motor3_Alm==1)
				{	j++;	}
			}
			break;	
		case M4TIM:
			for(i=0;i<n;i++)
			{
				if(Motor4_Tim==1)
				{	j++;	}
			}
			break;	
		case M4ALM:
			for(i=0;i<n;i++)
			{
				if(Motor4_Alm==1)
				{	j++;	}
			}
			break;	
			case M5TIM:
			for(i=0;i<n;i++)
			{
				if(Motor5_Tim==1)
				{	j++;	}
			}
			break;	
		case M5ALM:
			for(i=0;i<n;i++)
			{
				if(Motor5_Alm==1)
				{	j++;	}
			}
			break;
		case M6TIM:
			for(i=0;i<n;i++)
			{
				if(Motor6_Tim==1)
				{	j++;	}
			}
			break;	
		case M6ALM:
			for(i=0;i<n;i++)
			{
				if(Motor6_Alm==1)
				{	j++;	}
			}
			break;	
			case M7TIM:
			for(i=0;i<n;i++)
			{
				if(Motor7_Tim==1)
				{	j++;	}
			}
			break;	
	}
		if(j<284000)
		{	j=0;
			return 0;	
		}
		else
		{	j=0;		
			return 1;
		}
}

void KeyCheckAll(void)
{
	if(KeyCheck(M3TIM))
	{u2_printf("M3TIM\r\n");}
	if(KeyCheck(M3ALM))
	{u2_printf("M3ALM\r\n");}
	if(KeyCheck(M4TIM))
	{u2_printf("M4TIM\r\n");}
	if(KeyCheck(M4ALM))
	{u2_printf("M4ALM\r\n");}
	if(KeyCheck(M5TIM))
	{u2_printf("M5TIM\r\n");}
	if(KeyCheck(M5ALM))
	{u2_printf("M5ALM\r\n");}
	if(KeyCheck(M6TIM))
	{u2_printf("M6TIM\r\n");}
	if(KeyCheck(M6ALM))
	{u2_printf("M6ALM\r\n");}
	if(KeyCheck(M7TIM))
	{u2_printf("M7TIM\r\n");}
	
}

