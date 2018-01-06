#include "nand.h"
#include "delay.h"
#include "malloc.h"
#include "usart.h"

NAND_HandleTypeDef NAND_Handler;    //NAND FLASH句柄
nand_attriute nand_dev;             //nand重要参数结构体


/***********************************************************************
 函数名      ：HAL_NAND_MspInit(NAND_HandleTypeDef *hnand) 
 函数功能    ：NAND FALSH底层驱动,引脚配置，时钟使能
               此函数会被HAL_NAND_Init()调用
 输入        ：hnand
 输出        ：无
                           
************************************************************************/
void HAL_NAND_MspInit(NAND_HandleTypeDef *hnand)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_FMC_CLK_ENABLE();                //使能FMC时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();              //使能GPIOD时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();              //使能GPIOE时钟
    __HAL_RCC_GPIOG_CLK_ENABLE();              //使能GPIOG时钟
    
	//初始化PD6 R/B引脚
	GPIO_Initure.Pin=GPIO_PIN_6;
    GPIO_Initure.Mode=GPIO_MODE_INPUT;          //输入
    GPIO_Initure.Pull=GPIO_PULLUP;    			//上拉          
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	   
	//初始化PG9 NCE3引脚,FMC 总线的片选信号 3，为 NAND 片选信号
    GPIO_Initure.Pin=GPIO_PIN_9;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //复用
    GPIO_Initure.Pull=GPIO_NOPULL;    			//无上下拉          
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速
	GPIO_Initure.Alternate=GPIO_AF12_FMC;       //复用为FMC
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);  
	
    //初始化PD0,1,4,5,11,12,14,15
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|\
                     GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_Initure.Pull=GPIO_NOPULL;               //无上下拉              
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);

    //初始化PE7,8,9,10
    GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}


/*********************************************************************
*函数名       ：NAND_Init(void)
*函数功能     ：初始化NAND FLASH,配置相关控制参数和 FMC 时序
*输入         ：无
*输出         ：无

***********************************************************************/
u8 NAND_Init(void)
{
    FMC_NAND_PCC_TimingTypeDef ComSpaceTiming,AttSpaceTiming;
    
    //设置 NAND FLASH 的控制参数	
    NAND_Handler.Instance=FMC_NAND_DEVICE;
    NAND_Handler.Init.NandBank=FMC_NAND_BANK3;                          //NAND挂在BANK3上
    NAND_Handler.Init.Waitfeature=FMC_NAND_PCC_WAIT_FEATURE_DISABLE;    //关闭等待特性
    NAND_Handler.Init.MemoryDataWidth=FMC_NAND_PCC_MEM_BUS_WIDTH_8;     //数据总线宽度:8位数据宽度
    NAND_Handler.Init.EccComputation=FMC_NAND_ECC_DISABLE;              //不使用ECC
    NAND_Handler.Init.ECCPageSize=FMC_NAND_ECC_PAGE_SIZE_2048BYTE;      //ECC页大小为2k
    NAND_Handler.Init.TCLRSetupTime=0;                                  //设置TCLR(tCLR=CLE到RE的延时)=(TCLR+TSET+2)*THCLK,THCLK=1/180M=5.5ns
    NAND_Handler.Init.TARSetupTime=1;                                   //设置TAR(tAR=ALE到RE的延时)=(TAR+TSET+2)*THCLK,THCLK=1/180M=5.5n。   
   
	//ComSpace_Timing 用来设置 NAND 通用存储器空间时序
    ComSpaceTiming.SetupTime=2;         //建立时间
    ComSpaceTiming.WaitSetupTime=3;     //等待时间
    ComSpaceTiming.HoldSetupTime=2;     //保持时间
    ComSpaceTiming.HiZSetupTime=1;      //高阻态时间
   
	//AttSpace_Timing 用来设置 NAND 特性存储器空间时序
    AttSpaceTiming.SetupTime=2;         //建立时间
    AttSpaceTiming.WaitSetupTime=3;     //等待时间
    AttSpaceTiming.HoldSetupTime=2;     //保持时间
    AttSpaceTiming.HiZSetupTime=1;      //高阻态时间
    
    HAL_NAND_Init(&NAND_Handler,&ComSpaceTiming,&AttSpaceTiming); 
    NAND_Reset();       		        //复位NAND
    delay_ms(100);
    nand_dev.id=NAND_ReadID();	        //读取ID
	NAND_ModeSet(4);			        //设置为MODE4,高速模式 
  
    if(nand_dev.id==MT29F4G08ABADA)//NAND为MT29F4G08ABADA
    {
        nand_dev.page_totalsize=2112;	//nand一个page的总大小（包括spare区）
        nand_dev.page_mainsize=2048; 	//nand一个page的有效数据区大小
        nand_dev.page_sparesize=64;		//nand一个page的spare区大小
        nand_dev.block_pagenum=64;		//nand一个block所包含的page数目
        nand_dev.plane_blocknum=2048;	//nand一个plane所包含的block数目
        nand_dev.block_totalnum=4096; 	//nand的总block数目
    }else return 1;	//错误，返回
    return 0;
}

/*********************************************************************
*函数名       ：NAND_ModeSet(u8 mode)
*函数功能     ：读取NAND FLASH的ID
*输入         ：mode
*输出         ：0,成功;
                其他,失败

***********************************************************************/
u8 NAND_ModeSet(u8 mode)
{   
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_FEATURE;//发送设置特性命令
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=0X01;		//地址为0X01,设置mode
 	*(vu8*)NAND_ADDRESS=mode;					//P1参数,设置mode
	*(vu8*)NAND_ADDRESS=0;
	*(vu8*)NAND_ADDRESS=0;
	*(vu8*)NAND_ADDRESS=0; 
    if(NAND_WaitForReady()==NSTA_READY)return 0;//成功
    else return 1;								//失败
}

/*********************************************************************
*函数名       ：NAND_ReadID(void)
*函数功能     ：读取NAND FLASH的ID
*输入         ：无
*输出         ：0,成功;
                其他,失败

***********************************************************************/
u32 NAND_ReadID(void)
{
    u8 deviceid[5]; 
    u32 id;  
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_READID;   //发送读取ID命令
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=0X00;
	//ID一共有5个字节
    deviceid[0]=*(vu8*)NAND_ADDRESS;      
    deviceid[1]=*(vu8*)NAND_ADDRESS;  
    deviceid[2]=*(vu8*)NAND_ADDRESS; 
    deviceid[3]=*(vu8*)NAND_ADDRESS; 
    deviceid[4]=*(vu8*)NAND_ADDRESS;  
    //镁光的NAND FLASH的ID一共5个字节，但是为了方便我们只取4个字节组成一个32位的ID值
    //根据NAND FLASH的数据手册，只要是镁光的NAND FLASH，那么一个字节ID的第一个字节都是0X2C
    //所以我们就可以抛弃这个0X2C，只取后面四字节的ID值。
    id=((u32)deviceid[1])<<24|((u32)deviceid[2])<<16|((u32)deviceid[3])<<8|deviceid[4];
    return id;
}  

/*********************************************************************
*函数名       ：NAND_ReadStatus(void)
*函数功能     ：读NAND状态
*输入         ：无
*输出         ：NAND状态值
				bit0:0,成功;1,错误(编程/擦除/READ)
				bit6:0,Busy;1,Ready

***********************************************************************/
u8 NAND_ReadStatus(void)
{
    vu8 data=0; 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_READSTA;   //发送读状态命令
	data++;data++;data++;data++;data++;	           //加延时,防止-O2优化,导致的错误.
 	data=*(vu8*)NAND_ADDRESS;			           //读取状态值
    return data;
}

/*********************************************************************
*函数名       ：NAND_WaitForReady(void)
*函数功能     ：等待NAND准备好
*输入         ：无
*输出         ：NSTA_TIMEOUT 等待超时了
				NSTA_READY    已经准备好

***********************************************************************/
u8 NAND_WaitForReady(void)
{
    u8 status=0;
    vu32 time=0; 
	while(1)						            //等待ready
	{
		status=NAND_ReadStatus();	            //获取状态值
		if(status&NSTA_READY)break;
		time++;
		if(time>=0X1FFFF)return NSTA_TIMEOUT;   //超时
	}  
    return NSTA_READY;                          //准备好
}  

/*********************************************************************
*函数名       ：NAND_Reset(void)
*函数功能     ：复位NAND
*输入         ：无
*输出         ：0,成功;
				其他,失败

***********************************************************************/
u8 NAND_Reset(void)
{ 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_RESET;	 //复位NAND
    if(NAND_WaitForReady()==NSTA_READY)return 0; //复位成功
    else return 1;								 //复位失败
} 

/*********************************************************************
*函数名       ：NAND_WaitRB(void)
*函数功能     ：等待RB信号为某个电平
*输入         ：rb:0,等待RB==0；1,等待RB==1
*输出         ：0,成功;
				其他,失败

***********************************************************************/
u8 NAND_WaitRB(vu8 rb)
{
    vu16 time=0;  
	while(time<10000)
	{
		time++;
		if(NAND_RB==rb)return 0;
	}
	return 1;
}

/*********************************************************************
*函数名       ：NAND_Delay(vu32 i)
*函数功能     ：NAND延时
*输入         ：i
*输出         ：无

***********************************************************************/
void NAND_Delay(vu32 i)
{
	while(i>0)i--;
}

/*********************************************************************
*函数名       ：NAND_ReadPage()
*函数功能     ：读取NAND Flash的指定页指定列的数据
*输入         ：BlockNum:要读取的块地址，范围：0~（block_totalnum-1）=4095
				PageNum:要读取的页地址,范围:0~(block_pagenum)=63
				ColNum:要读取的列开始地址(也就是页内地址),范围:0~(page_totalsize-1)=2112-1
				NumByteToRead:读取字节数(不能跨页读)
*输出         ：读取到的值value

***********************************************************************/
u32 NAND_ReadPage(u32 BlockNum,u32 PageNum,u16 ColNum,u16 NumByteToRead)
{
    vu16 i=0;
	u8 res=0;
	u32 value=0;                                      //保存读取的数据
	BlockNum<<=6;
     *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_AREA_A;      //READ PAGE命令
    //发送地址（分五次发送）
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)ColNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(ColNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum|BlockNum);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>16);
	
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_AREA_TRUE1;    //READ PAGE命令
	
	res=NAND_WaitRB(0);			               //等待RB=0 
    if(res)return NSTA_TIMEOUT;	               //超时退出
	
    //下面2行代码是真正判断NAND是否准备好的
	res=NAND_WaitRB(1);			               //等待RB=1 
    if(res)return NSTA_TIMEOUT;                //超时退出
	if(NumByteToRead%NAND_ECC_SECTOR_SIZE)     //不是NAND_ECC_SECTOR_SIZE的整数倍，不进行ECC校验
	{ 
		//读取NAND FLASH中的值
		for(i=0;i<NumByteToRead;i++)           //NumByteToRead<4,最多读取32位数据
		{
			value|= ((*(vu8*)NAND_ADDRESS)<<(i*8));
		}		
	}
		if(NAND_WaitForReady()==NSTA_READY) 	
		{ return value;	}                      //成功并将读取到的值返回 	
}

/*********************************************************************
*函数名       ：NAND_WritePage()
*函数功能     ：在NAND一页中写入指定个字节的数据
*输入         ：BlockNum:要写入的块地址，范围：0~（block_totalnum-1）=4095
				PageNum:要写入的页地址,范围:0~(block_pagenum)=63
				ColNum:要写入的列开始地址(也就是页内地址),范围:0~(page_totalsize-1)=2112-1
				dat:要写入的数据
				NumByteToWrite:要写入的字节数，该值不能超过该页剩余字节数！！！
*输出         ：0,成功 
				其他,错误代码

***********************************************************************/
u32 NAND_WritePage(u32 BlockNum,u32 PageNum,u16 ColNum,u32 dat,u16 NumByteToRead)
{
    vu16 i=0;  
	BlockNum<<=6;
	*(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_WRITE0;    //WRITE PAGE命令，分两次发送 
    //发送地址
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)ColNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(ColNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum|BlockNum);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>16);
	
	NAND_Delay(30);                              //等待tADL   
	if(NumByteToRead%NAND_ECC_SECTOR_SIZE)       //不是NAND_ECC_SECTOR_SIZE的整数倍，不进行ECC校验
	{ 
		
		for(i=0;i<NumByteToRead;i++)             //读取NAND FLASH中的值
		{
			*(vu8*)NAND_ADDRESS= (vu8) (dat>>(i*8));
		}
	}
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_WRITE_TURE1;        //WRITE PAGE命令
    if(NAND_WaitForReady()!=NSTA_READY)return NSTA_ERROR;   //失败
    return 0;                                               //成功   
}

/*********************************************************************
*函数名       ：NAND_EraseBlock()
*函数功能     ：擦除一个块
*输入         ：BlockNum:要擦除的BLOCK编号,范围:0-(block_totalnum-1)0~4095
*输出         ：0,擦除成功
				其他,擦除失败

***********************************************************************/
u8 NAND_EraseBlock(u32 BlockNum)
{
	if(nand_dev.id==MT29F16G08ABABA)BlockNum<<=7;      	    //将块地址转换为页地址
    else if(nand_dev.id==MT29F4G08ABADA)BlockNum<<=6;
	
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_ERASE0;              //ERASE BLOCK 命令，分两次发送 
    //发送块地址
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)BlockNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>16);
	
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_ERASE1;              //ERASE BLOCK 命令
	
	if(NAND_WaitForReady()!=NSTA_READY)return NSTA_ERROR;    //失败
    return 0;	                                             //成功   
} 

/*********************************************************************
*函数名       ：NAND_EraseChip()
*函数功能     ：全片擦除NAND FLASH
*输入         ：无
*输出         ：无

***********************************************************************/
void NAND_EraseChip(void)
{
    u8 status;
    u16 i=0;
    for(i=0;i<nand_dev.block_totalnum;i++)                                   //循环擦除所有的块
    {
        status=NAND_EraseBlock(i);
        if(status)printf("Erase %d block fail!!，错误码为%d\r\n",i,status);   //擦除失败
    }
}

/*********************************************************************
*函数名       ：NAND_ECC_Get_OE()
*函数功能     ：获取ECC的奇数位/偶数位
*输入         ：oe:0,偶数位；1,奇数位
				eccval:输入的ecc值
*输出         ：计算后的ecc值(最多16位)

***********************************************************************/
u16 NAND_ECC_Get_OE(u8 oe,u32 eccval)
{
	u8 i;
	u16 ecctemp=0;
	for(i=0;i<24;i++)
	{
		if((i%2)==oe)
		{
			if((eccval>>i)&0X01)ecctemp+=1<<(i>>1); 
		}
	}
	return ecctemp;
} 

/*********************************************************************
*函数名       ：NAND_ECC_Correction()
*函数功能     ：ECC校正函数
*输入         ：eccrd:读取出来,原来保存的ECC值
				ecccl:读取数据时,硬件计算的ECC值
*输出         ：0,错误已修正
				其他,ECC错误(有大于2个bit的错误,无法恢复)

***********************************************************************/
u8 NAND_ECC_Correction(u8* data_buf,u32 eccrd,u32 ecccl)
{
	u16 eccrdo,eccrde,eccclo,ecccle;
	u16 eccchk=0;
	u16 errorpos=0; 
	u32 bytepos=0;  
	eccrdo=NAND_ECC_Get_OE(1,eccrd);	//获取eccrd的奇数位
	eccrde=NAND_ECC_Get_OE(0,eccrd);	//获取eccrd的偶数位
	eccclo=NAND_ECC_Get_OE(1,ecccl);	//获取ecccl的奇数位
	ecccle=NAND_ECC_Get_OE(0,ecccl); 	//获取ecccl的偶数位
	eccchk=eccrdo^eccrde^eccclo^ecccle;
	if(eccchk==0XFFF)	               //全1,说明只有1bit ECC错误
	{
		errorpos=eccrdo^eccclo; 
		printf("errorpos:%d\r\n",errorpos); 
		bytepos=errorpos/8; 
		data_buf[bytepos]^=1<<(errorpos%8);
	}else				               //不是全1,说明至少有2bit ECC错误,无法修复
	{
		printf("2bit ecc error or more\r\n");
		return 1;
	} 
	return 0;
}

