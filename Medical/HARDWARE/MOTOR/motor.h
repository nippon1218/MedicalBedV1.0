#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

/***************功能函数电机方向口*********************/

#define DIR3 PDout(7)       //3号电机方向信号线     
#define DIR4 PDout(3)       //4号电机方向信号线
#define DIR5 PDout(2)       //5号电机方向信号线 

#define DIR6 PCout(12)      //6号电机方向信号线 
#define DIR6_1 PAout(5)     //6坐便袋收紧电机方向信号线 
#define DIR6_2 PHout(13)    //6坐便袋收紧电机推杆驱动信号线 
#define PWM6_2 PCout(4)     //6坐便袋收紧电机推杆驱动脉冲线  

#define RELAY6 PHout(12)    //控制坐便袋收紧电机通断的继电器
#define DIR7 PAout(6)       //7号电机方向信号线 
                            //2号电机方向信号线
#define DIR2_DOWN PIout(8)  //下曲腿信号线
#define DIR2_UP PGout(10)   //上曲腿信号线

#define DIR1_DOWN PIout(3)  //支背下行信号线
#define DIR1_UP   PIout(7)  //支背上行信号线


/***************吊挂电机方向口*********************/

#define HANG_DIR1 PBout(6)   //1号吊挂电机方向信号线 
#define HANG_DIR2 PBout(7)   //2号吊挂电机方向信号线 
#define HANG_DIR3 PBout(8)   //3号吊挂电机方向信号线 
#define HANG_DIR4 PBout(9)   //4号吊挂电机方向信号线 

/***************吊挂电机脉冲口*********************/

#define HANG_PWM1 PHout(11)  //1号吊挂电机脉冲信号线 
#define HANG_PWM2 PGout(3)   //2号吊挂电机脉冲信号线  
#define HANG_PWM3 PDout(13)  //3号吊挂电机脉冲信号线 
#define HANG_PWM4 PGout(6)   //4号吊挂电机脉冲信号线 

/***************吊挂电机方向口*********************/
#define DG_Relay				PAout(12)		//吊挂继电器
/**************************************************************
                   功能函数电机驱动函数
                   arr：自动重装值
				   psc：时钟预分频数
***************************************************************/

void Motor_Dir_Init(void);                     //电机方向口初始化函数

void Motor_3_START(u16 arr,u16 psc);           //背部电机运行函数
void Motor_4_START(u16 arr,u16 psc);           //腰部电机运行函数
void Motor_4_Compensate(u8 dir,u16 time_arr,u16 arr,u16 psc);	//定时器补偿函数
void Motor_5_START(u16 arr,u16 psc);           //侧翻电机运行函数
void Motor_3_4_5_START_left(u16 arr,u16 psc);  //左/右翻电机运行函数
void Motor_3_4_5_START_right(u16 arr,u16 psc); //左/右翻电机运行函数
void Motor_6_START(u16 arr,u16 psc);           //坐便器电机运行函数
void Motor_6_1_START(u16 arr,u16 psc);         //坐便袋扎紧电机运行函数
void Motor_6_2_START(u8 dir,u32 pulse);        //坐便袋扎紧推杆驱动函数
void Motor_7_START(u16 arr,u16 psc);           //小桌子电机运行函数

void Push_Rod_Init(void);                      //电动推杆初始化函数
void Push_Rod_Start(u8 dir);                   //曲腿电机运行函数
void Push_Rod_Stop(void);                      //曲腿电机停止函数

void Motor_1_START(u8 dir);                    //支背电机运行函数
void Motor_1_STOP(void);                       //支背电机停止函数


void MotorStart(u8 MotorID,u8 dir,u16 arr);

void MotorStop(u8 MotorID);

void Motor345Start(u8 dir,u16 M3Arr,u16 M4Arr,u16 M5Arr);
void Motor345Stop();

void WashletRun(u8 dir,u16 TimArr,u16 TimPsc);
void BodyLeftRun(u8 dir,u16 M3Arr,u16 M4Arr,u16 TimArr);
void BodyRightRun(u8 dir,u16 M3Arr,u16 M4Arr,u16 TimArr);

void DeskRun(u8 dir,u16 HalfDist);	//小桌子程序
void DeskRun1(u8 dir,u16 Dist);	//小桌子程序



/***************功能函数电机停止函数*********************/

void Motor_3_STOP(void);                //背部电机停止函数
void Motor_4_STOP(void);                //腰部支背电机停止函数
void Motor_5_STOP(void);                //侧翻电机停止函数
void Motor_6_STOP(void);                //坐便器电机停止函数
void Motor_6_1_STOP(void);              //坐便袋扎紧电机停止函数
void Motor_7_STOP(void);                //小桌子电机停止函数
void Motor_3_4_5_STOP(void);            //左/右翻身电机停止函数
void Motor_All_Stop(void);              //所有电机停止


/**************************************************************
                   吊挂电机驱动函数
                   dir：控制吊挂电机速度
				   pulse：控制吊挂电机运行时间
***************************************************************/

void Hang_Init(void);                         //吊挂电机方向口、脉冲口初始化

void Auto_Hang_1(u16 dir,u32 pulse);          //吊挂电机1号电机驱动
void Auto_Hang_3(u16 dir,u32 pulse);          //吊挂电机3号电机驱动
void Auto_Hang_1_2(u16 dir,u32 pulse);        //吊挂1/2号电机同时驱动
void Auto_Hang_1_3(u16 dir,u32 pulse);        //吊挂1/3号电机同时驱动
void Auto_Hang_3_4(u16 dir,u32 pulse);        //吊挂3/4号电机同时驱动
void Auto_Hang_1_2_3_4(u16 dir,u32 pulse);    //吊挂1/2/3/4号电机同时驱动

void Hand_Hang_1(u16 dir,u32 pulse);          //吊挂电机1号电机驱动
void Hand_Hang_1_2(u16 dir,u32 pulse);        //吊挂1/2号电机同时驱动      

void Hand_Hang_3(u16 dir,u32 pulse);          //吊挂电机3号电机驱动
void Hand_Hang_3_4(u16 dir,u32 pulse);        //吊挂3/4号电机同时驱动

void Hand_Hang_1_3(u16 dir,u32 pulse);        //吊挂1/3号电机同时驱动
void Hand_Hang_1_2_3_4(u16 dir,u32 pulse);    //吊挂1/2/3/4号电机同时驱动


/**************************************************************
                   吊挂电机驱动函数--串口
                   dir：控制吊挂电机速度
				   pulse：控制吊挂电机运行时间
***************************************************************/

void Uart_Auto_Hang_1(u16 dir,u32 pulse);          //吊挂电机1号电机驱动
void Uart_Auto_Hang_3(u16 dir,u32 pulse);          //吊挂电机3号电机驱动
void Uart_Auto_Hang_1_2(u16 dir,u32 pulse);        //吊挂1/2号电机同时驱动
void Uart_Auto_Hang_1_3(u16 dir,u32 pulse);        //吊挂1/3号电机同时驱动
void Uart_Auto_Hang_3_4(u16 dir,u32 pulse);        //吊挂3/4号电机同时驱动
void Uart_Auto_Hang_1_2_3_4(u16 dir,u32 pulse);    //吊挂1/2/3/4号电机同时驱动

void Uart_Hand_Hang_1(u16 dir,u32 pulse);          //吊挂电机1号电机驱动
void Uart_Hand_Hang_1_2(u16 dir,u32 pulse);        //吊挂1/2号电机同时驱动      

void Uart_Hand_Hang_3(u16 dir,u32 pulse);          //吊挂电机3号电机驱动
void Uart_Hand_Hang_3_4(u16 dir,u32 pulse);        //吊挂3/4号电机同时驱动

void Uart_Hand_Hang_1_3(u16 dir,u32 pulse);        //吊挂1/3号电机同时驱动
void Uart_Hand_Hang_1_2_3_4(u16 dir,u32 pulse);    //吊挂1/2/3/4号电机同时驱动

void Uart_Motor_6_2_START(u8 dir,u32 pulse);       //坐便袋扎紧推杆驱动函数

#endif

