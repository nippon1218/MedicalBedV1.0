#ifndef __FUN_H
#define __FUN_H
#include "sys.h"
#include "usart.h"

extern u8 device_num;              //连接到WiFi的智能终端的个数

/********************机构限制运行最大角度********************************/

extern u8  back_angle_max;         //上位机设定支背运行角度
extern u8  leg_up_angle_max;       //上位机设定上曲腿运行角度
extern u8  leg_down_angle_max;     //上位机设定下曲腿运行角度
extern u8  body_left_angle_max;    //上位机设定左翻运行角度
extern u8  body_right_angle_max;   //上位机设定右翻运行角度
extern u8  desk_distance_max;      //上位机设定娱乐小桌子运动距离
extern u8  swash_dry_time_max;     //上位机设定娱乐坐便冲洗烘干时间/分钟

/*****************复位状态标志位，1：未复位 ； 0：复位到初始状态***********/

extern u8 back_flag;               //支背
extern u8 leg_up_flag;             //上曲腿
extern u8 leg_down_flag;           //下曲腿
extern u8 body_left_flag;          //左翻
extern u8 body_right_flag;         //右翻
extern u8 back_nursing_left_flag;  //左背部护理
extern u8 back_nursing_right_flag; //右背部护理
extern u8 waist_nursing_left_flag; //左腰部护理
extern u8 waist_nursing_right_flag;//右腰部护理
extern u8 washlet_flag;            //坐便器
extern u8 washlet_auto_flag;       //自动坐便器
extern u8 desk_flag;               //就餐娱乐一体桌
extern u8 jram_flag;               //肌肉按摩
extern u8 swash_dry_flag;          //冲洗烘干
extern u8 lock_flag;               //一键锁定程序
extern u8 fault_flag;              //电机故障标志位
extern u8 desk_front_flag;

extern u8 back_dir_flag;

//
extern u8 back_nursing_left_dir_flag;       //左背部护理方向标志位
extern u8 back_nursing_right_dir_flag;      //右背部护理方向标志位
extern u8 waist_nursing_left_dir_flag;      //左腰部护理方向标志位
extern u8 waist_nursing_right_dir_flag;     //右腰部护理方向标志位
extern u16 body_left_runed_arr;           //左翻，调整后的自动重装载值
extern u16 body_right_runed_arr;          //右翻，调整后的自动重装载值
extern u16 desk_runed_arr;								//桌子，调整后的自动重装载值




extern u8 swash_hand_flag;         //手动冲洗标志位
extern u8 dry_hand_flag;           //手动烘干标志位

extern u8 leg_down_state_flag;     //自动坐便时记录曲腿是否已处于动作状态
extern u8 back_state_flag;         //自动坐便时记录支背是否已处于动作状态

extern u8 armleg_left_flag;        //手动左肢
extern u8 armleg_right_flag;       //手动右肢
extern u8 armleg_left_right_flag;  //手动左右肢

extern u8 arm_fore_left_flag;      //左小臂
extern u8 leg_fore_left_flag;      //左小腿

extern u8 arm_fore_right_flag;     //右小臂
extern u8 leg_fore_right_flag;     //右小腿

extern u8 arm_post_left_flag;      //左大臂
extern u8 leg_post_left_flag;      //左大腿

extern u8 arm_post_right_flag;     //右大臂
extern u8 leg_post_right_flag;     //右大腿

extern u8 arm_fore_post_left_flag;       //左大小臂
extern u8 leg_fore_post_left_flag;       //左大小腿

extern u8 arm_fore_post_right_flag;      //右大小臂
extern u8 leg_fore_post_right_flag;      //右大小腿

extern u8 arm_fore_left_right_flag;      //左右小臂
extern u8 leg_fore_left_right_flag;      //左右小腿

extern u8 arm_post_left_right_flag;      //左右大臂
extern u8 leg_post_left_right_flag;      //左右大腿

extern u8 arm_fore_post_left_right_flag; //左右大小臂
extern u8 leg_fore_post_left_right_flag; //左右大小腿

/****************错误状态标志位******************************/

//驱动器返回-电机过载

extern u8 body_left_overload_3;        //左翻身
extern u8 body_left_overload_4;        //左翻身
extern u8 body_left_overload_5;        //左翻身

extern u8 body_right_overload_3;       //右翻身
extern u8 body_right_overload_4;       //右翻身
extern u8 body_right_overload_5;       //右翻身

extern u8 washlet_auto_overload;       //自动坐便
extern u8 desk_overload;               //就餐娱乐一体桌
extern u8 back_nursing_left_overload;  //左背部护理
extern u8 back_nursing_right_overload; //右背部护理
extern u8 waist_nursing_left_overload; //左腰部护理
extern u8 waist_nursing_right_overload;//右腰部护理

//驱动器返回-电机失步
extern u8 body_left_losepulse;          //左翻身
extern u8 body_right_losepulse;         //右翻身
extern u8 washlet_auto_losepulse;       //自动坐便
extern u8 desk_losepulse;               //就餐娱乐一体桌
extern u8 back_nursing_left_losepulse;  //左背部护理
extern u8 back_nursing_right_losepulse; //右背部护理
extern u8 waist_nursing_left_losepulse; //左腰部护理
extern u8 waist_nursing_right_losepulse;//右腰部护理

//程序运行出现动作干涉标志位
extern u8 back_interfere;                 //支背
extern u8 leg_up_interfere;               //上曲腿
extern u8 leg_down_interfere;             //下曲腿    
extern u8 leg_interfere;                  //曲腿
extern u8 body_left_interfere;            //左翻身
extern u8 body_right_interfere;           //右翻身
extern u8 washlet_auto_interfere;         //坐便器
extern u8 desk_interfere;                 //小桌子
extern u8 back_nursing_left_interfere;    //左背部护理
extern u8 back_nursing_right_interfere;   //右背部护理
extern u8 waist_nursing_left_interfere;   //左腰部护理
extern u8 waist_nursing_right_interfere;  //右腰部护理

/****************发送图片******************************/
extern u8 back_picture_k;                 //支背
extern u8 leg_up_picture_k;               //上曲腿
extern u8 leg_down_picture_k;             //下曲腿
extern u8 desk_picture_k;                 //小桌子
extern u8 washlet_picture_k;              //坐便器
extern u8 body_left_picture_k;            //左翻身
extern u8 left_motor5_picture_m;          //左小侧翻
extern u8 body_right_picture_k;           //右翻身
extern u8 right_motor5_picture_m;         //右小侧翻

extern u8 back_nursing_left_picture_k;    //左背部护理
extern u8 waist_nursing_left_picture_k;   //左腰部护理
extern u8 back_nursing_right_picture_k;   //右背部护理
extern u8 waist_nursing_right_picture_k;  //右腰部护理




/***************参数设定界面设置极限运行参数*********************/

extern u8  back_angle_lim;                 //上位机设定支背运行角度
extern u8  leg_up_angle_lim;               //上位机设定上曲腿运行角度
extern u8  leg_down_angle_lim;             //上位机设定下曲腿运行角度
extern u8  body_left_angle_lim;            //上位机设定左翻运行角度
extern u8  body_right_angle_lim;           //上位机设定右翻运行角度
extern u8  desk_distance_lim;              //上位机设定娱乐小桌子运动距离
extern u8  swash_dry_time;                 //上位机设定娱乐坐便冲洗烘干时间
extern u16 washlet_arr_lim;                //坐便器运行下，计时器的arr（重装载值）-6号电机

/****************吊挂康复训练脉冲值计算*******************/

extern u16 back_runed_arr;                 //支背，调整后的自动重装载值
extern u16 leg_up_runed_arr;
extern u16 leg_down_runed_arr;
//左肢
extern u32 arm_left_runed;                 //相对起始位置已运行脉冲数   
extern u32 arm_left_lim;                   //极限脉冲
extern u32 leg_left_runed;                 //相对起始位置已运行脉冲数   
extern u32 leg_left_lim;                   //极限脉冲

//右肢
extern u32 arm_right_runed;                //相对起始位置已运行脉冲数  
extern u32 arm_right_lim;                  //极限脉冲
extern u32 leg_right_runed;                //相对起始位置已运行脉冲数  
extern u32 leg_right_lim;                  //极限脉冲

//左右肢
extern u32 arm_left_right_runed;           //相对起始位置已运行脉冲数  
extern u32 arm_left_right_lim;             //极限脉冲
extern u32 leg_left_right_runed;           //相对起始位置已运行脉冲数  
extern u32 leg_left_right_lim;             //极限脉冲

//左小臂/小腿
extern u32 arm_fore_left_runed;            //相对起始位置已运行脉冲数  
extern u32 arm_fore_left_lim;              //极限脉冲
extern u32 leg_fore_left_runed;            //相对起始位置已运行脉冲数  
extern u32 leg_fore_left_lim;              //极限脉冲

//左大臂/大腿
extern u32 arm_post_left_lim;              //极限脉冲    
extern u32 arm_post_left_runed;            //相对起始位置已运行脉冲数
extern u32 leg_post_left_lim;              //极限脉冲    
extern u32 leg_post_left_runed;            //相对起始位置已运行脉冲数

//右小臂/小腿
extern u32 arm_fore_right_lim;             //极限脉冲     
extern u32 arm_fore_right_runed;           //相对起始位置已运行脉冲数
extern u32 leg_fore_right_lim;             //极限脉冲     
extern u32 leg_fore_right_runed;           //相对起始位置已运行脉冲数

//右大臂/大腿
extern u32 arm_post_right_lim;             //极限脉冲   
extern u32 arm_post_right_runed;           //相对起始位置已运行脉冲数
extern u32 leg_post_right_lim;             //极限脉冲   
extern u32 leg_post_right_runed;           //相对起始位置已运行脉冲数

//左右小臂/小腿
extern u32 arm_fore_left_right_lim;        //极限脉冲     
extern u32 arm_fore_left_right_runed;      //相对起始位置已运行脉冲数
extern u32 leg_fore_left_right_lim;        //极限脉冲     
extern u32 leg_fore_left_right_runed;      //相对起始位置已运行脉冲数

//左右大臂/大腿
extern u32 arm_post_left_right_lim;        //极限脉冲     
extern u32 arm_post_left_right_runed;      //相对起始位置已运行脉冲数
extern u32 leg_post_left_right_lim;        //极限脉冲     
extern u32 leg_post_left_right_runed;      //相对起始位置已运行脉冲数

/**************************冲洗烘干推杆******************************/

extern u32 push_rod_runed_pulse;           //相对起始位置已运行脉冲数
extern u32 push_rod_pulse_lim;             //极限脉冲

extern u32 swash_dry_runed_pulse;          //相对起始位置已运行脉冲数  
extern u32 swash_dry_pulse_lim;            //极限脉冲

/********************坐便袋收进前移动的推杆***************************/

extern u32 push_rod_tig_runed_pulse;       //相对起始位置已运行脉冲数  
extern u32 push_rod_tig_pulse_now;         //当前一次运行脉冲数
extern u32 push_rod_tig_pulse_lim;         //极限脉冲

/*********电机运行自动重装载值，控制电机运行速度******************/

extern unsigned int motor_back_freq;        //支背
extern unsigned int motor_body_freq;        //翻身
extern unsigned int motor_washlet_freq;     //坐便 
extern unsigned int motor_desk_freq;        //小桌子 
extern unsigned int motor_hang_freq;        //吊挂 

/**********定时器分频值psc，控制电机运行速度**********************/

extern unsigned int motor_timer_freq;      //定时器分频值psc   定时器频率=90M/50
extern unsigned int motor_timer_freq_1; 

/**********定时器分频值psc，控制电机总运行时间*******************/

extern u16 timer10_freq;
extern unsigned int timer10_freq_1;       //定时器分频值,
extern unsigned int timer10_arr_1s;       //定时1秒钟


extern u16 bodyleft_compleate;
extern u16 bodyright_compleate;
extern u8 body_left_dir_flag;
extern u8 body_right_dir_flag;
extern u8 muscle_massager_flag;

extern unsigned int motor_timer_freq;


/*********功能函数(手机APP、遥控器、电脑)-双键******************************/

void Fun_Back(void);                     //支背
void Fun_Leg_Up(void);                   //上曲腿
void Fun_Leg_Down(void);                 //下曲腿
void Fun_Body_Left(void);                //左翻
void Fun_Body_Right(void);               //右翻
void Fun_Desk(void);                     //办公娱乐一体桌
void Fun_Back_Nursing_Left(void);        //左背部护理
void Fun_Back_Nursing_Right(void);       //右背部护理 
void Fun_Waist_Nursing_Left(void);       //左腰部护理
void Fun_Waist_Nursing_Right(void);      //右腰部护理

//联调函数
void LDUART4(void);                      //联调函数
void WriteInUART4(char *p);              //将字符串写到串口4中

/**************护栏功能函数-单键*********************/

void GB_Back(void);                     //支背
void GB_Body_Left(void);                //左翻身
void GB_Body_Right(void);               //右翻身
void GB_Back_Nursing(void);             //背部护理
void GB_Waist_Nursing(void);            //腰部护理
u8   GB_Lock(void);                     //一键锁定

/**************自动坐便******************************/

void Washlet_Auto(void);                //自动坐便
void Washlet(u8 dir);                   //坐便器打开/关闭
void Back_Leg(void);                    //支背、下曲腿运行函数
u8   Weight(void);                      //称重
u8   Washlet_Weight(void); 
void Washlet_Tig(u8 dir);               //坐便袋收紧


void Swash_Dry(void);                   //冲洗烘干
void Swash_Hand(void);                  //手动冲洗
void Dry_Hand(void);                    //手动烘干
void Swash_Auto(void);                  //自动冲洗
void Dry_Auto(void);                    //自动烘干

void Muscle_Massager(void);             //肌肉按摩
void Heat(void);                        //加热

void IO_TEST(void);                     //IO口测试函数 

/**********************专家系统******************************/

void Exp_Back(void);                    //专家系统支背
void Exp_Body(void);                    //专家系统翻身
void Exp_Leg(void);                     //专家系统曲腿
void Exp_Washlet_Auto(void);            //专家系统自动坐便

/****************吊挂康复训练******************************/

//自动
void Auto_Arm_Leg_Left(int t);             //左肢康复训练
void Auto_Arm_Leg_Right(int t);            //右肢康复训练
void Auto_Arm_Leg_Left_Right(int t);       //左右肢康复训练 

//手动
void Hand_Arm_Fore_Left(void);             //左小臂
void Hand_Arm_Post_Left(void);             //左大臂
void Hand_Arm_Fore_Post_Left(void);        //左大小臂

void Hand_Leg_Fore_Left(void);             //左小腿
void Hand_Leg_Post_Left(void);             //左大腿
void Hand_Leg_Fore_Post_Left(void);        //左大小腿

void Hand_Arm_Fore_Right(void);            //右小臂
void Hand_Arm_Post_Right(void);            //右大臂
void Hand_Arm_Fore_Post_Right(void);       //右大小臂

void Hand_Leg_Fore_Right(void);            //右小腿
void Hand_Leg_Post_Right(void);            //右大腿
void Hand_Leg_Fore_Post_Right(void);       //右大小腿

void Hand_Arm_Fore_Left_Right(void);       //左右小臂
void Hand_Arm_Post_Left_Right(void);       //左右大臂
void Hand_Arm_Fore_Post_Left_Right(void);  //左右大小臂

void Hand_Leg_Fore_Left_Right(void);       //左右小腿
void Hand_Leg_Post_Left_Right(void);       //左右大腿
void Hand_Leg_Fore_Post_Left_Right(void);  //左右大小腿

/****************用于掉电复位函数********************************/

void Res_Power_Down(void);                 //掉电复位函数
void Res_Back(void);                       //支背复位 
void Res_Leg(void);                        //曲腿复位
void Res_Desk(void);                       //就餐娱乐一体桌复位
void Res_Motor5(u8 dir);                   //5号电机反向复位0;正向复位1
void Res_Body_Left(void);                  //左翻身复位
void Res_Body_Right(void);                 //右翻身复位

/*********获取上位机新的脉冲值********************************************/

void get_newangle_usart2(void);            //通过串口2获取上位机新的脉冲值
void get_newangle_wifi(void);              //通过WiFi获取上位机新的脉冲值

/*********将运行角度转化为脉冲数*****************************************/

u16 back_angle_to_arr(u8 angle);           //将支背运行角度转化为脉冲数
u16 leg_angle_to_arr(u8 angle);            //将曲腿运行角度转化为脉冲数
u16 body_angle_to_arr(u8 angle);           //将翻身运行角度转化为脉冲数
u16 desk_distance_to_arr(u8 distance);     //将桌子运行角度转化为脉冲数

void Read_Angle(void);                     //读取上位机设置的角度值和距离值

void Wifi_Send(u8 *data);                  //WiFi发送函数
void Wifi_ToPC(u8 *data);                  //WiFi发送给PC函数
void Wifi_ToPhone(u8 *data);               //WiFi发送给手机APP函数
void Wifi_ToRemote(u8 *data);              //WiFi发送给遥控器函数
void Wifi_ToGuard(u8 *data);               //WiFi发送给护栏函数
void Uart_ToStick(u8 *data);               //WiFi发送给PC棒函数

/***********************************************************************
                    
					     串口2函数（服务器）
					
************************************************************************/

/*********功能函数(移动智能终端)-双键******************************/

void Uart_Back(void);                  //支背
void Uart_Leg_Up(void);                //上曲腿
void Uart_Leg_Down(void);              //下曲腿
void Uart_Body_Left(void);             //左翻
void Uart_Body_Right(void);            //右翻
void Uart_Desk(void);                  //办公娱乐一体桌
void Uart_Desk1(void);
void Uart_Back_Nursing_Left(void);     //左背部护理
void Uart_Back_Nursing_Right(void);    //右背部护理 
void Uart_Waist_Nursing_Left(void);    //左腰部护理
void Uart_Waist_Nursing_Right(void);   //右腰部护理

//联动测试函数
void LDUART2(void);                    //功能函数联动
void WriteInUART2(char *p);            //将字符串写入到串口2的USART2_RX_BUF中
void LDUART2V2(void);
//调试电机函数
void MOTOR111(u8 dir);
void MOTOR222(u8 dir);
void MOTOR333(u8 dir);
void MOTOR444(u8 dir);
void MOTOR555(u8 dir);
void MOTOR666(u8 dir);
void MOTOR777(u8 dir);

void  TestAll(u8 dir);
void Hang1Test(u8 dir);
void Hang2Test(u8 dir);
void Hang3Test(u8 dir);
void Hang4Test(u8 dir);

/*********护栏功能函数-单键************************************************/

void Uart_GB_Back(void);                        //支背
void Uart_GB_Body_Left(void);                   //左翻身
void Uart_GB_Body_Right(void);                  //右翻身
void Uart_GB_Back_Nursing(void);                //背部护理
void Uart_GB_Waist_Nursing(void);               //腰部护理
u8   Uart_GB_Lock(void);                        //一键锁定

/**************自动坐便******************************/

void Uart_Washlet_Auto(void);                   //自动坐便
void Uart_Washlet(u8 dir);                      //坐便器打开/关闭
void Uart_Swash_Dry(void);                      //冲洗烘干
void Uart_Swash_Hand(void);                     //手动冲洗
void Uart_Dry_Hand(void);                       //手动烘干
void Uart_Swash_Auto(void);                     //自动冲洗
void Uart_Dry_Auto(void);                       //自动烘干
u8   Uart_Weight(void);                         //称重
void Uart_Washlet_Tig(u8 dir);                  //坐便袋收紧
void UartWashletTig(u8 dir,u32 TGArr,u32 SXArr);
void Uart_WashletTigOnly(u8 dir);
u8   Uart_Washlet_Weight(void);                 //称重
void Uart_Back_Leg(void);                       //支背下曲腿同时运行

/**********************专家系统******************************/

void Uart_Exp_Back(void);                       //专家系统支背
void Uart_Exp_Body(void);                       //专家系统翻身
void Uart_Exp_Leg(void);                        //专家系统曲腿
void Uart_Exp_Washlet_Auto(void);               //专家系统自动坐便

/****************吊挂康复训练******************************/

//自动
void Uart_Auto_Arm_Leg_Left(int t);             //左肢康复训练
void Uart_Auto_Arm_Leg_Right(int t);            //右肢康复训练
void Uart_Auto_Arm_Leg_Left_Right(int t);       //左右肢康复训练 

//手动
void Uart_Hand_Arm_Fore_Left(void);             //左小臂
void Uart_Hand_Arm_Post_Left(void);             //左大臂
void Uart_Hand_Arm_Fore_Post_Left(void);        //左大小臂

void Uart_Hand_Leg_Fore_Left(void);             //左小腿
void Uart_Hand_Leg_Post_Left(void);             //左大腿
void Uart_Hand_Leg_Fore_Post_Left(void);        //左大小腿

void Uart_Hand_Arm_Fore_Right(void);            //右小臂
void Uart_Hand_Arm_Post_Right(void);            //右大臂
void Uart_Hand_Arm_Fore_Post_Right(void);       //右大小臂

void Uart_Hand_Leg_Fore_Right(void);            //右小腿
void Uart_Hand_Leg_Post_Right(void);            //右大腿
void Uart_Hand_Leg_Fore_Post_Right(void);       //右大小腿

void Uart_Hand_Arm_Fore_Left_Right(void);       //左右小臂
void Uart_Hand_Arm_Post_Left_Right(void);       //左右大臂
void Uart_Hand_Arm_Fore_Post_Left_Right(void);  //左右大小臂

void Uart_Hand_Leg_Fore_Left_Right(void);       //左右小腿
void Uart_Hand_Leg_Post_Left_Right(void);       //左右大腿
void Uart_Hand_Leg_Fore_Post_Left_Right(void);  //左右大小腿

/****************用于掉电复位函数********************************/

void Uart_Res_Power_Down(void);                 //掉电复位函数
void Uart_Res_Power_Down1(void);
void Uart_Res_Power_Down2(void);

void Uart_Res_Back(void);                       //支背复位
void Uart_Res_Leg(void);                        //曲腿复位
void Uart_Res_Desk(void);                       //就餐娱乐一体桌复位
void Uart_Res_Motor5(u8 dir);                   //5号电机反向复位0;正向复位1
void Uart_Res_Body_Left(void);                  //左翻身复位
void Uart_Res_Body_Right(void);                 //右翻身复位

void FlagClear(void);

#endif

