#ifndef __HANG_
#define __HANG_

#include "sys.h"


void HangRun(u8 dir,u8 height,u8 hang1,u8 hang2,u8 hang3,u8 hang4);
void HangRunAuto(u8 dir,u8 turn,u8 ArmLeg,u8 times,u16 height);

//×Ô¶¯
void Uart_Auto_Arm_Leg(u8 dir,u8 turn,u8 ArmLeg,u8 times,u16 height);  //×óÖ«¿µ¸´ÑµÁ·


#endif