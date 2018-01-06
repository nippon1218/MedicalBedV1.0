#ifndef __WASHLET_H
#define __WASHLET_H
#include "sys.h"
#include "function.h"
#include "motor.h"
#include "pwm.h"


#define Up 1
#define Down 0

void WashLetAuto_V1(u8 dir,u16 arr);

void WashLet_V1(u8 dir,u16 arr);
void WashLet_V2(u8 dir,u16 arr);


#endif