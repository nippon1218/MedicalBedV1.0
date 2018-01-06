#ifndef __SPI_H
#define __SPI_H
#include "sys.h"

extern SPI_HandleTypeDef SPI5_Handler;        //SPI句柄

void SPI5_Init(void);                         //SPI口初始化，配置成主机模式
void SPI5_SetSpeed(u8 SPI_BaudRatePrescaler); //SPI速度设置函数
u8 SPI5_ReadWriteByte(u8 TxData);             //SPI5 读写一个字节函数

#endif
