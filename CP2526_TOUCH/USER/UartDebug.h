#ifndef _UART_DEBUG_H
#define _UART_DEBUG_H

#include "stm32f10x.h"
void Send_Data(int16_t ad1,int16_t ad2,int16_t ad3,int16_t ad4,int16_t ad5,int16_t ad6,int16_t ad7,int16_t ad8,int16_t ad9);
void Data_Send_Status(float rol,float pit,float yaw);
#endif