#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>	 //	 *itoa(),USART1_printf()这两个函数用到了这个头文件的定义


void USART1_Config(void);
void NVIC_Configuration(void);
int fputc(int ch, FILE *f);
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
void USART1_Send_Data(USART_TypeDef* USARTx,uint8_t *DataBuf,uint16_t DataNum);//发送数据到串口
#endif /* __USART1_H */
