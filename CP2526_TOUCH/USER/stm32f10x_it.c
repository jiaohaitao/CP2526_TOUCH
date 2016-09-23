/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include"usart1.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
extern __IO unsigned char SYS_1MS_FLAG;
extern __IO unsigned char SYS_10MS_FLAG;
extern __IO unsigned char SYS_15MS_FLAG;
extern __IO unsigned char SYS_20MS_FLAG;
extern __IO unsigned char SYS_50MS_FLAG;
extern __IO unsigned char SYS_100MS_FLAG;
extern __IO unsigned char SYS_1000MS_FLAG;

extern __IO unsigned char Sensor_Acc_Calibrate_In;
extern __IO unsigned char Sensor_Gyo_Calibrate_In;
extern __IO unsigned char Sensor_Mag_Calibrate_In;

extern __IO  unsigned char Imu_Start_Flag;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

void USART1_IRQHandler(void)
{
	u8 c;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
	    c=USART1->DR;
	  //	printf("%c",c);    //将接受到的数据直接返回打印
//		USART1_printf(USART1," %d ",c);
//		GPIO_WriteBit(GPIOC, GPIO_Pin_3,
//		(BitAction)((1-GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_3))));
		if(c=='A'){
			Sensor_Acc_Calibrate_In=1;
		}
		else if(c=='G'){
			Sensor_Gyo_Calibrate_In=1;
		}
		else if(c=='M'){
			Sensor_Mag_Calibrate_In=1;
		}
		else if(c=='I'){
			Imu_Start_Flag=1;
		}
		else{}
	} 
	 
}
//---------------------------------------------------------
//--函数： void TIM2_IRQHandler(void)
//--参数： 无
//--返回： 无
//--说明： 此为tim2中断服务函数，需要添加到stm32f10x_it.c中
//--		详见timer应用例程
//----------------------------------------------------------
 void TIM2_IRQHandler(void)
 {
	 static int timer_1ms_cnt=0;
	 static int timer_10ms_cnt=0;
	 static int timer_15ms_cnt=0;
	 static int timer_20ms_cnt=0;
	 static int timer_50ms_cnt=0;
	 static int timer_100ms_cnt=0;
	 static int timer_1000ms_cnt=0;
	 
 		if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
 	{	
 		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);  
		timer_1ms_cnt++;	
		timer_10ms_cnt++;
		timer_15ms_cnt++;
		timer_20ms_cnt++;
		timer_50ms_cnt++;
		timer_100ms_cnt++;
		timer_1000ms_cnt++;
		
		if(timer_1ms_cnt>=1){
			timer_1ms_cnt=0;
			SYS_1MS_FLAG=1;
		}
		
		if(timer_10ms_cnt>=10){
			timer_10ms_cnt=0;
			SYS_10MS_FLAG=1;
		}

		if(timer_15ms_cnt>=15){
			timer_15ms_cnt=0;
			SYS_15MS_FLAG=1;
		}

		if(timer_20ms_cnt>=20){
			timer_20ms_cnt=0;
			SYS_20MS_FLAG=1;
		}

		if(timer_50ms_cnt>=50){
			timer_50ms_cnt=0;
			SYS_50MS_FLAG=1;
		}

		if(timer_100ms_cnt>=100){
			timer_100ms_cnt=0;
			SYS_100MS_FLAG=1;
		}		
		
		if(timer_1000ms_cnt>=1000){
			timer_1000ms_cnt=0;
			SYS_1000MS_FLAG=1;
		}				
 	}	
 }

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)																							D
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
