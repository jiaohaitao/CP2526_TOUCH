// timer modul 
#include"timer.h"
#include "stm32f10x.h"
//--------------------------------------------------
//--函数： void Start_Timer(void)
//--参数： 无
//--返回： 无 （启动TIM时钟，使能TIM）
//--说明： 启动定时器 
//---------------------------------------------------
void Start_Timer(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}
//---------------------------------------------------
//--函数： void Stop_Tmer(void)
//--参数： 无
//--返回： 无（失能tim，关闭tim时钟）
//--停止定时器
//----------------------------------------------------
void Stop_Tmer(void)
{
	TIM_Cmd(TIM2,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,DISABLE);
}
//-----------------------------------------------------
//--函数： void Timer_Nvic_Configuration(void)
//--参数： 无
//--返回： 无 
//--说明： timer的nvic配置
//------------------------------------------------------
void Timer_Nvic_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;			
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//--------------------------------------------------------
//--函数：	void Timer_Configuration(void)
//--参数：  无
//--返回：  无
//--说明：	timer配置
//----------------------------------------------------------
void Timer_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_DeInit(TIM2);//复位tim2
	
	TIM_TimeBaseStructure.TIM_Period=10-1; //累计 TIM_Period个频率后产生一个更新或者中断 
	TIM_TimeBaseStructure.TIM_Prescaler=(7200-1);//时钟预分频数 72M/7200=10khz->0.1ms 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//采样分频 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
    TIM_ClearFlag(TIM2,TIM_FLAG_Update);//清除中断标志位
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//允许更新中断
    TIM_Cmd(TIM2, ENABLE);			//使能TIM2
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);		/*先关闭等待使用*/    

}
//---------------------------------------------------------
//--函数： void TIM2_IRQHandler(void)
//--参数： 无
//--返回： 无
//--说明： 此为tim2中断服务函数，需要添加到stm32f10x_it.c中
//--		详见timer应用例程
//----------------------------------------------------------
// void TIM2_IRQHandler(void)
// {
// 		if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
// 	{	
// 		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    
//   		 time++;
// 	}	
// }
