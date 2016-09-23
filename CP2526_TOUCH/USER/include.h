#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include "stm32f10x.h"
//#include "scheduler.h"
//#include "time.h"
//#include "init.h"
//#include "parameter.h"
//#include "pwm_in.h"
//#include "usart.h"
//#include "usbd_user_hid.h"
//#include "data_transfer.h"

//================系统===================

#define USE_US100           //使用us100型号超声波 
// #define USE_KS103					//使用ks103型号超声波

#define MAXMOTORS 		(4)		//电机数量
#define GET_TIME_NUM 	(5)		//设置获取时间的数组数量
#define CH_NUM 				(8) 	//接收机通道数量

#define USE_TOE_IN_UNLOCK 0 // 0：默认解锁方式，1：外八解锁方式
#define ANO_DT_USE_USART2 	//开启串口2数传功能
#define ANO_DT_USE_USB_HID	//开启飞控USBHID连接上位机功能
//=======================================
/***************中断优先级******************/
#define NVIC_GROUP NVIC_PriorityGroup_3		//中断分组选择
#define NVIC_PWMIN_P			1		//接收机采集
#define NVIC_PWMIN_S			1
#define NVIC_TIME_P       2		//暂未使用
#define NVIC_TIME_S       0
#define NVIC_UART_P				5		//暂未使用
#define NVIC_UART_S				1
#define NVIC_UART2_P			3		//串口2中断
#define NVIC_UART2_S			1
/***********************************************/

//================传感器===================
#define ACC_ADJ_EN 									//是否允许校准加速度计,(定义则允许)

#define OFFSET_AV_NUM 	50					//校准偏移量时的平均次数。
#define FILTER_NUM 			10					//滑动平均滤波数值个数

#define TO_ANGLE 				0.06103f 		//0.061036 //   4000/65536  +-2000   ???

#define FIX_GYRO_Y 			1.02f				//陀螺仪Y轴固有补偿
#define FIX_GYRO_X 			1.02f				//陀螺仪X轴固有补偿

#define TO_M_S2 				0.23926f   	//   980cm/s2    +-8g   980/4096
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度

#define MAX_ACC  4096.0f						//+-8G		加速度计量程
#define TO_DEG_S 500.0f      				//T = 2ms  默认为2ms ，数值等于1/T





typedef struct 
{
  s16 x;
	s16 y;
	s16 z;

}xyz_s16_t;

#endif

