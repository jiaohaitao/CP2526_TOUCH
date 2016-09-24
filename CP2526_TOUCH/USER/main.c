
#include "stm32f10x.h"
#include"usart1.h"
#include"led.h"
#include "timer.h"
#include "i2c_soft.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "imu.h"
#include "uartdebug.h"
#include "cp2526.h"

__IO unsigned char SYS_1MS_FLAG=0;
__IO unsigned char SYS_10MS_FLAG=0;
__IO unsigned char SYS_15MS_FLAG=0;
__IO unsigned char SYS_20MS_FLAG=0;
__IO unsigned char SYS_50MS_FLAG=0;
__IO unsigned char SYS_100MS_FLAG=0;
__IO unsigned char SYS_1000MS_FLAG=0;

__IO unsigned char Sensor_Acc_Calibrate_In=0;
__IO unsigned char Sensor_Gyo_Calibrate_In=0;
__IO unsigned char Sensor_Mag_Calibrate_In=0;

__IO unsigned char Imu_Start_Flag=0;

__IO unsigned char CpInit_Flag=0;


void Send_Data(int16_t ad1,int16_t ad2,int16_t ad3,int16_t ad4,int16_t ad5,int16_t ad6,int16_t ad7,int16_t ad8,int16_t ad9);
void Data_Send_Status(float rol,float pit,float yaw);


short Acc_Offset[3]={0};
short Gyo_Offset[3]={0};

short Mag_Max[3]={0};
short Mag_Min[3]={0};
short Mag_Offset[3]={0};


extern short AK8975_Mag_Data[3];
extern short Mpu6050_Acc_Data[3];
extern short Mpu6050_Gyo_Data[3];
extern float Mpu6050_Gyo_Deg_Data[3];

#define	RTMATH_PI                   3.1415926535
float m_gyroScale = RTMATH_PI / (16.4 * 180.0);
void EXTI_PE5_Config(void);
int main(void)
{
	int i=0;
	float tempyaw;
	LED_GPIO_Config(); 
	USART1_Config();
//	NVIC_Configuration();
	Timer_Configuration();
	Timer_Nvic_Configuration();
	I2c_Soft_Init();
	EXTI_PE5_Config();
//	MPU6050_Init(20); 

//	AK8975_Init();
	
	Start_Timer();
	LED_ON();
	if(CP2526_Init()==0){
		printf("CP2526 Init Ok.....\r\n");
	}
	else{
		printf("CP2526 Init error.....\r\n");
	}

	
	
	while(1){
		if(SYS_1MS_FLAG==1){
			SYS_1MS_FLAG=0;
		}
		
		if(SYS_10MS_FLAG==1){
			SYS_10MS_FLAG=0;
			if(CpInit_Flag==1){
				CpInit_Flag=0;
				CP2526_Read();
			}
		}

		if(SYS_15MS_FLAG==1){
			SYS_15MS_FLAG=0;
				
		}

		if(SYS_20MS_FLAG==1){
//			SYS_20MS_FLAG=0;
//			MPU6050_Read(); 		
//			ANO_AK8975_Read();				
//									
//			IMUupdate(0.01,Mpu6050_Gyo_Deg_Data[0] ,Mpu6050_Gyo_Deg_Data[1],Mpu6050_Gyo_Deg_Data[2],Mpu6050_Acc_Data[0],Mpu6050_Acc_Data[1],Mpu6050_Acc_Data[2],&Roll,&Pitch,&Yaw);
//			Send_Data(Mpu6050_Acc_Data[0],Mpu6050_Acc_Data[1],Mpu6050_Acc_Data[2],Mpu6050_Gyo_Data[0]*m_gyroScale*-1,Mpu6050_Gyo_Data[1]*m_gyroScale*-1,Mpu6050_Gyo_Data[2]*m_gyroScale,AK8975_Mag_Data[0],AK8975_Mag_Data[1],AK8975_Mag_Data[2]);
//			tempyaw=Yaw*-1;
//			Data_Send_Status(Roll,Pitch,tempyaw);
//			// Data_Send_Status(Mpu6050_Gyo_Data[0]*m_gyroScale,Mpu6050_Gyo_Data[1]*m_gyroScale,Mpu6050_Gyo_Data[2]*m_gyroScale);
//			
//			LED_ON_OFF();
		}

		if(SYS_50MS_FLAG==1){
			SYS_50MS_FLAG=0;
			if(CP2526_Read()==0)
				printf("\r\n-----key input\r\n");
		}		

		if(SYS_100MS_FLAG==1){
			SYS_100MS_FLAG=0;
		}		

		if(SYS_1000MS_FLAG==1){
			SYS_1000MS_FLAG=0;
			LED_ON_OFF();
		}				
	}
}
void GPIO_NVIC_Configuration()
{
	//NVIC初始化结构体，实例结构体
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//配置抢占，响应优先级。选择第一组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	
//	*     @arg NVIC_PriorityGroup_0: 0 bits for pre-emption priority
//  *                                4 bits for subpriority
//  *     @arg NVIC_PriorityGroup_1: 1 bits for pre-emption priority
//  *                                3 bits for subpriority
//  *     @arg NVIC_PriorityGroup_2: 2 bits for pre-emption priority
//  *                                2 bits for subpriority
//  *     @arg NVIC_PriorityGroup_3: 3 bits for pre-emption priority
//  *                                1 bits for subpriority
//  *     @arg NVIC_PriorityGroup_4: 4 bits for pre-emption priority
//  *                                0 bits for subpriority
//可知上面配置的为第1组，抢占优先级：1位；响应优先级：3位

	//配置中断向量号
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	//Pin0-Pin4这5个，有单独的中断向量
	//因为按键PE5对应的EXTI线为EXTI5，而从EXTI5~EXTI9线，由于它们是使用同一个中断向量的，
	//所以只能写入EXTI9_5_IRQn这个参数。这些可写入的参数可以在stm32f10x.h文件的IRQn类型定义中查找到。
	
	// 下面配置抢占，响应都为0，越小优先级越高，所以都是最高级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化，执行
	NVIC_Init(&NVIC_InitStructure);
}
//------------EXTI_PE5_Config()--------------------
//1. 使能EXTIx线的时钟和第二功能AFIO时钟
//2. 配置EXTIx线的中断优先级
//3. 配置EXTI 中断线I/O
//4. 选定要配置为EXTI的I/O口线和I/O口的工作模式
//5. EXTI 中断线工作模式配置
void EXTI_PE5_Config(void)
{
	//初始化gpio实例
	 GPIO_InitTypeDef GPIO_InitStructure;
	 //初始化exti外中断实例
	 EXTI_InitTypeDef EXTI_InitStructure;
	 
	 //开启外设时钟gpio&复用功能时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	// AFIO (alternate-function I/O)，指GPIO端口的复用功能，GPIO除了用作普通的输入输出(主功能)，
	//还可以作为片上外设的复用输入输出，如串口，ADC，这些就是复用功能
	//当把GPIO用作EXTI外部中断 或使用重映射功能的时候，必须开启AFIO时钟，而在使用默认复用功能的时候，就不必开启AFIO时钟了。
	
	//配置nvic中断控制器的函数
	//PE5，外中断，
	 GPIO_NVIC_Configuration();

	//gpio 复用是（包括映射外中断），仍要配置基本的gpio设置
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // 上拉输入
	 GPIO_Init(GPIOA, &GPIO_InitStructure);

	 //指定具体引脚对应外中断
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);
	 EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	 
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  //。把EXTI_Line5的模式设置为为中断模式EXTI_Mode_Interrupt。这个结构体成员也可以赋值为事件模式EXTI_Mode_Event ，
	 //这个模式不会立刻触发中断，而只是在寄存器上把相应的事件标置位置1，应用这个模式要不停地查询相应的寄存器。
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE; //使能
	 //执行中断设置
	 EXTI_Init(&EXTI_InitStructure);

}
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		LED_ON_OFF();
		CpInit_Flag=1;
		EXTI_ClearITPendingBit(EXTI_Line5);
		//EXTI_ClearITPendingBit() 清除中断标置位再退出中断服务函数。
	}	
}