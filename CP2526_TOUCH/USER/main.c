
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
	//NVIC��ʼ���ṹ�壬ʵ���ṹ��
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//������ռ����Ӧ���ȼ���ѡ���һ��
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
//��֪�������õ�Ϊ��1�飬��ռ���ȼ���1λ����Ӧ���ȼ���3λ

	//�����ж�������
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	//Pin0-Pin4��5�����е������ж�����
	//��Ϊ����PE5��Ӧ��EXTI��ΪEXTI5������EXTI5~EXTI9�ߣ�����������ʹ��ͬһ���ж������ģ�
	//����ֻ��д��EXTI9_5_IRQn�����������Щ��д��Ĳ���������stm32f10x.h�ļ���IRQn���Ͷ����в��ҵ���
	
	// ����������ռ����Ӧ��Ϊ0��ԽС���ȼ�Խ�ߣ����Զ�����߼�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//ʹ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ����ִ��
	NVIC_Init(&NVIC_InitStructure);
}
//------------EXTI_PE5_Config()--------------------
//1. ʹ��EXTIx�ߵ�ʱ�Ӻ͵ڶ�����AFIOʱ��
//2. ����EXTIx�ߵ��ж����ȼ�
//3. ����EXTI �ж���I/O
//4. ѡ��Ҫ����ΪEXTI��I/O���ߺ�I/O�ڵĹ���ģʽ
//5. EXTI �ж��߹���ģʽ����
void EXTI_PE5_Config(void)
{
	//��ʼ��gpioʵ��
	 GPIO_InitTypeDef GPIO_InitStructure;
	 //��ʼ��exti���ж�ʵ��
	 EXTI_InitTypeDef EXTI_InitStructure;
	 
	 //��������ʱ��gpio&���ù���ʱ��
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	// AFIO (alternate-function I/O)��ָGPIO�˿ڵĸ��ù��ܣ�GPIO����������ͨ���������(������)��
	//��������ΪƬ������ĸ�������������紮�ڣ�ADC����Щ���Ǹ��ù���
	//����GPIO����EXTI�ⲿ�ж� ��ʹ����ӳ�书�ܵ�ʱ�򣬱��뿪��AFIOʱ�ӣ�����ʹ��Ĭ�ϸ��ù��ܵ�ʱ�򣬾Ͳ��ؿ���AFIOʱ���ˡ�
	
	//����nvic�жϿ������ĺ���
	//PE5�����жϣ�
	 GPIO_NVIC_Configuration();

	//gpio �����ǣ�����ӳ�����жϣ�����Ҫ���û�����gpio����
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // ��������
	 GPIO_Init(GPIOA, &GPIO_InitStructure);

	 //ָ���������Ŷ�Ӧ���ж�
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);
	 EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	 
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  //����EXTI_Line5��ģʽ����ΪΪ�ж�ģʽEXTI_Mode_Interrupt������ṹ���ԱҲ���Ը�ֵΪ�¼�ģʽEXTI_Mode_Event ��
	 //���ģʽ�������̴����жϣ���ֻ���ڼĴ����ϰ���Ӧ���¼�����λ��1��Ӧ�����ģʽҪ��ͣ�ز�ѯ��Ӧ�ļĴ�����
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½����ж�
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE; //ʹ��
	 //ִ���ж�����
	 EXTI_Init(&EXTI_InitStructure);

}
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		LED_ON_OFF();
		CpInit_Flag=1;
		EXTI_ClearITPendingBit(EXTI_Line5);
		//EXTI_ClearITPendingBit() ����жϱ���λ���˳��жϷ�������
	}	
}