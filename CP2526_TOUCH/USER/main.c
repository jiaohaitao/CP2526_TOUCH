
#include "stm32f10x.h"
#include"usart1.h"
#include"led.h"
#include "timer.h"
#include "i2c_soft.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "imu.h"
#include "uartdebug.h"

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

int main(void)
{
	int i=0;
	float tempyaw;
	LED_GPIO_Config(); 
	USART1_Config();
	NVIC_Configuration();
	Timer_Configuration();
	Timer_Nvic_Configuration();
	MPU6050_Init(20); 

	AK8975_Init();
	
	Start_Timer();
	LED_ON();
	printf("-------------------F1 ANO IMU.....\r\n");
	
	
	while(1){
		if(SYS_1MS_FLAG==1){
			SYS_1MS_FLAG=0;
		}
		
		if(SYS_10MS_FLAG==1){
			SYS_10MS_FLAG=0;
		}

		if(SYS_15MS_FLAG==1){
			SYS_15MS_FLAG=0;
				
		}

		if(SYS_20MS_FLAG==1){
			SYS_20MS_FLAG=0;
			MPU6050_Read(); 		
			ANO_AK8975_Read();				
									
			IMUupdate(0.01,Mpu6050_Gyo_Deg_Data[0] ,Mpu6050_Gyo_Deg_Data[1],Mpu6050_Gyo_Deg_Data[2],Mpu6050_Acc_Data[0],Mpu6050_Acc_Data[1],Mpu6050_Acc_Data[2],&Roll,&Pitch,&Yaw);
			Send_Data(Mpu6050_Acc_Data[0],Mpu6050_Acc_Data[1],Mpu6050_Acc_Data[2],Mpu6050_Gyo_Data[0]*m_gyroScale*-1,Mpu6050_Gyo_Data[1]*m_gyroScale*-1,Mpu6050_Gyo_Data[2]*m_gyroScale,AK8975_Mag_Data[0],AK8975_Mag_Data[1],AK8975_Mag_Data[2]);
			tempyaw=Yaw*-1;
			Data_Send_Status(Roll,Pitch,tempyaw);
			// Data_Send_Status(Mpu6050_Gyo_Data[0]*m_gyroScale,Mpu6050_Gyo_Data[1]*m_gyroScale,Mpu6050_Gyo_Data[2]*m_gyroScale);
			
			LED_ON_OFF();
		}

		if(SYS_50MS_FLAG==1){
			SYS_50MS_FLAG=0;
		}		

		if(SYS_100MS_FLAG==1){
			SYS_100MS_FLAG=0;
		}		

		if(SYS_1000MS_FLAG==1){
			SYS_1000MS_FLAG=0;
			
		}				
	}
}
