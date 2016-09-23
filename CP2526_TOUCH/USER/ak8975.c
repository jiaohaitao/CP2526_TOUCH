/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：ak8975.c
 * 描述    ：电子罗盘驱动
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "ak8975.h"
#include "mymath.h"
#include "i2c_soft.h"

#define USE_AK8975
u8 ak8975_ok;
short AK8975_Mag_Data[3]={0};
short AK8975_Mag_Offset[3]={0};
unsigned char ak8975_offset_ok=0;
unsigned char ak8975_ok=0;

unsigned char AK8975_Get_Offset(void)
{	
	//-76,-36,-58
	AK8975_Mag_Offset[0]=-76;
	AK8975_Mag_Offset[1]=-36;
	AK8975_Mag_Offset[2]=-58;
	return 1;
}
unsigned char AK8975_Init(void)
{
	ak8975_offset_ok=AK8975_Get_Offset();
	
}
unsigned char ANO_AK8975_Run(void)
{
#ifdef USE_AK8975	
	return IIC_Write_1Byte(AK8975_ADDRESS,AK8975_CNTL,0x01);	
#else			
	return IIC_Write_1Byte(HMC5883_Addr,0x02,0x01);
#endif	
}


#define AK8975_MODE_POWERDOWN     0x0
#define AK8975_MODE_SINGLE        0x1
#define AK8975_MODE_SELFTEST      0x8
#define AK8975_MODE_FUSEROM       0xF

#define AK8975_RA_CNTL            0x0A

#define AK8975_FILTER_LENGTH	10
__IO short AK8975_FILTER_BUF[3][AK8975_FILTER_LENGTH]={0};
__IO int	AK8975_FILTER_SUM[3]={0};
unsigned int AK8975_FILTER_CNT=0;

void ANO_AK8975_Read_Mag_Data(void)
{
	int i=0,j=0;
	int16_t mag_temp[3];
	u8 ak8975_buffer[6]; //接收数据缓存
	
	I2C_FastMode = 0;
	
#ifdef 	USE_AK8975
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXL,&ak8975_buffer[0]); 
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXH,&ak8975_buffer[1]);
	mag_temp[0] = ((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;  //磁力计X轴

	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYL,&ak8975_buffer[2]);
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYH,&ak8975_buffer[3]);
	mag_temp[1] = -((((int16_t)ak8975_buffer[3]) << 8) | ak8975_buffer[2]) ;  //磁力计Y轴

	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZL,&ak8975_buffer[4]);
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZH,&ak8975_buffer[5]);
	mag_temp[2] = -((((int16_t)ak8975_buffer[5]) << 8) | ak8975_buffer[4]) ;  //磁力计Z轴	
#else	
	
	IIC_Read_1Byte(HMC5883_Addr,HMC5883_HXL,&ak8975_buffer[0]); 
	IIC_Read_1Byte(HMC5883_Addr,HMC5883_HXH,&ak8975_buffer[1]);
	mag_temp[0] = ((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;  //磁力计X轴

	IIC_Read_1Byte(HMC5883_Addr,HMC5883_HYL,&ak8975_buffer[2]);
	IIC_Read_1Byte(HMC5883_Addr,HMC5883_HYH,&ak8975_buffer[3]);
	mag_temp[1] = ((((int16_t)ak8975_buffer[3]) << 8) | ak8975_buffer[2]) ;  //磁力计Y轴

	IIC_Read_1Byte(HMC5883_Addr,HMC5883_HZL,&ak8975_buffer[4]);
	IIC_Read_1Byte(HMC5883_Addr,HMC5883_HZH,&ak8975_buffer[5]);
	mag_temp[2] = ((((int16_t)ak8975_buffer[5]) << 8) | ak8975_buffer[4]) ;  //磁力计Z轴		
#endif	
	if(ABS(mag_temp[0])>400||ABS(mag_temp[1])>400||ABS(mag_temp[2])>400)
		return;
	
	
	AK8975_Mag_Data[0] = mag_temp[0];
	AK8975_Mag_Data[1] = mag_temp[1];
	AK8975_Mag_Data[2] = mag_temp[2];
	
	if(ak8975_offset_ok==1){
		for(i=0;i<3;i++){
			AK8975_Mag_Data[i]-=AK8975_Mag_Offset[i];
		}
	}
	
	
//----------------------filter deal----------------------------	
	if(AK8975_FILTER_CNT>=AK8975_FILTER_LENGTH)
		AK8975_FILTER_CNT=0;
	
	for(i=0;i<3;i++){
		AK8975_FILTER_BUF[i][AK8975_FILTER_CNT]=AK8975_Mag_Data[i];
	}
	
	AK8975_FILTER_CNT++;
	AK8975_FILTER_SUM[0]=AK8975_FILTER_SUM[1]=AK8975_FILTER_SUM[2]=0;
	for(i=0;i<3;i++){
		for(j=0;j<AK8975_FILTER_LENGTH;j++){
			AK8975_FILTER_SUM[i]+=AK8975_FILTER_BUF[i][j];		
		}
	}
	
	AK8975_Mag_Data[0] = AK8975_FILTER_SUM[0]/AK8975_FILTER_LENGTH;
	AK8975_Mag_Data[1] = AK8975_FILTER_SUM[1]/AK8975_FILTER_LENGTH;
	AK8975_Mag_Data[2] = AK8975_FILTER_SUM[2]/AK8975_FILTER_LENGTH;
//--------------------------------------------------------------------	
	//AK8975采样触发
	ANO_AK8975_Run();
}


void ANO_AK8975_Read(void)
{
		//读取磁力计
		ANO_AK8975_Read_Mag_Data();
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

