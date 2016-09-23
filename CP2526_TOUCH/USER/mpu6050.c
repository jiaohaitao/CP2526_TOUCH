/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：mpu6050.c
 * 描述    ：6轴传感器mpu6050驱动
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "mpu6050.h"
#include "mymath.h"
#include "i2c_soft.h"

short Mpu6050_Acc_Data[3]={0};
short Mpu6050_Gyo_Data[3]={0};
float Mpu6050_Gyo_Deg_Data[3]={0.0};
short Mpu6050_Acc_Offset_Data[3]={0};
short Mpu6050_Gyo_Offset_Data[3]={0};

u8 mpu6050_buffer[14];
u8 mpu6050_ok=0;
unsigned char mpu6050_offset_ok=0;

#define MPU6050_FILTER_LENGTH	10
__IO short MPU6050_ACC_FILTER_BUF[3][MPU6050_FILTER_LENGTH]={0};
__IO short MPU6050_GYO_FILTER_BUF[3][MPU6050_FILTER_LENGTH]={0};
__IO int	MPU6050_ACC_FILTER_SUM[3]={0};
__IO int	MPU6050_GYO_FILTER_SUM[3]={0};
unsigned int MPU6050_FILTER_CNT=0;


unsigned char MPU6050_Get_Offset()
{
//		mpu6050.Acc_Offset.x=112;
//	mpu6050.Acc_Offset.y=91;
//	mpu6050.Acc_Offset.z=-144;	

//	mpu6050.Gyro_Offset.x=-32;
//	mpu6050.Gyro_Offset.y=29;
//	mpu6050.Gyro_Offset.z=-14;	
	Mpu6050_Acc_Offset_Data[0]=112;
	Mpu6050_Acc_Offset_Data[1]=91;
	Mpu6050_Acc_Offset_Data[2]=-144;	

	Mpu6050_Gyo_Offset_Data[0]=-32;
	Mpu6050_Gyo_Offset_Data[1]=29;
	Mpu6050_Gyo_Offset_Data[2]=-14;
	
	return 1;
}
void MPU6050_Read(void)
{
	int i=0,j=0;
	I2C_FastMode = 1;
	IIC_Read_nByte(MPU6050_ADDR,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
	
	Mpu6050_Acc_Data[0] = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
	Mpu6050_Acc_Data[1] = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
	Mpu6050_Acc_Data[2] = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;
 
	Mpu6050_Gyo_Data[0] = ((((int16_t)mpu6050_buffer[ 8]) << 8) | mpu6050_buffer[ 9]) ;
	Mpu6050_Gyo_Data[1] = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) ;
	Mpu6050_Gyo_Data[2] = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) ;		
	
	if(mpu6050_offset_ok==1){
		for(i=0;i<3;i++){
			Mpu6050_Acc_Data[i]-=Mpu6050_Acc_Offset_Data[i];
			Mpu6050_Gyo_Data[i]-=Mpu6050_Gyo_Offset_Data[i];
		}
	}


//----------------------filter deal----------------------------	
	if(MPU6050_FILTER_CNT>=MPU6050_FILTER_LENGTH)
		MPU6050_FILTER_CNT=0;
	
	for(i=0;i<3;i++){
		MPU6050_ACC_FILTER_BUF[i][MPU6050_FILTER_CNT]=Mpu6050_Acc_Data[i];
		MPU6050_GYO_FILTER_BUF[i][MPU6050_FILTER_CNT]=Mpu6050_Gyo_Data[i];
	}
	
	MPU6050_FILTER_CNT++;
	MPU6050_ACC_FILTER_SUM[0]=MPU6050_ACC_FILTER_SUM[1]=MPU6050_ACC_FILTER_SUM[2]=0;
	MPU6050_GYO_FILTER_SUM[0]=MPU6050_GYO_FILTER_SUM[1]=MPU6050_GYO_FILTER_SUM[2]=0;
	for(i=0;i<3;i++){
		for(j=0;j<MPU6050_FILTER_LENGTH;j++){
			MPU6050_ACC_FILTER_SUM[i]+=MPU6050_ACC_FILTER_BUF[i][j];		
			MPU6050_GYO_FILTER_SUM[i]+=MPU6050_GYO_FILTER_BUF[i][j];		
		}
	}
	
	Mpu6050_Acc_Data[0] = MPU6050_ACC_FILTER_SUM[0]/MPU6050_FILTER_LENGTH;
	Mpu6050_Acc_Data[1] = MPU6050_ACC_FILTER_SUM[1]/MPU6050_FILTER_LENGTH;
	Mpu6050_Acc_Data[2] = MPU6050_ACC_FILTER_SUM[2]/MPU6050_FILTER_LENGTH;
	
	Mpu6050_Gyo_Data[0] = MPU6050_GYO_FILTER_SUM[0]/MPU6050_FILTER_LENGTH;
	Mpu6050_Gyo_Data[1] = MPU6050_GYO_FILTER_SUM[1]/MPU6050_FILTER_LENGTH;
	Mpu6050_Gyo_Data[2] = MPU6050_GYO_FILTER_SUM[2]/MPU6050_FILTER_LENGTH;	
//--------------------------------------------------------------------	
	
	for(i=0;i<3;i++){
		Mpu6050_Gyo_Deg_Data[i] = (float)Mpu6050_Gyo_Data[i] *TO_ANGLE;
	}
}


/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
	u8 b;
	IIC_Read_nByte(dev, reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	mpu6050_ok = !( IIC_Write_1Byte(dev, reg, b) );
}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
	
	u8 b,mask;
	IIC_Read_nByte(dev, reg, 1, &b);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	b &= mask;
	b |= data;
	IIC_Write_1Byte(dev, reg, b);
}

/**************************实现函数********************************************
*函数原型:		
*功　　能:	    设置 采样率
*******************************************************************************/
void MPU6050_set_SMPLRT_DIV(uint16_t hz)
{
	IIC_Write_1Byte(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV,1000/hz - 1);
		//I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_SMPLRT_DIV, (1000/sample_rate - 1));
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
	
}
/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG,7, 3, 0x00);   //不自检
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG,7, 3, 0x00);   //不自检
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
enabled =1   睡觉
enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_ADDR, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/**************************实现函数********************************************
*函数原型:		
*功　　能:	    设置低通滤波截止频率
*******************************************************************************/
void MPU6050_setDLPF(uint8_t mode)
{
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void MPU6050_INT_Config()
{
//	GPIO_InitTypeDef GPIO_InitStructure;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
////	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//	GPIO_Init(GPIOD, &GPIO_InitStructure);
//	
//	GPIO_SetBits(GPIOD, GPIO_Pin_7);	

}
/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/

void Delay_ms(unsigned int x)
{
	unsigned int i,j;
	for(i=0;i<x;i++)
		for(j=0;j<50000;j++);
}
void MPU6050_Init(u16 lpf)
{ 
	u16 default_filter = 1;
	I2c_Soft_Init();
	MPU6050_INT_Config();
	
  switch(lpf)
	{
	case 5:
			default_filter = MPU6050_DLPF_BW_5;
			break;
	case 10:
			default_filter = MPU6050_DLPF_BW_10;
			break;
	case 20:
			default_filter = MPU6050_DLPF_BW_20;
			break;
	case 42:
			default_filter = MPU6050_DLPF_BW_42;
			break;
	case 98:
			default_filter = MPU6050_DLPF_BW_98;
			break;
	case 188:
			default_filter = MPU6050_DLPF_BW_188;
			break;
	case 256:
			default_filter = MPU6050_DLPF_BW_256;
			break;
	default:
			default_filter = MPU6050_DLPF_BW_42;
			break;
	}
	

	//设备复位
//	IIC_Write_1Byte(MPU6050_ADDR,MPU6050_RA_PWR_MGMT_1, 0x80);
	I2C_FastMode=0;
	MPU6050_setSleepEnabled(0); //进入工作状态
	Delay_ms(10);
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO); //设置时钟  0x6b   0x03
	Delay_ms(10);
	MPU6050_set_SMPLRT_DIV(1000);  //1000hz
	Delay_ms(10);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
	Delay_ms(10);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8);	//加速度度最大量程 +-8G
	Delay_ms(10);
	MPU6050_setDLPF(default_filter);  //42hz
	Delay_ms(10);
	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	Delay_ms(10);
	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
	Delay_ms(10);
	
	mpu6050_offset_ok=MPU6050_Get_Offset();
	
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
