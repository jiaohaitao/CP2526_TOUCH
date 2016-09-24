
#include "cp2526.h"
#include "i2c_soft.h"

#define CP2526_ADDR		0x2C

unsigned char CP2526_Init()
{
	unsigned char tempbuf[12],i,j;
	unsigned short Setth,Clrth;
	
	IIC_Read_nByte(CP2526_ADDR,0,2,tempbuf);
	if(tempbuf[0]==0x25&&tempbuf[1]==0x28){}
	else
		return 1;
	
	Setth=100;
	Clrth=50;
	
	//
	tempbuf[0]=tempbuf[1]=0x00;
	IIC_Write_nByte(CP2526_ADDR,0x05,2,tempbuf);	
	
	//
	tempbuf[0]=0x00;tempbuf[1]=0x3f;
	IIC_Write_nByte(CP2526_ADDR,0x06,2,tempbuf);
	
	//
	for(i=0;i<6;i++){
		tempbuf[2*i]=Setth>>8;
		tempbuf[2*i+1]=Setth;
	}
	IIC_Write_nByte(CP2526_ADDR,0x07,12,tempbuf);

	//
	for(i=0;i<6;i++){
		tempbuf[2*i]=Clrth>>8;
		tempbuf[2*i+1]=Clrth;
	}
	IIC_Write_nByte(CP2526_ADDR,0x13,12,tempbuf);
	
	//
	IIC_Read_nByte(CP2526_ADDR,0x33,2,tempbuf);	
	
	return 0;
}
//500ms per read
unsigned char CP2526_Read()
{
	unsigned char tempbuf[2];
	
	if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5)==0){
			IIC_Read_nByte(CP2526_ADDR,0x31,2,tempbuf);
		  IIC_Read_nByte(CP2526_ADDR,0x33,2,tempbuf);	
	}	
}