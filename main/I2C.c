#include "I2C.h"


void I2C_Configuration(void)			
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
		
	

	GPIO_InitStructure.GPIO_Pin = I2C_SCL | I2C_SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   					
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(I2C_PORT, &GPIO_InitStructure); 
	/*
	GPIO_InitStructure.GPIO_Pin = MPU6050_INT;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_Init(MPU6050_INT_PORT, &GPIO_InitStructure); 
	*/
	I2C_SCL_1; 
	I2C_SDA_1;
	I2C_DELAY;
}

void I2C_Delay(unsigned int dly)               
{
	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}


unsigned char I2C_START(void)
{ 
	I2C_SDA_1; 
 	I2C_NOP;
  // 
 	I2C_SCL_1; 
 	I2C_NOP;    
	//
 	if(!I2C_SDA_STATE) return I2C_BUS_BUSY;
	//
 	I2C_SDA_0;
 	I2C_NOP;
  //
 	I2C_SCL_0;  
 	I2C_NOP; 
	//
 	if(I2C_SDA_STATE) return I2C_BUS_ERROR;
	//
 	return I2C_READY;
}

void I2C_STOP(void)
{
 	I2C_SDA_0; 
 	I2C_NOP;
  // 
 	I2C_SCL_1; 
 	I2C_NOP;    
	//
 	I2C_SDA_1;
 	I2C_NOP;
}

void I2C_SendACK(void)
{
 	I2C_SDA_0;
 	I2C_NOP;
 	I2C_SCL_1;
 	I2C_NOP;
 	I2C_SCL_0; 
 	I2C_NOP;  
}

void I2C_SendNACK(void)
{
	I2C_SDA_1;
	I2C_NOP;
	I2C_SCL_1;
	I2C_NOP;
	I2C_SCL_0; 
	I2C_NOP;  
}

unsigned char I2C_SendByte(u8 i2c_data)
{
 	unsigned char i;
 	//
	I2C_SCL_0;
 	for(i=0;i<8;i++)
 	{  
  		if(i2c_data&0x80) I2C_SDA_1;
   		else I2C_SDA_0;
			//
  		i2c_data<<=1;
  		I2C_NOP;
			//
  		I2C_SCL_1;
  		I2C_NOP;
  		I2C_SCL_0;
  		I2C_NOP; 
 	}
	//
 	I2C_SDA_1; 
 	I2C_NOP;
 	I2C_SCL_1;
 	I2C_NOP;   
 	if(I2C_SDA_STATE)
 	{
  		I2C_SCL_0;
  		return I2C_NACK;
 	}
 	else
 	{
  		I2C_SCL_0;
  		return I2C_ACK;  
 	}    
}

unsigned char I2C_ReceiveByte_NoACK(void)
{
	unsigned char i,i2c_data;
	//
 	I2C_SDA_1;
 	I2C_SCL_0; 
 	i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		I2C_SCL_1;
  		I2C_NOP; 
  		i2c_data<<=1;
			//
  		if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  		I2C_SCL_0;  
  		I2C_NOP;         
 	}
	I2C_SendNACK();
 	return i2c_data;
}

unsigned char I2C_ReceiveByte_WithACK(void)
{
	unsigned char i,i2c_data;
	//
 	I2C_SDA_1;
 	I2C_SCL_0; 
 	i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		I2C_SCL_1;
  		I2C_NOP; 
  		i2c_data<<=1;
			//
  		if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  		I2C_SCL_0;  
  		I2C_NOP;         
 	}
	I2C_SendACK();
 	return i2c_data;
}

void I2C_Receive14Bytes(u8 *anbt_i2c_data_buffer)
{
	u8 i,j;
	u8 anbt_i2c_data;

	for(j=0;j<13;j++)
	{
		I2C_SDA_1;
		I2C_SCL_0; 
		anbt_i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		I2C_SCL_1;
  		I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		I2C_SCL_0;  
  		I2C_NOP;         
		}
		anbt_i2c_data_buffer[j]=anbt_i2c_data;
		I2C_SendACK();
	}
	//
	I2C_SDA_1;
	I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
  	I2C_SCL_1;
  	I2C_NOP; 
  	anbt_i2c_data<<=1;
			//
  	if(I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  	I2C_SCL_0;  
  	I2C_NOP;         
	}
	anbt_i2c_data_buffer[13]=anbt_i2c_data;
	I2C_SendNACK();
}
