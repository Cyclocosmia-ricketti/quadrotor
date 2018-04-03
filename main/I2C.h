#include "stm32f10x.h"
//

//
#define I2C_SDA			GPIO_Pin_11	 
#define I2C_SCL			GPIO_Pin_12
#define I2C_PORT   		GPIOA
//
#define I2C_SCL_0 		GPIO_ResetBits(I2C_PORT, I2C_SCL)
#define I2C_SCL_1 		GPIO_SetBits(I2C_PORT, I2C_SCL)
#define I2C_SDA_0 		GPIO_ResetBits(I2C_PORT, I2C_SDA)
#define I2C_SDA_1   	GPIO_SetBits(I2C_PORT, I2C_SDA)
//
#define I2C_SDA_STATE   	GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA)
#define I2C_DELAY 				I2C_Delay(100000)
#define I2C_NOP						I2C_Delay(10) 
//
#define I2C_READY					0x00
#define I2C_BUS_BUSY			0x01	
#define I2C_BUS_ERROR			0x02
//
#define I2C_NACK	  0x00 
#define I2C_ACK			0x01
//

//

//
void I2C_Configuration(void);
void I2C_Delay(unsigned int dly);
unsigned char  I2C_START(void);
void I2C_STOP(void);
void I2C_SendACK(void);
void I2C_SendNACK(void);
unsigned char  I2C_SendByte(unsigned char  i2c_data);
unsigned char  I2C_ReceiveByte_WithACK(void);
unsigned char  I2C_ReceiveByte_NoACK(void);
void I2C_Receive14Bytes(u8 *i2c_data_buffer);
