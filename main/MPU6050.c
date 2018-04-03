#include "mpu6050.h"
#include "I2C.h"
#include "systick.h"

SIX_AXIS_DATA  six_axis;

unsigned char mpu6050_data_buffer[14];
signed short int mpu6050_raw_data[7];

/********************************/
//     初始化函数子函数
/********************************/
void MPU6050_PWM_CFG_FUN(void);
void MPU6050_GYRO_CFG_FUN(void);
void MPU6050_ACCEL_CFG_FUN(void);
void MPU6050_EXIT_SLEEP_FUN(void);
void MPU6050_I2CBYPASS_CFG_FUN(void);
void MPU6050_USER_CTRL_FUN(void); 
void MPU6050_INT_CFG_FUN(void);
void MPU6050_Init(void);

void Get_MPU6050_Data(void)   
{
	unsigned char i;
	
	//
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR);					//圆点博士:发送陀螺仪写地址
	I2C_SendByte(MPU6050_ACCEL_DATA_ADDR);    //圆点博士:发送陀螺仪寄存器地址
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR+1);      	//圆点博士:发送陀螺仪读地址
	I2C_Receive14Bytes(mpu6050_data_buffer);		//圆点博士:读出陀螺仪寄存器值
	I2C_STOP();
	//				
	for(i=0;i<7;i++) mpu6050_raw_data[i]=(((signed short int)mpu6050_data_buffer[i*2]) << 8) | mpu6050_data_buffer[i*2+1];
}

void get_Accel_Gyro_Temp(SIX_AXIS_DATA *six_axis)
{
	Get_MPU6050_Data();
	six_axis->accel_x=mpu6050_raw_data[0];
	six_axis->accel_y=mpu6050_raw_data[1];
	six_axis->accel_z=mpu6050_raw_data[2];
	
	six_axis->gyro_x=mpu6050_raw_data[4];
	six_axis->gyro_y=mpu6050_raw_data[5];
	six_axis->gyro_z=mpu6050_raw_data[6];
	
	six_axis->Temperature=(float)(((float)mpu6050_raw_data[3])/340.0f + 36.53f);
}

void MPU6050_Init(void)	
{
	//
	MPU6050_PWM_CFG_FUN(); 				//圆点博士:设置内部时钟
	MPU6050_EXIT_SLEEP_FUN();    	//圆点博士:退出休眠模式
	MPU6050_GYRO_CFG_FUN();      	//圆点博士:设置陀螺仪量程
	MPU6050_ACCEL_CFG_FUN();     	//圆点博士:设置加速度量程
	MPU6050_USER_CTRL_FUN();
	MPU6050_I2CBYPASS_CFG_FUN(); 	//圆点博士:设置电磁读写模式
	MPU6050_INT_CFG_FUN();
	//
}

unsigned char MPU6050_GYRO_WHOAMI_FUN(void)
{
	unsigned char mpu6050_gyro_id;
	//
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR);					//圆点博士:发送陀螺仪写地址
	I2C_SendByte(MPU6050_GYRO_WHOAMI_ADDR);  	//圆点博士:发送陀螺仪ID地址
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR+1);      	//圆点博士:发送陀螺仪读地址
	mpu6050_gyro_id=I2C_ReceiveByte_NoACK();	//圆点博士:读出陀螺仪ID
	I2C_STOP();
	//
	return mpu6050_gyro_id;
	//
}

unsigned char MPU6050_READ_REG_FUN(unsigned char mpu6050_dev_addr,unsigned char mpu6050_reg_addr)   
{
	unsigned char mpu6050_reg;
	
	I2C_START();
	I2C_SendByte(mpu6050_dev_addr);					//圆点博士:发送陀螺仪写地址
	I2C_SendByte(mpu6050_reg_addr);    			//圆点博士:发送陀螺仪寄存器地址
	I2C_START();
	I2C_SendByte(mpu6050_dev_addr+1);      	//圆点博士:发送陀螺仪读地址
	mpu6050_reg=I2C_ReceiveByte_NoACK();		//圆点博士:读出陀螺仪寄存器值
	I2C_STOP();
	//
	return mpu6050_reg;
}

void MPU6050_PWM_CFG_FUN(void)   
{
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR);				//圆点博士:发送陀螺仪写地址
	I2C_SendByte(MPU6050_PWR_MGMT_1_ADDR);  //圆点博士:发送陀螺仪PWM地址
	I2C_SendByte(MPU6050_PWR_MGMT_1_VALUE); //圆点博士:发送陀螺仪PWM值
	I2C_STOP();
}
//
void MPU6050_GYRO_CFG_FUN(void)   
{
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR);				//圆点博士:发送陀螺仪写地址
	I2C_SendByte(MPU6050_GYRO_CFG_ADDR);   	//圆点博士:发送陀螺仪PWM地址
	I2C_SendByte(MPU6050_GYRO_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	I2C_STOP();
}
//
void MPU6050_ACCEL_CFG_FUN(void)   
{
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR);					//圆点博士:发送陀螺仪写地址
	I2C_SendByte(MPU6050_ACCEL_CFG_ADDR);   //圆点博士:发送陀螺仪PWM地址
	I2C_SendByte(MPU6050_ACCEL_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	I2C_STOP();
}

void MPU6050_EXIT_SLEEP_FUN(void)  
{
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR);					//圆点博士:发送陀螺仪写地址
	I2C_SendByte(MPU6050_PWR_MGMT_1_ADDR);  //圆点博士:发送陀螺仪PWM地址
	I2C_SendByte(MPU6050_EXIT_SLEEP_VALUE); //圆点博士:发送陀螺仪PWM值
	I2C_STOP();
}

void MPU6050_USER_CTRL_FUN(void)   
{
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR);			 //圆点博士:发送陀螺仪写地址
	I2C_SendByte(MPU6050_USER_CTRL_ADDR);  //圆点博士:发送陀螺仪PWM地址
	I2C_SendByte(MPU6050_USER_CTRL_VALUE); //圆点博士:发送陀螺仪PWM值
	I2C_STOP();
}

void MPU6050_I2CBYPASS_CFG_FUN(void)   
{
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR);					  //圆点博士:发送陀螺仪写地址
	I2C_SendByte(MPU6050_I2CBYPASS_CFG_ADDR);   //圆点博士:发送陀螺仪PWM地址
	I2C_SendByte(MPU6050_I2CBYPASS_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	I2C_STOP();
}

void MPU6050_INT_CFG_FUN(void)   
{
	I2C_START();
	I2C_SendByte(MPU6050_GYRO_ADDR);			//圆点博士:发送陀螺仪写地址
	I2C_SendByte(MPU6050_INT_CFG_ADDR);   //圆点博士:发送陀螺仪PWM地址
	I2C_SendByte(MPU6050_INT_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	I2C_STOP();
}


