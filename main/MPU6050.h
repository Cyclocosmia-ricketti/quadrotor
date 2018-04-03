

#include "stm32f10x.h"

#define ACC_SCALE	(16.0f*9.8f/32768.0f)
#define GYRO_SCALE	(250.0f/32768.0f)
#define DegToRad	0.0174533

#define MPU6050_GYRO_ADDR 			0xD0
#define MPU6050_GYRO_WHOAMI_ADDR 	0x75

#define MPU6050_USER_CTRL_ADDR		0x6A
#define MPU6050_USER_CTRL_VALUE		0x00

#define MPU6050_PWR_MGMT_1_ADDR		0x6B
#define MPU6050_PWR_MGMT_1_VALUE 	0x01
#define MPU6050_EXIT_SLEEP_VALUE 	0x01

#define MPU6050_GYRO_CFG_ADDR 		0x1B	//[4:3] Gyro Full Scale Select: 00 = +250dps, 01= +500dps, 10 = +1000dps, 11 = +2000dps
#define MPU6050_GYRO_CFG_VALUE 		0x00

#define MPU6050_ACCEL_CFG_ADDR 		0x1C	//[4:3] Accel Full Scale Select:¡À2g (00), ¡À4g (01), ¡À8g (10), ¡À16g (11)
#define MPU6050_ACCEL_CFG_VALUE 	0x18

#define MPU6050_I2CBYPASS_CFG_ADDR 		0x37
#define MPU6050_I2CBYPASS_CFG_VALUE 	0xB2		//bit7=1, bit6=0, bit5=1, bit4=1, bit3=0, bit2=0, bit1=1, bit0=0

#define MPU6050_INT_CFG_ADDR 		0x38
#define MPU6050_INT_CFG_VALUE 		0x01

#define MPU6050_ACCEL_DATA_ADDR 		0x3B

#define DEBUG_MPU6050
#define RAD_TO_DEG 57.2956f
#define DEG_TO_RAD 0.0174533f

typedef struct accel_gyro {

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	
	float Temperature;
	
}SIX_AXIS_DATA;

extern SIX_AXIS_DATA  six_axis;

unsigned char MPU6050_GYRO_WHOAMI_FUN(void);
unsigned char MPU6050_READ_REG_FUN(unsigned char mpu6050_dev_addr,unsigned char mpu6050_reg_addr);

void MPU6050_Init(void);
void Get_MPU6050_Data(void);
void get_Accel_Gyro_Temp(SIX_AXIS_DATA *six_axis);



