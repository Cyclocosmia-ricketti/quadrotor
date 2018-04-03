
#include "ANO_DT.h"
#include "mpu6050.h"
#include "Tim2_PID.h"
#include "Tim3_Cap.h"
#include "MahonyAHRS.h"


//数据拆分宏定义，在发送大于1字节的数据类型时，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;			//需要发送数据的标志
u8 data_to_send[50];	//发送数据缓存
u8 RxData;

  double pitch_rate_now; 
  double roll_rate_now;
  double yaw_rate_now;  

  extern double pitch_ctrl; 
  extern double roll_ctrl;
  extern double yaw_ctrl;
  extern double height_ctrl;

	extern double pitch_I;
	extern double roll_I;
	extern double yaw_I;

	extern double roll_rate_I;
	extern double pitch_rate_I;
	extern double yaw_rate_I;


	extern double pitch_error_rate;
	extern double roll_error_rate;
	
	extern double roll_target_rate[2];
	extern double pitch_target_rate[2];
	
	extern double pitch_msur[3];
	extern double roll_msur[3];
	extern double yaw_msur[3];
	
	extern double pitch_ctrl_init;
 extern double roll_ctrl_init;
 extern double yaw_ctrl_init;
 
 extern double pitch_bias_ctrl;
 extern double roll_bias_ctrl;
 extern double yaw_bias_ctrl;
 
	float test_angle_error_x,test_angle_error_y,test_angle_error_z;


uint16_t output1,output2,output3,output4;

PID_paras gyro_inter_PID[3];
PID_paras gyro_exter_PID[3];


/**********************************************************************
  * @name	ANO_DT_Data_Exchange
  * @brief  处理各种数据发送请求
  * @param  None
  * @retval None
  * @others	此函数应由用户每个工作周期调用一次
***********************************************************************/
void ANO_DT_Data_Exchange(void)
{
	static u8 cnt=0;
	
	if(cnt==0)
	{
		//ANO_DT_Send_Status(Angley,Anglex,Anglez,0,0,0);
		//ANO_DT_Send_Status(roll_rate_I*180/3.14159265359,pitch_rate_I*180/3.14159265359,yaw_msur[0]*180/3.14159265359,0,0,0);
		ANO_DT_Send_Status(roll_msur[0]*180/3.14159265359,pitch_msur[0]*180/3.14159265359,yaw_rate_I*180/3.14159265359,0,0,0);
		//ANO_DT_Send_Status(pitch_rate_now*180/3.14,pitch_msur[0]*180/3.14159265359,pitch_rate_I*180/3.14159265359,0,0,0);
		//ANO_DT_Send_Status(roll_rate_now*180/3.14,roll_msur[0]*180/3.14159265359,roll_rate_I*180/3.14159265359,0,0,0);
		//ANO_DT_Send_Status(roll_error_rate,kalAngleY,roll_rate_I,0,0,0);
		//ANO_DT_Send_Status(roll_rate_now,pitch_rate_now,yaw_rate_now,0,0,0);
		//ANO_DT_Send_Status(roll_ctrl,pitch_ctrl,yaw_ctrl,0,0,0);
		//ANO_DT_Send_Status(yaw_rate_now*180/3.14,pitch_msur[0]*180/3.14159265359,pitch_error_rate*180/3.14159265359,0,0,0);

			cnt++;

	}
	else if(cnt==1)
	{
		ANO_DT_Send_MotoPWM(output1,output2,output3,output4,0,0,0,0);

			cnt++;
	}
	else if(cnt==2)
	{
		//ANO_DT_Send_Senser(six_axis.accel_x,six_axis.accel_y,six_axis.accel_z,
//												six_axis.gyro_x,six_axis.gyro_y,six_axis.gyro_z,
//												0,0,0,0);
		ANO_DT_Send_Senser(pitch_rate_now*180/3.14159265*5,roll_rate_now*180/3.14159265*5,yaw_rate_now*180/3.14159265*5,
												-six_axis.gyro_x,six_axis.gyro_y,six_axis.gyro_z,
												pitch_ctrl*180/3.14159265*5,roll_ctrl*180/3.14159265*5,height_ctrl*5,0);
//		ANO_DT_Send_RCData(tempup1,yaw_ctrl_init,roll_ctrl_init,pitch_ctrl_init,0,0,0,0,0,0);//u16 thr,u16 yaw,u16 rol,u16 pit
		cnt=0;
	}


/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_version)
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
		ANO_DT_Send_PID(1,gyro_inter_PID[1].kp,gyro_inter_PID[1].ki,gyro_inter_PID[1].kd,
											gyro_inter_PID[0].kp,gyro_inter_PID[0].ki,gyro_inter_PID[0].kd,
											gyro_inter_PID[2].kp,gyro_inter_PID[2].ki,gyro_inter_PID[2].kd);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
		ANO_DT_Send_PID(2,gyro_exter_PID[1].kp,gyro_exter_PID[1].ki,gyro_exter_PID[1].kd,
											gyro_exter_PID[0].kp,gyro_exter_PID[0].ki,gyro_exter_PID[0].kd,
											gyro_exter_PID[2].kp,gyro_exter_PID[2].ki,gyro_exter_PID[2].kd);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid3)
	{
		f.send_pid3 = 0;
		ANO_DT_Send_PID(3,-pitch_bias_ctrl,roll_bias_ctrl,yaw_bias_ctrl,0,0,0,0,0,0);
	}
}

/**********************************************************************
  * @name	ANO_DT_Send_Data
  * @brief  使用USART实现发送一个数据帧
  * @param  u8 *dataToSend	需要发送的数据帧指针
			u8 length		需要发送的数据帧长度
  * @retval None
***********************************************************************/
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	u8 pos=0;
	while(pos!=length){
	USART1->DR=(u8)dataToSend[pos];
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){}
	pos++;
	}
}

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 sum = 0;
	u8 i;
	
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	

	for(i=0;i<6;i++)sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

/**********************************************************************
  * @name	ANO_DT_Data_Receive_Prepare
  * @brief  协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，
			格式正确的话再进行数据解析
  * @param  u8 data	USART接收到的字节
  * @retval None
  * @others 串口每收到一字节数据，则调用此函数一次
			此函数解析出符合格式的数据帧后，会自行调用数据解析函数
***********************************************************************/
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

/***********************************************************************
  * @name	ANO_DT_Data_Receive_Anl
  * @brief  协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数
			会首先对协议数据进行校验，校验通过后对数据进行解析，实现相应功能
  * @param  u8 *data_buf	数据缓存数组，即单个接收到的数据帧
			u8 num			数据帧长度
  * @retval None
***********************************************************************/
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	u8 i;
	
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
			TIM_SetCompare1(TIM4, 0);
			TIM_SetCompare2(TIM4, 0);
			TIM_SetCompare3(TIM4, 0);
			TIM_SetCompare4(TIM4, 0);
			TIM_Cmd(TIM2,DISABLE);	
		}
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
        gyro_inter_PID[1].kp= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        gyro_inter_PID[1].ki= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        gyro_inter_PID[1].kd= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        gyro_inter_PID[0].kp= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        gyro_inter_PID[0].ki= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        gyro_inter_PID[0].kd= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        gyro_inter_PID[2].kp= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        gyro_inter_PID[2].ki= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        gyro_inter_PID[2].kd= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
		gyro_exter_PID[1].kp= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        gyro_exter_PID[1].ki= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        gyro_exter_PID[1].kd= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        gyro_exter_PID[0].kp= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        gyro_exter_PID[0].ki= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        gyro_exter_PID[0].kd= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        gyro_exter_PID[2].kp= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        gyro_exter_PID[2].ki= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        gyro_exter_PID[2].kd= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
				pitch_bias_ctrl=- 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        roll_bias_ctrl= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        yaw_bias_ctrl= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		ANO_DT_Send_Check(*(data_buf+2),sum);
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}

/***********************************************************************
  * @name	ANO_DT_Send_Version
  * @brief  向上位机发送版本信息
  * @param  u8 hardware_type
			u16 hardware_ver
			u16 software_ver
			u16 protocol_ver
			u16 bootloader_ver
  * @retval None
***********************************************************************/
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{

}

/***********************************************************************
  * @name	ANO_DT_Send_Status
  * @brief  向上位机发送四旋翼姿态等基本信息
  * @param  float angle_rol	翻滚角
			float angle_pit	俯仰角
			float angle_yaw	自旋角
			s32 alt			气压值
			u8 fly_model	飞行模式
			u8 armed
  * @retval None
***********************************************************************/
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 i;
	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***********************************************************************
  * @name	ANO_DT_Send_Senser
  * @brief  向上位机发送四旋翼传感器数据
  * @param  s16 a_x,s16 a_y,s16 a_z	三轴加速度
			s16 g_x,s16 g_y,s16 g_z	三轴角加速度
			s16 m_x,s16 m_y,s16 m_z	三轴地磁强度
			s32 bar
  * @retval None
***********************************************************************/
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)
{
	u8 i;
	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***********************************************************************
  * @name	ANO_DT_Send_RCData
  * @brief  向上位机发送四旋翼接受到的遥控器数据
  * @param  u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6
  * @retval None
***********************************************************************/
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 i;
	u8 sum = 0;
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***********************************************************************
  * @name	ANO_DT_Send_Power
  * @brief  向上位机发送四旋翼的电池信息
  * @param  u16 votage	电池电压值
			u16 current	电池电流值
  * @retval None
***********************************************************************/
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 i;
	u8 _cnt=0;
	u16 temp;
	u8 sum = 0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***********************************************************************
  * @name	ANO_DT_Send_MotoPWM
  * @brief  向上位机发送四旋翼马达pwm占空比（70~190）
  * @param  u16 m_1,u16 m_2,u16 m_3,u16 m_4	四路马达pwm占空比（70~190）
			u16 m_5,u16 m_6,u16 m_7,u16 m_8	多轴保留接口，未使用
  * @retval None
***********************************************************************/
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 i;
	u8 sum = 0;
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***********************************************************************
  * @name	ANO_DT_Send_PID
  * @brief  向上位机发送四旋翼PID参数值
  * @param  u8 group	使用的PID参数组数
			float p1_p,float p1_i,float p1_d	PID1参数
			float p2_p,float p2_i,float p2_d	PID2参数
			float p3_p,float p3_i,float p3_d	PID3参数
  * @retval None
***********************************************************************/
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


/**********************************************************************
  * @name	USART1_Configuration
  * @brief  USART1初始化函数，配置USART1数据格式、波特率等参数
  * @param  None
  * @retval None
***********************************************************************/
void USART1_BlueTooth_Init( )
{
  		    
	USART_InitTypeDef USART_InitStructure; //串口设置恢复默认参数
	NVIC_InitTypeDef NVIC_InitStructure;    //定义一个中断结构体
	GPIO_InitTypeDef GPIO_InitStructure;    //定义GPIO初始化结构体

	//开启相关时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1 , ENABLE);
	
	//配置优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 	//通道设置为串口1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //中断响应优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 	//打开中断
	NVIC_Init(&NVIC_InitStructure);   					//初始化

	
	//将USART1 的TX 配置为复用推挽输出 AF_PP
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;     	//管脚位置定义。
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  //输出速度2MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;   //复用推挽输出 AF_PP
	GPIO_Init(GPIOA,&GPIO_InitStructure);     		//A组GPIO初始化
 
	//将USART1 的RX 配置为浮空输入 IN_FLOATING
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;   		//管脚位置定义
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  //输出速度2MHz 
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING; //浮空输入 IN_FLOATING                    
	GPIO_Init(GPIOA,&GPIO_InitStructure); 			//A组GPIO初始化

	//配置USART1数据格式、波特率等参数
	USART_InitStructure.USART_BaudRate = 115200; //波特率9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //1位停止字节
	USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;     //无流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      //打开Rx接收和Tx发送功能
	USART_Init(USART1, &USART_InitStructure);  //初始化
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // 若接收数据寄存器满，则产生中断
	USART_Cmd(USART1, ENABLE);                     //启动串口                   
                                                   
	//如下语句解决第1个字节无法正确发送出去的问题
	USART_ClearFlag(USART1, USART_FLAG_TC);     // 清发送完成标志
}


/**********************************************************************
  * @name	USART1_IRQHandler
  * @brief  USART1中断服务子程序
  * @param  None
  * @retval None
***********************************************************************/
void USART1_IRQHandler(void){
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET) //判断是否为接收中断
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);	//清除接受中断标志位
		RxData=USART_ReceiveData(USART1);				//读取接受数据
		ANO_DT_Data_Receive_Prepare(RxData);
	}
	
	if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET){	//判断是否为发送中断
		USART_ClearITPendingBit(USART1,USART_IT_TXE);	//清除发送中断标志位
	}
	
	if(USART_GetITStatus(USART1,USART_IT_TC)!=RESET){	//判断是否为发送完成中断
		USART_ClearITPendingBit(USART1,USART_IT_TC);	//清除发送完成中断标志位
	}
 }	 

