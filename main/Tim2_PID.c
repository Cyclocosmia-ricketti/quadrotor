#include "Tim2_PID.h"
#include "Tim3_Cap.h"
#include "math.h"

#include "mpu6050.h"
#include "I2C.h"
#include "ANO_DT.h"
#include "MahonyAHRS.h"

#define pitch 0
#define roll 1
#define yaw 2


#define pitch_max 2.5					//pitch 的积分截止值
#define yaw_max 0.5					//yaw 的积分截止值
#define roll_max 2.5						//roll 的积分截止值

#define pitch_rate_max 80					//pitch角速度 的积分截止值
#define yaw_rate_max 50					//yaw角速度 的积分截止值
#define roll_rate_max 80					//roll角速度 的积分截止值

#define pitch_target_rate_max 5
#define roll_target_rate_max 5
#define yaw_target_rate_max 5


u8 ready=0;

extern float test_angle_error_x,test_angle_error_y,test_angle_error_z;


extern PID_paras gyro_inter_PID[3];
extern PID_paras gyro_exter_PID[3];

 double t_update = 0.01;
 
 extern uint16_t output1,output2,output3,output4;

 double rate_0=0;					//平均转速占空比
 double rate_1=0;					//俯仰差动占空比
 double rate_2=0;					//侧翻差动占空比
 double rate_r=0;					//偏航差动占空比

 double pitch_ctrl=0;				//角度控制量（弧度制）
 double yaw_ctrl=0;				//角度控制量（弧度制）
 double yaw_stat;
 double roll_ctrl=0;				//角度控制量（弧度制）
 double height_ctrl=0;				//高度控制量
 
// double pitch_msur_init;				//角度测量量初始零偏（弧度制）
// double yaw_msur_init;				//角度测量量初始零偏（弧度制）
// double roll_msur_init;				//角度测量量初始零偏（弧度制）

 double pitch_msur[3];				//角度测量量（弧度制）
 double yaw_msur[3];				//角度测量量（弧度制）
 int yaw_circle_num;				//偏航圈数
 double roll_msur[3];				//角度测量量（弧度制）

 double pitch_err[3];			//角度误差量（弧度制）
 double yaw_err[3];				//角度误差量（弧度制）
 double roll_err[3];				//角度误差量（弧度制）

 double pitch_rate_I;					//角速度内环积分值（弧度制）
 double yaw_rate_I=90*3.14159265/180;					//角速度内环积分值（弧度制）
 double roll_rate_I;					//角速度内环积分值（弧度制）
 double roll_target_rate_I;
 
 double pitch_I;					//角度外环积分值（弧度制）
 double yaw_I;					//角度外环积分值（弧度制）
 double roll_I;					//角度外环积分值（弧度制）
 
 double pitch_target_rate[2]; 	//目标角速度（弧度制）
 double roll_target_rate[2]; 	//目标角速度（弧度制）
 double yaw_target_rate[2]; 	//目标角速度（弧度制）
 
 extern double pitch_rate_now; 
 extern double roll_rate_now;
 extern double yaw_rate_now;  
 
 double pitch_rate_pre;
 double roll_rate_pre;
 double yaw_rate_pre;
 
 double pitch_rate[4];
 double roll_rate[4];
 double yaw_rate[4];
 
 
 double pitch_error_rate ;
 double roll_error_rate  ;
 double yaw_error_rate	 ;
 double pitch_error_rate_diff ;
 double roll_error_rate_diff  ;
 double yaw_error_rate_diff		;
 
 double pitch_error_diff;
 double roll_error_diff;
 double yaw_error_diff;
 
 double pitch_rate_msur_init=-2.144*3.1415926/180;
 double roll_rate_msur_init= 1.4118*3.1415926/180;
 double yaw_rate_msur_init=		-1.630*3.1415926/180;
 
 double pitch_msur_init=0;
 double roll_msur_init=0;
 double yaw_msur_init=0;
 
 double init_count_num=0;
 
 double pitch_ctrl_init=0;
 double roll_ctrl_init=0;
 double yaw_ctrl_init=0;
 
 double pitch_bias_ctrl=0;
 double roll_bias_ctrl=0;
 double yaw_bias_ctrl=0;
 
 double height_ctrl_compens=0;
 

 
 //double x_before=0;				//角速度上一次测量量（弧度制）
 //double y_before=0;				//角速度上一次测量量（弧度制）
 //double z_before=0;				//角速度上一次测量量（弧度制）


void readInput() {
	if(tempup1<rc_thr_max) 	height_ctrl	=rc_thr_range*((double)tempup1-rc_thr_min)/(rc_thr_max-rc_thr_min);
	if(height_ctrl<10) height_ctrl=height_ctrl*4.5;
	else height_ctrl=45+(height_ctrl-10)*0.2;
	if(tempup2<rc_yaw_max && tempup2>rc_yaw_min) 	yaw_ctrl	=rc_yaw_range*((double)tempup2)/(rc_yaw_max-rc_yaw_min)-yaw_ctrl_init;
	if(tempup3<rc_pit_max && tempup3>rc_pit_min) 	pitch_ctrl	=rc_pit_range*((double)tempup3)/(rc_pit_max-rc_pit_min)-pitch_ctrl_init;
	if(tempup4<rc_rol_max && tempup4>rc_rol_min) 	roll_ctrl	=-rc_rol_range*((double)tempup4)/(rc_rol_max-rc_rol_min)-roll_ctrl_init;
	
	if(height_ctrl<40){
		pitch_rate_I=0;
		roll_rate_I=0;
		yaw_rate_I =90*3.14159265/180; 
		
		pitch_I =0;
		roll_I  =0;
		yaw_I   =0;
	}
}

void pwmOutput() 
{
	output1=(uint16_t)(10*(rate_0+rate_1-rate_r+70));
	output2=(uint16_t)(10*(rate_0+rate_2+rate_r+70));
	output3=(uint16_t)(10*(rate_0-rate_1-rate_r+70));
	output4=(uint16_t)(10*(rate_0-rate_2+rate_r+70));
	
	if(output1<720)output1=720;
	if(output2<720)output2=720;
	if(output3<720)output3=720;
	if(output4<720)output4=720;
	
	if(output1>1900)output1=1900;
	if(output2>1900)output2=1900;
	if(output3>1900)output3=1900;
	if(output4>1900)output4=1900;
	
	if(height_ctrl<10){
		output1=720;
		output2=720;
		output3=720;
		output4=720;
	}

	TIM_SetCompare1(TIM4, output1);
	TIM_SetCompare2(TIM4, output2);
	TIM_SetCompare3(TIM4, output3);
	TIM_SetCompare4(TIM4, output4);

}


void TIM2_PIB_Init(u16 arr,u16 psc){
	int i=0;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	
/******************************************************************/
	for(i=0;i<3;i++){
		gyro_inter_PID[i].kp=0;
		gyro_inter_PID[i].ki=0;
		gyro_inter_PID[i].kd=0;
		gyro_exter_PID[i].kp=0;
		gyro_exter_PID[i].ki=0;
		gyro_exter_PID[i].kd=0;
	}
/******************************************************************/
	gyro_inter_PID[pitch].kp=15.0;
	gyro_inter_PID[pitch].kd=0.125;
	gyro_inter_PID[pitch].ki=0.5;
	gyro_inter_PID[roll].kp=18.0;
	gyro_inter_PID[roll].kd=0.15;
	gyro_inter_PID[roll].ki=0.5;
	gyro_inter_PID[yaw].kp=12.0;
	gyro_inter_PID[yaw].kd=0.0;
	gyro_inter_PID[yaw].ki=6.0;
	
	gyro_exter_PID[roll].kp=3.0;
	gyro_exter_PID[roll].ki=0.0;
	gyro_exter_PID[pitch].kp=3.0;
	
	
	I2C_Configuration();
	MPU6050_Init();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 

	TIM_TimeBaseStructure. TIM_Period = arr;		  //配置计数阈值为999，超过时，自动清零，并触发中断
	TIM_TimeBaseStructure.TIM_Prescaler = psc;	   //	 时钟预分频值，除以多少
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 // 时钟分频倍数
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	 // 计数方式为向上计数

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);		 //	 初始化tim2
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //清除TIM2溢出中断标志
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //  使能TIM2的溢出更新中断
    TIM_Cmd(TIM2,ENABLE);					  //		   使能TIM2

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //要用同一个Group
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM2	溢出更新中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//	打断优先级为1，与上一个相同，不希望中断相互打断对方
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	//	响应优先级1，低于上一个，当两个中断同时来时，上一个先执行
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	t_update=(psc+1)*(arr+1)/72000000.0;
	
	readInput();
	yaw_ctrl_init=yaw_ctrl;
	pitch_ctrl_init=pitch_ctrl;
	roll_ctrl_init=roll_ctrl;
	
	ready=0;
	
}


void TIM2_IRQHandler(void)		//	  //TIM2的溢出更新中断响应函数，产生pwm波
{	
	
	int i=0;
	
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);		 //	 清空TIM2溢出中断响应函数标志位
	
	get_Accel_Gyro_Temp(&six_axis);
	
MahonyAHRSupdateIMU((float)(six_axis.gyro_x)*GYRO_SCALE*DegToRad-pitch_rate_msur_init,  	(float)(six_axis.gyro_y)*GYRO_SCALE*DegToRad-roll_rate_msur_init, 	(float)(six_axis.gyro_z)*GYRO_SCALE*DegToRad-yaw_rate_msur_init,
						(float)six_axis.accel_x/**ACC_SCALE*/,  			(float)six_axis.accel_y/**ACC_SCALE*/,				(float)six_axis.accel_z/**ACC_SCALE*/);	
	
//	test_angle_error_x=(float)six_axis.accel_x;
//	test_angle_error_y=(float)six_axis.accel_y;
//	test_angle_error_z=(float)six_axis.gyro_z;
	
	
	
	
	
	
	
	
		for(i=2;i>0;i--){
			pitch_msur[i]=pitch_msur[i-1];
			yaw_msur[i]=yaw_msur[i-1];
			roll_msur[i]=roll_msur[i-1];
		}
		pitch_msur[0]=(-Angley-0.8426)*3.1415926/180;
		roll_msur[0]= (Anglex-4.193)*3.1415926/180;
		yaw_msur[0]= Anglez*3.1415926/180;//z轴要矫正正负反馈
//	if (ready==0){
//			init_count_num+=1;
//		if(init_count_num>150&&init_count_num<500){
//			pitch_msur_init+=pitch_msur[0];
//			roll_msur_init+=roll_msur[0];
//			yaw_msur_init+=yaw_msur[0];
//		}
//		else if(init_count_num>=500){
//			pitch_msur_init/=(init_count_num-150);
//			roll_msur_init/=(init_count_num-150);
//			yaw_msur_init/=(init_count_num-150);
//			ready=1;
//		}
//	}
//	else {
		pitch_msur[0]-=pitch_msur_init;
		roll_msur[0]-=roll_msur_init;
		yaw_msur[0]-=yaw_msur_init;
	/**************************************************************/
	//更新PID参数
	/**************************************************************/
	
	//更新期望值

	readInput();
	pitch_ctrl+=pitch_bias_ctrl*3.141592653/180;
	roll_ctrl+=roll_bias_ctrl*3.141592653/180;

	// 更新误差向量
	for(i=2;i>0;i--){
		pitch_err[i]=pitch_err[i-1];
		yaw_err[i]=yaw_err[i-1];
		roll_err[i]=roll_err[i-1];
	}
	pitch_err[0]=pitch_msur[0]-pitch_ctrl;
	roll_err[0]=roll_msur[0]-roll_ctrl;
//	if(yaw_msur[0]-yaw_msur[1]>1.57)yaw_circle_num--;
//	if(yaw_msur[0]-yaw_msur[1]<-1.57)yaw_circle_num++;
//	if (yaw_ctrl>4*3.14/180||yaw_ctrl<-4*3.14/180) yaw_stat=yaw_msur[0]+yaw_circle_num*2*3.1415926535;
//	else yaw_ctrl=0;
//	yaw_err[0]=yaw_msur[0]+yaw_circle_num*2*3.1415926535-yaw_stat;//此处角度error可能因为跨过180度产生问题。要在姿态角解算中完成+-180的平缓衰减
	
//	if (yaw_ctrl>4*3.14/180||yaw_ctrl<-4*3.14/180) yaw_stat=yaw_msur[0];
//	else yaw_ctrl=0;
	yaw_stat=yaw_msur[0];
	yaw_err[0]=yaw_msur[0]-yaw_stat;//此处角度error可能因为跨过180度产生问题。要在姿态角解算中完成+-180的平缓衰减
	
	/**************************************************************/
	
	
	/**************************************************************/
	//PID计算
	/**************************************************************/
	//更新角速度缓冲器
	
	for(i=3;i>0;i--){
		pitch_rate[i]=pitch_rate[i-1];
		roll_rate[i] =roll_rate[i-1];
		yaw_rate[i]	 =yaw_rate[i-1];
	}
	pitch_rate[0]=-(six_axis.gyro_x /7510.0-pitch_rate_msur_init);
	roll_rate[0] =six_axis.gyro_y /7510.0-roll_rate_msur_init;
	yaw_rate[0]	 =six_axis.gyro_z /7510.0-yaw_rate_msur_init;
	
	pitch_rate[0]=pitch_rate[0]*0.45+pitch_rate[1]*0.55;
	roll_rate[0] =roll_rate[0]*0.45+roll_rate[1]*0.55;
	yaw_rate[0]	 =yaw_rate[0]*0.45+yaw_rate[1]*0.55;
	
	
	//定义基本量
	pitch_rate_pre = pitch_rate_now;
	 roll_rate_pre  = roll_rate_now;
	 yaw_rate_pre   = yaw_rate_now;
	
	pitch_rate_now=pitch_rate[0];
	roll_rate_now =roll_rate[0];
	yaw_rate_now	=yaw_rate[0];
	
	
	 
//	 pitch_rate_now = ((pitch_msur[0] - pitch_msur[1])/ t_update);
//	if 			(pitch_rate_now> 1.2/t_update) pitch_rate_now=pitch_rate_pre;
//	else if (pitch_rate_now<-1.2/t_update) pitch_rate_now=pitch_rate_pre;
//	 roll_rate_now  = ((roll_msur[0]  - roll_msur[1]) / t_update);
//	if      (roll_rate_now> 1.2/t_update) roll_rate_now=roll_rate_pre;
//	else if (roll_rate_now<-1.2/t_update) roll_rate_now=roll_rate_pre;
//	 yaw_rate_now   = ((yaw_msur[0]   - yaw_msur[1])  / t_update);

//		pitch_rate_now=-((pitch_rate[0]+pitch_rate[1]+pitch_rate[2]+pitch_rate[3])/4-pitch_rate_msur_init);
//		roll_rate_now=(roll_rate[0]+roll_rate[1]+roll_rate[2]+roll_rate[3])/4-roll_rate_msur_init;
//		yaw_rate_now=(yaw_rate[0]+yaw_rate[1]+yaw_rate[2]+yaw_rate[3])/4-yaw_rate_msur_init;
 
	//更新外环积分值
	if((pitch_I<pitch_max && pitch_I>-pitch_max)||pitch_I*pitch_err[0]<0)	pitch_I += pitch_err[0] * t_update;
	if((roll_I<roll_max   && roll_I>-roll_max)  ||roll_I *roll_err[0]<0)	roll_I  += roll_err[0]  * t_update;
	if((yaw_I<yaw_max     && yaw_I>-yaw_max)    ||yaw_I  *yaw_err[0]<0)		yaw_I		+= yaw_err[0]		* t_update;
	
	//更新外环微分值
	pitch_error_diff=(pitch_err[0]-pitch_err[1])/t_update;
	roll_error_diff=(roll_err[0]-roll_err[1])/t_update;
	yaw_error_diff=(yaw_err[0]-yaw_err[1])/t_update;
	
	//目标角速度限幅（角度模式）
	pitch_target_rate[1]=pitch_target_rate[0];
	roll_target_rate[1] =roll_target_rate[0];
  yaw_target_rate[1]	=yaw_target_rate[0];
	pitch_target_rate[0]=-gyro_exter_PID[pitch].kp * pitch_err[0]-gyro_exter_PID[pitch].ki*pitch_I-gyro_exter_PID[pitch].kd*pitch_error_diff;
	roll_target_rate[0] =-gyro_exter_PID[roll].kp  * roll_err[0] -gyro_exter_PID[roll].ki *roll_I	-gyro_exter_PID[roll].kd *roll_error_diff;
  yaw_target_rate[0]	=-gyro_exter_PID[yaw].kp	 * yaw_err[0]	 -gyro_exter_PID[yaw].ki	*yaw_I	-gyro_exter_PID[yaw].kd	 *yaw_error_diff  +yaw_ctrl;	
	if(pitch_target_rate[0]>pitch_target_rate_max) pitch_target_rate[0]=pitch_target_rate_max;
	if(roll_target_rate[0] >roll_target_rate_max)  roll_target_rate[0] =roll_target_rate_max;
	if(yaw_target_rate[0]	 >yaw_target_rate_max) 	 yaw_target_rate[0]	 =yaw_target_rate_max;
	if(pitch_target_rate[0]<-pitch_target_rate_max) pitch_target_rate[0]=-pitch_target_rate_max;
	if(roll_target_rate[0] <-roll_target_rate_max)  roll_target_rate[0] =-roll_target_rate_max;
	if(yaw_target_rate[0]	 <-yaw_target_rate_max) 	yaw_target_rate[0]	=-yaw_target_rate_max;
	
	//目标角速度限幅（角速度模式）
//	pitch_target_rate[1]=pitch_target_rate[0];
//	roll_target_rate[1] =roll_target_rate[0];
//	yaw_target_rate[1] =yaw_target_rate[0];
//	pitch_target_rate[0]=pitch_ctrl;
//	roll_target_rate[0] =roll_ctrl;
//	yaw_target_rate[0]	=yaw_ctrl;
	
	//计算（定义）外环输出作为内环输入
	 pitch_error_rate =(pitch_rate_now - pitch_target_rate[0]);
	 roll_error_rate  =(roll_rate_now  - roll_target_rate[0]);
	 yaw_error_rate		=(yaw_rate_now	 - yaw_target_rate[0]);
	 pitch_error_rate_diff =(((pitch_rate_now - pitch_target_rate[0])- (pitch_rate_pre-pitch_target_rate[1]))/ t_update);
	 roll_error_rate_diff  =(((roll_rate_now  - roll_target_rate[0])-  (roll_rate_pre -roll_target_rate[1]) )/ t_update);
	 yaw_error_rate_diff	 =(((yaw_rate_now   - yaw_target_rate[0])-   (yaw_rate_pre  -yaw_target_rate[1])  )/ t_update);
	
	//更新内环积分值
	if((pitch_rate_I*gyro_inter_PID[pitch].ki<pitch_rate_max && pitch_rate_I*gyro_inter_PID[pitch].ki>-pitch_rate_max)||pitch_rate_I*pitch_error_rate<0)	pitch_rate_I += pitch_error_rate * t_update;
//	if((roll_rate_I<roll_rate_max   && roll_rate_I>-roll_rate_max)  ||roll_rate_I *roll_error_rate<0){
//		roll_target_rate_I +=roll_target_rate[0]*t_update;
//		roll_rate_I  = roll_msur[0] -	roll_target_rate_I;
//	}
	if((roll_rate_I*gyro_inter_PID[roll].ki<roll_rate_max   && roll_rate_I*gyro_inter_PID[roll].ki>-roll_rate_max)  ||roll_rate_I *roll_error_rate<0)		roll_rate_I  += roll_error_rate  * t_update;
	if((yaw_rate_I*gyro_inter_PID[yaw].ki<yaw_rate_max     && yaw_rate_I*gyro_inter_PID[yaw].ki>-yaw_rate_max)    ||yaw_rate_I  *yaw_error_rate<0)		yaw_rate_I	 += yaw_error_rate	 * t_update;
	
	if(height_ctrl<40){
		pitch_rate_I=0;
		roll_rate_I=0;
		yaw_rate_I =90*3.14159265/180;
		roll_target_rate_I=0;		
		
		pitch_I =0;
		roll_I  =0;
		yaw_I   =0;
	}
	//双闭环PID解算转角动力
		height_ctrl_compens=(pitch_msur[0]*pitch_msur[0]+roll_msur[0]*roll_msur[0])/2;
	if(height_ctrl_compens>0.1)height_ctrl_compens=0.1;
		rate_0=height_ctrl*(1+height_ctrl_compens);
		rate_1=- gyro_inter_PID[pitch].kp * pitch_error_rate - gyro_inter_PID[pitch].kd * pitch_error_rate_diff - gyro_inter_PID[pitch].ki * pitch_rate_I;//-Pi*((pitch_err[0]-pitch_err[1])/t_update+P1/I1*pitch_err[0]);
		rate_2=- gyro_inter_PID[roll].kp  * roll_error_rate  - gyro_inter_PID[roll].kd  * roll_error_rate_diff  - gyro_inter_PID[roll].ki  * roll_rate_I;//-Pi*((roll_err[0]-roll_err[1])/t_update+P2/I2*roll_err[0]);
		rate_r=- gyro_inter_PID[yaw].kp		* yaw_error_rate	 - gyro_inter_PID[yaw].kd		* yaw_error_rate_diff		-	gyro_inter_PID[yaw].ki	 * yaw_rate_I;//-Pi*((yaw_err[0]-yaw_err[1])/t_update+Pr/Ir*yaw_err[0]);
	if(rate_r>15)rate_r=15;
	//更新前一时刻3轴角速度
	//x_before=six_axis.gyro_x/7505.7;
	//y_before=six_axis.gyro_y/7505.7;
	//z_before=six_axis.gyro_z/7505.7;
	//动力转换为占空比
		
		
		
	//占空比整合（叉形）
	 pwmOutput();

	 ANO_DT_Data_Exchange();
//}
	 GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}	

