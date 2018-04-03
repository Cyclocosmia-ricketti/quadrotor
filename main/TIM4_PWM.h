
#include "stm32f10x.h"

/**********************************************************
Tim4_PWM.c：

代码功能：四路输出PWM信号，实现对电机的控制

管脚定义：PB6,PB7,PB8,PB9

函数接口：void TIM4_PWM_Init(u16 arr,u16 psc);
		//功能：初始化相关硬件
		//参数：arr：时钟计数周期（建议值arr=1000）
		//	psc：时钟预分频（建议值psc=719）

		void TIM_SetCompare1(TIM4, uint16_t Compare)
		//功能：设置通道1输出的占空比
		//参数：Compare：高电平翻转时时钟计数值（默认值70）（范围70~190）

		void TIM_SetCompare2(TIM4, uint16_t Compare)
		//功能：设置通道2输出的占空比
		//参数：Compare：高电平翻转时时钟计数值（默认值70）（范围70~190）

		void TIM_SetCompare3(TIM4, uint16_t Compare)
		//功能：设置通道3输出的占空比
		//参数：Compare：高电平翻转时时钟计数值（默认值70）（范围70~190）

		void TIM_SetCompare4(TIM4, uint16_t Compare)
		//功能：设置通道4输出的占空比
		//参数：Compare：高电平翻转时时钟计数值（默认值70）（范围70~190）
**********************************************************/

void TIM4_PWM_Init(u16 arr,u16 psc);
