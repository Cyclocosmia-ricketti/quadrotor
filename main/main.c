
#include "stm32f10x.h"
#include <stdio.h>
#include "delay.h"

#include "Tim2_PID.h"
#include "Tim3_Cap.h"
#include "Tim4_PWM.h"
#include "ANO_DT.h"


void assert_failed(uint8_t* file, uint32_t line)
{
 printf("Wrong parameters value: file %s on line %d\r\n", file, line);
 while(1);
}


 
int main(void)
{	
	
	GPIO_InitTypeDef GPIO_InitStructure;    //����GPIO��ʼ���ṹ��
	
	TIM4_PWM_Init(5259,71);
	
	delay_nms(4000);
	
	TIM3_Cap_Init(0xffff,29);
	
		delay_nms(1000);
	
	TIM2_PIB_Init(525,719);
	USART1_BlueTooth_Init();
	

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;     //�ܽ�λ�ö��塣
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  //����ٶ�2MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;   //����������� AF_PP
	GPIO_Init(GPIOA,&GPIO_InitStructure);     //A��GPIO��ʼ��
	      
	while(1)
	{
	}	
}	 
