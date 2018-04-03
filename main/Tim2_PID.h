
#include "stm32f10x.h"




typedef struct {
	float kp;
	float ki;
	float kd;
}PID_paras;

extern u8 ready;

void TIM2_PIB_Init(u16 arr, u16 psc);

