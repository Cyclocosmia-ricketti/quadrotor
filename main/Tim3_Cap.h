#include "stm32f10x.h"

/**********************************************************
Tim3_Cap.c：
代码功能：四路捕获PWM输入信号，实现对遥控器信号的接受
管脚定义：PA6,PA7,PB0,PB1
**********************************************************/

#define rc_thr_max 4850
#define rc_thr_min 2458
#define rc_thr_range 120

#define rc_yaw_max 4850
#define rc_yaw_min 2440
#define rc_yaw_mid 3645
#define rc_yaw_range 0	//+-0 degree

#define rc_pit_max 4807
#define rc_pit_min 2412
#define rc_pit_mid 3567
#define rc_pit_range 0.42//0.348	//+-6 degree

#define rc_rol_max 4850
#define rc_rol_min 2440
#define rc_rol_mid 3645
#define rc_rol_range 0.42//0.348	//+-6 degree

extern u32 tempup1;	//捕获总高电平的时间 thr 油门 （min:2458 max:4850）
extern u32 tempup2;	//捕获总高电平的时间 yaw 偏航 （left:4850 right:2440 mid:3645）
extern u32 tempup3;	//捕获总高电平的时间 pit 俯仰 （front:4776 behind:2412 mid:3567）
extern u32 tempup4;	//捕获总高电平的时间 rol 翻滚 （left:4850 right:2440 mid:3645）

void TIM3_Cap_Init(u16 arr, u16 psc);

double Read1(void);

double Read2(void);

double Read3(void);

double Read4(void);
