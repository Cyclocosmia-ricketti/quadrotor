#include "systick.h"

static volatile uint32_t sysTickMillis = 0;
static uint32_t sysTickPerUs = 72;//系统内核时钟 ，单位（Mhz）
uint32_t idleMax=0;

void delay_x_ms(uint32_t xms)
{
	
	uint32_t x,i;
	for(i=0;i<xms;i++){
	if(idleMax!=0&&idleMax>700){
	x=idleMax;
	}else{
	x=732;//约等于1ms
	}
	while(x--){
	}
	}
}
void initSystick(void)
{
	sysTickPerUs = SystemCoreClock / 1000000;
	SysTick_Config(SystemCoreClock / 1000);//1ms中断
}


static  int systick_check_underflow(void)//返回24位递减计数器是否减到0标志
{
    return SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;
}


void SysTick_Handler(void)
{
    __disable_irq();
    systick_check_underflow();
    sysTickMillis++;
    __enable_irq();

   
}

unsigned int millis(void)
{
    return sysTickMillis;
}


unsigned int micros(void)
{
    uint32_t cycle, timeMs;

    do
    {
        timeMs = sysTickMillis;
        cycle = SysTick->VAL;
        __ASM volatile("nop");
        __ASM volatile("nop");
    }
    while (timeMs != sysTickMillis);

    if (systick_check_underflow())
    {
        timeMs++;
        cycle = SysTick->VAL;
    }

    return (timeMs * 1000) + (SysTick->LOAD + 1 - cycle) / sysTickPerUs;
}



