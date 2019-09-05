#include "delay.h"
#include "base_timer.h"

void delay_init()
{
	base_timer_initialize();
}

void delay_ms(u16 nms)
{
	base_timer_delay_ms(nms); 
}

void delay_us(u32 nus)
{
	base_timer_delay_us(nus);
}

uint64_t micros(void)
{
	uint64_t micros = base_timer_get_time() * 1000 + base_timer_get_us();
	return micros;
}
//end of file





































