#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "24l01.h" 	 
#include "nrf_protocol.h"
#include "exti.h"
#include "Encoder.h"
#include "timer.h"
 
#define TIM4_PERIOD 8000
#define RX_BUF_MAX 5
#define TX_BUF_MAX 5

/************************************************
 ALIENTEK战舰STM32开发板实验33
 无线通信 实验
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

u32 rate_value = 0; 
uint16_t rate_value_new = 0;
uint16_t rate_value_last = 0;
float rate= 0.0;

u16 count =0;


u16 current_count = 0;
u16 last_count = 0;
 int main(void)
 {	

	float angle = 0;
	delay_init();	    																			//延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);													//中断分组	
	uart_init(115200);	 																			//串口初始化为115200
	Encoder_init();
	capture_init();	

	while(1)
	{
		angle = ((current_count - 32768) % 4000 - 2000) * 0.09;
		if(rate_value != 0)
			rate = 	36000.0 / (float)rate_value ;
		else	
			rate = 0;
		printf("angle = %f\r\n", angle );
		printf("rate = %.4f\r\n", rate );
		delay_ms(1000);
	}
 
		
}



