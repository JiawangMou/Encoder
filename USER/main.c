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
 ALIENTEKս��STM32������ʵ��33
 ����ͨ�� ʵ��
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
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
	delay_init();	    																			//��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);													//�жϷ���	
	uart_init(115200);	 																			//���ڳ�ʼ��Ϊ115200
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



