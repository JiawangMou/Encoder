#include "exti.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "usart.h"
#include "24l01.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//外部中断 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   
//外部中断0服务程序
void EXTIx_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;


  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);						//使能复用功能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);	 				//使能PG端口时钟
    //GPIOG.6 中断线以及中断初始化配置   下降沿触发
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG,GPIO_PinSource6);

  	EXTI_InitStructure.EXTI_Line = EXTI_Line6;	
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 										//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;						//使能NRF24L01的IRQ引脚对应的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;			//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);
 
}


/*extern u8 RX_OK_FLAG; 
extern u8 rx_len;
extern u8 Rx_buf[RX_PLOAD_WIDTH];
extern u8 sta;
//外部中断EXTI9_5_IRQ服务程序

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		NRF24L01_CE = 0;
		sta=NRF24L01_Read_Reg(STATUS);  										//读取状态寄存器的值	 
		rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);								//读取接收到的数据长度
		NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_buf,rx_len);							//读取数据
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta);							//清除TX_DS或RX_DR中断标志
		delay_ms(1);
		sta=NRF24L01_Read_Reg(STATUS);  										//读取状态寄存器的值	 

		NRF24L01_FlushRX();														//防止接收端RX FIFO满时，不进行ACK，造成回传数据失败
		NRF24L01_CE = 1;
		RX_OK_FLAG = 1;
		EXTI_ClearITPendingBit(EXTI_Line6);  									//清除LINE6上的中断标志位  
	}

}*/

