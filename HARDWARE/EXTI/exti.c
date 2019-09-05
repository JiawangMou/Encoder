#include "exti.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "usart.h"
#include "24l01.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//�ⲿ�ж� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   
//�ⲿ�ж�0�������
void EXTIx_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;


  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);						//ʹ�ܸ��ù���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);	 				//ʹ��PG�˿�ʱ��
    //GPIOG.6 �ж����Լ��жϳ�ʼ������   �½��ش���
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG,GPIO_PinSource6);

  	EXTI_InitStructure.EXTI_Line = EXTI_Line6;	
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 										//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;						//ʹ��NRF24L01��IRQ���Ŷ�Ӧ���ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;			//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//�����ȼ�1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);
 
}


/*extern u8 RX_OK_FLAG; 
extern u8 rx_len;
extern u8 Rx_buf[RX_PLOAD_WIDTH];
extern u8 sta;
//�ⲿ�ж�EXTI9_5_IRQ�������

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		NRF24L01_CE = 0;
		sta=NRF24L01_Read_Reg(STATUS);  										//��ȡ״̬�Ĵ�����ֵ	 
		rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);								//��ȡ���յ������ݳ���
		NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_buf,rx_len);							//��ȡ����
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta);							//���TX_DS��RX_DR�жϱ�־
		delay_ms(1);
		sta=NRF24L01_Read_Reg(STATUS);  										//��ȡ״̬�Ĵ�����ֵ	 

		NRF24L01_FlushRX();														//��ֹ���ն�RX FIFO��ʱ��������ACK����ɻش�����ʧ��
		NRF24L01_CE = 1;
		RX_OK_FLAG = 1;
		EXTI_ClearITPendingBit(EXTI_Line6);  									//���LINE6�ϵ��жϱ�־λ  
	}

}*/

