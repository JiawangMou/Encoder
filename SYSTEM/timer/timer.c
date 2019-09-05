#include "timer.h"
#include "stm32f10x_tim.h"

#define TIM4_PERIOD 65535 

extern u32 rate_value;
extern uint16_t rate_value_last;
extern uint16_t rate_value_new;
extern u16 count;
void timer_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
//	TIM_DeInit(TIM3);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_Period = 65535;		 //������Ϊ1000��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	//��ʱ��������ģʽ �½��� ������ѡ��
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling ,TIM_ICPolarity_Falling);
	TIM_ICStructInit(&TIM_ICInitStructure);
	//�������˲� This parameter must be a value between 0x00 and 0x0F.
	TIM_ICInitStructure.TIM_ICFilter = 0x0f;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
	//�ö�ʱ����ʼֵ
	TIM_SetCounter(TIM3, 32768);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_Cmd(TIM3,ENABLE);
}

void capture_init()
{
	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	//����ģ����ջ�����ҡ������100����Ӧ������0.586ms���Ƶ���������-100����Ӧ������1.414ms
	TIM_TimeBaseInitStruct.TIM_Prescaler = (720-1);				//72MHZ/72���ڶ�ʱ��������1���Ӽ����Ĵ�����Ҳ����1MHZ����ÿ����һ��ʱ��Ϊ0.01ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM4_PERIOD;			//����65535��,��Ӧʱ����655.35ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	///����ʱ�ӷָ�:TDTS = Tck_tim


	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;				//����Ϊͨ��2
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;	//����Ϊ�½��ؼ��
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;//TIM Input 1, 2, 3 or 4 is selected to be connected to IC1, IC2, IC3 or IC4, respectively
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;			//Capture performed each time an edge is detected on the capture input
	TIM_ICInitStruct.TIM_ICFilter = 3<<4;						//Fsampling = Fck_int, N=8;
	
	TIM_ICInit(TIM4, &TIM_ICInitStruct);		

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//�жϷ���		
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;				//���ó�ʼ������TIM4���ж�	
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;		//������ռ���ȼ�Ϊ2
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;				//������Ӧ���ȼ��������ȼ���Ϊ0
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;				//ʹ�ܶ�ʱ��4����ж�
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_ITConfig(TIM4,TIM_IT_CC2,ENABLE);						//ʹ��TIM4��IC2ͨ���ж�
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler()
{
	if( TIM_GetITStatus(TIM4,TIM_IT_CC2) == 1 )
	{
		rate_value_last = rate_value_new;
		rate_value_new = TIM_GetCapture2(TIM4);
		rate_value = (u32)((int32_t)(rate_value_new - rate_value_last)  + (int32_t)count * 65536);
		count = 0;
	}
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) == 1)
	{
		if(count == 255)
			count = 0;
		else
			count++;
		TIM_ClearFlag(TIM4,TIM_IT_Update);
	}
}
