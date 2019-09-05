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
	TIM_TimeBaseStructure.TIM_Period = 65535;		 //编码器为1000线
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	//定时器编码器模式 下降沿 上升沿选择
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling ,TIM_ICPolarity_Falling);
	TIM_ICStructInit(&TIM_ICInitStructure);
	//编码器滤波 This parameter must be a value between 0x00 and 0x0F.
	TIM_ICInitStructure.TIM_ICFilter = 0x0f;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
	//置定时器初始值
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
	
	//数传模块接收机油门摇杆推满100，对应脉宽是0.586ms，推到负满量程-100，对应脉宽是1.414ms
	TIM_TimeBaseInitStruct.TIM_Prescaler = (720-1);				//72MHZ/72等于定时器计数器1秒钟计数的次数，也就是1MHZ，那每计数一次时间为0.01ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM4_PERIOD;			//计数65535次,对应时间是655.35ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	///设置时钟分割:TDTS = Tck_tim


	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;				//设置为通道2
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;	//设置为下降沿检测
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;//TIM Input 1, 2, 3 or 4 is selected to be connected to IC1, IC2, IC3 or IC4, respectively
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;			//Capture performed each time an edge is detected on the capture input
	TIM_ICInitStruct.TIM_ICFilter = 3<<4;						//Fsampling = Fck_int, N=8;
	
	TIM_ICInit(TIM4, &TIM_ICInitStruct);		

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//中断分组		
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;				//设置初始化的是TIM4的中断	
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;		//设置抢占优先级为2
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;				//设置响应优先级（子优先级）为0
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;				//使能定时器4这个中断
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_ITConfig(TIM4,TIM_IT_CC2,ENABLE);						//使能TIM4的IC2通道中断
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
