#include "24l01.h"
#include "delay.h"
#include "spi.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//NRF24L01驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
    
const u8 TX_ADDRESS[ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const u8 RX_ADDRESS[ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01};

/*===========================================================================
* 函数 ：NRF24L01_Init =》 初始化NRF24L01L01                                 * 
* 输入 ：mode，=TX_MODE, TX mode; =RX_MODE, RX mode	
* 说明 : 执行完此函数后，收发模式已经确定，所有的配置已经完成
		 这里的函数是配置成 ACK+DPL(动态包)模式
============================================================================*/
void NRF24L01_Init(NRF24L01_MODE RxTx_mode)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;
//	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOG, ENABLE);	 //使能PB,G端口时钟
    	
	//PB12上拉 防止W25X的干扰
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PB12上拉 防止W25X的干扰
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);                   //初始化指定IO
 	GPIO_SetBits(GPIOB,GPIO_Pin_12);                         //上拉				
 	
	//初始化NRF_CSN，NRF_CE，推免输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|				 //NRF_CSN
																GPIO_Pin_8;	 			 //NRF_CE 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);                   //初始化指定IO,推挽 
	
	//初始化NRF_IRQ
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;   			 //NRF_IRQ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;            //输入，上拉  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_SetBits(GPIOG,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);    //PG6,7,8上拉，CE引脚置高模拟机载板CE与VCC连在一起的情况					 
		 
	SPI2_Init();    										 //初始化SPI	 
	SPI2_SetSpeed(SPI_BaudRatePrescaler_16); 				 //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）

	while( NRF24L01_Check() );								 //检测NRF24L01硬件是否连接正常，如果连接正常以后才能进行下一步的配置
	//使能管道0动态包
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,ENAA_P0);						//使能管道0自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG+L01REG_FEATRUE,EN_DPL|EN_ACK_PAY);		//enable DPL and enable payload with ACK
	NRF24L01_Write_Reg(NRF_WRITE_REG+L01REG_DYNPD,DPL_P0);					//使能管道P0动态包
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,ERX_P0);						//使能通道0的接收地址 
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW,ADR_WIDTH);					//选择TX/RX的地址宽度
//	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);				//选择所有通道的有效数据宽度
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,ARD_2000US|ARC_15);			//设置自动重发间隔时间:500us + 86us;最大自动重发次数:15次
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     						//设置RF通信频率
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x26);						//设置TX发射参数,0db增益,250Kbps,低噪声增益开启	
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,ADR_WIDTH);	//写RX节点地址
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,ADR_WIDTH); 	//写TX节点地址 
	
	NRF24L01_FlushTX();														//复位TX FIFO指针 
	NRF24L01_FlushRX();														//复位RX FIFO指针 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,RX_OK|TX_OK|MAX_TX);			//清除TX_DS或MAX_RT中断标志
	if( RxTx_mode == RX_MODE )
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x3b);						//配置基本工作模式的参数;PWR_UP,EN_CRC,CRC_1byte,接收模式,屏蔽掉发送最大次数中断和发送完成中断，只打开了接收完成中断
	}
	if( RxTx_mode == TX_MODE )
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0a);						//配置基本工作模式的参数;PWR_UP,EN_CRC,CRC_1byte,发送模式 
	}
//由于TX FIFO 和 Rx FIFO,而且CE始终为高，所以此时NRF24L01要么为接受模式，要么被设置成发送模式，处于Standby-Ⅱ模式
	
//	NRF24L01_CE=0; 															//使能24L01,次PCB的CE引脚与VCC连在了一起（设计错误），一直置高；CE引脚置高模拟机载板CE与VCC连在一起的情况	
	 		 	 
}
//检测24L01是否存在
//返回值:0，成功;1，失败	
u8 NRF24L01_Check(void)
{
	u8 buf[5];
	u8 i;
	for(i=0;i<5;i++) buf[i] = TX_ADDRESS[i];
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   	 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=TX_ADDRESS[i])break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}	 	 
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //使能SPI传输
  	status =SPI2_ReadWriteByte(reg);//发送寄存器号 
  	SPI2_ReadWriteByte(value);      //写入寄存器的值
  	NRF24L01_CSN=1;                 //禁止SPI传输	   
  	return(status);       			//返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //使能SPI传输		
  	SPI2_ReadWriteByte(reg);   //发送寄存器号
  	reg_val=SPI2_ReadWriteByte(0XFF);//读取寄存器内容
  	NRF24L01_CSN = 1;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           											//使能SPI传输
  	status=SPI2_ReadWriteByte(reg);											//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI2_ReadWriteByte(0XFF);	//读出数据
  	NRF24L01_CSN=1;       													//关闭SPI传输
  	return status;        													//返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //使能SPI传输
  	status = SPI2_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI2_ReadWriteByte(*pBuf++); //写入数据	 
  	NRF24L01_CSN = 1;       //关闭SPI传输
  	return status;          //返回读到的状态值
}				   
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
	NRF24L01_CE=0;
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	NRF24L01_CE=1;//启动发送	   
	while(NRF24L01_IRQ!=0);//等待发送完成
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
	delay_ms(2);
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	sta=NRF24L01_Read_Reg(STATUS);  							//读取状态寄存器的值    	 
	if(sta&RX_OK)												//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);	//读取数据
		NRF24L01_FlushRX();										//清除RX FIFO寄存器 
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,(sta&RX_OK));	//清除TX_DS或MAX_RT中断标志
		return 0; 
	}	   
	if(sta & MAX_TX)											//最大输出可能的情况就是ACK应答出现问题
	{
		NRF24L01_FlushTX();
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,(sta&MAX_TX)); 	//清除MAX_RT中断标志
	}
	return 1;//没收到任何数据
}					    

/*===========================================================================
* 函数 ：NRF24L01_PowerDown_Mode() => 使NRF24L01掉电                               
============================================================================*/
void NRF24L01_PowerDown_Mode(void)
{
//	NRF24L01_CE=0;	    
    uint8_t configreg = NRF24L01_Read_Reg(NRF_READ_REG + CONFIG); //读取当前配置信息
  	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, (configreg&0xfd)); //将PWR_UP位置0，powerdown	
//	NRF24L01_CE=1;//CE为高,10us后启动发送
}

/*===========================================================================
* 函数 ：L01_FlushTX() => 复位TX FIFO指针                                   * 
============================================================================*/
void NRF24L01_FlushTX(void)
{
	NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
}
/*===========================================================================
* 函数 ：L01_FlushRX() => 复位RX FIFO指针                                   *
============================================================================*/
void NRF24L01_FlushRX(void)
{
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
}

/*===========================================================================
* 函数 ：L01_SetTRMode() => 设置L01模式                                     * 
* 输入 ：mode，=TX_MODE, TX mode; =RX_MODE, RX mode						
* 说明 ：此函数没有对NRF24L01进行完整的配置，在执行这个函数前需要保证已经执行过 
		 NRF24L01_Init()这个函数
============================================================================*/
void NRF24L01_SetTRMode(NRF24L01_MODE mode)
{
    uint8_t configreg = NRF24L01_Read_Reg(NRF_READ_REG + CONFIG);		//读取当前配置信息
	
	if(mode != (configreg & 0x01))										//需要转换到的模式与当前模式不同，如果相同则没有操作
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, (configreg&0xfd) );	//在转换NRF24L01的收发模式时，先将NRF24L01 powerdown,然后再重新设置模式和上电
		delay_ms(5);													//掉电后再转换到所指定的模式需要时间
		if		(mode == TX_MODE)       { configreg &= ~PRIM_RX; }
		else if (mode == RX_MODE)       { configreg |=  PRIM_RX; }	
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, configreg);			//上电并且设置Tx或者RX模式		
	}

}

void NRF24L01_ACK_W_Packet(u8 *Data,u8 Data_Length)
{
	NRF24L01_FlushTX();													//先清除TX FIFO(接收端)，防止写入FIFO的数据丢失连接造成的FIFO堵塞
	NRF24L01_Write_Buf(W_ACK_PAYLOAD,Data,Data_Length);					//向管道0写入W_ACK_PAYLOAD命令,并读取数据
}
