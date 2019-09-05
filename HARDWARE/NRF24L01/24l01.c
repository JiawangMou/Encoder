#include "24l01.h"
#include "delay.h"
#include "spi.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//NRF24L01��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
    
const u8 TX_ADDRESS[ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const u8 RX_ADDRESS[ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01};

/*===========================================================================
* ���� ��NRF24L01_Init =�� ��ʼ��NRF24L01L01                                 * 
* ���� ��mode��=TX_MODE, TX mode; =RX_MODE, RX mode	
* ˵�� : ִ����˺������շ�ģʽ�Ѿ�ȷ�������е������Ѿ����
		 ����ĺ��������ó� ACK+DPL(��̬��)ģʽ
============================================================================*/
void NRF24L01_Init(NRF24L01_MODE RxTx_mode)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;
//	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOG, ENABLE);	 //ʹ��PB,G�˿�ʱ��
    	
	//PB12���� ��ֹW25X�ĸ���
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PB12���� ��ֹW25X�ĸ���
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);                   //��ʼ��ָ��IO
 	GPIO_SetBits(GPIOB,GPIO_Pin_12);                         //����				
 	
	//��ʼ��NRF_CSN��NRF_CE���������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|				 //NRF_CSN
																GPIO_Pin_8;	 			 //NRF_CE 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);                   //��ʼ��ָ��IO,���� 
	
	//��ʼ��NRF_IRQ
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;   			 //NRF_IRQ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;            //���룬����  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_SetBits(GPIOG,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);    //PG6,7,8������CE�����ø�ģ����ذ�CE��VCC����һ������					 
		 
	SPI2_Init();    										 //��ʼ��SPI	 
	SPI2_SetSpeed(SPI_BaudRatePrescaler_16); 				 //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��

	while( NRF24L01_Check() );								 //���NRF24L01Ӳ���Ƿ�����������������������Ժ���ܽ�����һ��������
	//ʹ�ܹܵ�0��̬��
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,ENAA_P0);						//ʹ�ܹܵ�0�Զ�Ӧ��
	NRF24L01_Write_Reg(NRF_WRITE_REG+L01REG_FEATRUE,EN_DPL|EN_ACK_PAY);		//enable DPL and enable payload with ACK
	NRF24L01_Write_Reg(NRF_WRITE_REG+L01REG_DYNPD,DPL_P0);					//ʹ�ܹܵ�P0��̬��
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,ERX_P0);						//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW,ADR_WIDTH);					//ѡ��TX/RX�ĵ�ַ���
//	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);				//ѡ������ͨ������Ч���ݿ��
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,ARD_2000US|ARC_15);			//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:15��
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     						//����RFͨ��Ƶ��
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x26);						//����TX�������,0db����,250Kbps,���������濪��	
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,ADR_WIDTH);	//дRX�ڵ��ַ
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,ADR_WIDTH); 	//дTX�ڵ��ַ 
	
	NRF24L01_FlushTX();														//��λTX FIFOָ�� 
	NRF24L01_FlushRX();														//��λRX FIFOָ�� 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,RX_OK|TX_OK|MAX_TX);			//���TX_DS��MAX_RT�жϱ�־
	if( RxTx_mode == RX_MODE )
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x3b);						//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,CRC_1byte,����ģʽ,���ε������������жϺͷ�������жϣ�ֻ���˽�������ж�
	}
	if( RxTx_mode == TX_MODE )
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0a);						//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,CRC_1byte,����ģʽ 
	}
//����TX FIFO �� Rx FIFO,����CEʼ��Ϊ�ߣ����Դ�ʱNRF24L01ҪôΪ����ģʽ��Ҫô�����óɷ���ģʽ������Standby-��ģʽ
	
//	NRF24L01_CE=0; 															//ʹ��24L01,��PCB��CE������VCC������һ����ƴ��󣩣�һֱ�øߣ�CE�����ø�ģ����ذ�CE��VCC����һ������	
	 		 	 
}
//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
u8 NRF24L01_Check(void)
{
	u8 buf[5];
	u8 i;
	for(i=0;i<5;i++) buf[i] = TX_ADDRESS[i];
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=TX_ADDRESS[i])break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //ʹ��SPI����
  	status =SPI2_ReadWriteByte(reg);//���ͼĴ����� 
  	SPI2_ReadWriteByte(value);      //д��Ĵ�����ֵ
  	NRF24L01_CSN=1;                 //��ֹSPI����	   
  	return(status);       			//����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����		
  	SPI2_ReadWriteByte(reg);   //���ͼĴ�����
  	reg_val=SPI2_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
  	NRF24L01_CSN = 1;          //��ֹSPI����		    
  	return(reg_val);           //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           											//ʹ��SPI����
  	status=SPI2_ReadWriteByte(reg);											//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI2_ReadWriteByte(0XFF);	//��������
  	NRF24L01_CSN=1;       													//�ر�SPI����
  	return status;        													//���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����
  	status = SPI2_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI2_ReadWriteByte(*pBuf++); //д������	 
  	NRF24L01_CSN = 1;       //�ر�SPI����
  	return status;          //���ض�����״ֵ̬
}				   
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
	NRF24L01_CE=0;
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	NRF24L01_CE=1;//��������	   
	while(NRF24L01_IRQ!=0);//�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	delay_ms(2);
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	sta=NRF24L01_Read_Reg(STATUS);  							//��ȡ״̬�Ĵ�����ֵ    	 
	if(sta&RX_OK)												//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);	//��ȡ����
		NRF24L01_FlushRX();										//���RX FIFO�Ĵ��� 
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,(sta&RX_OK));	//���TX_DS��MAX_RT�жϱ�־
		return 0; 
	}	   
	if(sta & MAX_TX)											//���������ܵ��������ACKӦ���������
	{
		NRF24L01_FlushTX();
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,(sta&MAX_TX)); 	//���MAX_RT�жϱ�־
	}
	return 1;//û�յ��κ�����
}					    

/*===========================================================================
* ���� ��NRF24L01_PowerDown_Mode() => ʹNRF24L01����                               
============================================================================*/
void NRF24L01_PowerDown_Mode(void)
{
//	NRF24L01_CE=0;	    
    uint8_t configreg = NRF24L01_Read_Reg(NRF_READ_REG + CONFIG); //��ȡ��ǰ������Ϣ
  	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, (configreg&0xfd)); //��PWR_UPλ��0��powerdown	
//	NRF24L01_CE=1;//CEΪ��,10us����������
}

/*===========================================================================
* ���� ��L01_FlushTX() => ��λTX FIFOָ��                                   * 
============================================================================*/
void NRF24L01_FlushTX(void)
{
	NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
}
/*===========================================================================
* ���� ��L01_FlushRX() => ��λRX FIFOָ��                                   *
============================================================================*/
void NRF24L01_FlushRX(void)
{
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
}

/*===========================================================================
* ���� ��L01_SetTRMode() => ����L01ģʽ                                     * 
* ���� ��mode��=TX_MODE, TX mode; =RX_MODE, RX mode						
* ˵�� ���˺���û�ж�NRF24L01�������������ã���ִ���������ǰ��Ҫ��֤�Ѿ�ִ�й� 
		 NRF24L01_Init()�������
============================================================================*/
void NRF24L01_SetTRMode(NRF24L01_MODE mode)
{
    uint8_t configreg = NRF24L01_Read_Reg(NRF_READ_REG + CONFIG);		//��ȡ��ǰ������Ϣ
	
	if(mode != (configreg & 0x01))										//��Ҫת������ģʽ�뵱ǰģʽ��ͬ�������ͬ��û�в���
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, (configreg&0xfd) );	//��ת��NRF24L01���շ�ģʽʱ���Ƚ�NRF24L01 powerdown,Ȼ������������ģʽ���ϵ�
		delay_ms(5);													//�������ת������ָ����ģʽ��Ҫʱ��
		if		(mode == TX_MODE)       { configreg &= ~PRIM_RX; }
		else if (mode == RX_MODE)       { configreg |=  PRIM_RX; }	
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, configreg);			//�ϵ粢������Tx����RXģʽ		
	}

}

void NRF24L01_ACK_W_Packet(u8 *Data,u8 Data_Length)
{
	NRF24L01_FlushTX();													//�����TX FIFO(���ն�)����ֹд��FIFO�����ݶ�ʧ������ɵ�FIFO����
	NRF24L01_Write_Buf(W_ACK_PAYLOAD,Data,Data_Length);					//��ܵ�0д��W_ACK_PAYLOAD����,����ȡ����
}
