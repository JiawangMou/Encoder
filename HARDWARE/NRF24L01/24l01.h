#ifndef __24L01_H
#define __24L01_H	 		  
#include "sys.h"   
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������V3
//NRF24L01��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/17
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//NRF24L01�Ĵ�����������
#define NRF_READ_REG    	0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define NRF_WRITE_REG   	0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     	0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     	0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        	0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        	0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     	0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define R_RX_PL_WID     	0x60
#define W_ACK_PAYLOAD   	0xA8
#define W_TX_PAYLOAD_NOACK	0xB0
#define NOP             	0xFF  //�ղ���,����������״̬�Ĵ���	 
//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
	//reserved: bit7, only '0' allowed
    #define MASK_RX_DR   (1<<6) //RW, Mask interrupt caused by RX_DR
    #define MASK_TX_DS   (1<<5) //RW, Mask interrupt caused by TX_DS
    #define MASK_MAX_RT  (1<<4) //RW, Mask interrupt caused by MAX_RT
    #define EN_CRC       (1<<3) //RW, Enable CRC.
    #define CRCO         (1<<2) //RW, CRC encoding scheme, 0: 1 byte, 1: 2 bytes
    #define PWR_UP       (1<<1) //RW, Power up, 1: power up, 0: power down
    #define PRIM_RX      (1<<0) //RW, RX/TX control, 1: PRX, 0: PTX
#define EN_AA          0x01//ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
    //reserved: bit7-bit6:only '00' allowed
    #define ENAA_P5    (1<<5) //RW, Enable auto acknowledgement data pipe 5
    #define ENAA_P4    (1<<4) //RW, Enable auto acknowledgement data pipe 4
    #define ENAA_P3    (1<<3) //RW, Enable auto acknowledgement data pipe 3
    #define ENAA_P2    (1<<2) //RW, Enable auto acknowledgement data pipe 2
    #define ENAA_P1    (1<<1) //RW, Enable auto acknowledgement data pipe 1
    #define ENAA_P0    (1<<0) //RW, Enable auto acknowledgement data pipe 0
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
    //reserved: bit7-bit6:only '00' allowed
    #define ERX_P5     (1<<5) //RW, Enable data pipe 5
    #define ERX_P4     (1<<4) //RW, Enable data pipe 4
    #define ERX_P3     (1<<3) //RW, Enable data pipe 3
    #define ERX_P2     (1<<2) //RW, Enable data pipe 2
    #define ERX_P1     (1<<1) //RW, Enable data pipe 1
    #define ERX_P0     (1<<0) //RW, Enable data pipe 0
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
    // reserved: bit7-bit2, only '000000' allowed
    #define AW_RERSERVED 0x0 
    #define AW_3BYTES    0x1
    #define AW_4BYTES    0x2
    #define AW_5BYTES    0x3
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
    //for bit7-bit4, Auto transmission delay
    #define ARD_250US    (0x00<<4)
    #define ARD_500US    (0x01<<4)
    #define ARD_750US    (0x02<<4)
    #define ARD_1000US   (0x03<<4)
    #define ARD_2000US   (0x07<<4)
    //......
    #define ARD_4000US   (0x0F<<4)
    //for bit3-bit0, Auto retransmission Count
    #define ARC_DISABLE   0x00
    //......
    #define ARC_15        0x0F
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;

#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
    #define CONT_WAVE     7 //RW, Enables continuous carrier transmit when high
    //reserved bit6, only '0' allowed
    #define RF_DR_LOW     5 //RW, Set RF Data Rate to 250kbps
    #define PLL_LOCK      4 //RW, Force PLL lock signal, Only used in test
    #define RF_DR_HIGH    3 //RW, Low-High:00,1Mbps,01,2Mbps,10,250kbps,11,reserved
    //bit2-bit1:
        #define PWR_18DB  (0x00<<1)
        #define PWR_12DB  (0x01<<1)
        #define PWR_6DB   (0x02<<1)
        #define PWR_0DB   (0x03<<1)
    //reserved, bit0, Do not care
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
    //reserved, bit7, only '0' allowed
    #define RX_DR         6 //RW, Data ready RX FIFO interrupt
    #define TX_DS         5 //RW, Data sent TX FIFO interrupt
    #define MAX_RT        4 //RW, Maximum number of TX retransmits interrupt
    //for bit3-bit1, R, Data pipe number for the payload available for reading from
    //RX_FIFO, 000-101, Data pipe number, 110, not used, 111, RX FIFO empty
    #define TX_FULL_0     0 //R, TX FIFO full flag
#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define REG_RPD         0x09  //�ز����Ĵ���,bit0,�ز����;
    //bit7-bit1, reserved
    #define RPD           0 //Received power detector
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���

#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define NRF_FIFO_STATUS 0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;
    //bit7, reserved, only '0' allowed
    #define TX_REUSE      6 //R,
    #define TX_FULL_1     5 //R, TX FIFO full flag
    #define TX_EMPTY      4 //R, TX FIFO empty flag
    //bit3-bit2, reserved, only '00'
    #define RX_FULL       1 //R, RX FIFO full flag
    #define RX_EMPTY      0 //R, RX FIFO empty flag
	
#define L01REG_DYNPD      0x1C /**/
    //reserved, bit7-bit6, only '00' allowed
    #define DPL_P5    (1<<5) //RW, Enable dynamic payload length data pipe 5
    #define DPL_P4    (1<<4) //RW, Enable dynamic payload length data pipe 4
    #define DPL_P3    (1<<3) //RW, Enable dynamic payload length data pipe 3
    #define DPL_P2    (1<<2) //RW, Enable dynamic payload length data pipe 2
    #define DPL_P1    (1<<1) //RW, Enable dynamic payload length data pipe 1
    #define DPL_P0    (1<<0) //RW, Enable dynamic payload length data pipe 0

#define L01REG_FEATRUE    0x1D
    //reserved, bit7-bit3, only '0000' allowed
    #define EN_DPL      (1<<2)//RW, Enables Dynamic payloard length
    #define EN_ACK_PAY  (1<<1)//RW, Enable Payload with ACK
    #define EN_DYN_ACK  (1<<0)//RW, Enables the W_TX_PAYLOAD_NOACK command
	
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01������
#define NRF24L01_CE   PGout(8) //24L01Ƭѡ�ź�
#define NRF24L01_CSN  PGout(7) //SPIƬѡ�ź�	   
#define NRF24L01_IRQ  PGin(6)  //IRQ������������
//24L01���ͽ������ݿ�ȶ���
#define ADR_WIDTH    5   	//5�ֽڵĵ�ַ���

#define TX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32 		//32�ֽڵ��û����ݿ��

typedef enum{ TX_MODE, RX_MODE } NRF24L01_MODE; 		//TX_MODE->0,RX_MODE->1

									   	   

void NRF24L01_Init(NRF24L01_MODE RxTx_mode);	//��ʼ��
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 u8s);//д������
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 u8s);	//��������		  
u8 NRF24L01_Read_Reg(u8 reg);					//���Ĵ���
u8 NRF24L01_Write_Reg(u8 reg, u8 value);		//д�Ĵ���
u8 NRF24L01_Check(void);						//���24L01�Ƿ����
u8 NRF24L01_TxPacket(u8 *txbuf);				//����һ����������
u8 NRF24L01_RxPacket(u8 *rxbuf);				//����һ����������
void NRF24L01_PowerDown_Mode(void);				//����Ϊpower downģʽ
void NRF24L01_FlushTX(void);					//��λTX FIFOָ��
void NRF24L01_FlushRX(void);					//��λRX FIFOָ��
void NRF24L01_SetTRMode(NRF24L01_MODE mode);	//�����շ�ģʽ	
void NRF24L01_ACK_W_Packet(u8 *Data,u8 Data_Length);			//���Զ�Ӧ���payloadд��TX FIFO

#endif











