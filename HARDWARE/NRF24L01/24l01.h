#ifndef __24L01_H
#define __24L01_H	 		  
#include "sys.h"   
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板V3
//NRF24L01驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/17
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//NRF24L01寄存器操作命令
#define NRF_READ_REG    	0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_WRITE_REG   	0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     	0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     	0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        	0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        	0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     	0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define R_RX_PL_WID     	0x60
#define W_ACK_PAYLOAD   	0xA8
#define W_TX_PAYLOAD_NOACK	0xB0
#define NOP             	0xFF  //空操作,可以用来读状态寄存器	 
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
	//reserved: bit7, only '0' allowed
    #define MASK_RX_DR   (1<<6) //RW, Mask interrupt caused by RX_DR
    #define MASK_TX_DS   (1<<5) //RW, Mask interrupt caused by TX_DS
    #define MASK_MAX_RT  (1<<4) //RW, Mask interrupt caused by MAX_RT
    #define EN_CRC       (1<<3) //RW, Enable CRC.
    #define CRCO         (1<<2) //RW, CRC encoding scheme, 0: 1 byte, 1: 2 bytes
    #define PWR_UP       (1<<1) //RW, Power up, 1: power up, 0: power down
    #define PRIM_RX      (1<<0) //RW, RX/TX control, 1: PRX, 0: PTX
#define EN_AA          0x01//使能自动应答功能  bit0~5,对应通道0~5
    //reserved: bit7-bit6:only '00' allowed
    #define ENAA_P5    (1<<5) //RW, Enable auto acknowledgement data pipe 5
    #define ENAA_P4    (1<<4) //RW, Enable auto acknowledgement data pipe 4
    #define ENAA_P3    (1<<3) //RW, Enable auto acknowledgement data pipe 3
    #define ENAA_P2    (1<<2) //RW, Enable auto acknowledgement data pipe 2
    #define ENAA_P1    (1<<1) //RW, Enable auto acknowledgement data pipe 1
    #define ENAA_P0    (1<<0) //RW, Enable auto acknowledgement data pipe 0
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
    //reserved: bit7-bit6:only '00' allowed
    #define ERX_P5     (1<<5) //RW, Enable data pipe 5
    #define ERX_P4     (1<<4) //RW, Enable data pipe 4
    #define ERX_P3     (1<<3) //RW, Enable data pipe 3
    #define ERX_P2     (1<<2) //RW, Enable data pipe 2
    #define ERX_P1     (1<<1) //RW, Enable data pipe 1
    #define ERX_P0     (1<<0) //RW, Enable data pipe 0
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
    // reserved: bit7-bit2, only '000000' allowed
    #define AW_RERSERVED 0x0 
    #define AW_3BYTES    0x1
    #define AW_4BYTES    0x2
    #define AW_5BYTES    0x3
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
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
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;

#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
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
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
    //reserved, bit7, only '0' allowed
    #define RX_DR         6 //RW, Data ready RX FIFO interrupt
    #define TX_DS         5 //RW, Data sent TX FIFO interrupt
    #define MAX_RT        4 //RW, Maximum number of TX retransmits interrupt
    //for bit3-bit1, R, Data pipe number for the payload available for reading from
    //RX_FIFO, 000-101, Data pipe number, 110, not used, 111, RX FIFO empty
    #define TX_FULL_0     0 //R, TX FIFO full flag
#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#define OBSERVE_TX      0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define REG_RPD         0x09  //载波检测寄存器,bit0,载波检测;
    //bit7-bit1, reserved
    #define RPD           0 //Received power detector
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等

#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
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
//24L01操作线
#define NRF24L01_CE   PGout(8) //24L01片选信号
#define NRF24L01_CSN  PGout(7) //SPI片选信号	   
#define NRF24L01_IRQ  PGin(6)  //IRQ主机数据输入
//24L01发送接收数据宽度定义
#define ADR_WIDTH    5   	//5字节的地址宽度

#define TX_PLOAD_WIDTH  32  	//32字节的用户数据宽度
#define RX_PLOAD_WIDTH  32 		//32字节的用户数据宽度

typedef enum{ TX_MODE, RX_MODE } NRF24L01_MODE; 		//TX_MODE->0,RX_MODE->1

									   	   

void NRF24L01_Init(NRF24L01_MODE RxTx_mode);	//初始化
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 u8s);//写数据区
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 u8s);	//读数据区		  
u8 NRF24L01_Read_Reg(u8 reg);					//读寄存器
u8 NRF24L01_Write_Reg(u8 reg, u8 value);		//写寄存器
u8 NRF24L01_Check(void);						//检查24L01是否存在
u8 NRF24L01_TxPacket(u8 *txbuf);				//发送一个包的数据
u8 NRF24L01_RxPacket(u8 *rxbuf);				//接收一个包的数据
void NRF24L01_PowerDown_Mode(void);				//配置为power down模式
void NRF24L01_FlushTX(void);					//复位TX FIFO指针
void NRF24L01_FlushRX(void);					//复位RX FIFO指针
void NRF24L01_SetTRMode(NRF24L01_MODE mode);	//设置收发模式	
void NRF24L01_ACK_W_Packet(u8 *Data,u8 Data_Length);			//将自动应答的payload写入TX FIFO

#endif











