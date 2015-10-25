#ifndef __RF_H_
#define __RF_H_

#include "stm32f1xx_hal.h"

#define nRF_TX_ADR_WIDTH    5   //5�ֽڵĵ�ַ���
#define nRF_RX_ADR_WIDTH    5   //5�ֽڵĵ�ַ���
#define nRF_TX_PLOAD_WIDTH  32  //20�ֽڵ��û����ݿ��
#define nRF_RX_PLOAD_WIDTH  32  //20�ֽڵ��û����ݿ��


//NRF24L01�Ĵ�����������
#define nRF_READ_REG        0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define nRF_WRITE_REG       0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define nRF_RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define nRF_WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define nRF_FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define nRF_FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define nRF_REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define nRF_NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
//SPI(NRF24L01)�Ĵ�����ַ
#define nRF_CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define nRF_EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define nRF_EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define nRF_SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define nRF_SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define nRF_RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define nRF_RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define nRF_STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
#define nRF_MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define nRF_TX_OK   		0x20  //TX��������ж�
#define nRF_RX_OK   		0x40  //���յ������ж�
#define nRF_TIMEOUT			0x80  // ��ʱ��־

#define nRF_OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define nRF_CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define nRF_RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define nRF_RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define nRF_RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define nRF_RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define nRF_RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define nRF_RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define nRF_TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define nRF_RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define nRF_RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define nRF_RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define nRF_RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define nRF_RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define nRF_RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define nRF_FIFO_STATUS     0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;

#define nRF_State_TX_OK		(0x01<<0)		// 1. �������ݳɹ�
#define nRF_State_TX_MAX	(0x01<<1)		// 1. ��������������
#define nRF_State_RX_OK		(0x01<<2)		// 1. �������ݳɹ�

#define RF_CE_LOW()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
#define RF_CE_HIGH()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)

extern EventGroupHandle_t xRFEventGruop;
void rf_io_init(void);

#endif
