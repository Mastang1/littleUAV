/*******************************************************************
 *SYS ϵͳ�����ļ�
 *@brief 
 *@brief 
 *@time  2016.1.8
 *@editorС��&zin
 *�ɿذ���QQȺ551883670,����759421287@qq.com
 *����Ȩʹ����Ա����ֹʹ�á���ֹ���ģ�Υ��һ�����֣���Ȩ����
 ******************************************************************/
#include "sys.h"
#include "ADC_DMA_Config.h"
#include "SPI.h"
#include "nrf24l01.h"
#include "TIM.h"
#include "eeprom.h"
#include "remote.h"
#include <string.h>
	#include "I2C.h"
uint32_t SysTick_count = 0;


void _g_Init_sys(void)
{
	IIC_Init();//Ӳ��I2C��ʼ��
	NRF24L01_init(); //NRF��ʼ��
	
	while(test_AT24C02()!=0);//EEPROM��ʼ��ʧ��
	ADC1_GPIO_Config();//ADC���ų�ʼ��
	ADC1_Mode_Config();//ADC DMA ��ʼ��������DMA������ת��ADC����DMA�Զ��ض�DAC��ֵ�����������������ֶ�������ȡDAC

	RC_INIT();//ң��ֵ��ʼ��
}




//ͨ��Э�飺
//��ʼ֡0XAA,0XAF + ��ַ֡ 0X03 + ���ݳ��� + 12��ң������ + У���ֽ�


void SysTick_Handler(void)//�δ�ʱ��1ms�ж�
{   
	static uint8_t Count_10ms;
	
	SysTick_count++;	
	
	if(Count_10ms++>=10)
	{
		Count_10ms = 0;
		RC_Analy();//ң�����ݽ���
	}
}




