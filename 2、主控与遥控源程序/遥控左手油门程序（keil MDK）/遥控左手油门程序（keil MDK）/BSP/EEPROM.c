#include "EEPROM.h"
#include "delay.h"
#include "i2c.h"


#define AT24C02_ADDR 0xA0  //AT24c02  IIC��ַ


#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1
#define START_ADDR 0  //256�ֽ� //��ʼ��ַ
#define  STOP_ADDR 0XFF //������ַ
 






uint16_t read_AT24C02(uint8_t addr)
{
	uint16_t data;
////	delay_ms(100);
////	flag = IIC_Read_One_Byte(AT24C02_ADDR,addr); //��ȡ�Ƿ��Ѿ�У׼�����ݵı�־λ
////	if(flag == 0x55) //������������Ѿ�У׼���ˣ����ȡ���ݣ����򲻶�
//	{
//		u8 data_L;
//		u8 data_H;
//		delay_ms(100);
//		data_L = IIC_Read_One_Byte(AT24C02_ADDR,addr);
//		delay_ms(100);
//		data_H = IIC_Read_One_Byte(AT24C02_ADDR,addr+2);
//		return (data_L | (data_H<<8));//���ش���EEPROM��У׼ֵ
//	}
  IIC_read_Bytes(AT24C02_ADDR,addr,(uint8_t*)&data,2);
	return data; //�����ѵ�ǰ�������Ƿ�������д��ı�־λ��ʹ����˼���Ƿ��ٴ�д��
}


void write_AT24C02(uint8_t addr,uint16_t data)
{
//	u8 data_L;
//	u8 data_H;
//	delay_ms(100);
//	IIC_Write_One_Byte(AT24C02_ADDR,addr,0x55); //��ʾ�Ѿ�������У׼��������Զ�дУ׼ֵ
//	
//	data_L = data;
//	data_H = data>>8;
//	delay_ms(100);
//	IIC_Write_One_Byte(AT24C02_ADDR,addr,data_L);
	delay_ms(10);
//	IIC_Write_One_Byte(AT24C02_ADDR,addr+2,data_H);
	IIC_Write_Bytes(AT24C02_ADDR,addr,(uint8_t*)&data,2);
	
}




uint8_t test_AT24C02(void)//���ԣ��ڵ�ַ0x55���ڴ��ַд��0xAA����س����������������д�����ͬ����Գɹ�
{
	uint8_t test = 0;
  IIC_Write_One_Byte(AT24C02_ADDR,0x55,0xAA);
	delay_ms(1000);
	test = IIC_Read_One_Byte(AT24C02_ADDR,0x55);
	if(test != 0xAA) //�洢���쳣
		return FAILED;
	else
		return SUCCESS;//���سɹ�
}
	













