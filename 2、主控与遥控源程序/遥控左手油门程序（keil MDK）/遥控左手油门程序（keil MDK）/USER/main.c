/*******************************************************************
 *�������
 *@brief ����ʼ����������ѯ���õĺ���
 *@brief 
 *@time ZIN���Ӳ�Ʒ�뼼�� 2017.1.8
 *@editorС��&zin
 *�ɿذ���QQȺ551883670,����759421287@qq.com
 *����Ȩʹ����Ա����ֹʹ�á���ֹ���ģ�Υ��һ�����֣���Ȩ����
 *רҵ�ķɿز�����õķɿ�
 *@brief
 ͨ����ң�أ���������NRFͨ�ţ�ֻҪNRF��ַ��ƥ�䣬��ַ�����NRF24L01.C
 ��������֡�շ������������֡�ֲᡣ
 ******************************************************************/
#include "stm32f10x.h"
#include "sys.h"
#include "stdio.h"
#include "delay.h"
//����ϵͳʱ��,ʹ�ܸ�����ʱ��
void RCC_Configuration(void)
{
		RCC_DeInit();//������ RCC�Ĵ�������Ϊȱʡֵ  
  
    RCC_HSICmd(ENABLE);//ʹ��HSI    
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);//�ȴ�HSIʹ�ܳɹ�  
    //������������ܵ�64M
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  
    FLASH_SetLatency(FLASH_Latency_2);  
     
    RCC_HCLKConfig(RCC_SYSCLK_Div1);     
    RCC_PCLK1Config(RCC_HCLK_Div2);  
    RCC_PCLK2Config(RCC_HCLK_Div1);  
      
    //���� PLL ʱ��Դ����Ƶϵ��  48Mhz
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);//ʹ�ܻ���ʧ�� PLL,�����������ȡ��ENABLE����DISABLE   
    RCC_PLLCmd(ENABLE);//���PLL������ϵͳʱ��,��ô�����ܱ�ʧ��  
    //�ȴ�ָ���� RCC ��־λ���óɹ� �ȴ�PLL��ʼ���ɹ�  
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  
  
    //����ϵͳʱ�ӣ�SYSCLK�� ����PLLΪϵͳʱ��Դ  
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//ѡ����Ҫ��ϵͳʱ��   
    //�ȴ�PLL�ɹ�������ϵͳʱ�ӵ�ʱ��Դ  
    //  0x00��HSI ��Ϊϵͳʱ��   
    //  0x04��HSE��Ϊϵͳʱ��   
    //  0x08��PLL��Ϊϵͳʱ��    
    while(RCC_GetSYSCLKSource() != 0x08);//���뱻ѡ���ϵͳʱ�Ӷ�Ӧ������RCC_SYSCLKSource_PLL  
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
                           |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
						   |RCC_APB2Periph_ADC1  | RCC_APB2Periph_AFIO 
                           |RCC_APB2Periph_SPI1  
						   	, ENABLE );
	
   	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB1Periph_I2C1| RCC_APB1Periph_TIM3, ENABLE );
}



void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* ʹ�ܴ���1�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	//USART1  ����1ȫ���ж� 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //�����ȼ�1
	/*IRQͨ��ʹ��*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1*/
	NVIC_Init(&NVIC_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}

void BEEP_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    //ʹ��GPIO��ʱ��  CE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;          //NRF24L01 ģ��Ƭѡ�ź�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	BEEP_H;
}



//main���� ��������￪ʼִ��
int main(void)
{

	RCC_Configuration();//���þ���//ң��Ҫ��Ƚϵ͡�����ʹ��ʹ���ڲ�����8M/2��Ƶ*12��Ƶ=48MHz
	
	cycleCounterInit();//��ʼ��ÿ���δ�ʱ���ļ���ֵ��Ϊ��ʱ������׼��
	
	
	BEEP_INIT(); //����������
	{
		int32_t beep_cnt = 2000000;
		while(beep_cnt-->0)
		{
			
		}
		BEEP_L;
	}
	
	SysTick_Config(48000000 / 1000);	//����ϵͳ�δ�ʱ�� ����1ms����һ���ж�
	

	_g_Init_sys(); //10ms�жϴ����������SYS.C

	NVIC_Configuration();//�ж���������

	while(1)
	{
		//�������ݶ���10ms�жϴ����������SYS.C
	}
}










