/**************************************************************
 * 
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DEFINE.h"



//�ر��������������˿տ��ĵش��������ڽ��з��С������ǳ�������������ɽ����ر�ң�ء�

/**************************************************************
 * ����ϵͳ����Ϊ��ѯ���ж��н���Ľ��С�
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
int main(void)
{	
	cycleCounterInit();  //�õ�ϵͳÿ��us��ϵͳCLK������Ϊ�Ժ���ʱ�������͵õ���׼�ĵ�ǰִ��ʱ��ʹ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //2��bit����ռ���ȼ���2��bit�������ȼ�
	SysTick_Config(SystemCoreClock / 1000);	//ϵͳ�δ�ʱ��
	
	ALL_Init();
	
	
//	ALL_Init(do
//{
//	// TODO: enter the block content here
//	
//	
//} while ();
//);//ϵͳ��ʼ��
// 
	/*
	 whileѭ������һЩ�Ƚϲ���Ҫ�����飬������λ����LED���ơ�
	���๦�ܽ԰������жϣ����scheduler.c����Ĺ���
	*/
	while(1)  
	{		  
			PilotLED(); //LEDˢ��
	}
}










