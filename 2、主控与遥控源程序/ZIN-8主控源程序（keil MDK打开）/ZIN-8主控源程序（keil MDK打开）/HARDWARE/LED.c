/*******************************************************************
 *@title LED system
 *@brief flight light
 *@brief 
 *@time  2016.10.19
 *@editorС��&zin
 *�ɿذ���QQȺ551883670,����759421287@qq.com
 ******************************************************************/
#include "stm32f10x.h"
#include "LED.h"
#include "ALL_DATA.h"
//����ǰ��				 
#define fLED_H()  GPIOB->BSRR = GPIO_Pin_1
#define fLED_L()  GPIOB->BRR  = GPIO_Pin_1
#define fLED_Toggle()  GPIOB->ODR ^= GPIO_Pin_1

//-------------------------------------------------
//������				 
#define bLED_H()  GPIOB->BSRR = GPIO_Pin_9
#define bLED_L()  GPIOB->BRR  = GPIO_Pin_9
#define bLED_Toggle()  GPIOB->ODR ^= GPIO_Pin_9

//-------------------------------------------------
//---------------------------------------------------------
/*     you can select the LED statue on enum contains            */
sLED LED = {300,AllFlashLight};  //LED initial statue is off;
                             //default 300ms flash the status
/**************************************************************
 *  LED Init
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void LEDInit(void)	
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	AFIO->MAPR = 0X02000000; //ʹ��4����д �ͷ�ĳЩ����д��ص�����
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  , ENABLE);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_1;		     //LED12
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void PilotLED() //flash 300MS interval
{
	static uint32_t LastTime = 0;

	if(SysTick_count - LastTime < LED.FlashTime)
	{

		return;
	}
	else
		LastTime = SysTick_count;
	switch(LED.status)
	{
		case AlwaysOff:      //����   
			bLED_L();
			fLED_L();
			break;
		case AllFlashLight:				  //ȫ��ͬʱ��˸
			fLED_Toggle();			
			bLED_Toggle();				
		  break;
		case AlwaysOn:  //����
			bLED_H();
			fLED_H();
		  break;
		case AlternateFlash: //������˸
			bLED_H();
			fLED_L();
			LED.status = AllFlashLight;
			break;
		case WARNING:
			fLED_L();
			bLED_Toggle();
			LED.FlashTime = 100;
			break;
		case DANGEROURS:
			bLED_L();
			fLED_Toggle();
			LED.FlashTime = 70;
			break;
		default:
			LED.status = AlwaysOff;
			break;
	}
}

/**************************END OF FILE*********************************/



