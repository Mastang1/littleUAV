/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DEFINE.h"



//特别声明：请在无人空旷的地带或者室内进行飞行。遇到非常紧急的情况，可紧急关闭遥控。

/**************************************************************
 * 整个系统，分为轮询和中断有节奏的进行。
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
int main(void)
{	
	cycleCounterInit();  //得到系统每个us的系统CLK个数，为以后延时函数，和得到精准的当前执行时间使用
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //2个bit的抢占优先级，2个bit的子优先级
	SysTick_Config(SystemCoreClock / 1000);	//系统滴答时钟
	
	ALL_Init();
	
	
//	ALL_Init(do
//{
//	// TODO: enter the block content here
//	
//	
//} while ();
//);//系统初始化
// 
	/*
	 while循环里做一些比较不重要的事情，比如上位机和LED控制。
	其余功能皆安排在中断，请见scheduler.c里面的工作
	*/
	while(1)  
	{		  
			PilotLED(); //LED刷新
	}
}










