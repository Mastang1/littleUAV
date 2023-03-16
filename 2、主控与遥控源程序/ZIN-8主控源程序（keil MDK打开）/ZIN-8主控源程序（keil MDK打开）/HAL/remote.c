/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"


#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1
/*****************************************************************************************
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
	uint8_t RC_rxData[32];
void remote_unlock(void);	
void RC_Analy(void)  
{
		static uint16_t cnt;
/*             Receive  and check RC data                               */	
	if(NRF24L01_RxPacket(RC_rxData)==SUCCESS)
	{ 	

		uint8_t i;
		uint8_t CheckSum=0;
		cnt = 0;
		for(i=0;i<31;i++)
		{
			CheckSum +=  RC_rxData[i];
		}
		if(RC_rxData[31]==CheckSum && RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)  //如果接收到的遥控数据正确
		{
			  Remote.roll = ((uint16_t)RC_rxData[4]<<8) | RC_rxData[5];  //通道1
				Remote.roll = LIMIT(Remote.roll,1000,2000);
				Remote.pitch = ((uint16_t)RC_rxData[6]<<8) | RC_rxData[7];  //通道2
				Remote.pitch =LIMIT(Remote.pitch,1000,2000);
				Remote.thr = 	((uint16_t)RC_rxData[8]<<8) | RC_rxData[9];   //通道3
				Remote.thr = 	LIMIT(Remote.thr,1000,2000);
				Remote.yaw =  ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //通道4
				Remote.yaw =  LIMIT(Remote.yaw,1000,2000);
				Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //通道5  左上角按键都属于通道5  
				Remote.AUX1 =  LIMIT(Remote.AUX1,1000,2000);
				Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //通道6  右上角按键都属于通道6 
				Remote.AUX2 = LIMIT(Remote.AUX2,1000,2000);
				Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //通道7  左下边按键都属于通道7 
				Remote.AUX3 =  LIMIT(Remote.AUX3,1000,2000);
				Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //通道8  右下边按键都属于通道6  
				Remote.AUX4 = LIMIT(Remote.AUX4,1000,2000);		
				//---------------------------------------------------------------------------------
				//开始处理遥控数据
				{
							const float roll_pitch_ratio = 0.04f;  
					
							pidPitch.desired =-(Remote.pitch-1500)*roll_pitch_ratio;	 //将遥杆值作为飞行角度的期望值
							pidRoll.desired = -(Remote.roll-1500)*roll_pitch_ratio;
					    if(Remote.yaw>1820 )
							{	//以下为遥控控制偏航角 +-号代表方向 0.75代表控制偏航角的旋转量							
								pidYaw.desired += 0.45f;	
							}
							else if(Remote.yaw <1180)
							{
								pidYaw.desired -= 0.45f;	
							}
							//通道5作为定高控制
							if(Remote.AUX1>1700)
							{
								ALL_flag.height_lock = 1;
							}
							else
							{
								ALL_flag.height_lock = 0;							
							}								
							//限制油门不要太过猛烈
							Remote.thr *= 	0.9f;
							Remote.thr = 	LIMIT(Remote.thr,1000,2000);
				}
				remote_unlock();
			
		}
  }
//如果3秒没收到遥控数据，则判断遥控信号丢失，飞控在任何时候停止飞行，避免伤人。
//意外情况，使用者可紧急关闭遥控电源，飞行器会在3秒后立即关闭，避免伤人。
//立即关闭遥控，如果在飞行中会直接掉落，可能会损坏飞行器。
	else
	{
	
		
		cnt++;
		if(cnt>2000)
		{
			cnt = 0;
			ALL_flag.unlock = 0; 
			NRF24L01_init();
		}
	}
}

/*****************************************************************************************
 *  解锁判断
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
void remote_unlock(void)
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Remote.thr<1150 &&Remote.yaw<1200)                         //油门遥杆左下角锁定
	{
		status = EXIT_255;
	}
	
	switch(status)
	{
		case WAITING_1://等待解锁
 //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁			
			if(Remote.thr<1100)  //第一步        
			{			 
					 status = WAITING_2;				 
			}		
			break;
		case WAITING_2:  
			if(Remote.thr>1600)  //第二步         
			{			 
					 status = WAITING_3;				 
			}			
			break;
		case WAITING_3:
			if(Remote.thr<1100)  //第三步        
			{			 
					 status = WAITING_4;				 
			}			
			break;			
		case WAITING_4:	//解锁前准备	               
				ALL_flag.unlock = 1;   //解锁标志位
				status = PROCESS_31;   //进入控制
				LED.status = AlwaysOn; //LED常亮									
				 break;		
		case PROCESS_31:	//进入解锁状态
				if(Remote.thr<1020)
				{
					if(cnt++ > 2000)                                     // 油门遥杆处于最低6S自动上锁
					{								
						status = EXIT_255;								
					}
				}
				else if(!ALL_flag.unlock)                           //其它紧急情况可直接锁定飞控，比如DIY一键降落者
				{
					status = EXIT_255;				
				}
				else					
					cnt = 0;
			break;
		case EXIT_255: //进入锁定
			LED.status = AllFlashLight;	                                 //exit
			cnt = 0;
			LED.FlashTime = 100;  		                                   
			ALL_flag.unlock = 0;
			status = WAITING_1;
			break;
		default:
			status = EXIT_255;
			break;
	}
}
/***********************END OF FILE*************************************/







