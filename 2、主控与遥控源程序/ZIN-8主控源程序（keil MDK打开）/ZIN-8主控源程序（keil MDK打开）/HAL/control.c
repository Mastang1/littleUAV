/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"


//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1 
#undef EMERGENT
#define EMERGENT 0
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //结构体数组，将每一个数组放一个pid结构体，这样就可以批量操作各个PID的数据了  比如解锁时批量复位pid控制数据，新手明白这句话的作用就可以了
		,&pidHeightRate
		,&pidHeightHigh
};


/**************************************************************
 *  Height control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void HeightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;
		int16_t acc;       //当前你飞行器的加速度值
   	int16_t acc_error; //当前加速度减去重力加速度则为上下移动的加速度
   static int16_t acc_offset;//重力加速度值
	 static uint32_t high = 0; //当前高度
		static float thr_hold = 0; //进入高度时记录当前油门值

	//----------------------------------------------	
	{	//获取当前高度数据  ZIN-34  SPL06-001模块

		#define ZIN_34_ADDRESS 0xC0 //模块地址 最低位读写标志位 0：写  1：读
		#define HEIGHT_READ 0X33 //高度数据寄存器地址
		uint8_t data[3];
		
		if(IIC_read_Bytes(ZIN_34_ADDRESS,HEIGHT_READ,data,3) == FAILED)
    {
			pidHeightRate.out = Remote.thr -1000;//高度错误则改为油门输出
			return;//如果不支持定高，或者气压计有错误则直接返回。
		}			
		high = ((u32)data[0]<<16) | ((u16)data[1]<<8) | data[2];   //海拔高度
		if(high>10000000)//高度异常
			return;
	}	
	//----------------------------------------------	
	//----------------------------------------------	
	{ //获取垂直速度数据

		acc = (int16_t)GetNormAccz();//提取重力向量
		
			if(!ALL_flag.unlock) //取得静态加速度值
			{
				acc_offset = acc;
			}
			acc_error = acc - acc_offset;	
						
			{//此处做一个速度与高度的互补滤波 
				static float last_high;
				pidHeightRate.measured = (pidHeightRate.measured + acc_error * dt)*0.98f+0.02f*(high - last_high)/dt; //速度环永远是主调，所以互补滤波关键就抓在速度环这里
				last_high =  pidHeightHigh.measured = high;  //高度辅调
			}	
		}
	//----------------------------------------------紧急终止飞行
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
	//----------------------------------------------控制
	switch(status)
	{
		case WAITING_1: //检测定高
		  if(ALL_flag.height_lock && ALL_flag.unlock) 
			{
				LED.status = WARNING;
				status = WAITING_2;
			}
			break;
		case WAITING_2: //定高前准备
			thr_hold = Remote.thr -1000;  //记录定高时的油门
			status = PROCESS_31;
			break;
		
		case PROCESS_31://进入定高	
			{
					 static uint8_t set_high = 0;
					 
					 	 if(Remote.thr<1850 && Remote.thr>1150) //如果油门已回中，不调高度
						 {
									if(set_high == 0) //如果刚退出调高
									{
										set_high = 1;
										pidHeightHigh.desired = pidHeightHigh.measured;//记录油门回中的高度当做当前定高高度
									}
									pidUpdate(&pidHeightHigh,dt);    //调用PID处理函数来处理外环	俯仰角PID	
									pidHeightRate.desired = pidHeightHigh.out;  
						 }
						else if(Remote.thr>1700) //油门上拉则上升 调整高度
						{
							set_high = 0;
							pidHeightRate.desired = 300; //上升速度可调
						}
						else if	(Remote.thr<1200) //油门下拉则下降	调整高度	
						{
							set_high = 0;
							pidHeightRate.desired = -200; //下降速度可调
						}					 
				 }
								 
			pidUpdate(&pidHeightRate,dt); //再调用内环
				 
		  pidHeightRate.out += thr_hold;//加入悬停时的油门
				 
			if(!ALL_flag.height_lock)  //退出定高
			{
				LED.status = AlwaysOn ;
				status = EXIT_255;
			}
			break;
		case EXIT_255: //退出定高
			pidRest(&pPidObject[6],2);	//清除当前的定高输出值
			status = WAITING_1;//回到等待进入定高
			break;
		default:
			status = WAITING_1;
			break;	
	}	
		
		
}



/**************************************************************
 *  flight control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void FlightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //等待解锁
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //准备进入控制
			pidRest(pPidObject,8); //批量复位PID数据，防止上次遗留的数据影响本次控制

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //锁定偏航角
		  
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //正式进入控制
			if(Angle.pitch<-50||Angle.pitch>50||Angle.roll<-50||Angle.roll>50)//倾斜检测，大角度判定为意外情况，则紧急上锁			
				ALL_flag.unlock = EMERGENT;//打入紧急情况
			
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //内环测量值 角度/秒
			pidRateY.measured = MPU6050.gyroY * Gyro_G;
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
		
			pidPitch.measured = Angle.pitch; //外环测量值 单位：角度
		  pidRoll.measured = Angle.roll;
			pidYaw.measured = Angle.yaw;
		
		 	pidUpdate(&pidRoll,dt);    //调用PID处理函数来处理外环	横滚角PID		
			pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
			pidUpdate(&pidRateX,dt);  //再调用内环

		 	pidUpdate(&pidPitch,dt);    //调用PID处理函数来处理外环	俯仰角PID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //再调用内环

			CascadePID(&pidRateZ,&pidYaw,dt);	//也可以直接调用串级PID函数来处理
			break;
		case EXIT_255:  //退出控制
			pidRest(pPidObject,8);
			status = WAITING_1;//返回等待解锁
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
}

/**************************************************************
 *  电机输出
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
int16_t motor[4];
#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3] 

void MotorControl(void)
{	
	volatile static uint8_t status=WAITING_1;
	
	
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: //等待解锁	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2: //解锁完成后判断使用者是否开始拨动遥杆进行飞行控制
			if(Remote.thr>1100)
			{
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			{
				int16_t temp;
				if(ALL_flag.height_lock) //定高模式下 油门遥杆作为调整高度使用
				{		
					temp = pidHeightRate.out; //输出给电机的是定高输出值
				}
				else //正常飞行状态，油门正常使用
				{
					if(Remote.thr<1020) //油门太低了 则限制输出 以免在地上乱转
					{
						MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
						break;
					}
					temp = Remote.thr -1000; //输出给电机的是油门输出值
				}
				
				//将油门值作为基础值给PWM
				MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(temp,0,800); //留200给姿态控制
				
//再加上				
//以下输出的脉冲分配取决于电机PWM分布与飞控坐标体系。请看飞控坐标体系图解，与四个电机PWM分布分布	
//           机头      
//   PWM3     ♂       PWM2
//      *           *
//      	*       *
//    		  *   *
//      			*  
//    		  *   *
//      	*       *
//      *           *
//    PWM4           PWM1
//		pidRateX.out 横滚角串级PID输出 控制左右，可以看出1 2和3 4，左右两组电机同增同减
//    pidRateY.out 俯仰角串级PID输出 控制前后，可以看出2 3和1 4，前后两组电机同增同减
//		pidRateZ.out 横滚角串级PID输出 控制旋转，可以看出2 4和1 3，两组对角线电机同增同减	
				MOTOR1 +=    + pidRateX.out - pidRateY.out - pidRateZ.out;//; 姿态输出分配给各个电机的控制量,请看飞控坐标体系图解，与四个电机PWM分布分布
				MOTOR2 +=    + pidRateX.out + pidRateY.out + pidRateZ.out ;//;
				MOTOR3 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
				MOTOR4 +=    - pidRateX.out - pidRateY.out + pidRateZ.out;//;
			}	
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
			status = WAITING_1;	
			break;
		default:
			break;
	}
	
	
	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);  //更新PWM
	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);
	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);
	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);
} 
/************************************END OF FILE********************************************/ 
