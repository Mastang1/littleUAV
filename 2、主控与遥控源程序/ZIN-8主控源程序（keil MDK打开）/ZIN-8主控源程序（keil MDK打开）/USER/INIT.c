/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DEFINE.h"


volatile uint32_t SysTick_count; //系统时间计数
_st_Mpu MPU6050;   //MPU6050原始数据
_st_Mag AK8975;
_st_AngE Angle;    //当前角度姿态值
_st_Remote Remote; //遥控通道值


_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等


PidObject pidRateX; //内环PID数据
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID数据
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate;
PidObject pidHeightHigh;

void pid_param_Init(void); //PID控制参数初始化，改写PID并不会保存数据，请调试完成后直接在程序里更改 再烧录到飞控


/**************************************************************
 *  整个系统外设和传感器初始化
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void ALL_Init(void)
{
	IIC_Init();             //I2C初始化

	pid_param_Init();       //PID参数初始化
	 
	LEDInit();              //LED闪灯初始化

	MpuInit();              //MPU6050初始化

	NRF24L01_init();				//2.4G遥控通信初始化
	
	TIM2_PWM_Config();			//4路PWM初始化
	
	delay_ms(5000);         //延时等待输出气压计稳定，避免刚上电前几秒会有温升问题，海拔气压高度计模块会延迟几秒输出
	
	TIM3_Config();					//系统工作周期初始化 w 
	
}


/**************************************************************
 *  初始化PID参数
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void pid_param_Init(void)
{

	//4.7w rpm
//	pidRateX.kp = 1.0;//内环角速度PID值
//	pidRateY.kp = 1.0f;
//	pidRateZ.kp = 3.0f;
//	
//	pidRateX.ki = 0.2f;
//	pidRateY.ki = 0.2f;
//	pidRateZ.ki = 0.0f;	
//	
//	pidRateX.kd = 0.15f;
//	pidRateY.kd = 0.15f;
//	pidRateZ.kd = 0.3f;	
//	
//	pidPitch.kp = 6.5f;//外环角度PID值
//	pidRoll.kp = 6.5f;
//	pidYaw.kp = 5.0f;	
	
	//5.5w rpm 20190530
	pidRateX.kp = 1.0;//内环角速度PID值
	pidRateY.kp = 1.0f;
	pidRateZ.kp = 3.0f;
	
	pidRateX.ki = 0.75f;
	pidRateY.ki = 0.75f;
	pidRateZ.ki = 0.0f;	
	
	pidRateX.kd = 0.13f;
	pidRateY.kd = 0.13f;
	pidRateZ.kd = 0.3f;	
	
	pidPitch.kp = 3.5f;//外环角度PID值
	pidRoll.kp = 3.5f;
	pidYaw.kp = 3.0f;	
	
	pidHeightRate.kp = 0.5f;
	pidHeightRate.kd = 0.3f;
	
//	pidHeightHigh.ki = 0.05f;
	pidHeightHigh.kp = 0.16f;
}















