/**************************************************************
 * 
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DEFINE.h"


volatile uint32_t SysTick_count; //ϵͳʱ�����
_st_Mpu MPU6050;   //MPU6050ԭʼ����
_st_Mag AK8975;
_st_AngE Angle;    //��ǰ�Ƕ���ֵ̬
_st_Remote Remote; //ң��ͨ��ֵ


_st_ALL_flag ALL_flag; //ϵͳ��־λ������������־λ��


PidObject pidRateX; //�ڻ�PID����
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //�⻷PID����
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate;
PidObject pidHeightHigh;

void pid_param_Init(void); //PID���Ʋ�����ʼ������дPID�����ᱣ�����ݣ��������ɺ�ֱ���ڳ�������� ����¼���ɿ�


/**************************************************************
 *  ����ϵͳ����ʹ�������ʼ��
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void ALL_Init(void)
{
	IIC_Init();             //I2C��ʼ��

	pid_param_Init();       //PID������ʼ��
	 
	LEDInit();              //LED���Ƴ�ʼ��

	MpuInit();              //MPU6050��ʼ��

	NRF24L01_init();				//2.4Gң��ͨ�ų�ʼ��
	
	TIM2_PWM_Config();			//4·PWM��ʼ��
	
	delay_ms(5000);         //��ʱ�ȴ������ѹ���ȶ���������ϵ�ǰ��������������⣬������ѹ�߶ȼ�ģ����ӳټ������
	
	TIM3_Config();					//ϵͳ�������ڳ�ʼ�� w 
	
}


/**************************************************************
 *  ��ʼ��PID����
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void pid_param_Init(void)
{

	//4.7w rpm
//	pidRateX.kp = 1.0;//�ڻ����ٶ�PIDֵ
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
//	pidPitch.kp = 6.5f;//�⻷�Ƕ�PIDֵ
//	pidRoll.kp = 6.5f;
//	pidYaw.kp = 5.0f;	
	
	//5.5w rpm 20190530
	pidRateX.kp = 1.0;//�ڻ����ٶ�PIDֵ
	pidRateY.kp = 1.0f;
	pidRateZ.kp = 3.0f;
	
	pidRateX.ki = 0.75f;
	pidRateY.ki = 0.75f;
	pidRateZ.ki = 0.0f;	
	
	pidRateX.kd = 0.13f;
	pidRateY.kd = 0.13f;
	pidRateZ.kd = 0.3f;	
	
	pidPitch.kp = 3.5f;//�⻷�Ƕ�PIDֵ
	pidRoll.kp = 3.5f;
	pidYaw.kp = 3.0f;	
	
	pidHeightRate.kp = 0.5f;
	pidHeightRate.kd = 0.3f;
	
//	pidHeightHigh.ki = 0.05f;
	pidHeightHigh.kp = 0.16f;
}















