/**************************************************************
 * 
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
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
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //�ṹ�����飬��ÿһ�������һ��pid�ṹ�壬�����Ϳ���������������PID��������  �������ʱ������λpid�������ݣ�����������仰�����þͿ�����
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
		int16_t acc;       //��ǰ��������ļ��ٶ�ֵ
   	int16_t acc_error; //��ǰ���ٶȼ�ȥ�������ٶ���Ϊ�����ƶ��ļ��ٶ�
   static int16_t acc_offset;//�������ٶ�ֵ
	 static uint32_t high = 0; //��ǰ�߶�
		static float thr_hold = 0; //����߶�ʱ��¼��ǰ����ֵ

	//----------------------------------------------	
	{	//��ȡ��ǰ�߶�����  ZIN-34  SPL06-001ģ��

		#define ZIN_34_ADDRESS 0xC0 //ģ���ַ ���λ��д��־λ 0��д  1����
		#define HEIGHT_READ 0X33 //�߶����ݼĴ�����ַ
		uint8_t data[3];
		
		if(IIC_read_Bytes(ZIN_34_ADDRESS,HEIGHT_READ,data,3) == FAILED)
    {
			pidHeightRate.out = Remote.thr -1000;//�߶ȴ������Ϊ�������
			return;//�����֧�ֶ��ߣ�������ѹ���д�����ֱ�ӷ��ء�
		}			
		high = ((u32)data[0]<<16) | ((u16)data[1]<<8) | data[2];   //���θ߶�
		if(high>10000000)//�߶��쳣
			return;
	}	
	//----------------------------------------------	
	//----------------------------------------------	
	{ //��ȡ��ֱ�ٶ�����

		acc = (int16_t)GetNormAccz();//��ȡ��������
		
			if(!ALL_flag.unlock) //ȡ�þ�̬���ٶ�ֵ
			{
				acc_offset = acc;
			}
			acc_error = acc - acc_offset;	
						
			{//�˴���һ���ٶ���߶ȵĻ����˲� 
				static float last_high;
				pidHeightRate.measured = (pidHeightRate.measured + acc_error * dt)*0.98f+0.02f*(high - last_high)/dt; //�ٶȻ���Զ�����������Ի����˲��ؼ���ץ���ٶȻ�����
				last_high =  pidHeightHigh.measured = high;  //�߶ȸ���
			}	
		}
	//----------------------------------------------������ֹ����
	if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;
	//----------------------------------------------����
	switch(status)
	{
		case WAITING_1: //��ⶨ��
		  if(ALL_flag.height_lock && ALL_flag.unlock) 
			{
				LED.status = WARNING;
				status = WAITING_2;
			}
			break;
		case WAITING_2: //����ǰ׼��
			thr_hold = Remote.thr -1000;  //��¼����ʱ������
			status = PROCESS_31;
			break;
		
		case PROCESS_31://���붨��	
			{
					 static uint8_t set_high = 0;
					 
					 	 if(Remote.thr<1850 && Remote.thr>1150) //��������ѻ��У������߶�
						 {
									if(set_high == 0) //������˳�����
									{
										set_high = 1;
										pidHeightHigh.desired = pidHeightHigh.measured;//��¼���Ż��еĸ߶ȵ�����ǰ���߸߶�
									}
									pidUpdate(&pidHeightHigh,dt);    //����PID�������������⻷	������PID	
									pidHeightRate.desired = pidHeightHigh.out;  
						 }
						else if(Remote.thr>1700) //�������������� �����߶�
						{
							set_high = 0;
							pidHeightRate.desired = 300; //�����ٶȿɵ�
						}
						else if	(Remote.thr<1200) //�����������½�	�����߶�	
						{
							set_high = 0;
							pidHeightRate.desired = -200; //�½��ٶȿɵ�
						}					 
				 }
								 
			pidUpdate(&pidHeightRate,dt); //�ٵ����ڻ�
				 
		  pidHeightRate.out += thr_hold;//������ͣʱ������
				 
			if(!ALL_flag.height_lock)  //�˳�����
			{
				LED.status = AlwaysOn ;
				status = EXIT_255;
			}
			break;
		case EXIT_255: //�˳�����
			pidRest(&pPidObject[6],2);	//�����ǰ�Ķ������ֵ
			status = WAITING_1;//�ص��ȴ����붨��
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
		case WAITING_1: //�ȴ�����
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //׼���������
			pidRest(pPidObject,8); //������λPID���ݣ���ֹ�ϴ�����������Ӱ�챾�ο���

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //����ƫ����
		  
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //��ʽ�������
			if(Angle.pitch<-50||Angle.pitch>50||Angle.roll<-50||Angle.roll>50)//��б��⣬��Ƕ��ж�Ϊ������������������			
				ALL_flag.unlock = EMERGENT;//����������
			
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
			pidRateY.measured = MPU6050.gyroY * Gyro_G;
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
		
			pidPitch.measured = Angle.pitch; //�⻷����ֵ ��λ���Ƕ�
		  pidRoll.measured = Angle.roll;
			pidYaw.measured = Angle.yaw;
		
		 	pidUpdate(&pidRoll,dt);    //����PID�������������⻷	�����PID		
			pidRateX.desired = pidRoll.out; //���⻷��PID�����Ϊ�ڻ�PID������ֵ��Ϊ����PID
			pidUpdate(&pidRateX,dt);  //�ٵ����ڻ�

		 	pidUpdate(&pidPitch,dt);    //����PID�������������⻷	������PID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //�ٵ����ڻ�

			CascadePID(&pidRateZ,&pidYaw,dt);	//Ҳ����ֱ�ӵ��ô���PID����������
			break;
		case EXIT_255:  //�˳�����
			pidRest(pPidObject,8);
			status = WAITING_1;//���صȴ�����
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;
}

/**************************************************************
 *  ������
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
	
	
	if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: //�ȴ�����	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //������������������Ϊ0
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2: //������ɺ��ж�ʹ�����Ƿ�ʼ����ң�˽��з��п���
			if(Remote.thr>1100)
			{
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			{
				int16_t temp;
				if(ALL_flag.height_lock) //����ģʽ�� ����ң����Ϊ�����߶�ʹ��
				{		
					temp = pidHeightRate.out; //�����������Ƕ������ֵ
				}
				else //��������״̬����������ʹ��
				{
					if(Remote.thr<1020) //����̫���� ��������� �����ڵ�����ת
					{
						MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
						break;
					}
					temp = Remote.thr -1000; //�������������������ֵ
				}
				
				//������ֵ��Ϊ����ֵ��PWM
				MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(temp,0,800); //��200����̬����
				
//�ټ���				
//����������������ȡ���ڵ��PWM�ֲ���ɿ�������ϵ���뿴�ɿ�������ϵͼ�⣬���ĸ����PWM�ֲ��ֲ�	
//           ��ͷ      
//   PWM3     ��       PWM2
//      *           *
//      	*       *
//    		  *   *
//      			*  
//    		  *   *
//      	*       *
//      *           *
//    PWM4           PWM1
//		pidRateX.out ����Ǵ���PID��� �������ң����Կ���1 2��3 4������������ͬ��ͬ��
//    pidRateY.out �����Ǵ���PID��� ����ǰ�󣬿��Կ���2 3��1 4��ǰ��������ͬ��ͬ��
//		pidRateZ.out ����Ǵ���PID��� ������ת�����Կ���2 4��1 3������Խ��ߵ��ͬ��ͬ��	
				MOTOR1 +=    + pidRateX.out - pidRateY.out - pidRateZ.out;//; ��̬����������������Ŀ�����,�뿴�ɿ�������ϵͼ�⣬���ĸ����PWM�ֲ��ֲ�
				MOTOR2 +=    + pidRateX.out + pidRateY.out + pidRateZ.out ;//;
				MOTOR3 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
				MOTOR4 +=    - pidRateX.out - pidRateY.out + pidRateZ.out;//;
			}	
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //������������������Ϊ0
			status = WAITING_1;	
			break;
		default:
			break;
	}
	
	
	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);  //����PWM
	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);
	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);
	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);
} 
/************************************END OF FILE********************************************/ 
