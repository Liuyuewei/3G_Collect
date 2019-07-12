#include "timer3.h"
#include "stm32f4xx_hal.h"
#include "common.h"


TIM_HandleTypeDef TIM3_Handler; 									//��ʱ�����
//������ʱ��
void TIM3_Start(void)
{
	HAL_TIM_Base_Start_IT(&TIM3_Handler); 							//ʹ�ܶ�ʱ�� 3 �Ͷ�ʱ�� 3 �����ж�
}

//ֹͣ��ʱ��
void TIM3_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&TIM3_Handler);
}

//ͨ�ö�ʱ�� 3 �жϳ�ʼ��
//arr���Զ���װֵ�� psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ�� 3!(��ʱ�� 3 ���� APB1 �ϣ�ʱ��Ϊ HCLK/2)

void TIM3_Init(rt_uint32_t arr,rt_uint32_t psc)
{
	TIM3_Handler.Instance			=	TIM3; 						//ͨ�ö�ʱ�� 3
	TIM3_Handler.Init.Prescaler		=	psc;						//��Ƶϵ��
	TIM3_Handler.Init.CounterMode	=	TIM_COUNTERMODE_UP; 		//���ϼ�����
	TIM3_Handler.Init.Period		=	arr; 						//�Զ�װ��ֵ
	TIM3_Handler.Init.ClockDivision	=	TIM_CLOCKDIVISION_DIV1; 	//ʱ�ӷ�Ƶ����
	
	HAL_TIM_Base_Init(&TIM3_Handler); 								//��ʼ����ʱ�� 3
	HAL_TIM_Base_Start_IT(&TIM3_Handler); 							//ʹ�ܶ�ʱ�� 3 �Ͷ�ʱ�� 3 �����ж�
}


//��ʱ�� 3 �жϷ�����
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TIM3_Handler);
}

//������
rt_uint8_t num = 0;

//��ʱ�� 3 �жϷ���������   
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==(&TIM3_Handler))
	{
		if(num % 2)
		{
			rt_pin_write(MOTOR_STEP,PIN_HIGH);
		}
		else
		{
			rt_pin_write(MOTOR_STEP,PIN_LOW);
			//����
			step_num ++;
		}	
	}
	num ++;
}