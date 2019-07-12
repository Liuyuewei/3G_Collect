#include "timer3.h"
#include "stm32f4xx_hal.h"
#include "common.h"


TIM_HandleTypeDef TIM3_Handler; 									//定时器句柄
//启动定时器
void TIM3_Start(void)
{
	HAL_TIM_Base_Start_IT(&TIM3_Handler); 							//使能定时器 3 和定时器 3 更新中断
}

//停止定时器
void TIM3_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&TIM3_Handler);
}

//通用定时器 3 中断初始化
//arr：自动重装值。 psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器 3!(定时器 3 挂在 APB1 上，时钟为 HCLK/2)

void TIM3_Init(rt_uint32_t arr,rt_uint32_t psc)
{
	TIM3_Handler.Instance			=	TIM3; 						//通用定时器 3
	TIM3_Handler.Init.Prescaler		=	psc;						//分频系数
	TIM3_Handler.Init.CounterMode	=	TIM_COUNTERMODE_UP; 		//向上计数器
	TIM3_Handler.Init.Period		=	arr; 						//自动装载值
	TIM3_Handler.Init.ClockDivision	=	TIM_CLOCKDIVISION_DIV1; 	//时钟分频因子
	
	HAL_TIM_Base_Init(&TIM3_Handler); 								//初始化定时器 3
	HAL_TIM_Base_Start_IT(&TIM3_Handler); 							//使能定时器 3 和定时器 3 更新中断
}


//定时器 3 中断服务函数
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TIM3_Handler);
}

//脉冲数
rt_uint8_t num = 0;

//定时器 3 中断服务函数调用   
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
			//步数
			step_num ++;
		}	
	}
	num ++;
}