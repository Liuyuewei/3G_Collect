#ifndef __TIMER3_H__
#define __TIMER3_H__

#include <rtthread.h>

#include "rtdevice.h"

//��ʼ����ʱ��
void TIM3_Init(rt_uint32_t arr,rt_uint32_t psc);
//������ʱ��
void TIM3_Start(void);
//ֹͣ��ʱ��
void TIM3_Stop(void);

#endif

