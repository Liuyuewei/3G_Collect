#ifndef __TIMER3_H__
#define __TIMER3_H__

#include <rtthread.h>

#include "rtdevice.h"

//初始化定时器
void TIM3_Init(rt_uint32_t arr,rt_uint32_t psc);
//启动定时器
void TIM3_Start(void);
//停止定时器
void TIM3_Stop(void);

#endif

