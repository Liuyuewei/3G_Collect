#ifndef COMMON_H__
#define COMMON_H__
#include "rtconfig.h"
#include <board.h>


//线程优先级
#define RT_THREAD_PRIORITY_MOTOR 	RT_THREAD_PRIORITY_MAX	- 6
#define RT_THREAD_PRIORITY_ADC 		RT_THREAD_PRIORITY_MAX	- 5
#define RT_THREAD_PRIORITY_LED 		RT_THREAD_PRIORITY_MAX	- 4

/* defined the MOTOR_STEP pin: PD4 */
#define MOTOR_STEP    GET_PIN(D, 4)

extern rt_uint32_t step_num;

#endif

