/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "led_run.h"
#include "adc_collect.h"
#include "drv_dac_onchip.h"
#include "motor.h"

int main(void)
{
	thread_init_led();
//	thread_init_adc();
	thread_init_motor();
	return RT_EOK;
}
