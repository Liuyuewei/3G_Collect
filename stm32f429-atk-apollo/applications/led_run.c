#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "finsh.h"
#include "common.h"
#include "led_run.h"

/* defined the LED0 pin: PB1 */
#define LED1_PIN    GET_PIN(B, 0)
#define LED0_PIN    GET_PIN(B, 1)


ALIGN(RT_ALIGN_SIZE)
static char led_stack[512];

//线程控制块
static struct rt_thread led_thread;

//硬件初始化
void rt_hw_led_init()
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

}

static void led_thread_entry(void *parameter)
{
	int count = 1;
	rt_hw_led_init();
	
    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
		rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
		rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
    }
}


int thread_init_led(void)
{
	rt_err_t result;

    /* init adc thread */
    result = rt_thread_init(&led_thread,
                            "led",
                            led_thread_entry,
                            RT_NULL,
                            &led_stack[0],
                            sizeof(led_stack),
                            RT_THREAD_PRIORITY_LED,
                            10);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led_thread);
    }
    return 0;
}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(thread_init_led, led collect);



