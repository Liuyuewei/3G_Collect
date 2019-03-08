#include <rtthread.h>
#include <rtdevice.h>
#include "finsh.h"
#include "adc_collect.h"
#include "common.h"
#include "board.h"
//#include "drv_ad568x.h"
#include "string.h" 

rt_device_t dac_device;
/**********************ADCͨ����**************************/
#define CHANNEL_NUM		7
//ADC��ض���
#define ADC_DEV_NAME 	"adc1"

#define ADC_DEV_CHA0	0
#define ADC_DEV_CHA1	1
#define ADC_DEV_CHA5	5
#define ADC_DEV_CHA8	8
#define ADC_DEV_CHA9	9
#define ADC_DEV_CHA14	14
#define ADC_DEV_CHA15	15

rt_uint8_t chanel[CHANNEL_NUM]=
{
	ADC_DEV_CHA0,
	ADC_DEV_CHA1,
	ADC_DEV_CHA5,
	ADC_DEV_CHA8,
	ADC_DEV_CHA9,
	ADC_DEV_CHA14,
	ADC_DEV_CHA15
};

#define REFER_VOLTAGE	330			/* �ο���ѹ 3.3V,���ݾ��ȳ���100����2λС��*/
#define CONVERT_BITS	(1<<12)	 	/* ת��λ��Ϊ12λ */


ALIGN(RT_ALIGN_SIZE)
static char adc_stack[512];
//�߳̿��ƿ�
static struct rt_thread adc_thread;
rt_adc_device_t adc_dev;			/* ADC �豸��� */  

void adc_init(void)
{
	//����ADC�豸
	adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
	if(adc_dev == RT_NULL)
	{
		rt_kprintf("adc run failed! can't find %s device!\n", ADC_DEV_NAME);
	}
	
	
	 /* ʹ���豸 */
//	for(rt_uint8_t i=0;i<CHANNEL_NUM;i++)
//	{
//		rt_adc_enable(adc_dev, chanel[i]);
//	}
	
	rt_adc_enable(adc_dev,ADC_DEV_CHA0);
	rt_adc_enable(adc_dev,ADC_DEV_CHA5);
}

//�ĸ�ͨ��ѭ���ɼ�
void adc_collect(void)
{
	rt_uint32_t value;					/* ADCֵ */
	rt_uint32_t vol;					/* ADC�ɼ���ѹֵ */
	for(rt_uint8_t i=0;i<CHANNEL_NUM;i++)
	{
		/* ��ȡ����ֵ */
		value = rt_adc_read(adc_dev, chanel[i]);
		/* ת��Ϊ��Ӧ��ѹֵ */
		vol = value * REFER_VOLTAGE / CONVERT_BITS;
		rt_kprintf("VOL%02d:  %d.%02dV \r\n",chanel[i], vol / 100, vol % 100);
	}		
	rt_kprintf("*****************************\r\n");
}

static void adc_thread_entry(void *parameter)
{
	rt_uint32_t value;					/* ADCֵ */
	rt_uint32_t vol;					/* ADC�ɼ���ѹֵ */
	//��ʼ��Ӳ���ӿ�
//	rt_hw_gpio_init();
	//ADC��ʼ��
	adc_init();
	

	while(1)
	{
		value = rt_adc_read(adc_dev, ADC_DEV_CHA0);
		/* ת��Ϊ��Ӧ��ѹֵ */
		vol = value * REFER_VOLTAGE / CONVERT_BITS;
		rt_kprintf("VOL%02d:  %d.%02dV \r\n",ADC_DEV_CHA0, vol / 100, vol % 100);
		
		rt_thread_mdelay(500);
		
		value = rt_adc_read(adc_dev, ADC_DEV_CHA5);
		/* ת��Ϊ��Ӧ��ѹֵ */
		vol = value * REFER_VOLTAGE / CONVERT_BITS;
		rt_kprintf("VOL%02d:  %d.%02dV \r\n",ADC_DEV_CHA5, vol / 100, vol % 100);
		rt_kprintf("*****************************\r\n");
		rt_thread_delay(500);
	}
}


int thread_init_adc(void)
{
	rt_err_t result;

    /* init adc thread */
    result = rt_thread_init(&adc_thread,
                            "adc",
                            adc_thread_entry,
                            RT_NULL,
                            &adc_stack[0],
                            sizeof(adc_stack),
                            RT_THREAD_PRIORITY_ADC,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&adc_thread);
    }
    return 0;
}

/* ������ msh �����б��� */
MSH_CMD_EXPORT(thread_init_adc, adc collect);



