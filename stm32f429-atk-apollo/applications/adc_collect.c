#include <rtthread.h>
#include <rtdevice.h>
#include "finsh.h"
#include "adc_collect.h"
#include "common.h"
#include "board.h"
//#include "drv_ad568x.h"
#include "string.h" 
#include "drv_dac_onchip.h"


rt_device_t dac_device;
/**********************ADC通道数**************************/
#define CHANNEL_NUM		7
//ADC相关定义
#define ADC_DEV_NAME 	"adc1"

#define ADC_DEV_CHA0	0
#define ADC_DEV_CHA1	1
#define ADC_DEV_CHA5	5
#define ADC_DEV_CHA8	8
#define ADC_DEV_CHA9	9
#define ADC_DEV_CHA14	14
#define ADC_DEV_CHA15	15


//ADC采集
#define ADC_COL_NUM		20
#define ADC_LOSE_NUM	5

//DAC初始电压值
#define DAC_VOL			0.0

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

#define REFER_VOLTAGE	330			/* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS	(1<<12)	 	/* 转换位数为12位 */



ALIGN(RT_ALIGN_SIZE)
static char adc_stack[512];
//线程控制块
static struct rt_thread adc_thread;
rt_adc_device_t adc_dev;			/* ADC 设备句柄 */  

rt_device_t dac_dev;				/* DAC 设备句柄 */ 

//连续采集控制标志
#define START_COL	1
#define STOP_COL	0
rt_uint8_t start_flag = 0;			//1：启动	0：停止


////dac设备初始化
void dac_init(void)
{
	double vol = DAC_VOL;
	dac_dev = rt_device_find(DAC_DEV_NAME);
	if(dac_dev == RT_NULL)
	{
		rt_kprintf("can't find %s device!\n", DAC_DEV_NAME);
	}
	
	rt_device_open(dac_dev,RT_DEVICE_FLAG_RDWR);
	
	rt_device_write(dac_dev,0,&vol,sizeof(double));

}

//adc设备初始化
void adc_init(void)
{
	//查找ADC设备
	adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
	if(adc_dev == RT_NULL)
	{
		rt_kprintf("can't find %s device!\n", ADC_DEV_NAME);
	}
	
	
	 /* 使能设备 */
//	for(rt_uint8_t i=0;i<CHANNEL_NUM;i++)
//	{
//		rt_adc_enable(adc_dev, chanel[i]);
//	}
	
	rt_adc_enable(adc_dev,ADC_DEV_CHA0);
	rt_adc_enable(adc_dev,ADC_DEV_CHA5);
}

//四个通道循环采集
void adc_collect(void)
{
	rt_uint32_t value;					/* ADC值 */
	rt_uint32_t vol;					/* ADC采集电压值 */
	for(rt_uint8_t i=0;i<CHANNEL_NUM;i++)
	{
		/* 读取采样值 */
		value = rt_adc_read(adc_dev, chanel[i]);
		/* 转换为对应电压值 */
		vol = value * REFER_VOLTAGE / CONVERT_BITS;
		rt_kprintf("VOL%02d:  %d.%02dV \r\n",chanel[i], vol / 100, vol % 100);
	}		
	rt_kprintf("*****************************\r\n");
}

#define SIGNAL_CHANNEL	0
static void adc_thread_entry(void *parameter)
{
	rt_uint32_t value = 0;					/* ADC值 */
	rt_uint32_t vol = 0;					/* ADC采集电压值 */
	
	//DAC初始化
	dac_init();
	//ADC初始化
	adc_init();
	

	while(1)
	{
		if(start_flag == START_COL)
		{
#if	SIGNAL_CHANNEL
			value = 0;
			for(rt_uint8_t i = 0;i<ADC_COL_NUM;i++)
			{
				if(i>=ADC_LOSE_NUM)
				value += rt_adc_read(adc_dev, ADC_DEV_CHA0);	
			}
			value /= ADC_COL_NUM - ADC_LOSE_NUM;
			/* 转换为对应电压值 */
			vol = value * REFER_VOLTAGE / CONVERT_BITS;
			rt_kprintf("VOL%02d:  %d.%02dV \r\n",ADC_DEV_CHA0, vol / 100, vol % 100);
			
			rt_thread_mdelay(10);
#endif
			value = 0;
			for(rt_uint8_t j = 0;j<ADC_COL_NUM;j++)
			{
				if(j>=ADC_LOSE_NUM)
				value += rt_adc_read(adc_dev, ADC_DEV_CHA5);	
			}
			value /= ADC_COL_NUM - ADC_LOSE_NUM;
			/* 转换为对应电压值 */
			vol = value * REFER_VOLTAGE / CONVERT_BITS;
#if SIGNAL_CHANNEL
			rt_kprintf("VOL%02d:  %d.%02dV \r\n",ADC_DEV_CHA5, vol / 100, vol % 100);
			rt_kprintf("*****************************\r\n");
#else
			rt_kprintf("VOL:  %d.%02dV \r\n", vol / 100, vol % 100);
#endif
		}		
		rt_thread_delay(1000);
	}
}

//初始化线程
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

//通过命令设置DAC输出电压及读取ADC 5通道采集到的电压值
#include "string.h"
void vol(int argc,char ** argv)
{
	double vol_set = 0;
	rt_uint8_t channel = 0;
	rt_uint32_t value = 0;					/* ADC值 */
	rt_uint32_t vol_read = 0;
	
	if(argc < 2)
	{
		rt_kprintf("vol set 1.5 or vol read !\n\r");
		return ;
	}
	if(!rt_strcmp(argv[1],"set"))
	{
		
		vol_set = atof(argv[2]);
		rt_device_write(dac_dev,0,&vol_set,sizeof(double));		
	}
	
	else if(!rt_strcmp(argv[1],"read"))
	{			
		for(rt_uint8_t j = 0;j<ADC_COL_NUM;j++)
		{
			if(j>=ADC_LOSE_NUM)
			value += rt_adc_read(adc_dev, ADC_DEV_CHA5);	
		} 
		value /= ADC_COL_NUM - ADC_LOSE_NUM;
		/* 转换为对应电压值 */
		vol_read = value * REFER_VOLTAGE / CONVERT_BITS;
		rt_kprintf("VOL: %d.%02dV \r\n", vol_read / 100, vol_read % 100);		
	}
	else if(!rt_strcmp(argv[1],"start"))
	{			
		start_flag = START_COL;
	}
	else if(!rt_strcmp(argv[1],"stop"))
	{			
		start_flag = STOP_COL;	
	}
	else
	{
		rt_kprintf("please input right parameter !\n\r");
	}
}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(thread_init_adc, adc collect);
MSH_CMD_EXPORT(vol, vol set or read!);


