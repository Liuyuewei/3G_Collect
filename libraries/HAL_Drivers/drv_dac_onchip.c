#include "drv_dac_onchip.h"

DAC_HandleTypeDef hdac;

static int stm32_dac_init(void)
{
	int result = RT_EOK;

	DAC_ChannelConfTypeDef sConfig = {0};
	
	 /**DAC Initialization 
	*/
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK)
	{
		Error_Handler();
	}
	/**DAC channel OUT1 config 
	*/
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1); //开启 DAC 通道 1
	
	return RT_EOK;
}	

//设置通道 1 输出电压
//vol:0~3300,代表 0~3.3V
static rt_size_t dac_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	double temp;
	temp = *(double *)buffer;
	
	temp=temp*4095/3.3;
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,temp);//12 位右对齐数据格式设置 DAC 值
	
	return size;
}

rt_err_t rt_hw_dac_init(void)
{
	rt_err_t result = RT_EOK;
	
	rt_device_t device;
	
	//申请的空间大小sizeof()函数的参数应该是一个数组，而不是指针类型，这个一定得注意
	device = rt_malloc(sizeof(struct rt_device));
	
	rt_memset(device,0,sizeof(struct rt_device));
	
	//dac初始化
	stm32_dac_init();
	
	device->type = RT_Device_Class_Miscellaneous;
	device->init = RT_NULL;
	device->open = RT_NULL;
	device->close = RT_NULL;
	device->read = RT_NULL;
	device->write = dac_write;
	device->control = RT_NULL;
	

	device->user_data   = RT_NULL;

	/* register a character device */
	result = rt_device_register(device, DAC_DEV_NAME, RT_DEVICE_FLAG_RDWR);
	
	return result;
}

MSH_CMD_EXPORT(rt_hw_dac_init,rt_hw_dac_init);
/* 在应用中初始化 */
INIT_BOARD_EXPORT(rt_hw_dac_init);


