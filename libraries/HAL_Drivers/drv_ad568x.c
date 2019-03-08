#include "drv_ad568x.h"
#include <board.h>
#include "drv_spi.h"
#include "rtdbg.h"

AD568X_Device_t	*AD568X_Device;

#define AD568X_LDAC		GET_PIN(D,8)

unsigned char currentPowerRegValue = 0; 

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Initializes the device.
 *
 * @param ad568x - Device version.
 *                 Example: AD5684  - 12-bit DAC(no internal vRef).
 *                          AD5686  - 16-bit DAC(no internal vRef).
 *                          AD5684R - 12-bit DAC(with internal vRef).
 *                          AD5685R - 14-bit DAC(with internal vRef).
 *                          AD5685R - 16-bit DAC(with internal vRef).
 *
 * @return status - Result of the initialization procedure.
 *					Example: 0x0 - SPI peripheral was not initialized.
 *				  			 0x1 - SPI peripheral is initialized.
*******************************************************************************/
#ifdef	RT_USING_AD5684
#define AD568X_BIT_NUM	12

#elif	RT_USING_AD5684R
#define AD568X_BIT_NUM	12

#elif RT_USING_AD5685R
#define AD568X_BIT_NUM	14

#elif RT_USING_AD5686 || RT_USING_AD5686R
#define AD568X_BIT_NUM	16

#endif


/***************************************************************************//**
 * @brief Writes a 24-bit data-word to the Input Register of the device.
 *
 * @param registerValue - Value of the register.
 *
 * @return none.
*******************************************************************************/
void AD568X_SetInputRegister(struct rt_spi_device *device,unsigned long registerValue)
{
    unsigned char registerWord[3] = {0, 0, 0};
    unsigned char* dataPointer    = (unsigned char*)&registerValue;

    registerWord[0] = dataPointer[2];
    registerWord[1] = dataPointer[1];
    registerWord[2] = dataPointer[0];
    
    rt_spi_send(AD568X_Device->spiDevice, registerWord, 3);
}

/***************************************************************************//**
 * @brief Puts the device in a specific power mode.
 *
 * @param channel - Channel option.
 *                  Example: AD568X_ADDR_DAC_A
 *                           AD568X_ADDR_DAC_B
 *                           AD568X_ADDR_DAC_C
 *                           AD568X_ADDR_DAC_D
 * @param pwrMode - Power mode of the device.
 *                  Example: AD568X_PD_NORMAL   - Normal operation
 *                           AD568X_PD_1K       - 1 kOhm to GND
 *                           AD568X_PD_100K     - 100 kOhm to GND
 *                           AD568X_PD_3STATE   - Three-state
 *                               
 *
 * @return none.
*******************************************************************************/
void AD568X_PowerMode(struct rt_spi_device *device,unsigned char channel, unsigned char pwrMode)
{    
    switch(channel)
    {
        case AD568X_ADDR_DAC_A:
            currentPowerRegValue &= ~AD568X_PWR_PDA(0x3);   // Clear power bits
            currentPowerRegValue |= AD568X_PWR_PDA(pwrMode);
            break;
        case AD568X_ADDR_DAC_B:
            currentPowerRegValue &= ~AD568X_PWR_PDB(0x3);
            currentPowerRegValue |= AD568X_PWR_PDB(pwrMode);
            break;
        case AD568X_ADDR_DAC_C:
            currentPowerRegValue &= ~AD568X_PWR_PDC(0x3);
            currentPowerRegValue |= AD568X_PWR_PDC(pwrMode);
            break;
        case AD568X_ADDR_DAC_D:
            currentPowerRegValue &= ~AD568X_PWR_PDD(0x3);
            currentPowerRegValue |= AD568X_PWR_PDD(pwrMode);
            break;
    }
    AD568X_SetInputRegister(device,AD568X_CMD(AD568X_CMD_POWERMODE) | 
                             currentPowerRegValue);
}

/***************************************************************************//**
 * @brief Resets the device(clears the outputs to either zero scale or 
          midscale).
 *
 * @param resetOutput - The output values of the device at power-up or reset.
 *                      Example: 
 *                    AD568X_RST_ZERO_SCALE - outputs are cleared to zero scale.
 *                    AD568X_RST_MIDSCALE   - outputs are cleared to midscale.
 *
 * @return none.
*******************************************************************************/
void AD568X_Reset(struct rt_spi_device *device,unsigned char resetOutput)
{
   
    AD568X_SetInputRegister(device,AD568X_CMD(AD568X_CMD_SOFT_RESET));
}



/***************************************************************************//**
 * @brief Select internal or external voltage reference.
 *
 * @param vRefMode - Voltage reference option.
 *                   Example: AD568X_INT_REF_ON  - Internal reference is used.
 *                            AD568X_INT_REF_OFF - External reference is used.
 *
 * @return none.
*******************************************************************************/
void AD568X_InternalVoltageReference(struct rt_spi_device *device,unsigned char vRefMode)
{
    AD568X_SetInputRegister(device,AD568X_CMD(AD568X_CMD_INT_REF_SETUP) | 
                            vRefMode);
}

/***************************************************************************//**
 * @brief Write data to the Input Register or to DAC Register of a channel.
 *
 * @param writeCommand - The write command option.
 *                       Example: 
*                        AD568X_CMD_WR_INPUT_N    - Write to Input Register n.
 *                       AD568X_CMD_WR_UPDT_DAC_N - Write to and update DAC n.
 * @param channel - Channel option.
 *                  Example: AD568X_ADDR_DAC_A
 *                           AD568X_ADDR_DAC_B
 *                           AD568X_ADDR_DAC_C
 *                           AD568X_ADDR_DAC_D
 * @param data -  Data value to write.
 *
 * @return none.
*******************************************************************************/
void AD568X_WriteFunction(struct rt_spi_device *device,
						  unsigned char writeCommand, 
                          unsigned char channel, 
                          unsigned short data)
{
    unsigned char shiftValue = 0;
    
    /* Different types of devices have different data bits positions. */
    shiftValue = 16 - AD568X_BIT_NUM;
    AD568X_SetInputRegister(device,AD568X_CMD(writeCommand) |
                            AD568X_ADDR(channel) | 
                            ((long)AD568X_DATA_BITS(data) << shiftValue));
}

/***************************************************************************//**
 * @brief Reads back the binary value written to one of the channels.
 *
 * @param dacChannelAddr - Channel address.
 *                         Example: AD568X_ADDR_DAC_A
 *                                  AD568X_ADDR_DAC_B
 *                                  AD568X_ADDR_DAC_C
 *                                  AD568X_ADDR_DAC_D 
 *
 * @return 12-bit value of the selected channel.
*******************************************************************************/
unsigned short AD568X_ReadBack(struct rt_spi_device *device,unsigned char dacChannelAddr)
{
    unsigned long channelValue = 0;
    unsigned char shiftValue   = 0;
    unsigned char rxBuffer[3]  = {0, 0, 0};
    
    /* Different types of devices have different data bits positions. */
    shiftValue = 16 - AD568X_BIT_NUM;
    AD568X_SetInputRegister(device,AD568X_CMD(AD568X_CMD_SET_READBACK) | 
                             AD568X_ADDR(dacChannelAddr));
    
    rt_spi_recv(AD568X_Device->spiDevice, rxBuffer, 3);
    
    channelValue = ((long)rxBuffer[0] << 8) | rxBuffer[1];
    channelValue >>= shiftValue;
    
    return channelValue;
}

/***************************************************************************//**
 * @brief Selects the output voltage of the selected channel.
 *
 * @param channel - Channel option.
 *                  Example: AD568X_ADDR_DAC_A
 *                           AD568X_ADDR_DAC_B
 *                           AD568X_ADDR_DAC_C
 *                           AD568X_ADDR_DAC_D
 * @param outputVoltage - Output voltage value.
 * @param vRef - Value of the voltage reference used. If GAIN pin is tied to Vdd
 *               vRef value is multiplied by 2 inside this function.
 *
 * @return The actual value of the output voltage.
*******************************************************************************/
float AD568X_SetVoltage(struct rt_spi_device *device,unsigned char channel, 
                        float outputVoltage, 
                        float vRef)
{
    unsigned short binaryValue   = 0;
    float          actualVoltage = 0;
    

    binaryValue = (unsigned short)(outputVoltage * (1ul << AD568X_BIT_NUM) / 
                                  vRef);
    AD568X_WriteFunction(device,AD568X_CMD_WR_UPDT_DAC_N, channel, binaryValue);
    actualVoltage = (float)(vRef * binaryValue) / (1ul << AD568X_BIT_NUM);
    
    return actualVoltage;
}
void AD568X_Init(struct rt_spi_device *device)
{
	// Enable internal voltage reference. 
	AD568X_InternalVoltageReference(device,AD568X_INT_REF_OFF);
	// Power-up all channels. 
	AD568X_PowerMode(device,AD568X_ADDR_DAC_A, AD568X_PD_NORMAL);
	AD568X_PowerMode(device,AD568X_ADDR_DAC_B, AD568X_PD_NORMAL);
	AD568X_PowerMode(device,AD568X_ADDR_DAC_C, AD568X_PD_NORMAL);
	AD568X_PowerMode(device,AD568X_ADDR_DAC_D, AD568X_PD_NORMAL);
	// Reset the device. 
	AD568X_Reset(device,AD568X_RST_ZERO_SCALE);
	// Set the channel A output voltage to 0.45 V and read back the raw value written to DAC A register.
	AD568X_SetVoltage(device,AD568X_ADDR_DAC_A, VOL, 3.303);
	//AD568X_ReadBack(AD568X_ADDR_DAC_A);
	// Set the channel B output voltage to 0.5 V. 
	AD568X_SetVoltage(device,AD568X_ADDR_DAC_B, VOL, 3.303);
	AD568X_SetVoltage(device,AD568X_ADDR_DAC_C, VOL, 3.303);
	AD568X_SetVoltage(device,AD568X_ADDR_DAC_D, VOL, 3.303);
}

static rt_err_t AD568X_init(rt_device_t dev)
{

    return RT_EOK;
}
static rt_err_t AD568X_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}
static rt_err_t AD568X_close(rt_device_t dev)
{
    return RT_EOK;
}
static rt_size_t AD568X_read(rt_device_t dev,
                                   rt_off_t pos,
                                   void* buffer,
                                   rt_size_t size)
{
    return size;
}
static rt_size_t AD568X_write(rt_device_t dev,
                                    rt_off_t pos,
                                    const void* buffer,
                                    rt_size_t size)
{
    return size;
}

static rt_err_t AD568X_control(rt_device_t dev, int cmd, void *args)
{
	AD568X_Device_t * AD568X_device;
	AD568xData_t  data;

	data = (AD568xData_t)args;
	
    RT_ASSERT(dev != RT_NULL);
	
	AD568X_device = (AD568X_Device_t *)dev;
	
	switch(cmd)
	{
		case Init:
			
			AD568X_Init(AD568X_device->spiDevice);
		
		break;
		
		case SetVoltage:
			
			AD568X_SetVoltage(AD568X_device->spiDevice,data->channel,data->outputVoltage,3.303);
			
		break;
		
		default:
			break;
	}
	
    return RT_EOK;
}


//模式配置
void rt_hw_spi_dac_cfg(void)
{
	struct rt_spi_configuration cfg;
	
	cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_1 | RT_SPI_MSB;
    cfg.max_hz = 8 * 1000 *1000;                           /* 8M */

    rt_spi_configure(AD568X_Device->spiDevice, &cfg);
}
//AD568X设备初始化
int rt_hw_spi_dac_init(void)
{
	rt_err_t result;
	
	//需要给指针申请地址，并进行清零，否则在进行spi参数配置时会出现错误
	AD568X_Device = (AD568X_Device_t *)rt_malloc(sizeof(AD568X_Device_t));
	if(AD568X_Device == NULL)
	{
		rt_kprintf("AD568X_Device malloc failed\n");
		return -RT_ENOMEM;
	}
	rt_memset(AD568X_Device,0,sizeof(AD568X_Device_t));
	
	//设置AD568X的LDAC引脚为低，则DAC的输出数据会实时更新。
	rt_pin_mode(AD568X_LDAC,PIN_MODE_OUTPUT);
	rt_pin_write(AD568X_LDAC,PIN_LOW);
	
	//使能GPIOB时钟
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	//将设备挂载到总线上，并初始化了片选引脚	PB12
	result = rt_hw_spi_device_attach(AD568X_SPI_BUS_NAME, AD568X_SPI_DEVICE_NAME, GPIOB, GPIO_PIN_12);
	if(result != RT_EOK)
	{
		rt_kprintf("spi device attach failed!\r\n");
	}
	
	
	AD568X_Device->spiDevice = (struct rt_spi_device *)rt_device_find(AD568X_SPI_DEVICE_NAME);
	
	if(AD568X_Device->spiDevice == RT_NULL)
	{
		rt_kprintf("spi device find failed!\r\n");
	}
	
	//进行模式配置
	rt_hw_spi_dac_cfg();
	
	/* register device */
    AD568X_Device->Device.type    = RT_Device_Class_Char;
    AD568X_Device->Device.init    = AD568X_init;
    AD568X_Device->Device.open    = AD568X_open;
    AD568X_Device->Device.close   = AD568X_close;
    AD568X_Device->Device.read    = AD568X_read;
    AD568X_Device->Device.write   = AD568X_write;
    AD568X_Device->Device.control = AD568X_control;

	 /* no private */
    AD568X_Device->Device.user_data = RT_NULL;
	
	
	rt_device_register(&AD568X_Device->Device, AD568X_DEVICE,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}

/* 导出到自动初始化，在初始化应用时启动 */
INIT_APP_EXPORT(rt_hw_spi_dac_init);
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(rt_hw_spi_dac_init, dac device init);

