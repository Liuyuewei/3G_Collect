#include <rtthread.h>
#include <rtdevice.h>
#include "motor.h"
#include "common.h"
#include "timer3.h"


/********************定时器**********************/
// 如果ARR再小的话，则motor线程无法及时处理定时器释放的信号量
// ARR 越大 电机频率越低
#define ARR		25				//自动重装载寄存器周期的值	
#define PSC		(90000 / 2)		//时钟频率除数的预分频值		90M / 9000 = 10K	//差了一个2倍的关系

//细分数
#define MICRO_STEP	400

#define	STATE_0		((75 * MICRO_STEP) / 360)
#define	STATE_1		((53 * MICRO_STEP) / 360)
#define	STATE_2		((53 * MICRO_STEP) / 360)
#define	STATE_3		((75 * MICRO_STEP) / 360)
#define	STATE_4		((5 * MICRO_STEP) / 360)


//鼎智丝杆28电机导程
#define MOTOR_L_PER	0.635

#define MOVE_1_DZ	((8 / MOTOR_L_PER) * MICRO_STEP)		//移动9.9mm
#define MOVE_2_DZ	((13 / MOTOR_L_PER) * MICRO_STEP)		//移动13.5mm

//电机需要的脉冲数
rt_uint32_t rec_step_num = 0;
//电机当前位置	0：零点	1：测试	2：卸杯
rt_uint8_t motor_state	= 0;

#define STATE_1_DZ	1		//			   上:锁轴 下:放松	即零点位置
#define STATE_2_DZ	2		//零点往后退9.9mm, 上:放松 下:放松	即测试位
#define STATE_3_DZ	3		//零点往前推13.5mm,上:锁轴 下:卸杯	即卸杯锁轴位

//读取限位开关状态	1：遮挡	0：不遮挡
#define	LIMIT_STATE				rt_pin_read(MOTOR_LIMIT)

//按下为低电平
#define TEST_STATE				rt_pin_read(TEST)
#define UNLOAD_CUP_STATE		rt_pin_read(UNLOAD_CUP)

//限位标志位
rt_uint8_t limit_flag = 0;
//按键状态标志位
rt_uint8_t unload_cup_flag = 0;
rt_uint8_t test_flag = 0;
//电机上电时到达零位标志
rt_uint8_t zero_flag = 0;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t motor_stack[ 512 ];

/* 线程的TCB控制块 */
static struct rt_thread motor_thread;
/* defined the MOTOR_DIR pin: PD5 */
#define MOTOR_DIR    GET_PIN(D, 5)
/* defined the MOTOR_LIMIT pin: PD5 */
#define MOTOR_LIMIT    GET_PIN(H, 6)
/* defined the 卸杯按键 pin: PE9 */
#define UNLOAD_CUP    GET_PIN(E, 9)
/* defined the 测试按键 pin: PE7 */
#define TEST    GET_PIN(E, 7)

//卸杯按键中断
void unload_cup_cb(void *args)
{
	if((zero_flag == 1) && (test_flag != 1))
	{
		//当前在零点位置
		if(motor_state == STATE_1_DZ)
		{
			rec_step_num = MOVE_2_DZ;
			unload_cup_flag = 1;
			step_num = 0;
			//往前推
			rt_pin_write(MOTOR_DIR,PIN_LOW);
			TIM3_Start();
		}
		//在测试位
		else if(motor_state == STATE_2_DZ)
		{
			rec_step_num = MOVE_1_DZ + MOVE_2_DZ;
			unload_cup_flag = 1;
			step_num = 0;
			//往前推
			rt_pin_write(MOTOR_DIR,PIN_LOW);
			TIM3_Start();
		}
	}
}

//测试按键中断
void test_cb(void *args)
{
	if((zero_flag == 1) && (unload_cup_flag != 1))
	{
		//当前在零点位置
		if(motor_state == STATE_1_DZ)
		{
			rec_step_num = MOVE_1_DZ;
			test_flag = 1;
			step_num = 0;
			//往后拉
			rt_pin_write(MOTOR_DIR,PIN_HIGH);
			TIM3_Start();
		}
		//在卸杯位
		else if(motor_state == STATE_3_DZ)
		{
			rec_step_num = MOVE_1_DZ + MOVE_2_DZ;
			test_flag = 1;
			step_num = 0;
			//往后拉
			rt_pin_write(MOTOR_DIR,PIN_HIGH);
			TIM3_Start();
		}
	}
}	



//电机方向、及限位控制引脚模式配置
void rt_hw_motor_init(void)
{
	/* set MOTOR_STEP pin mode to output */
    rt_pin_mode(MOTOR_STEP, PIN_MODE_OUTPUT);
	/* set MOTOR_DIR pin mode to output */
    rt_pin_mode(MOTOR_DIR, PIN_MODE_OUTPUT);
	/* set MOTOR_LIMIT pin mode to input */
    rt_pin_mode(MOTOR_LIMIT, PIN_MODE_INPUT_PULLDOWN);
	
	/* set UNLOAD_CUP pin mode to input */
    rt_pin_mode(UNLOAD_CUP, PIN_MODE_INPUT_PULLDOWN);
	rt_pin_attach_irq(UNLOAD_CUP,PIN_IRQ_MODE_RISING,unload_cup_cb,RT_NULL);
	rt_pin_irq_enable(UNLOAD_CUP,PIN_IRQ_ENABLE);
	
	/* set TEST pin mode to input */
    rt_pin_mode(TEST, PIN_MODE_INPUT_PULLDOWN);
	rt_pin_attach_irq(TEST,PIN_IRQ_MODE_RISING,test_cb,RT_NULL);
	rt_pin_irq_enable(TEST,PIN_IRQ_ENABLE);
}

//呜志28陀螺电机
void motor_28_tuoluo_test(void)
{
	//状态机
	rt_uint8_t state = 0;
    rt_hw_motor_init();
	rt_thread_delay(RT_TICK_PER_SECOND * 4);
	//一上电逆时针转
	rt_pin_write(MOTOR_DIR,PIN_HIGH);
	TIM3_Init(ARR - 1,PSC - 1);	//初始化定时器并设置中断周期
	
	while(1)
	{
		//一上电逆时针旋转，当到达限位处即零点
		if(LIMIT_STATE == 0)
		{
			TIM3_Stop();
			step_num = 0;
			//每个状态延时1s
			rt_thread_delay(2000);
			TIM3_Start();
			break;
		}
		rt_thread_mdelay(1);	
	}
	while(1)
	{
		//卸杯状态：上锁轴 下卸杯	逆时针转5度
		if(step_num >= STATE_4)
		{
			TIM3_Stop();
			step_num = 0;
			//顺时针转
			rt_pin_write(MOTOR_DIR,PIN_LOW);
			//每个状态延时1s
			rt_thread_delay(2000);
			TIM3_Start();
			state = 0;
			//找到零点后退出
			break;
		}
		rt_thread_mdelay(1);
	}

	//启动之后电机自动往反运动
    while (1)
    {
		switch(state)
		{
			
			case 0:
				//开机、上杯状态：上锁轴 下放松	顺时针转75度
				if(step_num >= 83)
				{
					TIM3_Stop();
					step_num = 0;
					//顺时针转
					rt_pin_write(MOTOR_DIR,PIN_LOW);
					//每个状态延时1s
					rt_thread_delay(2000);
					TIM3_Start();
					state = 1;
				}
			
			
				
			break;	
			
			case 1:
				//测试状态：上放松 下放松	顺时针转53度
				if(step_num >= 59)
				{
					TIM3_Stop();
					step_num = 0;
					//顺时针转
					rt_pin_write(MOTOR_DIR,PIN_HIGH);
					//每个状态延时1s
					rt_thread_delay(2000);
					TIM3_Start();
					state = 2;
				}
				
			break;	
			
			case 2:
				//测试状态：上锁轴 下放松	逆时针转53度
				if(step_num >= 59)
				{
					TIM3_Stop();
					step_num = 0;
					//顺时针转
					rt_pin_write(MOTOR_DIR,PIN_HIGH);
					//每个状态延时1s
					rt_thread_delay(2000);
					TIM3_Start();
					state = 3;
				}
				
			break;
				
			case 3:
				//卸杯状态：上锁轴 下卸杯	逆时针转75度
				if(step_num >= 83)
				{
					TIM3_Stop();
					step_num = 0;
					//顺时针转
					rt_pin_write(MOTOR_DIR,PIN_LOW);
					//每个状态延时1s
					rt_thread_delay(10000);
					TIM3_Start();
					state = 0;
				}
				
			break;	
			
			default:
			break;
		}
		
		rt_thread_mdelay(1);	//
    }
}
//呜志20杯托电机
void motor_20_beituo_test(void)
{
	rt_hw_motor_init();
	rt_thread_delay(RT_TICK_PER_SECOND * 4);
	//一上电逆时针转
	rt_pin_write(MOTOR_DIR,PIN_HIGH);
	TIM3_Init(ARR - 1,PSC - 1);	//初始化定时器并设置中断周期
	
	while(1)
	{
		rt_thread_mdelay(1000);
	}
}
//鼎智28带丝杆电机
void motor_28_shigan_test(void)
{
	rt_hw_motor_init();
	rt_thread_delay(RT_TICK_PER_SECOND * 4);
	//一上电逆时针转
	rt_pin_write(MOTOR_DIR,PIN_HIGH);
	TIM3_Init(ARR - 1,PSC - 1);	//初始化定时器并设置中断周期
	
	//读到低电平,说明压住限位了
	if(LIMIT_STATE)
	{
		//往前推
		rt_pin_write(MOTOR_DIR,PIN_LOW);
	}
	else
	{
		//往后拉
		rt_pin_write(MOTOR_DIR,PIN_HIGH);
	}
	limit_flag = LIMIT_STATE;
	//找到零点位后电机停止并退出循环
	while(1)
	{
		if(limit_flag != LIMIT_STATE)
		{
			TIM3_Stop();
			motor_state = STATE_1_DZ;	//到达零位
			step_num = 0;
			zero_flag = 1;
			break;
		}	
		
		rt_thread_mdelay(10);
	}
	while(1)
	{	
		//测试按下
		if(test_flag  && (step_num >= rec_step_num))
		{
			TIM3_Stop();
			test_flag = 0;
			step_num = 0;	
			motor_state = STATE_2_DZ;	//到达测试位
			limit_flag = LIMIT_STATE;
		}
		
		//卸杯按下
		if(unload_cup_flag && (step_num >= rec_step_num))
		{
			TIM3_Stop();
			unload_cup_flag = 0;
			step_num = 0;	
			motor_state = STATE_3_DZ;	//到达卸杯位
			limit_flag = LIMIT_STATE;
			rt_thread_mdelay(1000);		//延时500ms
	
		}
		if(motor_state == STATE_3_DZ)
		{
			//往后拉
			rt_pin_write(MOTOR_DIR,PIN_HIGH);
			TIM3_Start();
			if(limit_flag != LIMIT_STATE)
			{
				TIM3_Stop();
				step_num = 0;
				motor_state = STATE_1_DZ;	//到达零位
				zero_flag = 1;
			}	
		}
		rt_thread_mdelay(10);
	}
	
	
	
	
	
	
}
//线程入口函数
static void motor_thread_entry(void *parameter)
{
	while(1)
	{
//		motor_28_tuoluo_test();
		
		motor_20_beituo_test();
		
//		motor_28_shigan_test();
		
		
	}
}

int thread_init_motor(void)
{
    rt_err_t result;

    /* init led thread */
    result = rt_thread_init(&motor_thread,
                            "motor",
                            motor_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&motor_stack[0],
                            sizeof(motor_stack),
                            RT_THREAD_PRIORITY_MOTOR,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&motor_thread);
    }
    return 0;
}