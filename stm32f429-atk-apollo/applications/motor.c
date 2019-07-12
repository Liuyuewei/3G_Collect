#include <rtthread.h>
#include <rtdevice.h>
#include "motor.h"
#include "common.h"
#include "timer3.h"


/********************��ʱ��**********************/
// ���ARR��С�Ļ�����motor�߳��޷���ʱ����ʱ���ͷŵ��ź���
// ARR Խ�� ���Ƶ��Խ��
#define ARR		25				//�Զ���װ�ؼĴ������ڵ�ֵ	
#define PSC		(90000 / 2)		//ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ		90M / 9000 = 10K	//����һ��2���Ĺ�ϵ

//ϸ����
#define MICRO_STEP	400

#define	STATE_0		((75 * MICRO_STEP) / 360)
#define	STATE_1		((53 * MICRO_STEP) / 360)
#define	STATE_2		((53 * MICRO_STEP) / 360)
#define	STATE_3		((75 * MICRO_STEP) / 360)
#define	STATE_4		((5 * MICRO_STEP) / 360)


//����˿��28�������
#define MOTOR_L_PER	0.635

#define MOVE_1_DZ	((8 / MOTOR_L_PER) * MICRO_STEP)		//�ƶ�9.9mm
#define MOVE_2_DZ	((13 / MOTOR_L_PER) * MICRO_STEP)		//�ƶ�13.5mm

//�����Ҫ��������
rt_uint32_t rec_step_num = 0;
//�����ǰλ��	0�����	1������	2��ж��
rt_uint8_t motor_state	= 0;

#define STATE_1_DZ	1		//			   ��:���� ��:����	�����λ��
#define STATE_2_DZ	2		//���������9.9mm, ��:���� ��:����	������λ
#define STATE_3_DZ	3		//�����ǰ��13.5mm,��:���� ��:ж��	��ж������λ

//��ȡ��λ����״̬	1���ڵ�	0�����ڵ�
#define	LIMIT_STATE				rt_pin_read(MOTOR_LIMIT)

//����Ϊ�͵�ƽ
#define TEST_STATE				rt_pin_read(TEST)
#define UNLOAD_CUP_STATE		rt_pin_read(UNLOAD_CUP)

//��λ��־λ
rt_uint8_t limit_flag = 0;
//����״̬��־λ
rt_uint8_t unload_cup_flag = 0;
rt_uint8_t test_flag = 0;
//����ϵ�ʱ������λ��־
rt_uint8_t zero_flag = 0;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t motor_stack[ 512 ];

/* �̵߳�TCB���ƿ� */
static struct rt_thread motor_thread;
/* defined the MOTOR_DIR pin: PD5 */
#define MOTOR_DIR    GET_PIN(D, 5)
/* defined the MOTOR_LIMIT pin: PD5 */
#define MOTOR_LIMIT    GET_PIN(H, 6)
/* defined the ж������ pin: PE9 */
#define UNLOAD_CUP    GET_PIN(E, 9)
/* defined the ���԰��� pin: PE7 */
#define TEST    GET_PIN(E, 7)

//ж�������ж�
void unload_cup_cb(void *args)
{
	if((zero_flag == 1) && (test_flag != 1))
	{
		//��ǰ�����λ��
		if(motor_state == STATE_1_DZ)
		{
			rec_step_num = MOVE_2_DZ;
			unload_cup_flag = 1;
			step_num = 0;
			//��ǰ��
			rt_pin_write(MOTOR_DIR,PIN_LOW);
			TIM3_Start();
		}
		//�ڲ���λ
		else if(motor_state == STATE_2_DZ)
		{
			rec_step_num = MOVE_1_DZ + MOVE_2_DZ;
			unload_cup_flag = 1;
			step_num = 0;
			//��ǰ��
			rt_pin_write(MOTOR_DIR,PIN_LOW);
			TIM3_Start();
		}
	}
}

//���԰����ж�
void test_cb(void *args)
{
	if((zero_flag == 1) && (unload_cup_flag != 1))
	{
		//��ǰ�����λ��
		if(motor_state == STATE_1_DZ)
		{
			rec_step_num = MOVE_1_DZ;
			test_flag = 1;
			step_num = 0;
			//������
			rt_pin_write(MOTOR_DIR,PIN_HIGH);
			TIM3_Start();
		}
		//��ж��λ
		else if(motor_state == STATE_3_DZ)
		{
			rec_step_num = MOVE_1_DZ + MOVE_2_DZ;
			test_flag = 1;
			step_num = 0;
			//������
			rt_pin_write(MOTOR_DIR,PIN_HIGH);
			TIM3_Start();
		}
	}
}	



//������򡢼���λ��������ģʽ����
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

//��־28���ݵ��
void motor_28_tuoluo_test(void)
{
	//״̬��
	rt_uint8_t state = 0;
    rt_hw_motor_init();
	rt_thread_delay(RT_TICK_PER_SECOND * 4);
	//һ�ϵ���ʱ��ת
	rt_pin_write(MOTOR_DIR,PIN_HIGH);
	TIM3_Init(ARR - 1,PSC - 1);	//��ʼ����ʱ���������ж�����
	
	while(1)
	{
		//һ�ϵ���ʱ����ת����������λ�������
		if(LIMIT_STATE == 0)
		{
			TIM3_Stop();
			step_num = 0;
			//ÿ��״̬��ʱ1s
			rt_thread_delay(2000);
			TIM3_Start();
			break;
		}
		rt_thread_mdelay(1);	
	}
	while(1)
	{
		//ж��״̬�������� ��ж��	��ʱ��ת5��
		if(step_num >= STATE_4)
		{
			TIM3_Stop();
			step_num = 0;
			//˳ʱ��ת
			rt_pin_write(MOTOR_DIR,PIN_LOW);
			//ÿ��״̬��ʱ1s
			rt_thread_delay(2000);
			TIM3_Start();
			state = 0;
			//�ҵ������˳�
			break;
		}
		rt_thread_mdelay(1);
	}

	//����֮�����Զ������˶�
    while (1)
    {
		switch(state)
		{
			
			case 0:
				//�������ϱ�״̬�������� �·���	˳ʱ��ת75��
				if(step_num >= 83)
				{
					TIM3_Stop();
					step_num = 0;
					//˳ʱ��ת
					rt_pin_write(MOTOR_DIR,PIN_LOW);
					//ÿ��״̬��ʱ1s
					rt_thread_delay(2000);
					TIM3_Start();
					state = 1;
				}
			
			
				
			break;	
			
			case 1:
				//����״̬���Ϸ��� �·���	˳ʱ��ת53��
				if(step_num >= 59)
				{
					TIM3_Stop();
					step_num = 0;
					//˳ʱ��ת
					rt_pin_write(MOTOR_DIR,PIN_HIGH);
					//ÿ��״̬��ʱ1s
					rt_thread_delay(2000);
					TIM3_Start();
					state = 2;
				}
				
			break;	
			
			case 2:
				//����״̬�������� �·���	��ʱ��ת53��
				if(step_num >= 59)
				{
					TIM3_Stop();
					step_num = 0;
					//˳ʱ��ת
					rt_pin_write(MOTOR_DIR,PIN_HIGH);
					//ÿ��״̬��ʱ1s
					rt_thread_delay(2000);
					TIM3_Start();
					state = 3;
				}
				
			break;
				
			case 3:
				//ж��״̬�������� ��ж��	��ʱ��ת75��
				if(step_num >= 83)
				{
					TIM3_Stop();
					step_num = 0;
					//˳ʱ��ת
					rt_pin_write(MOTOR_DIR,PIN_LOW);
					//ÿ��״̬��ʱ1s
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
//��־20���е��
void motor_20_beituo_test(void)
{
	rt_hw_motor_init();
	rt_thread_delay(RT_TICK_PER_SECOND * 4);
	//һ�ϵ���ʱ��ת
	rt_pin_write(MOTOR_DIR,PIN_HIGH);
	TIM3_Init(ARR - 1,PSC - 1);	//��ʼ����ʱ���������ж�����
	
	while(1)
	{
		rt_thread_mdelay(1000);
	}
}
//����28��˿�˵��
void motor_28_shigan_test(void)
{
	rt_hw_motor_init();
	rt_thread_delay(RT_TICK_PER_SECOND * 4);
	//һ�ϵ���ʱ��ת
	rt_pin_write(MOTOR_DIR,PIN_HIGH);
	TIM3_Init(ARR - 1,PSC - 1);	//��ʼ����ʱ���������ж�����
	
	//�����͵�ƽ,˵��ѹס��λ��
	if(LIMIT_STATE)
	{
		//��ǰ��
		rt_pin_write(MOTOR_DIR,PIN_LOW);
	}
	else
	{
		//������
		rt_pin_write(MOTOR_DIR,PIN_HIGH);
	}
	limit_flag = LIMIT_STATE;
	//�ҵ����λ����ֹͣ���˳�ѭ��
	while(1)
	{
		if(limit_flag != LIMIT_STATE)
		{
			TIM3_Stop();
			motor_state = STATE_1_DZ;	//������λ
			step_num = 0;
			zero_flag = 1;
			break;
		}	
		
		rt_thread_mdelay(10);
	}
	while(1)
	{	
		//���԰���
		if(test_flag  && (step_num >= rec_step_num))
		{
			TIM3_Stop();
			test_flag = 0;
			step_num = 0;	
			motor_state = STATE_2_DZ;	//�������λ
			limit_flag = LIMIT_STATE;
		}
		
		//ж������
		if(unload_cup_flag && (step_num >= rec_step_num))
		{
			TIM3_Stop();
			unload_cup_flag = 0;
			step_num = 0;	
			motor_state = STATE_3_DZ;	//����ж��λ
			limit_flag = LIMIT_STATE;
			rt_thread_mdelay(1000);		//��ʱ500ms
	
		}
		if(motor_state == STATE_3_DZ)
		{
			//������
			rt_pin_write(MOTOR_DIR,PIN_HIGH);
			TIM3_Start();
			if(limit_flag != LIMIT_STATE)
			{
				TIM3_Stop();
				step_num = 0;
				motor_state = STATE_1_DZ;	//������λ
				zero_flag = 1;
			}	
		}
		rt_thread_mdelay(10);
	}
	
	
	
	
	
	
}
//�߳���ں���
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