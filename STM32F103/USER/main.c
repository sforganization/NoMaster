
#include "delay.h"
#include "sys.h"
#include "usart1.h"
#include "usart2.h"
#include "usart3.h"
#include "as608.h"

#include "timer.h"

#include "as608.h"
//#include "pwm.h"


#define _MAININC_
#include "SysComment.h"
#undef _MAININC_
#define usart2_baund			57600//����2�����ʣ�����ָ��ģ�鲨���ʸ��ģ�ע�⣺ָ��ģ��Ĭ��57600��



void SysTickTask(void);



/*******************************************************************************
* ����: 
* ����: ������һ����Ƭ���ϵ����\�ϵ�
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void LockGPIOInit(void)
{
	/* ����IOӲ����ʼ���ṹ����� */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��(����)KEY1���Ŷ�ӦIO�˿�ʱ�� */
	LOCK_ZERO_RCC_CLOCKCMD(LOCK_RCC_CLOCKGPIO, ENABLE);


	GPIO_InitStructure.GPIO_Pin =  LOCK_01_PIN | LOCK_02_PIN | LOCK_03_PIN |
                                   LOCK_04_PIN | LOCK_05_PIN | LOCK_06_PIN | 
                                   LOCK_07_PIN | LOCK_08_PIN | LOCK_09_PIN | 
                                   LOCK_10_PIN | LOCK_11_PIN | LOCK_12_PIN | 
                                   LOCK_13_PIN | LOCK_14_PIN | LOCK_15_PIN | 
                                   LOCK_16_PIN ;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//

	GPIO_Init(LOCK_GPIO, &GPIO_InitStructure);
}

/*******************************************************************************
* ����: 
* ����: �����io
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_ZeroGPIOInit(void)
{
	/* ����IOӲ����ʼ���ṹ����� */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��(����)KEY1���Ŷ�ӦIO�˿�ʱ�� */
	LOCK_ZERO_RCC_CLOCKCMD(LOCK_ZERO_RCC_CLOCKGPIO, ENABLE);
    
	GPIO_InitStructure.GPIO_Pin =   LOCK_ZERO_01_PIN | LOCK_ZERO_02_PIN | LOCK_ZERO_03_PIN |
                                    LOCK_ZERO_04_PIN | LOCK_ZERO_05_PIN | LOCK_ZERO_06_PIN | 
                                    LOCK_ZERO_07_PIN | LOCK_ZERO_08_PIN | LOCK_ZERO_09_PIN | 
                                    LOCK_ZERO_10_PIN | LOCK_ZERO_11_PIN | LOCK_ZERO_12_PIN | 
                                    LOCK_ZERO_13_PIN | LOCK_ZERO_14_PIN | LOCK_ZERO_15_PIN | 
                                    LOCK_ZERO_16_PIN ;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������

	GPIO_Init(LOCK_ZERO_GPIO, &GPIO_InitStructure);
}

/*******************************************************************************
* ����: 
* ����: �������io  һ����һ������������ת����
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_DirGPIOInit(void)
{
	/* ����IOӲ����ʼ���ṹ����� */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��(����)���Ŷ�ӦIO�˿�ʱ��                         + */
	MOTO_P_RCC_CLOCKCMD(MOTO_P_RCC_CLOCKGPIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin =   MOTO_01_PIN_P | MOTO_02_PIN_P | MOTO_03_PIN_P |
                                    MOTO_04_PIN_P | MOTO_05_PIN_P | MOTO_06_PIN_P | 
                                    MOTO_07_PIN_P | MOTO_08_PIN_P | MOTO_09_PIN_P | 
                                    MOTO_10_PIN_P | MOTO_11_PIN_P | MOTO_12_PIN_P | 
                                    MOTO_13_PIN_P | MOTO_14_PIN_P | MOTO_15_PIN_P | 
                                    MOTO_16_PIN_P ;
	;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//

	GPIO_Init(MOTO_P_GPIO, &GPIO_InitStructure);



    
	/* ʹ��(����)���Ŷ�ӦIO�˿�ʱ��                     -*/
	MOTO_N_RCC_CLOCKCMD(MOTO_N_RCC_CLOCKGPIO, ENABLE);
    
	GPIO_InitStructure.GPIO_Pin =   MOTO_01_PIN_N | MOTO_02_PIN_N | MOTO_03_PIN_N |
                                    MOTO_04_PIN_N | MOTO_05_PIN_N | MOTO_06_PIN_N | 
                                    MOTO_07_PIN_N | MOTO_08_PIN_N | MOTO_09_PIN_N | 
                                    MOTO_10_PIN_N | MOTO_11_PIN_N | MOTO_12_PIN_N | 
                                    MOTO_13_PIN_N | MOTO_14_PIN_N | MOTO_15_PIN_N | 
                                    MOTO_16_PIN_N ;

	;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//

	GPIO_Init(MOTO_N_GPIO, &GPIO_InitStructure);
}


int main(void)
{
	delay_init();									//��ʱ������ʼ��	  
	NVIC_Configuration();							//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�				 

	usart1_init(115200);
    Usart1_SendByte(0x88);
	usart3_init(115200); //��ʱ����
    Usart3_SendByte(0x66);
   
	usart2_init(usart2_baund);						//��ʼ������2,������ָ��ģ��ͨѶ
	PS_StaGPIO_Init();								//��ʼ��FR��״̬����  ʶ��ָ�ư�ѹ����ߵ�ƽ����Ϊ���ù�ϵ��ʼ�������ŵ�KEY_GPIO_Init����
	LockGPIOInit();
	Moto_ZeroGPIOInit();
	Moto_DirGPIOInit();
    
	//GENERAL_TIMx_PWM_Init();   //ע����tim�жϵ�����
	SysInit();

	GENERAL_TIMx_Configuration();
	while (1)
	{
//		MainTask();

		if (SysTask.nTick)
		{
			SysTask.nTick--;
			SysTickTask();

		}
	}
}


/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void SysTickTask(void)
{
	vu16 static 	u16SecTick = 0; 				//�����
	u8  i;

	if (u16SecTick++ >= 1000) //������
	{
		u16SecTick			= 0;
	}

	if (SysTask.nWaitTime)
	{
		SysTask.nWaitTime--;
	}
    
	if (SysTask.u16SaveTick)
    {   
		SysTask.u16SaveTick--;
        if(SysTask.u16SaveTick == 0)
            SysSaveData();
    }


    if(SysTask.u16SaveWaitTick)
        SysTask.u16SaveWaitTick--;

    
    for(i = 0; i < GLASS_MAX_CNT; i++)
    {
        if(SysTask.Moto_StateTime[i])   SysTask.Moto_StateTime[i]--;
        if(SysTask.Moto_RunTime[i])     SysTask.Moto_RunTime[i]--;
        if(SysTask.Moto_WaitTime[i])    SysTask.Moto_WaitTime[i]--;
        if(SysTask.Lock_StateTime[i])   SysTask.Lock_StateTime[i]--;
        if(SysTask.Lock_OffTime[i])     SysTask.Lock_OffTime[i]--;
    }

    

	//			if (state)
	//			{
	//				state				= 0;
	//				GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	//			}
	//			else 
	//			{
	//				state				= 1;
	//				GPIO_SetBits(GPIOC, GPIO_Pin_13);
	//			}
}


