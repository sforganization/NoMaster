
#include "usart1.h" 
#include "usart3.h" 	 
#include "string.h"	 
#include "delay.h"

#include "SysComment.h"
#include "stm32f10x_it.h"
#include "stm32f10x_iwdg.h"
#include "san_flash.h"

#include "as608.h"
#include "bmp.h"


#define     SLAVE_WAIT_TIME         3000    //3s �ӻ�Ӧ��ʱ��

typedef enum 
{		
	ACK_SUCCESS = 0, //�ɹ� 
	ACK_FAILED,      //ʧ��
	ACK_AGAIN,       //�ٴ�����
	ACK_ERROR,       //�������
	ACK_DEF = 0xFF,
}Finger_Ack;  



//�ڲ�����
static vu16 	mDelay;
u8    g_GlassSendAddr   = 0;   //Ҫ���͵��ֱ��ַ
u8    g_FingerAck       = 0;   //Ӧ����������



u32 g_au32DataSaveAddr[6] = { 0x0807D000, 
                            0x0807D800,
                            0x0807E000,
                            0x0807E800, 
                            0x0807F000, 
                            0x0807F800};

vu32            g_au16MoteTime[][3] =
{
    //{
    //  MOTO_TIME_OFF, 0, 0
    //},
    {
        MOTO_TIME_TPD, 43200000, 43200000
    },
    {
        MOTO_TIME_650, 120000, 942000
    },
    {
        MOTO_TIME_750, 120000, 800000
    },
    {
        MOTO_TIME_850, 120000, 693000
    },
    {
        MOTO_TIME_1000, 120000, 570000
    },
    {
        MOTO_TIME_1950, 120000, 234000
    },
};


//�ڲ�����
void SysTickConfig(void);


/*******************************************************************************
* ����: SysTick_Handler
* ����: ϵͳʱ�ӽ���1MS
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void SysTick_Handler(void)
{
	static u16		Tick_1S = 0;

	mSysTick++;
	mSysSec++;
	mTimeRFRX++;

	if (mDelay)
		mDelay--;

	if (++Tick_1S >= 1000)
	{
		Tick_1S 			= 0;

		if (mSysIWDGDog)
		{
			IWDG_ReloadCounter();					/*ιSTM32����Ӳ����*/

			if ((++mSysSoftDog) > 5) /*��system  DOG 2S over*/
			{
				mSysSoftDog 		= 0;
				NVIC_SystemReset();
			}
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
void DelayUs(uint16_t nCount)
{
	u32 			del = nCount * 5;

	//48M 0.32uS
	//24M 0.68uS
	//16M 1.02us
	while (del--)
		;
}


/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void DelayMs(uint16_t nCount)
{
	unsigned int	ti;

	for (; nCount > 0; nCount--)
	{
		for (ti = 0; ti < 4260; ti++)
			; //16M/980-24M/1420 -48M/2840
	}
}


/*******************************************************************************
* ����: Strcpy()
* ����: 
* �β�:		
* ����: ��
* ˵��: 
******************************************************************************/
void Strcpy(u8 * str1, u8 * str2, u8 len)
{
	for (; len > 0; len--)
	{
		*str1++ 			= *str2++;
	}
}


/*******************************************************************************
* ����: Strcmp()
* ����: 
* �β�:		
* ����: ��
* ˵��: 
******************************************************************************/
bool Strcmp(u8 * str1, u8 * str2, u8 len)
{
	for (; len > 0; len--)
	{
		if (*str1++ != *str2++)
			return FALSE;
	}

	return TRUE;
}


/*******************************************************************************
* ����: Sys_DelayMS()
* ����: ϵͳ�ӳٺ���
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Sys_DelayMS(uint16_t nms)
{
	mDelay				= nms + 1;

	while (mDelay != 0x0)
		;
}


/*******************************************************************************
* ����: Sys_LayerInit
* ����: ϵͳ��ʼ��
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Sys_LayerInit(void)
{
	SysTickConfig();

	mSysSec 			= 0;
	mSysTick			= 0;
	SysTask.mUpdate 	= TRUE;
}


/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Sys_IWDGConfig(u16 time)
{
	/* д��0x5555,�����������Ĵ���д�빦�� */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* ����ʱ�ӷ�Ƶ,40K/64=0.625K()*/
	IWDG_SetPrescaler(IWDG_Prescaler_64);

	/* ι��ʱ�� TIME*1.6MS .ע�ⲻ�ܴ���0xfff*/
	IWDG_SetReload(time);

	/* ι��*/
	IWDG_ReloadCounter();

	/* ʹ�ܹ���*/
	IWDG_Enable();
}


/*******************************************************************************
* ����: Sys_IWDGReloadCounter
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Sys_IWDGReloadCounter(void)
{
	mSysSoftDog 		= 0;						//ι��
}


/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void SysTickConfig(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1)
			;
	}

	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x0);

#if (								SYSINFOR_PRINTF == 1)
	printf("SysTickConfig:Tick=%d/Second\r\n", 1000);
#endif
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void LedInit(void) //

{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(LED_RCC_CLOCKGPIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(LED_GPIO, LED_PIN);
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void FingerTouchTask(void)
{
	SearchResult seach;
	u8 ensure;
	static u8 u8MatchCnt = 0;						//ƥ��ʧ�ܴ�����Ĭ��ƥ��MATCH_FINGER_CNT�� 

	if ((SysTask.bEnFingerTouch == TRUE))
	{
		switch (SysTask.TouchState)
		{
			case TOUCH_MATCH_INIT:
				if (PS_Sta) //��ָ�ư���
				{
					SysTask.nWaitTime	= 100;		//һ��ʱ���ټ��
					SysTask.TouchState	= TOUCH_MATCH_AGAIN;
					u8MatchCnt			= 0;
				}
				break;

			case TOUCH_MATCH_AGAIN:
				if (SysTask.nWaitTime == 0) //��ָ�ư���
				{
					ensure				= PS_GetImage();

					if (ensure == 0x00) //���������ɹ�
					{
						//PS_ValidTempleteNum(&u16FingerID); //����ָ�Ƹ���
						ensure				= PS_GenChar(CharBuffer1);
						if (ensure == 0x00) //���������ɹ�
						{
							ensure				= PS_Search(CharBuffer1, 0, FINGER_MAX_CNT, &seach);
							if (ensure == 0) //ƥ��ɹ�
							{
								SysTask.u16FingerID = seach.pageID;
								SysTask.nWaitTime	= SysTask.nTick + 1000; //��ʱһ��ʱ�� ʵ���ʱֻ����  ʱ300ms
								SysTask.TouchSub	= TOUCH_SUB_INIT;
								SysTask.TouchState	= TOUCH_IDLE; //�˳�
                                SysTask.bEnFingerTouch = FALSE;    //�ɹ������ָֹ��ģ������
                                //ƥ��ɹ������͵�ƽ��ˣ�Ӧ��
                                g_FingerAck = ACK_SUCCESS;
                                if(SysTask.SendState == SEND_IDLE)
                                    SysTask.SendState = SEND_TABLET_FINGER;
							}
							else if (u8MatchCnt >= MATCH_FINGER_CNT)
							{
                                SysTask.bEnFingerTouch = FALSE;    //�ɹ������ָֹ��ģ������
                                //ƥ��ʧ�ܣ����͵�ƽ��ˣ�Ӧ��
                                g_FingerAck = ACK_FAILED;
                                if(SysTask.SendState == SEND_IDLE)
                                    SysTask.SendState = SEND_TABLET_FINGER;
								SysTask.TouchState	= TOUCH_IDLE; //�˳�
							}
						}
					}

					if (ensure) //ƥ��ʧ��
					{
					    SysTask.nWaitTime = 300; //��ʱһ��ʱ����ȥ���
						u8MatchCnt++;
					}
				}
				break;

			case TOUCH_ADD_USER:
				{
					switch (SysTask.TouchSub)
					{
						case TOUCH_SUB_INIT: // *
							if (SysTask.nWaitTime == 0)
							{
								SysTask.TouchSub	= TOUCH_SUB_ENTER;
							}
							break;

						case TOUCH_SUB_ENTER: //
							if ((PS_Sta) && (SysTask.nWaitTime == 0)) //�����ָ�ư���
							{
								ensure				= PS_GetImage();

								if (ensure == 0x00)
								{
									ensure				= PS_GenChar(CharBuffer1); //��������

									if (ensure == 0x00)
									{
										SysTask.nWaitTime	= 1000; //��ʱһ��ʱ����ȥ�ɼ�
										SysTask.TouchSub	= TOUCH_SUB_AGAIN; //�����ڶ���				
										//Please press  agin
										//�ɹ������͵�ƽ��ˣ�Ӧ������ٰ�һ��
                                        g_FingerAck = ACK_AGAIN;
                                        if(SysTask.SendState == SEND_IDLE)
                                            SysTask.SendState = SEND_TABLET_FINGER;
										
									}
								}
							}
							break;

						case TOUCH_SUB_AGAIN:
							if ((PS_Sta) && (SysTask.nWaitTime == 0)) //�����ָ�ư���
							{
								ensure				= PS_GetImage();
								if (ensure == 0x00)
								{
									ensure				= PS_GenChar(CharBuffer2); //��������

									if (ensure == 0x00)
									{
										ensure				= PS_Match(); //�Ա�����ָ��

										if (ensure == 0x00) //�ɹ�
										{

											ensure				= PS_RegModel(); //����ָ��ģ��

											if (ensure == 0x00)
											{

												ensure				= PS_StoreChar(CharBuffer2, SysTask.u16FingerID); //����ģ��

												if (ensure == 0x00)
												{
													SysTask.TouchSub	= TOUCH_SUB_WAIT;
													SysTask.nWaitTime	= 3000; //��ʱһ��ʱ���˳�
                                                    SysTask.bEnFingerTouch = FALSE;    //�ɹ������ָֹ��ģ������
													//�ɹ������͵�ƽ��ˣ�Ӧ��
                                                    g_FingerAck = ACK_SUCCESS;
                                                    if(SysTask.SendState == SEND_IDLE)
                                                        SysTask.SendState = SEND_TABLET_FINGER;
												}
												else 
												{ 
													//ƥ��ʧ�ܣ����͵�ƽ��ˣ�Ӧ��
													SysTask.TouchSub	= TOUCH_SUB_ENTER;
													SysTask.nWaitTime	= 3000; //��ʱһ��ʱ���˳�
												}
											}
											else 
											{ 
												//ʧ�ܣ����͵�ƽ��ˣ�Ӧ��
                                                g_FingerAck = ACK_FAILED;
                                                if(SysTask.SendState == SEND_IDLE)
                                                    SysTask.SendState = SEND_TABLET_FINGER;
                                                    
												SysTask.TouchSub	= TOUCH_SUB_ENTER;
												SysTask.nWaitTime	= 3000; //��ʱһ��ʱ���˳�
											}
										}
										else 
										{ 
											//ʧ�ܣ����͵�ƽ��ˣ�Ӧ��
                                            g_FingerAck = ACK_FAILED;
                                            if(SysTask.SendState == SEND_IDLE)
                                                SysTask.SendState = SEND_TABLET_FINGER;
                                                
											SysTask.TouchSub	= TOUCH_SUB_ENTER;
											SysTask.nWaitTime	= 3000; //��ʱһ��ʱ���˳�
										}
									}
								}
							}
							break;

						case TOUCH_SUB_WAIT: // *
							if ((SysTask.nWaitTime == 0))
							{
								SysTask.TouchState	= TOUCH_IDLE;
                                SysTask.bEnFingerTouch = FALSE;    //�ɹ������ָֹ��ģ������
							}
							break;

						default:
							break;
					}
				}
				break;

			case TOUCH_DEL_USER:
				{
					switch (SysTask.TouchSub)
					{
						case TOUCH_SUB_INIT: // *
							if (SysTask.nWaitTime == 0)
							{
					            u8MatchCnt			= 0;
								SysTask.TouchSub	= TOUCH_SUB_ENTER;
							}
							break;

						case TOUCH_SUB_ENTER: //
							if (SysTask.nWaitTime)
							{
								break;
							}
                            
							ensure = PS_DeletChar(SysTask.u16FingerID, 1);
                            if (ensure == 0)
							{
								SysTask.TouchSub	= TOUCH_SUB_WAIT;
								SysTask.nWaitTime	= 1000; //��ʱһ��ʱ���˳�
							}
							else  //ɾ��ʧ��
							{
							    u8MatchCnt++;
								SysTask.nWaitTime	= 1000; //��ʱһ��ʱ���ٴ�ɾ��
                                if(u8MatchCnt >= 3) //ʧ���˳�
                                {
                                    //���͵�ƽ�����
                                    g_FingerAck = ACK_FAILED;
                                    if(SysTask.SendState == SEND_IDLE)
                                        SysTask.SendState = SEND_TABLET_FINGER;
                                                
                                    SysTask.TouchSub    = TOUCH_SUB_WAIT;
                                }
							}
							break;

						case TOUCH_SUB_WAIT: // *
							if ((SysTask.nWaitTime == 0))
							{
								SysTask.TouchState	= TOUCH_IDLE;
                                SysTask.bEnFingerTouch = FALSE;    //�ɹ������ָֹ��ģ������
							}
							break;

						default:
							break;
					}
				}
				break;
                
            case TOUCH_IDLE:
			default:
				break;
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
void LedTask(void)
{
	if(SysTask.LED_Mode != SysTask.LED_ModeSave)
    {
        if(SysTask.LED_Mode == LED_MODE_ON)
            GPIO_ResetBits(LED_GPIO, LED_PIN);
        else
            GPIO_SetBits(LED_GPIO, LED_PIN);
        
        SysTask.LED_ModeSave = SysTask.LED_Mode;
    }   
}

/*******************************************************************************
* ����: 
* ����: У���
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
u8 CheckSum(u8 *pu8Addr, u8 u8Count)
{
    u8 i = 0;
    u8 u8Result = 0;

    for(i = 0; i < u8Count; i++)
    {
        u8Result += pu8Addr[i];
    }

    return ~u8Result;
}

/*******************************************************************************
* ����: 
* ����: �ӻ�������
*		��ͷ + �ӻ���ַ +      �ֱ��ַ + ���� + ���ݰ�[3] + У��� + ��β
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void SlavePackageSend(u8 u8SlaveAddr, u8 u8GlassAddr, u8 u8Cmd, u8 *u8Par)
{
	u8 i				= 0;
	u8 u8SendArr[9] 	=
	{
		0
	};

	u8SendArr[0]		= 0xAA; 					//��ͷ
	u8SendArr[1]		= u8SlaveAddr;					//�ӻ���ַ
	u8SendArr[2]		= u8GlassAddr;					//�ֱ��ַ
	u8SendArr[3]		= u8Cmd;					//����
	u8SendArr[4]		= u8Par[0]; 				//����1
	u8SendArr[5]		= u8Par[1]; 				//����2
	u8SendArr[6]		= u8Par[2]; 				//����3

    u8SendArr[7] = CheckSum(u8SendArr, 7);         //У���
	u8SendArr[8]		= 0x55; 					//��β


	for (i = 0; i < 8; i++)
	{
		USART_SendData(USART3, u8SendArr[i]);
	}
}

#define RECV_PAKAGE_CHECK         10   //   10�����ݼ���

/*
static int Usart3JudgeStr(void)
{
    //    ��ͷ + ���� +����[3]        +����[3]    +  У��� + ��β
    //    ��ͷ��0XAA
    //    ��� CMD_UPDATE   �� CMD_READY 
    //    ������ ��ַ��LEDģʽ������ɾ��ָ��ID,
    //    ���ݣ��������� + ���� + ʱ�䣩
    //    У��ͣ�
    //    ��β�� 0X55

    //��������ƽ��İ���Ϊ���֡���һ���ǵ�������һ����ȫ��������

	const char *data;

    u8 u8Cmd;
	u8 i;
    u8 str[8] = {0}; 
	str[0]=0xAA;

    u8 u8CheckSum = 0;

	data=strstr((const char*)USART3_RX_BUF,(const char*)str);
	if(!data)
        return -1;
     
    if( *(data + 9) != 0X55) //��β
        return -1;
        
    for(i = 0; i < RECV_PAKAGE_CHECK; i++)
        u8CheckSum += *(data + i);
    
    if(u8CheckSum != 0x55)  //����Ͳ����ڰ�β ����ʧ��
        return -1;

    
    u8Cmd = *(data + 1);

    switch(u8Cmd)

    {
        case CMD_READY:
            SysTask.bTabReady   = TRUE;
            break;
            
        case CMD_UPDATE:
            if(*(data + 2) ==  0xFF)   //��ȫ���ֱ�
            {
                for(i = 0 ; i < GLASS_MAX_CNT; i++)
                {
                    SysTask.WatchState[i].u8LockState  = *(data + 5);
                    SysTask.WatchState[i].u8Dir        = *(data + 6);
                    SysTask.WatchState[i].MotoTime     = *(data + 7);

                    SysTask.Moto_Mode[i]        =  (MotoFR)*(data + 6);
                    SysTask.Moto_Time[i]        =  (MotoTime_e)*(data + 7);
                    SysTask.Moto_RunTime[i]     =  g_au16MoteTime[SysTask.Moto_Time[i]][1];
                    SysTask.Moto_WaitTime[i]    =  g_au16MoteTime[SysTask.Moto_Time[i]][2];
                    //SysTask.LockMode[i]         =  *(data + 5);  //��״̬���ڴ˸��£���Ϊ������Ҫ����֮��
                        
                }
            }
            else  //�����ֱ��״̬����
            {
                SysTask.WatchState[*(data + 2)].u8LockState  = *(data + 5);
                SysTask.WatchState[*(data + 2)].u8Dir        = *(data + 6);
                SysTask.WatchState[*(data + 2)].MotoTime     = *(data + 7);
                SysTask.Moto_Mode[*(data + 2)]        =  (MotoFR)*(data + 6);
                SysTask.Moto_Time[*(data + 2)]        =  (MotoTime_e)*(data + 7);
                SysTask.Moto_RunTime[*(data + 2)]     =  g_au16MoteTime[SysTask.Moto_Time[i]][1];
                SysTask.Moto_WaitTime[*(data + 2)]    =  g_au16MoteTime[SysTask.Moto_Time[i]][2];
               // SysTask.LockMode[*(data + 2)]         =  *(data + 5);//��״̬���ڴ˸��£���Ϊ������Ҫ����֮��
            }     

            
            SysTask.u16SaveTick = SAVE_TICK_TIME;   //���ݸ��±���
            break;
            
        case CMD_ADD_USER:
            if(*(data + 2) <= FINGER_MAX_CNT)
            {
                SysTask.bEnFingerTouch  = TRUE;
                SysTask.TouchState      = TOUCH_ADD_USER;
                SysTask.u16FingerID     = *(data + 2);
            }
            break;
                
        case CMD_DEL_USER:
            if(*(data + 2) <= FINGER_MAX_CNT)
            {
                SysTask.bEnFingerTouch  = TRUE;
                SysTask.TouchState      = TOUCH_DEL_USER;
                SysTask.u16FingerID     = *(data + 2);
            }
            break;
            
        case CMD_OPEN_LOCK:
            if(*(data + 2) ==  0xFF)   //ȫ����״̬
            {
                for(i = 0 ; i < GLASS_MAX_CNT; i++)
                {
                    SysTask.LockMode[i] = LOCK_ON; 
                }
            }
            else  //������״̬����
            {
               SysTask.LockMode[*(data + 2)] = LOCK_ON;  //���ת������״̬����OFF?
            } 
            break;
            
        case CMD_RUN_MOTO:   
            if(*(data + 2) ==  0xFF)   //��ȫ���ֱ�
            {
                for(i = 0 ; i < GLASS_MAX_CNT; i++)
                {
                    SysTask.LockMode[i]     = LOCK_OFF;  //���ת������״̬����OFF?
    				SysTask.Moto_State[i]	= MOTO_STATE_INIT;
                }
            }
            else  //�����ֱ��״̬����
            {
                SysTask.LockMode[*(data + 2)]     = LOCK_OFF;  //���ת������״̬����OFF?
				SysTask.Moto_State[*(data + 2)]	  = MOTO_STATE_INIT;
            } 
            break;
            
        case CMD_LED_MODE:   
            if((*(data + 2) ==  LED_MODE_ON) || (*(data + 2) ==  LED_MODE_OFF))   //LEDģʽ��Ч
            {
              SysTask.LED_Mode = *(data + 2);
            } 
            
            SysTask.u16SaveTick = SAVE_TICK_TIME;   //���ݸ��±���
            break;
            
        default:
            //���͵�ƽ��ˣ�Ӧ��    �������
            g_FingerAck = ACK_ERROR;
            if(SysTask.SendState == SEND_IDLE)
                SysTask.SendState = SEND_TABLET_FINGER;
            break;
    }
                
   return 0;   //           
}
*/

/*******************************************************************************
* ����: �����հ���������ƽ�巢�����������������
* ����: �ж��жϽ��յ�������û�и������ݰ� ���߾�����
* �β�:		
* ����: -1, ʧ�� 0:�ɹ�
* ˵��: 
*******************************************************************************/
static int Usart1JudgeStr(void)
{
    //    ��ͷ + ���� +����[3]        +����[3]    +  У��� + ��β
    //    ��ͷ��0XAA
    //    ��� CMD_UPDATE   �� CMD_READY 
    //    ������ ��ַ��LEDģʽ������ɾ��ָ��ID,
    //    ���ݣ��������� + ���� + ʱ�䣩
    //    У��ͣ�
    //    ��β�� 0X55

    //��������ƽ��İ���Ϊ���֡���һ���ǵ�������һ����ȫ��������

	const char *data;

    u8 u8Cmd;
	u8 i;
    u8 str[8] = {0}; 
	str[0]=0xAA;

    u8 u8CheckSum = 0;

	data=strstr((const char*)USART1_RX_BUF,(const char*)str);
	if(!data)
        return -1;
     
    if( *(data + 9) != 0X55) //��β
        return -1;
        
    for(i = 0; i < RECV_PAKAGE_CHECK; i++)
        u8CheckSum += *(data + i);
    
    if(u8CheckSum != 0x55)  //����Ͳ����ڰ�β ����ʧ��
        return -1;

    
    u8Cmd = *(data + 1);

    switch(u8Cmd)

    {
        case CMD_READY:
            SysTask.bTabReady   = TRUE;
            break;
            
        case CMD_UPDATE:
            if(*(data + 2) ==  0xFF)   //��ȫ���ֱ�
            {
                for(i = 0 ; i < GLASS_MAX_CNT; i++)
                {
                    SysTask.WatchState[i].u8LockState  = *(data + 5);
                    SysTask.WatchState[i].u8Dir        = *(data + 6);
                    SysTask.WatchState[i].MotoTime     = *(data + 7);

                    SysTask.Moto_Mode[i]        =  (MotoFR)*(data + 6);
                    SysTask.Moto_Time[i]        =  (MotoTime_e)*(data + 7);
                    SysTask.Moto_RunTime[i]     =  g_au16MoteTime[SysTask.Moto_Time[i]][1];
                    SysTask.Moto_WaitTime[i]    =  g_au16MoteTime[SysTask.Moto_Time[i]][2];
                    //SysTask.LockMode[i]         =  *(data + 5);  //��״̬���ڴ˸��£���Ϊ������Ҫ����֮��
                        
                }
            }
            else  //�����ֱ��״̬����
            {
                SysTask.WatchState[*(data + 2)].u8LockState  = *(data + 5);
                SysTask.WatchState[*(data + 2)].u8Dir        = *(data + 6);
                SysTask.WatchState[*(data + 2)].MotoTime     = *(data + 7);
                SysTask.Moto_Mode[*(data + 2)]        =  (MotoFR)*(data + 6);
                SysTask.Moto_Time[*(data + 2)]        =  (MotoTime_e)*(data + 7);
                SysTask.Moto_RunTime[*(data + 2)]     =  g_au16MoteTime[SysTask.Moto_Time[i]][1];
                SysTask.Moto_WaitTime[*(data + 2)]    =  g_au16MoteTime[SysTask.Moto_Time[i]][2];
               // SysTask.LockMode[*(data + 2)]         =  *(data + 5);//��״̬���ڴ˸��£���Ϊ������Ҫ����֮��
            }     

            
            SysTask.u16SaveTick = SAVE_TICK_TIME;   //���ݸ��±���
            break;
            
        case CMD_ADD_USER:
            if(*(data + 2) <= FINGER_MAX_CNT)
            {
                SysTask.bEnFingerTouch  = TRUE;
                SysTask.TouchState      = TOUCH_ADD_USER;
                SysTask.u16FingerID     = *(data + 2);
            }
            break;
                
        case CMD_DEL_USER:
            if(*(data + 2) <= FINGER_MAX_CNT)
            {
                SysTask.bEnFingerTouch  = TRUE;
                SysTask.TouchState      = TOUCH_DEL_USER;
                SysTask.u16FingerID     = *(data + 2);
            }
            break;
            
        case CMD_OPEN_LOCK:
            if(*(data + 2) ==  0xFF)   //ȫ����״̬
            {
                for(i = 0 ; i < GLASS_MAX_CNT; i++)
                {
                    SysTask.LockMode[i] = LOCK_ON; 
                }
            }
            else  //������״̬����
            {
               SysTask.LockMode[*(data + 2)] = LOCK_ON;  //���ת������״̬����OFF?
            } 
            break;
            
        case CMD_RUN_MOTO:   
            if(*(data + 2) ==  0xFF)   //��ȫ���ֱ�
            {
                for(i = 0 ; i < GLASS_MAX_CNT; i++)
                {
                    SysTask.LockMode[i]     = LOCK_OFF;  //���ת������״̬����OFF?
    				SysTask.Moto_State[i]	= MOTO_STATE_INIT;
                }
            }
            else  //�����ֱ��״̬����
            {
                SysTask.LockMode[*(data + 2)]     = LOCK_OFF;  //���ת������״̬����OFF?
				SysTask.Moto_State[*(data + 2)]	  = MOTO_STATE_INIT;
            } 
            break;
            
        case CMD_LED_MODE:   
            if((*(data + 2) ==  LED_MODE_ON) || (*(data + 2) ==  LED_MODE_OFF))   //LEDģʽ��Ч
            {
              SysTask.LED_Mode = *(data + 2);
            } 
            
            SysTask.u16SaveTick = SAVE_TICK_TIME;   //���ݸ��±���
            break;
            
        default:
            //���͵�ƽ��ˣ�Ӧ��    �������
            g_FingerAck = ACK_ERROR;
            if(SysTask.SendState == SEND_IDLE)
                SysTask.SendState = SEND_TABLET_FINGER;
            break;
    }
                
   return 0;   //           
}


/*******************************************************************************
* ����: 
* ����: ƽ����������͵����������
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void TabletToHostTask(void)
{
    if(USART1_RX_STA & 0X8000)//���յ�ƽ������
    {
        Usart1JudgeStr();
        USART1_RX_STA   =   0; //���״̬
    }
    
//    if(USART3_RX_STA & 0X8000)//���յ�ƽ������
//    {
//        Usart3JudgeStr();
//        USART3_RX_STA   =   0; //���״̬
//    }
}

/*******************************************************************************
* ����: 
* ����: ��������ȫ��״̬��ƽ��
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void SendToTablet(u8 u8GlassAddr)
{
    //    ��ͷ + ��ַ + ���� + ���ݰ�[48] + У��� + ��β
    //    ��ͷ��0XAA
    //    �ֱ��ַ��0X8~�����λΪ1��ʾȫ��                       ~:��ʾ�м����ֱ�
    //              0x01����һ���ֱ�
    //    ���
    //    ���ݣ��������� + ���� + ʱ�䣩* 16 = 48
    //    У��ͣ�
    //    ��β�� 0X55
    u8 i;
    u8 u8aSendArr[53] = {0};
    SendArrayUnion_u SendArrayUnion;

    if(u8GlassAddr == FINGER_ACK_ADDR) //Ӧ���ַ
    {
        u8aSendArr[2] = g_FingerAck;       //��������  Ӧ���������
    }else
    {
        memcpy(SendArrayUnion.WatchState, SysTask.WatchState, sizeof(SysTask.WatchState));

        u8aSendArr[2] = 0x11;       //�������ݱ����� 
    }
    
    u8aSendArr[0] = 0xAA;
    u8aSendArr[1] = u8GlassAddr;
    for(i = 0; i < 48; i++)
    {
        u8aSendArr[3 + i] = SendArrayUnion.u8SendArray[i];
    }
    u8aSendArr[51] = CheckSum(u8aSendArr, 51); //У���
    u8aSendArr[52] = 0x55;  //��βs 

    for(i = 0; i < 53; i++)
    {
        Usart1_SendByte(u8aSendArr[i]);
    }
}

/*******************************************************************************
* ����: 
* ����: ���͸�ƽ�������
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void SendTabletTask(void)
{
	switch (SysTask.SendState)
	{
		case SEND_INIT:
		    if(SysTask.bTabReady == TRUE)   
           {
                //SLAVE_SEND_ALL; 
                g_GlassSendAddr = GLASS_SEND_ALL | GLASS_MAX_CNT;
    			SendToTablet(g_GlassSendAddr);  //�����ֱ����
    			SysTask.SendState = SEND_IDLE;
            }
			break;
        
        case SEND_TABLET_FINGER: //����ģ��Ӧ���ƽ��
            SendToTablet(FINGER_ACK_ADDR);  //
            SysTask.SendState = SEND_IDLE;
            break;

        /* ���ڽ׶�û��ʹ���ⲿ�ֵ����ݣ���Ϊ����Ӧ����飬���������ⲿ����ʱ�ò���
		case SEND_TABLET_SINGLE://���͵����ӻ�״̬��ƽ��
			SendToTablet(g_SlaveSendAddr, g_GlassSendAddr);
			break;
        */

        
		case SEND_IDLE:
		default:
    		SysTask.SendState = SEND_IDLE;
			break;
	}
}


/*******************************************************************************
* ����: 
* ����: �����ӷ��͸��ӻ����͵����������
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void HostToTabletTask(void)
{
//    static u8 i = 0;
//    static u8 j = 0;

//    static u8 state = 0;


//SysTask.WatchStateSave, SysTask.SlaveState
//    switch(state)
//        {
//        case HtoS_CHECK:
//            if(     (SysTask.WatchStateSave.MotoTime != SysTask.WatchState.MotoTime)
//               ||   (SysTask.WatchStateSave.MotoTime != SysTask.WatchState.MotoTime)
//               ||   (SysTask.WatchStateSave.MotoTime != SysTask.WatchState.MotoTime))

//            {
//                state = HtoS_SEND;
//            }
//            else
//            {
//                if(i == 7){
//                    j++;
//                    if(j == 4) j = 0;
//                    i = 0;
//                 }
//                else
//                    i++;
//            }
//            break;
//        case HtoS_SEND:
//            SysTask.WatchState.MotoTime = SysTask.WatchStateSave.MotoTime;
//            SysTask.WatchState.MotoTime = SysTask.WatchStateSave.MotoTime;
//            SysTask.WatchState.MotoTime = SysTask.WatchStateSave.MotoTime;
//            state = HtoS_WAIT;
//            break;
//        case HtoS_WAIT:
//            if(SysTask.u16HtoSWaitTime == 0)
//                state = HtoS_CHECK;
//            break;
//        default:
//            break;
//    }
}


/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_01_Task(void)
{
	static u8 u8State	= 0;

	//A���
	switch (SysTask.Moto_State[GLASS_01])
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime[GLASS_01] == 0)
			{
				SysTask.Moto_RunTime[GLASS_01] = g_au16MoteTime[SysTask.Moto_Time[GLASS_01]][1];
				SysTask.Moto_State[GLASS_01]	= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_01] = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_01] == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //
				SysTask.Moto_State[GLASS_01]	= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_01] == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //
				SysTask.Moto_State[GLASS_01]	= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_01] == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //
				SysTask.Moto_State[GLASS_01]	= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_01] = SysTask.Moto_Mode[GLASS_01];
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_01] != SysTask.Moto_ModeSave[GLASS_01])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_01]	= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime[GLASS_01] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_01])
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_01] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_01] = g_au16MoteTime[SysTask.Moto_Time[GLASS_01]][2];
						SysTask.Moto_SubState[GLASS_01] = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_01] == 0)
					{
						SysTask.Moto_RunTime[GLASS_01] = g_au16MoteTime[SysTask.Moto_Time[GLASS_01]][1];
						SysTask.Moto_SubState[GLASS_01] = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_01] == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //
							SysTask.Moto_State[GLASS_01]	= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_01] == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //
							SysTask.Moto_State[GLASS_01]	= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_01] != SysTask.Moto_ModeSave[GLASS_01])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_01]	= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_01])
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_01] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_01] = g_au16MoteTime[SysTask.Moto_Time[GLASS_01]][2];
						SysTask.Moto_SubState[GLASS_01] = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //
					}
					else if (SysTask.Moto_StateTime[GLASS_01] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_01_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime[GLASS_01] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_01] == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_01] = g_au16MoteTime[SysTask.Moto_Time[GLASS_01]][1];
						SysTask.Moto_SubState[GLASS_01] = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_01] == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_01_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_01_PIN_P); //

				SysTask.Moto_State[GLASS_01]	= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_02_Task(void)
{
	static u8 u8State	= 0;

	//_02���
	switch (SysTask.Moto_State[GLASS_02]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_02] == 0)
			{
				SysTask.Moto_RunTime[GLASS_02] = g_au16MoteTime[SysTask.Moto_Time[GLASS_02] ][1];
				SysTask.Moto_State[GLASS_02]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_02]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_02]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //
				SysTask.Moto_State[GLASS_02]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_02]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //
				SysTask.Moto_State[GLASS_02]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_02]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //
				SysTask.Moto_State[GLASS_02]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_02]  = SysTask.Moto_Mode[GLASS_02] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_02]  != SysTask.Moto_ModeSave[GLASS_02])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_02]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_02] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_02] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_02] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_02]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_02] ][2];
						SysTask.Moto_SubState[GLASS_02]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_02]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_02] = g_au16MoteTime[SysTask.Moto_Time[GLASS_02] ][1];
						SysTask.Moto_SubState[GLASS_02]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_02]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //
							SysTask.Moto_State[GLASS_02]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_02]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //
							SysTask.Moto_State[GLASS_02]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_02]  != SysTask.Moto_ModeSave[GLASS_02])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_02]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_02] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_02] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_02]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_02] ][2];
						SysTask.Moto_SubState[GLASS_02]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_02] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_02_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_02] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_02]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_02] = g_au16MoteTime[SysTask.Moto_Time[GLASS_02] ][1];
						SysTask.Moto_SubState[GLASS_02]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_02]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_02_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_02_PIN_P); //

				SysTask.Moto_State[GLASS_02]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_03_Task(void)
{
	static u8 u8State	= 0;

	//_03���
	switch (SysTask.Moto_State[GLASS_03]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_03] == 0)
			{
				SysTask.Moto_RunTime[GLASS_03] = g_au16MoteTime[SysTask.Moto_Time[GLASS_03] ][1];
				SysTask.Moto_State[GLASS_03]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_03]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_03]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //
				SysTask.Moto_State[GLASS_03]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_03]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //
				SysTask.Moto_State[GLASS_03]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_03]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //
				SysTask.Moto_State[GLASS_03]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_03] = SysTask.Moto_Mode[GLASS_03] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_03]  != SysTask.Moto_ModeSave[GLASS_03])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_03]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_03] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_03] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_03] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_03]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_03] ][2];
						SysTask.Moto_SubState[GLASS_03]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_03]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_03] = g_au16MoteTime[SysTask.Moto_Time[GLASS_03] ][1];
						SysTask.Moto_SubState[GLASS_03]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_03]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //
							SysTask.Moto_State[GLASS_03]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_03]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //
							SysTask.Moto_State[GLASS_03]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_03]  != SysTask.Moto_ModeSave[GLASS_03])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_03]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_03] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_03] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_03]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_03] ][2];
						SysTask.Moto_SubState[GLASS_03]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_03] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_03_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_03] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_03]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_03] = g_au16MoteTime[SysTask.Moto_Time[GLASS_03] ][1];
						SysTask.Moto_SubState[GLASS_03]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_03]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_03_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_03_PIN_P); //

				SysTask.Moto_State[GLASS_03]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_04_Task(void)
{
	static u8 u8State	= 0;

	//_04���
	switch (SysTask.Moto_State[GLASS_04]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_04] == 0)
			{
				SysTask.Moto_RunTime[GLASS_04] = g_au16MoteTime[SysTask.Moto_Time[GLASS_04] ][1];
				SysTask.Moto_State[GLASS_04]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_04]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_04]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //
				SysTask.Moto_State[GLASS_04]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_04]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //
				SysTask.Moto_State[GLASS_04]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_04]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //
				SysTask.Moto_State[GLASS_04]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_04] = SysTask.Moto_Mode[GLASS_04] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_04]  != SysTask.Moto_ModeSave[GLASS_04])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_04]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_04] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_04] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_04] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_04]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_04] ][2];
						SysTask.Moto_SubState[GLASS_04]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_04]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_04] = g_au16MoteTime[SysTask.Moto_Time[GLASS_04] ][1];
						SysTask.Moto_SubState[GLASS_04]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_04]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //
							SysTask.Moto_State[GLASS_04]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_04]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //
							SysTask.Moto_State[GLASS_04]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_04]  != SysTask.Moto_ModeSave[GLASS_04])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_04]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_04] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_04] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_04]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_04] ][2];
						SysTask.Moto_SubState[GLASS_04]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_04] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_04_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_04] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_04]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_04] = g_au16MoteTime[SysTask.Moto_Time[GLASS_04] ][1];
						SysTask.Moto_SubState[GLASS_04]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_04]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_04_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_04_PIN_P); //

				SysTask.Moto_State[GLASS_04]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_05_Task(void)
{
	static u8 u8State	= 0;

	//_05���
	switch (SysTask.Moto_State[GLASS_05]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_05] == 0)
			{
				SysTask.Moto_RunTime[GLASS_05] = g_au16MoteTime[SysTask.Moto_Time[GLASS_05] ][1];
				SysTask.Moto_State[GLASS_05]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_05]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_05]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //
				SysTask.Moto_State[GLASS_05]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_05]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //
				SysTask.Moto_State[GLASS_05]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_05]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //
				SysTask.Moto_State[GLASS_05]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_05] = SysTask.Moto_Mode[GLASS_05] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_05]  != SysTask.Moto_ModeSave[GLASS_05])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_05]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_05] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_05] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_05] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_05]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_05] ][2];
						SysTask.Moto_SubState[GLASS_05]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_05]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_05] = g_au16MoteTime[SysTask.Moto_Time[GLASS_05] ][1];
						SysTask.Moto_SubState[GLASS_05]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_05]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //
							SysTask.Moto_State[GLASS_05]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_05]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //
							SysTask.Moto_State[GLASS_05]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_05]  != SysTask.Moto_ModeSave[GLASS_05])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_05]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_05] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_05] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_05]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_05] ][2];
						SysTask.Moto_SubState[GLASS_05]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_05] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_05_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_05] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_05]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_05] = g_au16MoteTime[SysTask.Moto_Time[GLASS_05] ][1];
						SysTask.Moto_SubState[GLASS_05]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_05]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_05_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_05_PIN_P); //

				SysTask.Moto_State[GLASS_05]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_06_Task(void)
{
	static u8 u8State	= 0;

	//_06���
	switch (SysTask.Moto_State[GLASS_06]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_06] == 0)
			{
				SysTask.Moto_RunTime[GLASS_06] = g_au16MoteTime[SysTask.Moto_Time[GLASS_06] ][1];
				SysTask.Moto_State[GLASS_06]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_06]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_06]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //
				SysTask.Moto_State[GLASS_06]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_06]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //
				SysTask.Moto_State[GLASS_06]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_06]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //
				SysTask.Moto_State[GLASS_06]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_06] = SysTask.Moto_Mode[GLASS_06] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_06]  != SysTask.Moto_ModeSave[GLASS_06])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_06]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_06] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_06] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_06] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_06]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_06] ][2];
						SysTask.Moto_SubState[GLASS_06]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_06]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_06] = g_au16MoteTime[SysTask.Moto_Time[GLASS_06] ][1];
						SysTask.Moto_SubState[GLASS_06]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_06]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //
							SysTask.Moto_State[GLASS_06]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_06]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //
							SysTask.Moto_State[GLASS_06]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_06]  != SysTask.Moto_ModeSave[GLASS_06])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_06]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_06] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_06] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_06]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_06] ][2];
						SysTask.Moto_SubState[GLASS_06]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_06] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_06_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_06] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_06]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_06] = g_au16MoteTime[SysTask.Moto_Time[GLASS_06] ][1];
						SysTask.Moto_SubState[GLASS_06]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_06]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_06_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_06_PIN_P); //

				SysTask.Moto_State[GLASS_06]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_07_Task(void)
{
	static u8 u8State	= 0;

	//_07���
	switch (SysTask.Moto_State[GLASS_07]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_07] == 0)
			{
				SysTask.Moto_RunTime[GLASS_07] = g_au16MoteTime[SysTask.Moto_Time[GLASS_07] ][1];
				SysTask.Moto_State[GLASS_07]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_07]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_07]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //
				SysTask.Moto_State[GLASS_07]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_07]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //
				SysTask.Moto_State[GLASS_07]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_07]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //
				SysTask.Moto_State[GLASS_07]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_07] = SysTask.Moto_Mode[GLASS_07] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_07]  != SysTask.Moto_ModeSave[GLASS_07])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_07]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_07] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_07] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_07] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_07]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_07] ][2];
						SysTask.Moto_SubState[GLASS_07]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_07]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_07] = g_au16MoteTime[SysTask.Moto_Time[GLASS_07] ][1];
						SysTask.Moto_SubState[GLASS_07]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_07]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //
							SysTask.Moto_State[GLASS_07]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_07]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //
							SysTask.Moto_State[GLASS_07]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_07]  != SysTask.Moto_ModeSave[GLASS_07])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_07]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_07] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_07] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_07]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_07] ][2];
						SysTask.Moto_SubState[GLASS_07]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_07] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_07_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_07] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_07]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_07] = g_au16MoteTime[SysTask.Moto_Time[GLASS_07] ][1];
						SysTask.Moto_SubState[GLASS_07]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_07]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_07_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_07_PIN_P); //

				SysTask.Moto_State[GLASS_07]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_08_Task(void)
{
	static u8 u8State	= 0;

	//_08���
	switch (SysTask.Moto_State[GLASS_08]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_08] == 0)
			{
				SysTask.Moto_RunTime[GLASS_08] = g_au16MoteTime[SysTask.Moto_Time[GLASS_08] ][1];
				SysTask.Moto_State[GLASS_08]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_08]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_08]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //
				SysTask.Moto_State[GLASS_08]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_08]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //
				SysTask.Moto_State[GLASS_08]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_08]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //
				SysTask.Moto_State[GLASS_08]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_08] = SysTask.Moto_Mode[GLASS_08] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_08]  != SysTask.Moto_ModeSave[GLASS_08])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_08]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_08] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_08] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_08] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_08]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_08] ][2];
						SysTask.Moto_SubState[GLASS_08]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_08]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_08] = g_au16MoteTime[SysTask.Moto_Time[GLASS_08] ][1];
						SysTask.Moto_SubState[GLASS_08]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_08]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //
							SysTask.Moto_State[GLASS_08]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_08]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //
							SysTask.Moto_State[GLASS_08]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_08]  != SysTask.Moto_ModeSave[GLASS_08])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_08]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_08] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_08] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_08]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_08] ][2];
						SysTask.Moto_SubState[GLASS_08]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_08] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_08_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_08] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_08]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_08] = g_au16MoteTime[SysTask.Moto_Time[GLASS_08] ][1];
						SysTask.Moto_SubState[GLASS_08]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_08]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_08_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_08_PIN_P); //

				SysTask.Moto_State[GLASS_08]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_09_Task(void)
{
	static u8 u8State	= 0;

	//_09���
	switch (SysTask.Moto_State[GLASS_09]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_09] == 0)
			{
				SysTask.Moto_RunTime[GLASS_09] = g_au16MoteTime[SysTask.Moto_Time[GLASS_09] ][1];
				SysTask.Moto_State[GLASS_09]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_09]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_09]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //
				SysTask.Moto_State[GLASS_09]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_09]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //
				SysTask.Moto_State[GLASS_09]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_09]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //
				SysTask.Moto_State[GLASS_09]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_09] = SysTask.Moto_Mode[GLASS_09] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_09]  != SysTask.Moto_ModeSave[GLASS_09])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_09]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_09] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_09] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_09] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_09]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_09] ][2];
						SysTask.Moto_SubState[GLASS_09]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_09]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_09] = g_au16MoteTime[SysTask.Moto_Time[GLASS_09] ][1];
						SysTask.Moto_SubState[GLASS_09]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_09]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //
							SysTask.Moto_State[GLASS_09]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_09]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //
							SysTask.Moto_State[GLASS_09]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_09]  != SysTask.Moto_ModeSave[GLASS_09])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_09]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_09] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_09] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_09]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_09] ][2];
						SysTask.Moto_SubState[GLASS_09]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_09] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_09_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_09] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_09]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_09] = g_au16MoteTime[SysTask.Moto_Time[GLASS_09] ][1];
						SysTask.Moto_SubState[GLASS_09]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_09]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_09_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_09_PIN_P); //

				SysTask.Moto_State[GLASS_09]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_10_Task(void)
{
	static u8 u8State	= 0;

	//_10���
	switch (SysTask.Moto_State[GLASS_10]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_10] == 0)
			{
				SysTask.Moto_RunTime[GLASS_10] = g_au16MoteTime[SysTask.Moto_Time[GLASS_10] ][1];
				SysTask.Moto_State[GLASS_10]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_10]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_10]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //
				SysTask.Moto_State[GLASS_10]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_10]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //
				SysTask.Moto_State[GLASS_10]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_10]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //
				SysTask.Moto_State[GLASS_10]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_10] = SysTask.Moto_Mode[GLASS_10] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_10]  != SysTask.Moto_ModeSave[GLASS_10])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_10]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_10] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_10] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_10] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_10]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_10] ][2];
						SysTask.Moto_SubState[GLASS_10]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_10]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_10] = g_au16MoteTime[SysTask.Moto_Time[GLASS_10] ][1];
						SysTask.Moto_SubState[GLASS_10]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_10]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //
							SysTask.Moto_State[GLASS_10]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_10]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //
							SysTask.Moto_State[GLASS_10]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_10]  != SysTask.Moto_ModeSave[GLASS_10])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_10]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_10] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_10] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_10]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_10] ][2];
						SysTask.Moto_SubState[GLASS_10]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_10] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_10_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_10] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_10]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_10] = g_au16MoteTime[SysTask.Moto_Time[GLASS_10] ][1];
						SysTask.Moto_SubState[GLASS_10]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_10]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_10_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_10_PIN_P); //

				SysTask.Moto_State[GLASS_10]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_11_Task(void)
{
	static u8 u8State	= 0;

	//_11���
	switch (SysTask.Moto_State[GLASS_11] )
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_11] == 0)
			{
				SysTask.Moto_RunTime[GLASS_11] = g_au16MoteTime[SysTask.Moto_Time[GLASS_11] ][1];
				SysTask.Moto_State[GLASS_11]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_11]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_11]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //
				SysTask.Moto_State[GLASS_11]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_11]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //
				SysTask.Moto_State[GLASS_11]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_11]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //
				SysTask.Moto_State[GLASS_11]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_11] = SysTask.Moto_Mode[GLASS_11] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_11]  != SysTask.Moto_ModeSave[GLASS_11])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_11]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_11] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_11] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_11] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_11]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_11] ][2];
						SysTask.Moto_SubState[GLASS_11]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_11]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_11] = g_au16MoteTime[SysTask.Moto_Time[GLASS_11] ][1];
						SysTask.Moto_SubState[GLASS_11]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_11]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //
							SysTask.Moto_State[GLASS_11]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_11]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //
							SysTask.Moto_State[GLASS_11]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_11]  != SysTask.Moto_ModeSave[GLASS_11])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_11]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_11] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_11] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_11]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_11] ][2];
						SysTask.Moto_SubState[GLASS_11]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_11] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_11_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_11] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_11]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_11] = g_au16MoteTime[SysTask.Moto_Time[GLASS_11] ][1];
						SysTask.Moto_SubState[GLASS_11]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_11]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_11_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_11_PIN_P); //

				SysTask.Moto_State[GLASS_11]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_12_Task(void)
{
	static u8 u8State	= 0;

	//_12���
	switch (SysTask.Moto_State[GLASS_12]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_12] == 0)
			{
				SysTask.Moto_RunTime[GLASS_12] = g_au16MoteTime[SysTask.Moto_Time[GLASS_12] ][1];
				SysTask.Moto_State[GLASS_12]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_12]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_12]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //
				SysTask.Moto_State[GLASS_12]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_12]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //
				SysTask.Moto_State[GLASS_12]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_12]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //
				SysTask.Moto_State[GLASS_12]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_12] = SysTask.Moto_Mode[GLASS_12] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_12]  != SysTask.Moto_ModeSave[GLASS_12])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_12]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_12] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_12] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_12] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_12]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_12] ][2];
						SysTask.Moto_SubState[GLASS_12]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_12]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_12] = g_au16MoteTime[SysTask.Moto_Time[GLASS_12] ][1];
						SysTask.Moto_SubState[GLASS_12]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_12]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //
							SysTask.Moto_State[GLASS_12]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_12]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //
							SysTask.Moto_State[GLASS_12]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_12]  != SysTask.Moto_ModeSave[GLASS_12])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_12]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_12] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_12] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_12]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_12] ][2];
						SysTask.Moto_SubState[GLASS_12]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_12] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_12_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_12] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_12]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_12] = g_au16MoteTime[SysTask.Moto_Time[GLASS_12] ][1];
						SysTask.Moto_SubState[GLASS_12]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_12]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_12_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_12_PIN_P); //

				SysTask.Moto_State[GLASS_12]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_13_Task(void)
{
	static u8 u8State	= 0;

	//_13���
	switch (SysTask.Moto_State[GLASS_13]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_13] == 0)
			{
				SysTask.Moto_RunTime[GLASS_13] = g_au16MoteTime[SysTask.Moto_Time[GLASS_13] ][1];
				SysTask.Moto_State[GLASS_13]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_13]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_13]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //
				SysTask.Moto_State[GLASS_13]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_13]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //
				SysTask.Moto_State[GLASS_13]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_13]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //
				SysTask.Moto_State[GLASS_13]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_13] = SysTask.Moto_Mode[GLASS_13] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_13]  != SysTask.Moto_ModeSave[GLASS_13])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_13]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_13] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_13] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_13] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_13]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_13] ][2];
						SysTask.Moto_SubState[GLASS_13]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_13]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_13] = g_au16MoteTime[SysTask.Moto_Time[GLASS_13] ][1];
						SysTask.Moto_SubState[GLASS_13]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_13]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //
							SysTask.Moto_State[GLASS_13]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_13]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //
							SysTask.Moto_State[GLASS_13]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_13]  != SysTask.Moto_ModeSave[GLASS_13])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_13]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_13] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_13] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_13]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_13] ][2];
						SysTask.Moto_SubState[GLASS_13]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_13] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_13_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_13] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_13]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_13] = g_au16MoteTime[SysTask.Moto_Time[GLASS_13] ][1];
						SysTask.Moto_SubState[GLASS_13]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_13]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_13_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_13_PIN_P); //

				SysTask.Moto_State[GLASS_13]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}
/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_14_Task(void)
{
	static u8 u8State	= 0;

	//_14���
	switch (SysTask.Moto_State[GLASS_14]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_14] == 0)
			{
				SysTask.Moto_RunTime[GLASS_14] = g_au16MoteTime[SysTask.Moto_Time[GLASS_14] ][1];
				SysTask.Moto_State[GLASS_14]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_14]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_14]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //
				SysTask.Moto_State[GLASS_14]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_14]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //
				SysTask.Moto_State[GLASS_14]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_14]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //
				SysTask.Moto_State[GLASS_14]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_14] = SysTask.Moto_Mode[GLASS_14] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_14]  != SysTask.Moto_ModeSave[GLASS_14])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_14]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_14] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_14] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_14] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_14]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_14] ][2];
						SysTask.Moto_SubState[GLASS_14]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_14]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_14] = g_au16MoteTime[SysTask.Moto_Time[GLASS_14] ][1];
						SysTask.Moto_SubState[GLASS_14]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_14]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //
							SysTask.Moto_State[GLASS_14]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_14]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //
							SysTask.Moto_State[GLASS_14]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_14]  != SysTask.Moto_ModeSave[GLASS_14])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_14]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_14] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_14] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_14]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_14] ][2];
						SysTask.Moto_SubState[GLASS_14]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_14] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_14_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_14] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_14]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_14] = g_au16MoteTime[SysTask.Moto_Time[GLASS_14] ][1];
						SysTask.Moto_SubState[GLASS_14]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_14]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_14_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_14_PIN_P); //

				SysTask.Moto_State[GLASS_14]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}


/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_15_Task(void)
{
	static u8 u8State	= 0;

	//_15���
	switch (SysTask.Moto_State[GLASS_15]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_15] == 0)
			{
				SysTask.Moto_RunTime[GLASS_15] = g_au16MoteTime[SysTask.Moto_Time[GLASS_15] ][1];
				SysTask.Moto_State[GLASS_15]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_15]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_15]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //
				SysTask.Moto_State[GLASS_15]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_15]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //
				SysTask.Moto_State[GLASS_15]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_15]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //
				SysTask.Moto_State[GLASS_15]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_15] = SysTask.Moto_Mode[GLASS_15] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_15]  != SysTask.Moto_ModeSave[GLASS_15])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_15]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_15] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_15] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_15] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_15]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_15] ][2];
						SysTask.Moto_SubState[GLASS_15]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_15]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_15] = g_au16MoteTime[SysTask.Moto_Time[GLASS_15] ][1];
						SysTask.Moto_SubState[GLASS_15]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_15]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //
							SysTask.Moto_State[GLASS_15]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_15]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //
							SysTask.Moto_State[GLASS_15]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_15]  != SysTask.Moto_ModeSave[GLASS_15])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_15]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_15] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_15] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_15]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_15] ][2];
						SysTask.Moto_SubState[GLASS_15]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_15] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_15_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_15] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_15]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_15] = g_au16MoteTime[SysTask.Moto_Time[GLASS_15] ][1];
						SysTask.Moto_SubState[GLASS_15]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_15]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_15_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_15_PIN_P); //

				SysTask.Moto_State[GLASS_15]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Moto_16_Task(void)
{
	static u8 u8State	= 0;

	//_16���
	switch (SysTask.Moto_State[GLASS_16]) 
	{
		case MOTO_STATE_INIT:
			if (SysTask.Moto_StateTime [GLASS_16] == 0)
			{
				SysTask.Moto_RunTime[GLASS_16] = g_au16MoteTime[SysTask.Moto_Time[GLASS_16] ][1];
				SysTask.Moto_State[GLASS_16]= MOTO_STATE_CHANGE_DIR;
				SysTask.Moto_SubState[GLASS_16]  = MOTO_SUB_STATE_RUN;
			}

			break;

		case MOTO_STATE_CHANGE_TIME:
			break;

		case MOTO_STATE_CHANGE_DIR:
			if (SysTask.Moto_Mode[GLASS_16]  == MOTO_FR_FWD)
			{
				GPIO_SetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //��ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //
				SysTask.Moto_State[GLASS_16]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_16]  == MOTO_FR_REV)
			{
				GPIO_SetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //��ת
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //
				SysTask.Moto_State[GLASS_16]= MOTO_STATE_RUN_NOR;
			}
			else if (SysTask.Moto_Mode[GLASS_16]  == MOTO_FR_FWD_REV)
			{ //����ת
				u8State 			= 0;
				GPIO_SetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //����ת
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //
				SysTask.Moto_State[GLASS_16]= MOTO_STATE_RUN_CHA;
			}

			SysTask.Moto_ModeSave[GLASS_16] = SysTask.Moto_Mode[GLASS_16] ;
			break;

		case MOTO_STATE_RUN_NOR: //��ͨ����״̬
			if (SysTask.Moto_Mode[GLASS_16]  != SysTask.Moto_ModeSave[GLASS_16])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_16]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				SysTask.Moto_StateTime [GLASS_16] = 200;
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_16] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_16] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_16]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_16] ][2];
						SysTask.Moto_SubState[GLASS_16]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //
					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_16]  == 0)
					{
						SysTask.Moto_RunTime[GLASS_16] = g_au16MoteTime[SysTask.Moto_Time[GLASS_16] ][1];
						SysTask.Moto_SubState[GLASS_16]  = MOTO_SUB_STATE_RUN;

						if (SysTask.Moto_Mode[GLASS_16]  == MOTO_FR_FWD)
						{
							GPIO_SetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //��ת
							GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //
							SysTask.Moto_State[GLASS_16]= MOTO_STATE_RUN_NOR;
						}
						else if (SysTask.Moto_Mode[GLASS_16]  == MOTO_FR_REV)
						{
							GPIO_SetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //��ת
							GPIO_ResetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //
							SysTask.Moto_State[GLASS_16]= MOTO_STATE_RUN_NOR;
						}
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_RUN_CHA: //����ת����״̬
			if (SysTask.Moto_Mode[GLASS_16]  != SysTask.Moto_ModeSave[GLASS_16])
			{
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //����ͣ���ת��
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //����ͣ���ת��
				SysTask.Moto_State[GLASS_16]= MOTO_STATE_INIT; //����ʱһ��ʱ���ٷ�ת
				break;
			}

			switch (SysTask.Moto_SubState[GLASS_16] )
			{
				case MOTO_SUB_STATE_RUN:
					if (SysTask.Moto_RunTime[GLASS_16] == 0)
					{
						SysTask.Moto_WaitTime[GLASS_16]  = g_au16MoteTime[SysTask.Moto_Time[GLASS_16] ][2];
						SysTask.Moto_SubState[GLASS_16]  = MOTO_SUB_STATE_WAIT;
						GPIO_ResetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //ֹͣ
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //
					}
					else if (SysTask.Moto_StateTime [GLASS_16] == 0)
					{
						if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_16_PIN) == 0) // //��⵽���  
						{

							SysTask.Moto_StateTime [GLASS_16] = SysTask.nTick + 500; //��⵽0�㣬һ��ʱ����ٴμ�⣬�ܿ�

							if (u8State == 0)
							{
								u8State 			= 1;
								GPIO_SetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //��ת
								GPIO_ResetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //

							}
							else 
							{
								u8State 			= 0;
								GPIO_SetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //����ת
								GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //
							}
						}

					}

					break;

				case MOTO_SUB_STATE_WAIT:
					if (SysTask.Moto_WaitTime[GLASS_16]  == 0)
					{
						u8State 			= 0;
						SysTask.Moto_RunTime[GLASS_16] = g_au16MoteTime[SysTask.Moto_Time[GLASS_16] ][1];
						SysTask.Moto_SubState[GLASS_16]  = MOTO_SUB_STATE_RUN;
						GPIO_SetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //����ת
						GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //
					}

					break;

				default:
					break;
			}

			break;

		case MOTO_STATE_STOP:
			if (SysTask.Moto_WaitTime[GLASS_16]  == 0)
			{
				GPIO_ResetBits(MOTO_N_GPIO, MOTO_16_PIN_N); //ֹͣ
				GPIO_ResetBits(MOTO_P_GPIO, MOTO_16_PIN_P); //

				SysTask.Moto_State[GLASS_16]= MOTO_STATE_IDLE;
			}

			break;

		case MOTO_STATE_WAIT:
			break;

		case MOTO_STATE_IDLE:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_01_Task(void)
{
    switch (SysTask.Lock_State[GLASS_01] ) //_01��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_01]  != SysTask.LockModeSave[GLASS_01] )
            {
                if (SysTask.LockMode[GLASS_01]  == LOCK_ON)  //��Ҫ����
                {
                    if (SysTask.Moto_WaitTime[GLASS_01] != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_01] = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_01_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_01]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_01] = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_01]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_01]  = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_01]  = SysTask.LockMode[GLASS_01] ;
                        SysTask.Lock_OffTime[GLASS_01] = LOCK_OFFTIME;
                    }
                }
                else //��Ҫ����
                {
                    SysTask.LockModeSave[GLASS_01]  = SysTask.LockMode[GLASS_01] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_01_PIN);
                    SysTask.Lock_OffTime[GLASS_01]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:  //���� 40ms �����е��ٿ���
            if (SysTask.Lock_StateTime[GLASS_01]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_01_PIN) == 0) //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_01_PIN);
                    SysTask.Lock_State[GLASS_01]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_02_Task(void)
{
    switch (SysTask.Lock_State[GLASS_02] ) //_02��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_02]  != SysTask.LockModeSave[GLASS_02] )
            {
                if (SysTask.LockMode[GLASS_02]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_02]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_02]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_02_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_02]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_02]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_02]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_02] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_02]  = SysTask.LockMode[GLASS_02] ;
                        SysTask.Lock_OffTime[GLASS_02] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_02]  = SysTask.LockMode[GLASS_02] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_02_PIN);
                    SysTask.Lock_OffTime[GLASS_02]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_02]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_02_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_02_PIN);
                    SysTask.Lock_State[GLASS_02]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_03_Task(void)
{
    switch (SysTask.Lock_State[GLASS_03] ) //_03��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_03]  != SysTask.LockModeSave[GLASS_03] )
            {
                if (SysTask.LockMode[GLASS_03]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_03]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_03]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_03_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_03]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_03]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_03]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_03] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_03]  = SysTask.LockMode[GLASS_03] ;
                        SysTask.Lock_OffTime[GLASS_03] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_03]  = SysTask.LockMode[GLASS_03] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_03_PIN);
                    SysTask.Lock_OffTime[GLASS_03]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_03]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_03_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_03_PIN);
                    SysTask.Lock_State[GLASS_03]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_04_Task(void)
{
    switch (SysTask.Lock_State[GLASS_04] ) //_04��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_04]  != SysTask.LockModeSave[GLASS_04] )
            {
                if (SysTask.LockMode[GLASS_04]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_04]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_04]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_04_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_04]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_04]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_04]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_04] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_04]  = SysTask.LockMode[GLASS_04] ;
                        SysTask.Lock_OffTime[GLASS_04] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_04]  = SysTask.LockMode[GLASS_04] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_04_PIN);
                    SysTask.Lock_OffTime[GLASS_04]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_04]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_04_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_04_PIN);
                    SysTask.Lock_State[GLASS_04]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}
/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_05_Task(void)
{
    switch (SysTask.Lock_State[GLASS_05] ) //_05��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_05]  != SysTask.LockModeSave[GLASS_05] )
            {
                if (SysTask.LockMode[GLASS_05]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_05]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_05]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_05_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_05]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_05]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_05]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_05] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_05]  = SysTask.LockMode[GLASS_05] ;
                        SysTask.Lock_OffTime[GLASS_05] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_05]  = SysTask.LockMode[GLASS_05] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_05_PIN);
                    SysTask.Lock_OffTime[GLASS_05]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_05]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_05_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_05_PIN);
                    SysTask.Lock_State[GLASS_05]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_06_Task(void)
{
    switch (SysTask.Lock_State[GLASS_06] ) //_06��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_06]  != SysTask.LockModeSave[GLASS_06] )
            {
                if (SysTask.LockMode[GLASS_06]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_06]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_06]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_06_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_06]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_06]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_06]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_06] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_06]  = SysTask.LockMode[GLASS_06] ;
                        SysTask.Lock_OffTime[GLASS_06] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_06]  = SysTask.LockMode[GLASS_06] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_06_PIN);
                    SysTask.Lock_OffTime[GLASS_06]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_06]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_06_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_06_PIN);
                    SysTask.Lock_State[GLASS_06]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_07_Task(void)
{
    switch (SysTask.Lock_State[GLASS_07] ) //_07��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_07]  != SysTask.LockModeSave[GLASS_07] )
            {
                if (SysTask.LockMode[GLASS_07]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_07]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_07]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_07_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_07]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_07]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_07]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_07] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_07]  = SysTask.LockMode[GLASS_07] ;
                        SysTask.Lock_OffTime[GLASS_07] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_07]  = SysTask.LockMode[GLASS_07] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_07_PIN);
                    SysTask.Lock_OffTime[GLASS_07]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_07]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_07_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_07_PIN);
                    SysTask.Lock_State[GLASS_07]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_08_Task(void)
{
    switch (SysTask.Lock_State[GLASS_08] ) //_08��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_08]  != SysTask.LockModeSave[GLASS_08] )
            {
                if (SysTask.LockMode[GLASS_08]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_08]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_08]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_08_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_08]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_08]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_08]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_08] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_08]  = SysTask.LockMode[GLASS_08] ;
                        SysTask.Lock_OffTime[GLASS_08] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_08]  = SysTask.LockMode[GLASS_08] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_08_PIN);
                    SysTask.Lock_OffTime[GLASS_08]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_08]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_08_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_08_PIN);
                    SysTask.Lock_State[GLASS_08]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_09_Task(void)
{
    switch (SysTask.Lock_State[GLASS_09] ) //_09��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_09]  != SysTask.LockModeSave[GLASS_09] )
            {
                if (SysTask.LockMode[GLASS_09]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_09]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_09]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_09_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_09]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_09]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_09]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_09] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_09]  = SysTask.LockMode[GLASS_09] ;
                        SysTask.Lock_OffTime[GLASS_09] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_09]  = SysTask.LockMode[GLASS_09] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_09_PIN);
                    SysTask.Lock_OffTime[GLASS_09]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_09]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_09_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_09_PIN);
                    SysTask.Lock_State[GLASS_09]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_10_Task(void)
{
    switch (SysTask.Lock_State[GLASS_10] ) //_10��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_10]  != SysTask.LockModeSave[GLASS_10] )
            {
                if (SysTask.LockMode[GLASS_10]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_10]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_10]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_10_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_10]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_10]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_10]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_10] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_10]  = SysTask.LockMode[GLASS_10] ;
                        SysTask.Lock_OffTime[GLASS_10] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_10]  = SysTask.LockMode[GLASS_10] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_10_PIN);
                    SysTask.Lock_OffTime[GLASS_10]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_10]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_10_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_10_PIN);
                    SysTask.Lock_State[GLASS_10]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_11_Task(void)
{
    switch (SysTask.Lock_State[GLASS_11] ) //_11��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_11]  != SysTask.LockModeSave[GLASS_11] )
            {
                if (SysTask.LockMode[GLASS_11]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_11]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_11]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_11_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_11]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_11]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_11]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_11] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_11]  = SysTask.LockMode[GLASS_11] ;
                        SysTask.Lock_OffTime[GLASS_11] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_11]  = SysTask.LockMode[GLASS_11] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_11_PIN);
                    SysTask.Lock_OffTime[GLASS_11]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_11]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_11_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_11_PIN);
                    SysTask.Lock_State[GLASS_11]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_12_Task(void)
{
    switch (SysTask.Lock_State[GLASS_12] ) //_12��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_12]  != SysTask.LockModeSave[GLASS_12] )
            {
                if (SysTask.LockMode[GLASS_12]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_12]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_12]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_12_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_12]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_12]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_12]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_12] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_12]  = SysTask.LockMode[GLASS_12] ;
                        SysTask.Lock_OffTime[GLASS_12] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_12]  = SysTask.LockMode[GLASS_12] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_12_PIN);
                    SysTask.Lock_OffTime[GLASS_12]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_12]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_12_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_12_PIN);
                    SysTask.Lock_State[GLASS_12]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_13_Task(void)
{
    switch (SysTask.Lock_State[GLASS_13] ) //_13��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_13]  != SysTask.LockModeSave[GLASS_13] )
            {
                if (SysTask.LockMode[GLASS_13]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_13]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_13]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_13_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_13]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_13]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_13]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_13] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_13]  = SysTask.LockMode[GLASS_13] ;
                        SysTask.Lock_OffTime[GLASS_13] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_13]  = SysTask.LockMode[GLASS_13] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_13_PIN);
                    SysTask.Lock_OffTime[GLASS_13]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_13]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_13_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_13_PIN);
                    SysTask.Lock_State[GLASS_13]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_14_Task(void)
{
    switch (SysTask.Lock_State[GLASS_14] ) //_14��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_14]  != SysTask.LockModeSave[GLASS_14] )
            {
                if (SysTask.LockMode[GLASS_14]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_14]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_14]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_14_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_14]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_14]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_14]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_14] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_14]  = SysTask.LockMode[GLASS_14] ;
                        SysTask.Lock_OffTime[GLASS_14] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_14]  = SysTask.LockMode[GLASS_14] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_14_PIN);
                    SysTask.Lock_OffTime[GLASS_14]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_14]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_14_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_14_PIN);
                    SysTask.Lock_State[GLASS_14]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_15_Task(void)
{
    switch (SysTask.Lock_State[GLASS_15] ) //_15��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_15]  != SysTask.LockModeSave[GLASS_15] )
            {
                if (SysTask.LockMode[GLASS_15]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_15]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_15]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_15_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_15]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_15]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_15]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_15] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_15]  = SysTask.LockMode[GLASS_15] ;
                        SysTask.Lock_OffTime[GLASS_15] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_15]  = SysTask.LockMode[GLASS_15] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_15_PIN);
                    SysTask.Lock_OffTime[GLASS_15]  = 0;
                }
            }
            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_15]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_15_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_15_PIN);
                    SysTask.Lock_State[GLASS_15]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void Lock_16_Task(void)
{
    switch (SysTask.Lock_State[GLASS_16] ) //_16��
    {
        case LOCK_STATE_DETECT:
            if (SysTask.LockMode[GLASS_16]  != SysTask.LockModeSave[GLASS_16] )
            {
                if (SysTask.LockMode[GLASS_16]  == LOCK_ON)
                {
                    if (SysTask.Moto_WaitTime[GLASS_16]  != 0) //��������ֹͣ״̬
                    {
                        SysTask.Moto_WaitTime[GLASS_16]  = 0;
                    }

                    if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_16_PIN) == 0) // //��⵽��� ����
                    {
                        SysTask.Lock_StateTime[GLASS_16]  = MOTO_DEBOUNCE; //��ʱ40ms�����е�
                        SysTask.Moto_WaitTime[GLASS_16]  = MOTO_DEBOUNCE;
                        SysTask.Lock_State[GLASS_16]  = LOCK_STATE_DEBOUNSE;
                        SysTask.Moto_State[GLASS_16] = MOTO_STATE_STOP; //ֹͣת��
                        SysTask.LockModeSave[GLASS_16]  = SysTask.LockMode[GLASS_16] ;
                        SysTask.Lock_OffTime[GLASS_16] = LOCK_OFFTIME;
                    }
                }
                else 
                {
                    SysTask.LockModeSave[GLASS_16]  = SysTask.LockMode[GLASS_16] ;
                    GPIO_ResetBits(LOCK_GPIO, LOCK_16_PIN);
                    SysTask.Lock_OffTime[GLASS_16]  = 0;
                }
            }

            break;

        case LOCK_STATE_DEBOUNSE:
            if (SysTask.Lock_StateTime[GLASS_16]  == 0)
            {
                if (GPIO_ReadInputDataBit(LOCK_ZERO_GPIO, LOCK_ZERO_16_PIN) == 0) // //��⵽��� ����
                {
                    GPIO_SetBits(LOCK_GPIO, LOCK_16_PIN);
                    SysTask.Lock_State[GLASS_16]  = LOCK_STATE_DETECT;
                }
            }

            break;

        default:
            break;
    }
}



/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/

void MainTask(void)
{
	LedTask();
	SendTabletTask();						//���͸�ƽ������
	TabletToHostTask(); 					//����ƽ������������
	HostToTabletTask(); 					//�������͸��ӻ�������������
	FingerTouchTask();								//ָ��ģ��
	Moto_01_Task();
    Moto_02_Task();
    Moto_03_Task();
    Moto_04_Task();
    Moto_05_Task();
    Moto_06_Task();
    Moto_07_Task();
    Moto_08_Task();
    Moto_09_Task();
    Moto_11_Task();
    Moto_12_Task();
    Moto_13_Task();
    Moto_14_Task();
    Moto_15_Task();
    Moto_16_Task();

    
    Lock_01_Task();
    Lock_02_Task();
    Lock_03_Task();
    Lock_04_Task();
    Lock_05_Task();
    Lock_06_Task();
    Lock_07_Task();
    Lock_08_Task();
    Lock_09_Task();
    Lock_10_Task();
    Lock_11_Task();
    Lock_12_Task();
    Lock_13_Task();
    Lock_14_Task();
    Lock_15_Task();
    Lock_16_Task();
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void SysSaveData(void)
{
    static u8 u8Status = 0;
    static u8 u8count = 0;
    u8 i;

    switch(u8Status)
    {
        case SAVE_INIT:
            memset(SysTask.u16FlashData, 0, sizeof(SysTask.u16FlashData)); //��0
            memcpy(SysTask.u16FlashData, SysTask.WatchState, sizeof(SysTask.WatchState));
            SysTask.u16FlashData[DATA_READ_CNT] = SysTask.LED_Mode;
            u8Status = SAVE_WRITE;
            break;

        case SAVE_WRITE:
            if(!FLASH_WriteMoreData(g_au32DataSaveAddr[SysTask.u16AddrOffset], SysTask.u16FlashData, sizeof(SysTask.u16FlashData)))
            {
                u8count++;
                u8Status = SAVE_WAIT;
                SysTask.u16SaveWaitTick = 100;// 100ms ��ʱ������д��
            }
            else
            {
                u8Status = SAVE_EXIT;
            }
            break;

        case SAVE_WAIT:

            if(SysTask.u16SaveWaitTick == 0)
            {
                u8Status = SAVE_WRITE;
                if(u8count++ >= 7) //ÿҳ����8��
                {
                    if(++SysTask.u16AddrOffset < ADDR_MAX_OFFSET) //��5ҳ
                    {
                        for(i = 0; i < 3; i++)  //��д8�����
                        {
                            if(!FLASH_WriteMoreData(g_au32DataSaveAddr[0], &SysTask.u16AddrOffset, 1))
                                break;
                            delay_ms(20);   
                        } 
                    }
                    else
                        {
                        u8Status = SAVE_EXIT;
                    }
                }
            }
            break;
            
        case SAVE_EXIT: 
            u8count = 0;
            break;
        default: 
            u8Status = SAVE_EXIT;
            break;

    }
}

/*******************************************************************************
* ����: 
* ����: 
* �β�:		
* ����: ��
* ˵��: 
*******************************************************************************/
void SysInit(void)
{
    u8 i, j;

    
    SysTask.u16AddrOffset = FLASH_ReadHalfWord(g_au32DataSaveAddr[0]); //��ȡ
	if((SysTask.u16AddrOffset == 0xffff) || (SysTask.u16AddrOffset >= ADDR_MAX_OFFSET ) || (SysTask.u16AddrOffset == 0 ))  //��ʼ״̬ flash��0xffff
    {
        SysTask.u16AddrOffset = 1;
        
        memset(SysTask.u16FlashData, 0, sizeof(SysTask.u16FlashData));

        for(i = 0; i < 8; i++)  //��д8�����
        {
            if(!FLASH_WriteMoreData(g_au32DataSaveAddr[0], &SysTask.u16AddrOffset, 1))
                break;
            delay_ms(20);   
        }


        for(i = 0; i < GLASS_MAX_CNT; i++)
        {
            SysTask.WatchState[i].u8LockState  = 0;
            SysTask.WatchState[i].u8Dir        = MOTO_FR_FWD;
            SysTask.WatchState[i].MotoTime     = MOTO_TIME_650;
        }
        memcpy(SysTask.u16FlashData, SysTask.WatchState, sizeof(SysTask.WatchState));
        SysTask.u16FlashData[DATA_READ_CNT]    = LED_MODE_ON; //LEDģʽ
    
        FLASH_WriteMoreData(g_au32DataSaveAddr[SysTask.u16AddrOffset], SysTask.u16FlashData, sizeof(SysTask.u16FlashData));
    }   
    
	FLASH_ReadMoreData(g_au32DataSaveAddr[SysTask.u16AddrOffset], SysTask.u16FlashData, sizeof(SysTask.u16FlashData));

   
    for(i = 0; i < DATA_READ_CNT; i++)
    {
        if(SysTask.u16FlashData[i] == 0xFFFF)  //������FLASH�д�
        {
            for(j = 0; j < GLASS_MAX_CNT; j++)
            {
                SysTask.WatchState[j].u8LockState  = 0;
                SysTask.WatchState[j].u8Dir        = MOTO_FR_FWD;
                SysTask.WatchState[j].MotoTime     = MOTO_TIME_650;
            }
            SysTask.u16FlashData[DATA_READ_CNT]    = LED_MODE_ON; //LEDģʽ
            memcpy(SysTask.u16FlashData, SysTask.WatchState, sizeof(SysTask.WatchState));
            break;
        }
        
    }
    
    memcpy(SysTask.WatchState, SysTask.u16FlashData, sizeof(SysTask.WatchState));

    SysTask.LED_Mode            = SysTask.u16FlashData[DATA_READ_CNT];
    if((SysTask.LED_Mode != LED_MODE_ON) && (SysTask.LED_Mode != LED_MODE_OFF))
    {
        SysTask.LED_Mode = LED_MODE_ON; //LEDģʽ
    }
    
    for(i = 0; i < GLASS_MAX_CNT; i++)
    {
        SysTask.Moto_Mode[i]        =  (MotoFR)(SysTask.WatchState[i].u8Dir);
        SysTask.Moto_Time[i]        =  (MotoTime_e)(SysTask.WatchState[i].MotoTime);
        SysTask.Moto_RunTime[i]     =  g_au16MoteTime[SysTask.Moto_Time[i]][1];
        SysTask.Moto_WaitTime[i]    =  g_au16MoteTime[SysTask.Moto_Time[i]][2];
        SysTask.LockMode[i]         =  LOCK_OFF;  //��״̬���ڴ˸��£���Ϊ������Ҫ����֮��
    }
    
    
	SysTask.TouchState	        = TOUCH_MATCH_INIT;
	SysTask.TouchSub	        = TOUCH_SUB_INIT;
    SysTask.bEnFingerTouch      = FALSE;
	SysTask.nWaitTime	        = 0;
	SysTask.nTick		        = 0;
    SysTask.u16SaveTick         = 0;

	SysTask.SendState	        = SEND_INIT;
	SysTask.SendSubState        = SEND_SUB_INIT;
    SysTask.bTabReady           = FALSE;
    SysTask.LED_ModeSave        = LED_MODE_DEF;  // ��֤��һ�ο�������ģʽ



    for(i = 0; i < GLASS_MAX_CNT; i++)
    {
	    SysTask.Moto_StateTime[i]          = 0;
        SysTask.Moto_State[i]	           = MOTO_STATE_INIT;//��֤����������ת
        SysTask.Moto_SubState[i]	       = MOTO_SUB_STATE_RUN;
    	SysTask.Lock_StateTime[i]          = 0;			//        
    }
//    SysTask.u16BootTime = 3000;   //3��ȴ��ӻ�������ȫ
}


