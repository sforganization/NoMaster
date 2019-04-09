

#ifndef __SYSCOMMENT_H__
#define __SYSCOMMENT_H__

#ifdef _MAININC_
#define EXTERN

#else

#define EXTERN					extern
#endif


/* 头文件  ------------------------------------------------------------*/
#include "stm32f10x.h"



#define SAVE_TIME           10000    //10秒无动作自动保存现在的状态信息
#define DATA_READ_CNT       24       //按16 bit存储，16个手表共占用 48 字节 所以取24
#define LED_MODE_CNT        1        //LED模式 按16字节存储 会存在截断现象
#define PASSWD_CNT          3        //保存密码个数 按16字节存储   6位密码占用3

#define LOCK_OFFTIME        4000
#define SAVE_TICK_TIME      30      //有数据更新30S后写入


#define     FINGER_MAX_CNT          10    //允许最大指纹用户个数
#define     GLASS_SEND_ALL          0X80  //最高位为1表示所有的手表
#define     GLASS_MAX_CNT           16    //手表总数
#define     MCU_ACK_ADDR         0xFF  //应答地址
#define     FINGER_ACK_TYPE         0x01  //指纹应答地址
#define     ZERO_ACK_TYPE         0x02  //零点应答地址
#define     ERROR_ACK_TYPE         0xFF  //其他应答


#define     LOCK_OFF                1
#define     LOCK_ON                 0

#define     GLASS_01                 0
#define     GLASS_02                 1
#define     GLASS_03                 2
#define     GLASS_04                 3
#define     GLASS_05                 4
#define     GLASS_06                 5
#define     GLASS_07                 6
#define     GLASS_08                 7
#define     GLASS_09                 8
#define     GLASS_10                 9
#define     GLASS_11                 10
#define     GLASS_12                 11
#define     GLASS_13                 12
#define     GLASS_14                 13
#define     GLASS_15                 14
#define     GLASS_16                 15




#define FLASH_SAVE_ADDR         0X0800C004 	    //设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
										        //否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
                                                //512k 共255页，在250页保存的是在哪个页保存数据信息（251~255）共5页，
                                                //如果251页flash失效则保存到252页，依次类推，大概可以写10k * 5 = 50k次
                                                //按每天修改10次计算最保守可以用13年
										
/*
*  因为数据存储经常擦除，所以写入与读出不符合时表明flash已经坏
*/
	

/*
		页		起始地址					    结束地址
		255		0x0807F800		~			0X0807FFFF
		254		0x0807F000		~			0X0807F7FF
		253		0x0807E800		~			0X0807EFFF
		252		0x0807E000		~			0X0807F7FF
		251		0x0807D800		~			0X0807DFFF
		250		0x0807D000		~			0X0807F7FF
    
*/

#define     DATA_SAVE_ADDR      0x0807D000
#define		DATA_SAVE_SAVE_1	0x0807F800
#define		DATA_SAVE_SAVE_2	0x0807F000
#define		DATA_SAVE_SAVE_3	0x0807E800
#define		DATA_SAVE_SAVE_4	0x0807E000
#define		DATA_SAVE_SAVE_5	0x0807D800




#define VOID					void	




#define VOID					void	


//联合体由于数据大小端问题--所有变量必须使用字节声明
//设备信息
#define LED_SHOWFRE_DNS 		300
#define LED_SHOWFRE_OK			1000


#define FINGER_MAX_CNT			10   //指纹模块最多存储个数

#define DEBOUNCE_TIME			40 //40ms 防抖
#define MATCH_FINGER_CNT		2

#define ADDR_MAX_OFFSET         6


#define MOTO_DEBOUNCE			4	//40ms 延时到达中点位置



#define LED_RCC_CLOCKCMD             RCC_APB2PeriphClockCmd
#define LED_RCC_CLOCKGPIO            RCC_APB2Periph_GPIOB
#define LED_GPIO                     GPIOB

#define LED_PIN                      GPIO_Pin_13  //LED


#define LOCK_ZERO_RCC_CLOCKCMD              RCC_APB2PeriphClockCmd
#define LOCK_ZERO_RCC_CLOCKGPIO             RCC_APB2Periph_GPIOD
#define LOCK_ZERO_GPIO                      GPIOD

#define LOCK_ZERO_01_PIN                 GPIO_Pin_0
#define LOCK_ZERO_02_PIN                 GPIO_Pin_1
#define LOCK_ZERO_03_PIN                 GPIO_Pin_2
#define LOCK_ZERO_04_PIN                 GPIO_Pin_3
#define LOCK_ZERO_05_PIN                 GPIO_Pin_4
#define LOCK_ZERO_06_PIN                 GPIO_Pin_5
#define LOCK_ZERO_07_PIN                 GPIO_Pin_6
#define LOCK_ZERO_08_PIN                 GPIO_Pin_7
#define LOCK_ZERO_09_PIN                 GPIO_Pin_8
#define LOCK_ZERO_10_PIN                 GPIO_Pin_9
#define LOCK_ZERO_11_PIN                 GPIO_Pin_10
#define LOCK_ZERO_12_PIN                 GPIO_Pin_11
#define LOCK_ZERO_13_PIN                 GPIO_Pin_12
#define LOCK_ZERO_14_PIN                 GPIO_Pin_13
#define LOCK_ZERO_15_PIN                 GPIO_Pin_14
#define LOCK_ZERO_16_PIN                 GPIO_Pin_15

#define MOTO_P_RCC_CLOCKCMD             RCC_APB2PeriphClockCmd
#define MOTO_P_RCC_CLOCKGPIO            RCC_APB2Periph_GPIOE
#define MOTO_P_GPIO                     GPIOE

#define MOTO_01_PIN_P                 GPIO_Pin_0  //PWM+
#define MOTO_02_PIN_P                 GPIO_Pin_1  //PWM+
#define MOTO_03_PIN_P                 GPIO_Pin_2  //PWM+
#define MOTO_04_PIN_P                 GPIO_Pin_3  //PWM+
#define MOTO_05_PIN_P                 GPIO_Pin_4  //PWM+
#define MOTO_06_PIN_P                 GPIO_Pin_5  //PWM+
#define MOTO_07_PIN_P                 GPIO_Pin_6  //PWM+
#define MOTO_08_PIN_P                 GPIO_Pin_7  //PWM+
#define MOTO_09_PIN_P                 GPIO_Pin_8  //PWM+
#define MOTO_10_PIN_P                 GPIO_Pin_9  //PWM+
#define MOTO_11_PIN_P                 GPIO_Pin_10  //PWM+
#define MOTO_12_PIN_P                 GPIO_Pin_11  //PWM+
#define MOTO_13_PIN_P                 GPIO_Pin_12  //PWM+
#define MOTO_14_PIN_P                 GPIO_Pin_13  //PWM+
#define MOTO_15_PIN_P                 GPIO_Pin_14  //PWM+
#define MOTO_16_PIN_P                 GPIO_Pin_15  //PWM+




#define MOTO_N_RCC_CLOCKCMD             RCC_APB2PeriphClockCmd
#define MOTO_N_RCC_CLOCKGPIO            RCC_APB2Periph_GPIOF
#define MOTO_N_GPIO                     GPIOF

#define MOTO_01_PIN_N                 GPIO_Pin_0  //PWM-
#define MOTO_02_PIN_N                 GPIO_Pin_1  //PWM-
#define MOTO_03_PIN_N                 GPIO_Pin_2  //PWM-
#define MOTO_04_PIN_N                 GPIO_Pin_3  //PWM-
#define MOTO_05_PIN_N                 GPIO_Pin_4  //PWM-
#define MOTO_06_PIN_N                 GPIO_Pin_5  //PWM-
#define MOTO_07_PIN_N                 GPIO_Pin_6  //PWM-
#define MOTO_08_PIN_N                 GPIO_Pin_7  //PWM-
#define MOTO_09_PIN_N                 GPIO_Pin_8  //PWM-
#define MOTO_10_PIN_N                 GPIO_Pin_9  //PWM-
#define MOTO_11_PIN_N                 GPIO_Pin_10  //PWM-
#define MOTO_12_PIN_N                 GPIO_Pin_11  //PWM-
#define MOTO_13_PIN_N                 GPIO_Pin_12  //PWM-
#define MOTO_14_PIN_N                 GPIO_Pin_13  //PWM-
#define MOTO_15_PIN_N                 GPIO_Pin_14  //PWM-
#define MOTO_16_PIN_N                 GPIO_Pin_15  //PWM-


#define LOCK_RCC_CLOCKCMD             RCC_APB2PeriphClockCmd
#define LOCK_RCC_CLOCKGPIO            RCC_APB2Periph_GPIOG
#define LOCK_GPIO                     GPIOG
         
#define LOCK_01_PIN                 GPIO_Pin_0  //锁
#define LOCK_02_PIN                 GPIO_Pin_1  //锁
#define LOCK_03_PIN                 GPIO_Pin_2  //锁
#define LOCK_04_PIN                 GPIO_Pin_3  //锁
#define LOCK_05_PIN                 GPIO_Pin_4  //锁
#define LOCK_06_PIN                 GPIO_Pin_5  //锁
#define LOCK_07_PIN                 GPIO_Pin_6  //锁
#define LOCK_08_PIN                 GPIO_Pin_7  //锁
#define LOCK_09_PIN                 GPIO_Pin_8  //锁
#define LOCK_10_PIN                 GPIO_Pin_9  //锁
#define LOCK_11_PIN                 GPIO_Pin_10  //锁
#define LOCK_12_PIN                 GPIO_Pin_11  //锁
#define LOCK_13_PIN                 GPIO_Pin_12  //锁
#define LOCK_14_PIN                 GPIO_Pin_13  //锁
#define LOCK_15_PIN                 GPIO_Pin_14  //锁
#define LOCK_16_PIN                 GPIO_Pin_15  //锁


#define PASSWD_COUNT			0x6   //密码位数 6位
#define REMOTE_SHOW_TIME		20   //接收到遥控 led显示时间，超时则退出
#define MOTO_ZERO_DETECT		20   //零点检测，最长时间，检测不到直接退出 ,实测全速一圈为10S左右

//    static u16 u16WriteCount = 0;  //重写次数
//    static u16 u16EraseCount = 0;  //重擦写次数
//                FLASH_ReadMoreData(FLASH_SAVE_ADDR, readData, 8);
//                FLASH_WriteMoreData(FLASH_SAVE_ADDR, writeData, 8);
//                FLASH_ReadMoreData(FLASH_SAVE_ADDR, readData, 8);             





typedef enum 
{
    FALSE = 0, TRUE = !FALSE
} bool;
  
typedef enum 
{
    LED_MODE_ON = 0,    //常开 
    LED_MODE_OFF,
    LED_MODE_DEF = 0XFF, 
} LEDMode_T;

    
typedef enum 
{
   TOUCH_MATCH_INIT = 0, 
   TOUCH_MATCH_AGAIN, //再次匹配
   TOUCH_ADD_USER, //增加用户
   TOUCH_DEL_USER, //删除用户
   TOUCH_WAIT,
   TOUCH_IDLE,
   TOUCH_DEF = 0XFF, 
} TouchState_T;


typedef enum 
{
    TOUCH_SUB_INIT = 0, 
    TOUCH_SUB_ENTER,  //
    TOUCH_SUB_AGAIN,  //
    TOUCH_SUB_WAIT, 
    TOUCH_SUB_DEF = 0XFF,
} TouchSub_T;    



typedef enum 
{
    MOTO_TIME_TPD = 0, //每天的动作模式，工作12小时，停止12小时
    MOTO_TIME_650, //旋转2分钟，停止942S
    MOTO_TIME_750,  //旋转2分钟，停止800S
    MOTO_TIME_850,  //旋转2分钟，停止693S
    MOTO_TIME_1000, //旋转2分钟，停止570S
    MOTO_TIME_1950, //旋转2分钟，停止234S

    
    MOTO_TIME_OFF = 0XFE,  //开锁状态下，电机停止

    MOTO_TIME_DEF = 0XFF,
}MotoTime_e;  
    

typedef enum 
{
    MOTO_FR_FWD = 0,  //正转
    MOTO_FR_REV,      //反转
    MOTO_FR_FWD_REV, //正反转
    MOTO_FR_STOP, //停止
    MOTO_FR_DEF = 0XFF,
}MotoFR;  

typedef enum 
{
    SEND_INIT = 0,     // 初始状态为查询主机下的各从机状态   从机的状态只保存在各个从机中
    SEND_TABLET_ALL,         //发送全部从机给平板
    SEND_TABLET_FINGER,      //发送指纹模块应答给平板
    SEND_TABLET_ZERO,//发送零点应答给平板
    SEND_WAIT,         //等待状态
    SEND_IDLE,         //空闲状态
    SEND_DEF = 0XFF,
}SendState_T;  


typedef enum 
{
    SEND_SUB_INIT = 0,     // 初始状态为查询主机下的各从机状态   从机的状态只保存在各个从机中
    SEND_SUB_IDLE,         //空闲状态
    SEND_SUB_WAIT,         //等待状态 ，等待从机应答
    SEND_SUB_DEF = 0XFF,
}SendSubState_T; 

typedef enum 
{
    HtoS_CHECK = 0,     // host to slave init
    HtoS_SEND,         //发送状态
    HtoS_WAIT,         //等待状态 ，等待一段时间，再发送，给时间从机响应
    HtoS_DEF = 0XFF,
}HtoS_State_T; 
    
    
typedef enum SAVE
{
    SAVE_INIT = 0, 
    SAVE_WRITE,
    SAVE_WAIT,
    SAVE_EXIT,
    SAVE_DEF = 0XFF, 
} Save_T;
   

typedef enum 
{
    CMD_GET_ATTR = 0,       // 获取属性    0
    CMD_ACK,                // 应答1
    CMD_UPDATE,             // 更新包2
    CMD_READY,              // 就绪包3
    CMD_ADD_USER,              // 增加用户指纹4
    CMD_DEL_USER,              // 删除用户指纹5
    CMD_LOCK_MODE,             // 开锁或者运行状态6
    CMD_empt,             // 保留命令
    CMD_LED_MODE,              // LED模式7
    CMD_PASSWD_ON,           // 密码模式，开启指纹8
    CMD_PASSWD_OFF,           // 密码模式，关闭指纹9
    
    CMD_DEF = 0XFF, 
}SendCmd_T; 

typedef struct WATCH_STATE
{
    u8 u8LockState; //锁状态,不用上传，但要下载，因为开锁后一段时间是自动关闭，在平板端处理就可以
    u8 u8Dir; //转动方向
    u8 MotoTime; //转动时间      MotoTime_e  联合体类型
}WatchState_t;  //手表状态


//typedef union SENDARRAY
//{
//    u8 u8SendArray[96];
//    SlaveState_t SlaveState[4];
//}SendArrayUnion_u; //发送数组联合体

typedef enum 
{
    MOTO_STATE_INIT = 0,  //
    MOTO_STATE_CHANGE_TIME,  //
    MOTO_STATE_CHANGE_DIR,  //
    MOTO_STATE_RUN_NOR,  // 正转或者反转运行状态
    MOTO_STATE_RUN_CHA,  // 正反转运行状态
    MOTO_STATE_STOP,
    MOTO_STATE_WAIT,  //
    MOTO_STATE_IDLE,  //
    MOTO_STATE_DEF = 0XFF,
}MotoState;  
    
typedef enum 
{
    MOTO_SUB_STATE_RUN = 0,  //
    MOTO_SUB_STATE_WAIT,  //
    MOTO_SUB_STATE_DEF = 0XFF,
}MotoSubState;  
    
typedef enum 
{
    LOCK_STATE_DETECT = 0,  //
    LOCK_STATE_DEBOUNSE,  //防抖
    LOCK_STATE_SENT,  //开锁，发送3个50ms低电平
    LOCK_STATE_OFF,  //完全掉电操作
    LOCK_STATE_DEF = 0XFF,
}LockState_T;  
    
typedef union SENDARRAY
{
    u8 u8SendArray[48];
    WatchState_t    WatchState[16];                     //最大16个手表
}SendArrayUnion_u; //发送数组联合体

typedef struct   
{
    WatchState_t    WatchState[GLASS_MAX_CNT];
    u16             u16LedMode;
    u8              u8Passwd[PASSWD_CNT * 2];
}PackData_t;

typedef union   PACK_U
{
    PackData_t      Data;
    u8              u8FlashData[(DATA_READ_CNT + LED_MODE_CNT + PASSWD_CNT) * 2];
    u16             u16FlashData[DATA_READ_CNT + LED_MODE_CNT + PASSWD_CNT];       //因为Flash读取为16个bit为单位，后续再转换注意字节序还有大小端的问题
}FlashData_u;

typedef struct 
{
    bool			mUpdate;

    vu16			nTick;								//节拍器
    vu16			nLoadTime;							//显示加载时间

    vu32 			nWaitTime;							//状态机延时
    TouchState_T	TouchState;
    TouchSub_T	    TouchSub;
    bool 			bEnFingerTouch;						//全能指纹识别功能
	u16             u16FingerID; 					    //指纹模版ID

    SendState_T     SendState;
    SendSubState_T  SendSubState;
    u8              u8WatchCount;
    u16             u16AddrOffset;
    u16             u16SaveTick;                        //延时保存状态信息时间
    u16             u16SaveWaitTick;                    //保存状态机延时
    bool 			bEnSaveData;						//s全能保存
    
    WatchState_t    WatchState[GLASS_MAX_CNT];          //最大16个手表
    WatchState_t    WatchStateSave[GLASS_MAX_CNT];      //最大16个手表
    FlashData_u     FlashData;
    
    bool 			bTabReady;							//平板上电准备就绪
    u8              u8Passwd[PASSWD_CNT * 2];           //密码保存

    u8              u8LedMode;                          //LED模式
    u8              u8LedModeSave;                      //保存LED模式

    vu32            Moto_StateTime[GLASS_MAX_CNT];      //状态机延时
    MotoState       Moto_State[GLASS_MAX_CNT];          //电机运行状态
    MotoSubState    Moto_SubState[GLASS_MAX_CNT];       //电机运行子状态
    MotoFR          Moto_Mode[GLASS_MAX_CNT];           //电机模式
    MotoFR          Moto_ModeSave[GLASS_MAX_CNT];       //电机模式保存
    MotoTime_e      Moto_Time[GLASS_MAX_CNT];           //电机运行时间枚举
    vu32 			Moto_RunTime[GLASS_MAX_CNT];		//电机运行时间
    vu32 			Moto_WaitTime[GLASS_MAX_CNT];		//电机停止时间
    LockState_T     Lock_State[GLASS_MAX_CNT];          //锁状态机
    u8              LockMode[GLASS_MAX_CNT];            // 锁状态 1 开 or 0 关
    u8              LockModeSave[GLASS_MAX_CNT];        // 锁状态 1 开 or 0 关
    vu32 			Lock_StateTime[GLASS_MAX_CNT];		//状态机延时
    vu32            Lock_OffTime[GLASS_MAX_CNT];        //锁关闭时间

    
    u8              LED_Mode;                           // LED模式
    u8              LED_ModeSave;                       // LED模式
} SYS_TASK;


/* 全局变量 -----------------------------------------------------------*/

/* 全局变量 -----------------------------------------------------------*/
EXTERN vu8		mSysIWDGDog; //软狗标记
EXTERN vu32 	mSysSoftDog; //软狗计数器 
EXTERN vu16 	mSysTick; //节拍器
EXTERN vu16 	mSysSec; //节拍器
EXTERN vu16 	mTimeRFRX; //接收间隔-仿真

EXTERN SYS_TASK SysTask;


EXTERN void Sys_DelayMS(uint16_t nms);
EXTERN void Sys_GetMac(u8 * mac);
EXTERN void Sys_LayerInit(void);
EXTERN void Sys_IWDGConfig(u16 time);
EXTERN void Sys_IWDGReloadCounter(void);
EXTERN void Sys_1s_Tick(void);

EXTERN void DelayUs(uint16_t nCount);
EXTERN void DelayMs(uint16_t nCount);
EXTERN void Strcpy(u8 * str1, u8 * str2, u8 len);
EXTERN bool Strcmp(u8 * str1, u8 * str2, u8 len);

EXTERN void MainTask(void);
EXTERN void SysInit(void);
EXTERN void SysSaveData(void);


#endif

