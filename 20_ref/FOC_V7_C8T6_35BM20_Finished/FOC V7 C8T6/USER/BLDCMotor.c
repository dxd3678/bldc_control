#include "BLDCmotor.h"
#include "FOCMotor.h"
#include "foc_utils.h"
#include "gpio.h"
#include <stdio.h>
#include "MagneticSensor.h" 
#include "tim.h"
#include "FlashStorage.h"
#include "Kalman.h"
#include "pid.h"
#include "math.h"
#include <stdlib.h>
#include "adc.h"
#include "CurrentSense.h"
#include "can.h"
#include "OLED_H.h"


// 定义启用和禁用电机的宏，通过GPIO操作实现电机的启停控制
#define M1_Enable 	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET)    // 定义启用电机的宏，通过设置GPIOA的第4脚为高电平
#define M1_Disable 	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET)  // 定义禁用电机的宏，通过设置GPIOA的第4脚为低电平
#define PWM_Period 1280  // 定义PWM周期


extern float target;  // 声明一个外部变量，用于接收目标值（如速度或位置）
extern float Temperature;
long sensor_direction;  // 传感器方向
float voltage_power_supply;  // 电源电压
float voltage_limit;  // 电压限制
float voltage_sensor_align;  // 传感器对齐时使用的电压
int pole_pairs;  // 极对数
unsigned long open_loop_timestamp;  // 开环控制的时间戳
float velocity_limit;  // 速度限制
int pattern=0;  // 模式标志
int Gear_Switch_Number=4;  // 齿轮切换数量
float Damping_Vale=0.1;  // 阻尼系数
float shaft_angle_1090;  // 轴角度

int alignSensor(void);  // 声明传感器校准函数
float velocityOpenloop(float target_velocity);  // 开环速度控制函数
float angleOpenloop(float target_angle);  // 开环角度控制函数



int Music_T_L[12] = {3822, 3608, 3405, 3214, 3034, 2863, 2703, 2551, 2408, 2272, 2145, 2025}; // 低八度
int Music_T_M[12] = {1911, 1804, 1703, 1607, 1517, 1432, 1351, 1276, 1204, 1136, 1073, 1012}; // 中八度
int Music_T_H[12] = {956, 902, 851, 804, 758, 716, 676, 638, 602, 568, 536, 506}; // 高八度


// 低八度、中八度、高八度的音符频率常量
#define NOTE_L1 3822  // 低八度 1
#define NOTE_L2 3608  // 低八度 2
#define NOTE_L3 3405  // 低八度 3
#define NOTE_L4 3214  // 低八度 4
#define NOTE_L5 3034  // 低八度 5
#define NOTE_L6 2863  // 低八度 6
#define NOTE_L7 2703  // 低八度 7

#define NOTE_M1 1911  // 中八度 1
#define NOTE_M2 1804  // 中八度 2
#define NOTE_M3 1703  // 中八度 3
#define NOTE_M4 1607  // 中八度 4
#define NOTE_M5 1517  // 中八度 5
#define NOTE_M6 1432  // 中八度 6
#define NOTE_M7 1351  // 中八度 7

#define NOTE_H1 956   // 高八度 1
#define NOTE_H2 902   // 高八度 2
#define NOTE_H3 851   // 高八度 3
#define NOTE_H4 804   // 高八度 4
#define NOTE_H5 758   // 高八度 5
#define NOTE_H6 716   // 高八度 6
#define NOTE_H7 676   // 高八度 7

#define REST  0       // 休止符

extern const int hetangyeushe_music[][3];

void Beep_PlayNotes(uint8_t num, uint16_t notes[][2]);
void Beep_Play(uint16_t period);
/******************************************************************************/
void Motor_init(void)
{
	printf("MOT: Init\r\n");  // 打印初始化信息
	
	if(voltage_sensor_align > voltage_limit)
		voltage_sensor_align = voltage_limit;  // 如果校准电压超过电压限制，则将校准电压限制为电压上限
	
	pole_pairs=7;  // 默认极对数设为7
	sensor_direction=UNKNOWN;  // 默认传感器方向未知
	M1_Enable;  // 启动电机
	printf("MOT: Enable driver.\r\n");  // 打印启动电机驱动的信息
}
/******************************************************************************/
void Motor_initFOC(void)
{

	
	if((Flash_ReadMotorParam(&pole_pairs, &zero_electric_angle, (int*)&sensor_direction) == -1 )|| (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) ) //尝试读取Flash数据
	{
		if(alignSensor()) // 如果传感器校准成功
		{
			Flash_SaveMotorParam(pole_pairs, zero_electric_angle, sensor_direction); // 将校准后的参数保存到Flash
		}
	}
	
	// 更新轴角度
	angle_prev=getAngle();  // 获取当前角度作为先前角度
	HAL_Delay(5);
	shaft_angle = shaftAngle();  // 获取当前轴角度
	
	HAL_Delay(200);
}
/******************************************************************************/
int alignSensor(void)
{
	long i;
	float angle;
	float mid_angle,end_angle;
	float moved;
	
	printf("MOT: Align sensor.\r\n");  // 打印传感器校准开始的信息
	
	// 寻找自然方向
	// 向前移动一个电气转动周期


	for(i=0; i<=500; i++)
	{
		angle = _3PI_2 + _2PI * i / 500.0;  // 计算角度
		setPhaseVoltage(voltage_sensor_align/3, 0,  angle);  // 设置相应的相电压
		HAL_Delay(2);
	}
	mid_angle=getAngle();  // 获取中间角度
	
	for(i=500; i>=0; i--) 
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(voltage_sensor_align/3, 0,  angle);
		HAL_Delay(2);
	}
	end_angle=getAngle();  // 获取结束角度
	setPhaseVoltage(0, 0, 0);  // 关闭相电压
	HAL_Delay(200);

	

	
	printf("mid_angle=%.4f\r\n",mid_angle);
	printf("end_angle=%.4f\r\n",end_angle);
	
	moved =  fabs(mid_angle - end_angle);  // 计算移动的角度差
	if((mid_angle == end_angle)||(moved < 0.02))  // 如果中间角度和结束角度相同或几乎没有移动
	{
		printf("MOT: Failed to notice movement loop222.\r\n");
		M1_Disable;  // 如果没有检测到移动，禁用电机
		return 0;
	}
	else if(mid_angle < end_angle)
	{
		printf("MOT: sensor_direction==CCW\r\n");  // 传感器方向为逆时针
		sensor_direction=CCW;
	}
	else
	{
		printf("MOT: sensor_direction==CW\r\n");  // 传感器方向为顺时针
		sensor_direction=CW;
	}
	
	printf("MOT: PP check: ");  // 检查极对数
	if( fabs(moved*pole_pairs - _2PI) > 0.5 )  // 如果计算出的极对数与实际极对数的误差较大
	{
		printf("fail - estimated pp:");
		pole_pairs=_2PI/moved+0.5;  // 重新计算极对数，四舍五入
		printf("%d\r\n",pole_pairs);
	}
	else
		printf("OK!\r\n");
	
	setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);  // 设置零点偏移角度的相电压
	HAL_Delay(700);
	zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*getAngle(), pole_pairs));  // 计算并归一化电气角度
	HAL_Delay(20);
	printf("MOT: Zero elec. angle:");
	printf("%.4f\r\n",zero_electric_angle);
	
	setPhaseVoltage(0, 0, 0);  // 关闭相电压
	HAL_Delay(200);
	
	return 1;  // 校准成功，返回1
}


/******************************************************************************/
void loopFOC(void)
{
    // 检查控制器是否设置为角度开环或速度开环，如果是，则直接返回不执行后续代码
    if(controller == Type_angle_openloop || controller == Type_velocity_openloop) return;
    
    // 读取实际的轴角度
    shaft_angle = shaftAngle();
    // 根据轴角度计算电角度，需要先调用shaftAngle函数
    electrical_angle = electricalAngle();
    
    // 根据扭矩控制的类型执行相应的控制策略
    switch(torque_controller)
    {
        case Type_voltage:  // 电压型控制，无需执行特别操作
            break;
        case Type_dc_current:  // 直流电流控制，此处未实现
            break;
        case Type_foc_current:  // FOC电流控制，此处未实现
            break;
        default:  // 未选择有效的扭矩控制类型，输出错误信息
            printf("MOT: no torque control selected!");
            break;
    }
    

    
    // 设置相位电压，FOC的核心功能
    extern uint8_t beepPlaying;
    if(beepPlaying == 0) // 如果不在蜂鸣器播放状态，则正常输出电压
        setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}


// 多级旋钮模式：用于返回目标角度   KEY_NUMBER:需要产生几个挡位；
float get_goal_angle(float KEY_NUMBER)
{								
  float goal_angle_h=0;
  int set_angle_h=0;
	float set_=0;   
	for(int i=0;i<KEY_NUMBER;i++)//循环扫描，根据当前的电机的角度来判断，当前的角度在哪个挡位的范围内
	{		
		set_ = (i/KEY_NUMBER) * _2PI ;
		if(i==0)//因为是单向的多档开关，所以0挡位的时候，只要判断 【当前角度<范围】就行
		{
			if(shaft_angle <= set_+ _2PI/KEY_NUMBER/2)
			{
				set_angle_h=0;break;
			}
		}
		else
		if(i==KEY_NUMBER-1)//最后一档的时候也是，最后一档并不是从2pi开始算的，第六挡是5.24开始算的，所以下面计算六档的范围的时候要特殊处理
							// <-1->	 <-2->     <-3->    <-4->    <-5->   <-6->
							//   [0-----1.05-----2.09-----3.14-----4.19-----5.24-----6.28]   
		{
			if(shaft_angle >= ((KEY_NUMBER-1)/KEY_NUMBER)*_2PI - _2PI/KEY_NUMBER/2 )
			{
				set_angle_h=KEY_NUMBER-1;break;
			}
		}
		else
		{//判断中间2345挡位，因为中间挡位都有左右两侧的范围。
			if(shaft_angle >= set_- _2PI/KEY_NUMBER/2 && shaft_angle <= set_+ _2PI/KEY_NUMBER/2)
			{
				set_angle_h=i;
				break;
			}
		}
	}
	//根据算出来的set_angle_h挡位，除以全挡位的数量，就是一个百分比，把这个百分比乘以2pi就可以得到目标角度。
	goal_angle_h=(float)(set_angle_h/(KEY_NUMBER))*_2PI;
	return goal_angle_h;
}

// 多级旋钮模式2：可以无限旋转   上面的是最高档、最低档限位，这个函数不会产生限位。
float get_goal_angle_two(float KEY_NUMBER)
{		
	int set_angle_h=0;
	float set_angle_h_two=shaft_angle;
	while(set_angle_h_two>_2PI)set_angle_h_two-=_2PI;
	while(set_angle_h_two<0   )set_angle_h_two=_2PI+set_angle_h_two;
	
	float goal_angle_h[200];
	for(int i=0;i<KEY_NUMBER;i++)
	{
		goal_angle_h[i] = (i/KEY_NUMBER) * _2PI ;
	}
	
	for(int i=0;i<KEY_NUMBER;i++)
	{
		if(i==0)
		{
			if(set_angle_h_two >= _2PI- _2PI/KEY_NUMBER/2 )
			{
					set_angle_h=KEY_NUMBER;
					break;			
			}
			else
			if(set_angle_h_two <= goal_angle_h[i]  + _2PI/KEY_NUMBER/2)
			{
					set_angle_h=i;
					break;							
			}
		}

		else
		{
			if(set_angle_h_two >= goal_angle_h[i]- _2PI/KEY_NUMBER/2 && set_angle_h_two <= goal_angle_h[i]  + _2PI/KEY_NUMBER/2)
			{
					set_angle_h=i;
					break;			
			}
		}
	}

	return (float)(set_angle_h/(KEY_NUMBER))*_2PI;

}

extern void Beep_PlayNotes(uint8_t num, uint16_t notes[][2]);

void key_scan()
{
    static int key_count=0,key_lock=0,first_flag=0;
    
    if(first_flag==0)
    {
        if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3))first_flag=1;
        return;
    }
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==0)
    {
        if(key_count<=1000)key_count++;
        if(key_count==1000&&key_lock==0)
        {
            Beep_PlayNotes(3,(uint16_t[][2]){{956,200},{758,200},{638,500}}); // 播放开机音效

            key_lock=0;
            pattern/=10;
            pattern*=10;  // 先把小数去掉
            pattern+=10;  // 增加控制模式
            if(pattern>=80)
            {
                pattern=10;  // 重置控制模式
            }
        }
    }
    else
    {
        key_lock=0;
        key_count=0;
    }
}


char scan_key_ABCD(void) {
    static uint8_t countA = 0, countB = 0, countC = 0, countD = 0;
    static char lockedA = 0, lockedB = 0, lockedC = 0, lockedD = 0;

    // 读取每个按键的状态
    char stateD = HAL_GPIO_ReadPin(KEYA_GPIO_Port, KEYA_Pin);
    char stateC = HAL_GPIO_ReadPin(KDYB_GPIO_Port, KDYB_Pin);
    char stateB = HAL_GPIO_ReadPin(KEYC_GPIO_Port, KEYC_Pin);
    char stateA = HAL_GPIO_ReadPin(KEYD_GPIO_Port, KEYD_Pin);

    // 检测按键A
    if (stateA==0) {
        if (!lockedA && ++countA >= 10) {
            lockedA = 1; // 锁定按键
            return 'A';
        }
    } else {
        countA = 0;
        lockedA = 0;
    }

    // 检测按键B
    if (stateB==0) {
        if (!lockedB && ++countB >= 10) {
            lockedB = 1; // 锁定按键
            return 'B';
        }
    } else {
        countB = 0;
        lockedB = 0;
    }

    // 检测按键C
    if (stateC==0) {
        if (!lockedC && ++countC >= 10) {
            lockedC = 1; // 锁定按键
            return 'C';
        }
    } else {
        countC = 0;
        lockedC = 0;
    }

    // 检测按键D
    if (stateD==0) {
        if (!lockedD && ++countD >= 10) {
            lockedD = 1; // 锁定按键
            return 'D';
        }
    } else {
        countD = 0;
        lockedD = 0;
    }

    return '0'; // 如果没有按键被按下
}
// 定义CAN发送数据包的结构体
typedef struct
{
	uint32_t mailbox;                // 邮箱ID，用于标识CAN发送消息的邮箱
	CAN_TxHeaderTypeDef hdr;         // CAN发送消息的头部，包含ID、帧格式等信息
	uint8_t payload[8];              // 发送的数据负载，最大为8字节
} CAN_TxPacketTypeDef;

// 定义CAN接收数据包的结构体
typedef struct
{
	CAN_RxHeaderTypeDef hdr;         // CAN接收消息的头部，包含ID、帧格式等信息
	uint8_t payload[8];              // 接收的数据负载，最大为8字节
} CAN_RxPacketTypeDef;


/// CAN过滤器寄存器位宽类型定义
typedef union
{
    __IO uint32_t value;
    struct
    {
        uint8_t REV : 1;			///< [0]    ：未使用
        uint8_t RTR : 1;			///< [1]    : RTR（数据帧或远程帧标志位）
        uint8_t IDE : 1;			///< [2]    : IDE（标准帧或扩展帧标志位）
        uint32_t EXID : 18;			///< [21:3] : 存放扩展帧ID
        uint16_t STID : 11;			///< [31:22]: 存放标准帧ID
    } Sub;
} CAN_FilterRegTypeDef;


#define CAN_BASE_ID 0x100						///< CAN标准ID，最大11位，也就是0x7FF

#define CAN_FILTER_MODE_MASK_ENABLE 1		///< CAN过滤器模式选择：=0：列表模式  =1：屏蔽模式

#define CAN_ID_TYPE_STD_ENABLE      1       ///< CAN过滤ID类型选择：=1：标准ID，=0：扩展ID

void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;
    CAN_FilterRegTypeDef IDH = {0};
    CAN_FilterRegTypeDef IDL = {0};

#if CAN_ID_TYPE_STD_ENABLE
    IDH.Sub.STID = (CAN_BASE_ID >> 16) & 0xFFFF;		// 标准ID高16位
    IDL.Sub.STID = (CAN_BASE_ID & 0xFFFF);					// 标准ID低16位
#else
    IDH.Sub.EXID = (CAN_BASE_ID >> 16) & 0xFFFF;		// 扩展ID高16位
    IDL.Sub.EXID = (CAN_BASE_ID & 0xFFFF);				// 扩展ID低16位
    IDL.Sub.IDE  = 1;									// 扩展帧标志位置位
#endif
    sFilterConfig.FilterBank           = 0;												// 设置过滤器组编号
#if CAN_FILTER_MODE_MASK_ENABLE
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;							// 屏蔽位模式
#else
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;							// 列表模式
#endif
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;							// 32位宽
    sFilterConfig.FilterIdHigh         = IDH.value;										// 标识符寄存器一ID高十六位，放入扩展帧位
    sFilterConfig.FilterIdLow          = IDL.value;										// 标识符寄存器一ID低十六位，放入扩展帧位
    sFilterConfig.FilterMaskIdHigh     = IDH.value;										// 标识符寄存器二ID高十六位，放入扩展帧位
    sFilterConfig.FilterMaskIdLow      = IDL.value;										// 标识符寄存器二ID低十六位，放入扩展帧位
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;									// 过滤器组关联到FIFO0
    sFilterConfig.FilterActivation     = ENABLE;										// 激活过滤器
    sFilterConfig.SlaveStartFilterBank = 14;											// 设置从CAN的起始过滤器编号，本单片机只有一个CAN，顾此参数无效
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


// CAN发送函数，传入发送数据包指针
uint8_t CAN_Transmit(CAN_TxPacketTypeDef* packet)
{
    // 调用HAL库函数将CAN消息添加到发送队列，如果返回值不是HAL_OK则表示发送失败
	if(HAL_CAN_AddTxMessage(&hcan, &packet->hdr, packet->payload, &packet->mailbox) != HAL_OK)
		return 1;                    // 发送失败返回1
	return 0;                        // 发送成功返回0
}

// CAN初始化函数
void CAN_Init(void)
{
    MX_CAN_Init();                   // 初始化CAN硬件
    CAN_Filter_Config();             // 配置CAN过滤器
    HAL_CAN_Start(&hcan);            // 启动CAN接口
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 启用CAN接收中断
}

// 用于全局CAN发送数据包
CAN_TxPacketTypeDef g_CanTxPacket;

// 设置CAN发送数据包的函数
void CAN_SetTxPacket(void)
{
	g_CanTxPacket.hdr.StdId = 0x100; // 设置标准ID为0x100
	g_CanTxPacket.hdr.IDE = CAN_ID_STD; // 使用标准ID类型
	g_CanTxPacket.hdr.DLC = 8;       // 数据长度为8字节
	g_CanTxPacket.hdr.TransmitGlobalTime = DISABLE; // 禁止全局时间戳
	
	// 初始化负载数据为0到7
	for(int i = 0; i < 8; i++)
	{
		g_CanTxPacket.payload[i] = i;
	}
}

// Kalman滤波器，用于处理轴速度和电流的滤波
Kalman kalman_shaft_velocity_Filter; // 轴速度的卡尔曼滤波器
Kalman kalman_current_Iq_Filter;     // 电流Iq的卡尔曼滤波器

// 用于存储不同模式下的速度
float pattern_40_velocity, pattern_50_velocity;

// 目标角度、速度和电流
float Target_angle = 0;
float Target_speed = _2PI;
float Target_current = 2;
char show_one_flag_2=1;

extern float Motor_Target_angle[6];
extern float Motor_Target_speed[6];
extern float Motor_Target_current[6];
extern float Motor_Feedback_angle[6];
extern float Motor_Feedback_speed[6];
extern float Motor_Feedback_current[6];
extern uint8_t motorID;       //初始电机ID

void Key_ABCD_Task()
{
	char key_ABCD=scan_key_ABCD();
	if(key_ABCD=='A')
	{
		OLED_Clear();show_one_flag_2=1;
		if(pattern>=1010)pattern-=10;OLED_Show_Number(6,0,4,pattern);
	}
	if(key_ABCD=='B')
	{
		OLED_Clear();show_one_flag_2=1;
		if(pattern<=1090)pattern+=10;OLED_Show_Number(6,0,4,pattern);
	}
	if(key_ABCD=='C')
	{
		show_one_flag_2=1;
		switch(pattern)
		{
				case 1000://低速高扭矩模式，每次循环增加万分之一圈弧度，所以速度不确定。
					if(Target_current<=6)Target_current+=1;
					break;
        case 1010: // 位置模式 + 电流限制
					if(Target_current<=6)Target_current+=1;
					break;
        case 1020: // 力反馈  （要两个电机配合，一个电机可以感受到另外一个电机的阻力）
					if(Target_current<=6)Target_current+=1;
					break;
        case 1030: // 位置模式 + 速度限制 + 电流限制
					Target_speed+=1;
					break;
        case 1040: // 速度模式 + 电流限制--------------速度换->电流换
					//Target_speed+=_2PI;
					if(Target_current<=6)Target_current+=1;
					break;
        case 1050: // 转动旋钮   慢慢停止
					break;
        case 1060: // 转动旋钮   不停止
					break;
        case 1070: // 阻尼旋钮
					if(Damping_Vale<1)Damping_Vale+=0.1;
					break;
        case 1080: // 多档开关
					Gear_Switch_Number++;
					break;
        case 1090: // 多档开关
					Gear_Switch_Number++;
					break;
			}
		}
	if(key_ABCD=='D')
	{
		show_one_flag_2=1;
		switch(pattern)
		{
				case 1000://低速高扭矩模式，每次循环增加万分之一圈弧度，所以速度不确定。
					if(Target_current>1)Target_current--;
					break;
        case 1010: // 位置模式 + 电流限制
					if(Target_current>1)Target_current--;
					break;
        case 1020: // 力反馈  （要两个电机配合，一个电机可以感受到另外一个电机的阻力）
					if(Target_current>1)Target_current--;
					break;
        case 1030: // 位置模式 + 速度限制 + 电流限制
					Target_speed-=1;
					if(Target_speed<=0)Target_speed=1;
					break;
        case 1040: // 速度模式 + 电流限制--------------速度换->电流换
		//			Target_speed-=_2PI;
			//		if(Target_speed<=0)Target_speed=1;
					if(Target_current>1)Target_current--;
					break;
        case 1050: // 转动旋钮   慢慢停止
					break;
        case 1060: // 转动旋钮   不停止
					break;
        case 1070: // 阻尼旋钮
					if(Damping_Vale>0.1)Damping_Vale-=0.1;
					break;
        case 1080: // 多档开关
					if(Gear_Switch_Number>1)Gear_Switch_Number--;
					break;
        case 1090: // 多档开关
					if(Gear_Switch_Number>1)Gear_Switch_Number--;
					break;
			}
		}
}

void OLED_display_Task()
{
	static int show_one_flag = 0;
	static float shaft_angle_flag=0;
	static float Motor_Feedback_angle_show_buf=0;

	switch(pattern)
	{
			case 1000://  低速高扭矩模式，每次循环增加万分之一圈弧度，所以速度不确定。
				if(show_one_flag != pattern)// 根据当前模式（pattern）更新OLED显示内容
				{
						show_one_flag = pattern;
						OLED_Show_String(0, 0, "High Torque");	
						OLED_Show_String(2, 0, "Target_Ang:");        
					
				}
				
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;
					OLED_Show_Number(4, 6*16+8, 2, Target_current);             
				}
				break;
			case 1010: // 位置模式 + 电流限制
				if(show_one_flag != pattern)// 根据当前模式（pattern）更新OLED显示内容
				{
						show_one_flag = pattern;
						OLED_Show_String(0, 0, "Angle Current   ");
						OLED_Show_String(2, 0, "Target_Ang:");        
						OLED_Show_String(4, 0, "Max_Current:");                 
				}
				// 显示目标角度和最大电流
				
				if(Motor_Feedback_angle_show_buf!=Motor_Feedback_angle[1])
				{
					Motor_Feedback_angle_show_buf=Motor_Feedback_angle[1];
					OLED_Show_Number_xiao(2, 5*16, Motor_Feedback_angle[1]);               					
				}
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;
					OLED_Show_Number(4, 6*16+8, 2, Target_current);             
				}
				break;
			case 1020: // 力反馈  （要两个电机配合，一个电机可以感受到另外一个电机的阻力）
				if(show_one_flag != pattern)
				{
						show_one_flag = pattern;
						OLED_Show_String(0, 0, "Force Feedback");
						OLED_Show_String(2, 0, "Target_Ang:");        
						OLED_Show_String(4, 0, "Max_Current:");    
				}               
				// 显示目标角度、最大电流和角度系数
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;

					OLED_Show_Number(4, 6*16+8, 2, Target_current);    
				}
				if(Motor_Feedback_angle_show_buf!=Motor_Feedback_angle[1])
				{
					Motor_Feedback_angle_show_buf=Motor_Feedback_angle[1];
					OLED_Show_Number_xiao(2, 5*16, Motor_Feedback_angle[1]);               					
				}
				
		break;
			case 1030: // 位置模式 + 速度限制 + 电流限制
				if(show_one_flag != pattern) 
				{
					show_one_flag = pattern;
					OLED_Show_String(0, 0, "Angle Curr Speed");   
					OLED_Show_String(2, 0, "Target_Ang:");        
					OLED_Show_String(4, 0, "Max_Current:");    
					OLED_Show_String(6, 0, "Max_Speed:");   
				}                   
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;

					OLED_Show_Number(4, 6*16+8, 2, Target_current); 
					OLED_Show_Number_xiao(6, 6*16-16,Target_speed);               					
				}
				if(Motor_Feedback_angle_show_buf!=Motor_Feedback_angle[1])
				{
					Motor_Feedback_angle_show_buf=Motor_Feedback_angle[1];
					OLED_Show_Number_xiao(2, 5*16, Motor_Feedback_angle[1]);               					
				}
				break;
			case 1040: // 速度模式 + 电流限制--------------速度换->电流换
				if(show_one_flag != pattern)
				{
					show_one_flag = pattern;
					OLED_Show_String(0, 0, "Speed Current   ");
					OLED_Show_String(2, 0, "Target_Speed:");                   
					OLED_Show_String(4, 0, "Max_Current:");                    
				}                   
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;
					OLED_Show_Number_xiao(2, 6*16-16,Target_speed);                
					OLED_Show_Number(4, 6*16+8, 2, Target_current);             

				}
				
				break;
			case 1050: // 转动旋钮   慢慢停止
				if(show_one_flag != pattern)
				{
					show_one_flag = pattern;
					OLED_Show_String(0, 0, "Knod Slow Stop");
					OLED_Show_String(2, 0, "Speed:");              
				}                   
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;
					OLED_Show_Number_xiao(2, 6*16-16, Motor_Feedback_speed[0]/100);					
				}
				
				break;
			case 1060: // 转动旋钮   不停止
				if(show_one_flag != pattern)
				{
					show_one_flag = pattern;
					OLED_Show_String(0, 0, "Knod no Stop");
					OLED_Show_String(2, 0, "Speed:");              
				}                   
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;
					OLED_Show_Number_xiao(2, 6*16-16, Motor_Feedback_speed[0]/100);
				}
				break;
			case 1070: // 阻尼旋钮
				if(show_one_flag != pattern)
				{
					show_one_flag = pattern;
					OLED_Show_String(0, 0, "Damping Knod");          
					OLED_Show_String(2, 0, "D_Vale:");        
				}                   
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;
					OLED_Show_Number_xiao(2, 5*16, Damping_Vale);   
				}
				break;
			case 1080: // 多档开关
				if(show_one_flag != pattern)
				{
					show_one_flag = pattern;
					OLED_Show_String(0, 0, "Gear Switch");                   
					OLED_Show_String(2, 0, "Number:");     
					OLED_Show_String(4, 0, "Target_Ang:");        
					
				}                   
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;
					OLED_Show_Number(2, 6*16+8, 2, Gear_Switch_Number);
				}
				if(shaft_angle_flag!=shaft_angle)
				{
					shaft_angle_flag=shaft_angle;
					OLED_Show_Number(4, 6*16-8,3, shaft_angle*360/_2PI); 
				}
				break;
			case 1090: // 多档开关
				if(show_one_flag != pattern)
				{
					show_one_flag = pattern;
					OLED_Show_String(0, 0, "Gear Switch two");                   
					OLED_Show_String(2, 0, "Number:");  
					OLED_Show_String(4, 0, "Target_Ang:");        
					
				}                   
				if(show_one_flag_2==1)
				{
					show_one_flag_2=0;
					OLED_Show_Number(2, 6*16+8, 2, Gear_Switch_Number);
				}
				if(shaft_angle_flag!=shaft_angle)
				{
					shaft_angle_flag=shaft_angle;
					OLED_Show_Number(4, 6*16-8,3, shaft_angle*360/_2PI); 
				}
				break;

			case 1100: // 播放音乐
				if(show_one_flag != pattern)
				{
					show_one_flag = pattern;
					OLED_Show_String(0, 0, "Play Music");                   					
				}                   

				break;

			
		}

		
}

/*
函数 setPhaseVoltage 说明：
功能：根据DQ坐标系下的电压分量（Uq和Ud）以及电气角度，计算并设置三相电压的占空比。这是空间矢量PWM（SVPWM）算法的一部分，用于实现高效的电机控制。
参数：
Uq：DQ坐标系下的Q轴电压分量。
Ud：DQ坐标系下的D轴电压分量。
angle_el：当前的电气角度，单位为弧度。

全局变量说明：
shaft_angle：电机当前的实际轴角度。
electrical_angle：电机当前的电气角度，考虑了极对数和偏移量。
shaft_velocity：电机当前的轴速度。
current_sp：目标电流设定值。
shaft_velocity_sp：目标轴速度设定值。
shaft_angle_sp：目标轴角度设定值。
voltage：表示DQ坐标系下的电压。
current：表示DQ坐标系下的电流。
torque_controller：力矩控制器类型，用于控制电机的输出力矩。
controller：运动控制器类型，用于整体运动控制。
sensor_offset：传感器偏移量，用于校准角度。
zero_electric_angle：零电气角度，用	于参考电气角度的计算。

*/

// 设置目标电压的函数，输入参数为新的目标电压值
void setTargetVotage(float new_target)
{
//    static int Temperature_cs=0; // 静态变量，用于在函数调用之间保持值

//key_scan(); // 扫描按键状态
	Key_ABCD_Task();
	OLED_display_Task();
    switch(pattern) // 根据当前模式进行不同的操作
    {
        case 0:    
            CAN_Init(); // 初始化CAN通信
            CAN_SetTxPacket(); // 设置CAN发送包
            pattern=1000; // 将模式切换到

				break;
        case 10: // 位置模式（用于测试）

            current = getFOCCurrents(electrical_angle); // 获取FOC电流，根据电气角度计算
            // 计算目标电压，使用位置PID控制器根据目标角度与实际轴角度的误差lu
        //   voltage.q = PIDoperator(&P_angle, (0 - shaft_angle));       	
						voltage.q = 2;				
						voltage.q=_constrain(voltage.q,-3,3);
        //    printf("%.3f,%.3f\n", current.q,0.0); // 可选的调试输出，打印轴角度
            // voltage.q = PID_Calculate(&P_angle, shaft_angle, 3); // 另一种PID计算方式，注释掉
            break;
				
				case 1000://低速高扭矩模式，每次循环增加万分之一圈弧度，所以速度不确定。
				{

					Motor_Feedback_angle[1]=50;
					PID_velocity.P = -0.3;
						double Increment = 0.000628318530718*0.5;         // 每次增量，单位：弧度							
						if(motorID==1)
						{
							current=getFOCCurrents(electrical_angle);	

							static float TargetAngle_buf=0;
								if (TargetAngle_buf < Motor_Feedback_angle[1]) {
									TargetAngle_buf += Increment;
									// 防止超过目标角度
									if (TargetAngle_buf > Motor_Feedback_angle[1]) {
											TargetAngle_buf = Motor_Feedback_angle[1];
									}
							} else if (TargetAngle_buf > Motor_Feedback_angle[1]) {
									TargetAngle_buf -= Increment;
									// 防止低于目标角度
									if (TargetAngle_buf < Motor_Feedback_angle[1]) {
											TargetAngle_buf = Motor_Feedback_angle[1];
									}
							}
							// 计算目标电压，使用位置PID控制器根据目标角度与实际轴角度的误差

							voltage.q = PIDoperator(&P_angle, (TargetAngle_buf - shaft_angle));  
							
							voltage.q = _constrain( voltage.q , -5 , 5);// 用于限制电流

							voltage.q = PIDoperator(&PID_current_q, (voltage.q - current.q)); 				
						}
						else
						if(motorID==2)////电机2就是一个阻尼旋钮 给 电机1发送目标位置。
						{
							
							shaft_velocity = getVelocity();
							shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter,shaft_velocity);
							voltage.q=-(shaft_velocity/20);									
						}
					}	
						break;
        case 1010: // 位置模式 + 电流限制
						
						// 把电机ID2的 位置 给电机ID1，把电机ID1的电流 （就是电机1的阻力） 给 电机DI2 
						if(motorID==1)
						{
							PID_current_q.D=0;
						//	Target_current = 5;// ID1的电流限制设置大一点，这样感觉直观一点
							current = getFOCCurrents(electrical_angle); // 获取FOC电流
							// 计算目标电压，使用位置PID控制器
							voltage.q = PIDoperator(&P_angle, (-Motor_Feedback_angle[1] - shaft_angle));         
							// 以下代码用于限制电流，已被注释掉
							#define CURRENT_LIMIT Target_current
							if(voltage.q >= CURRENT_LIMIT) voltage.q = CURRENT_LIMIT;
							if(voltage.q <= -CURRENT_LIMIT) voltage.q = -CURRENT_LIMIT;

							// 使用电流PID控制器进一步调整目标电压
							voltage.q = PIDoperator(&PID_current_q, (voltage.q - current.q)); 				
						}
						else
						if(motorID==2)////电机2就是一个阻尼旋钮 给 电机1发送目标位置。
						{

							shaft_velocity = getVelocity();
							shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter,shaft_velocity);
							voltage.q=-(shaft_velocity/20);									
						}
						
            break;
        
        case 1020: // 力反馈  （要两个电机配合，一个电机可以感受到另外一个电机的阻力）
						// 把电机ID2的 位置 给电机ID1，把电机ID1的电流 （就是电机1的阻力） 给 电机DI2 
						if(motorID==1)
						{
							PID_current_q.D=0;
						//	Target_current = 5;// ID1的电流限制设置大一点，这样感觉直观一点
							current = getFOCCurrents(electrical_angle); // 获取FOC电流
							// 计算目标电压，使用位置PID控制器
							voltage.q = PIDoperator(&P_angle, (-Motor_Feedback_angle[1] - shaft_angle));         
							// 以下代码用于限制电流，已被注释掉
							#define CURRENT_LIMIT Target_current
							if(voltage.q >= CURRENT_LIMIT) voltage.q = CURRENT_LIMIT;
							if(voltage.q <= -CURRENT_LIMIT) voltage.q = -CURRENT_LIMIT;
							// 使用电流PID控制器进一步调整目标电压
							voltage.q = PIDoperator(&PID_current_q, (voltage.q - current.q)); 
						//	printf("%.2f,%.2f\n", shaft_velocity, current.q);
						}
						else
						if(motorID==2)//电机2就是一个阻尼旋钮，这个 阻尼 和ID1电机的电流成正比
						{

							shaft_velocity = getVelocity();
							shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter,shaft_velocity);
							voltage.q=-(shaft_velocity/20 - ( Motor_Feedback_current[0]/15.0));// 电机2的 阻力来源于 （阻尼感 + 电机1的电流）									
						}				//  shaft_velocity/20  电机2 自己的阻力        
            break;	//	Motor_Feedback_current[0]/15.0
        
        case 1030: // 位置模式 + 速度限制 + 电流限制
									
//						Motor_Feedback_angle[1] = 6.28;
						if(motorID==1)
						{
							PID_current_q.D=0;
						//							Target_current = 2;// ID1的电流限制设置大一点，这样感觉直观一点
							PID_velocity.P = -0.1;   //0.5
							PID_velocity.I = 0; // 重置速度PID积分项
															
							current = getFOCCurrents(electrical_angle); // 获取FOC电流						
							shaft_velocity = getVelocity(); // 获取轴的速度
							shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter, shaft_velocity); // 使用卡尔曼滤波器处理速度
	
							
							// 计算目标电压，使用位置PID控制器
							voltage.q = PIDoperator(&P_angle, (+Motor_Feedback_angle[1]+ shaft_angle)); 

							#define VELOCITY_LIMIT_12 Target_speed // 定义速度限制的宏
							// 限制目标电压在速度最大值和最小值之间
							if(voltage.q >=  VELOCITY_LIMIT_12) voltage.q =  VELOCITY_LIMIT_12;
							if(voltage.q <= -VELOCITY_LIMIT_12) voltage.q = -VELOCITY_LIMIT_12;    
							// 使用速度PID控制器调整目标电压，考虑速度误差
							voltage.q = PIDoperator(&PID_velocity, (voltage.q -shaft_velocity/3)); 

//							#define CURRENT_LIMIT_12 Target_current // 定义电流限制的宏
//							// 限制目标电压在电流最大值和最小值之间
//							if(voltage.q >= CURRENT_LIMIT_12) voltage.q = CURRENT_LIMIT_12;
//							if(voltage.q <= -CURRENT_LIMIT_12) voltage.q = -CURRENT_LIMIT_12;
							// 使用电流PID控制器进一步调整目标电压，考虑电流误差 
	//						voltage.q = PIDoperator(&PID_current_q, (voltage.q - current.q)); 
						}
						else
						if(motorID==2)//
						{

							shaft_velocity = getVelocity();
							shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter,shaft_velocity);
							voltage.q=-(shaft_velocity/20-Motor_Feedback_current[0]/15.0);// 电机2的 阻力来源于 （阻尼感 + 电机1的电流）									
						}

						
				break;
        case 1040: // 速度模式 + 电流限制--------------速度换->电流换
					
						Target_speed=_2PI;
						PID_velocity.P = -0.4;  //0.5
						PID_current_q.D=0.003;
						PID_current_q.P=1.0;

            current = getFOCCurrents(electrical_angle); // 获取FOC电流
            shaft_velocity = getVelocity(); // 获取轴的速度
            shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter, shaft_velocity); // 使用卡尔曼滤波器处理速度
            // 计算目标电压，使用速度PID控制器根据目标速度与实际速度的误差
            voltage.q = PIDoperator(&PID_velocity, (Target_speed - shaft_velocity));              
        
            #define CURRENT_LIMIT_12 Target_current // 定义电流限制的宏
            // 限制目标电压在电流最大值和最小值之间
            if(voltage.q >= CURRENT_LIMIT_12) voltage.q = CURRENT_LIMIT_12;
            if(voltage.q <= -CURRENT_LIMIT_12) voltage.q = -CURRENT_LIMIT_12;
            // 使用电流PID控制器进一步调整目标电压，考虑电流误差 
            voltage.q = PIDoperator(&PID_current_q, (voltage.q - current.q)); 
				
				
						static int cs_1040;
						if(cs_1040++>=50)
						{
							cs_1040=0;
							printf("%.2f,%.2f,%.2f\n", current.q , 0.0,shaft_velocity);
						}
            break;                
        
        case 1050: // 转动旋钮   慢慢停止
						PID_current_q.P=0.9;
						PID_velocity.P = -0.4;  //0.5
            shaft_velocity = getVelocity(); // 获取轴的速度
            shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter, shaft_velocity); // 使用卡尔曼滤波器处理速度
            voltage.q =-shaft_velocity / 10; // 适当增加一点助力，转起来丝滑一点。
				    voltage.q = _constrain(voltage.q, -2, 2);

						static int cs_40=0; // 静态变量，用于计时
				
            if(abs(shaft_velocity) >= 0.5) // 如果速度绝对值大于等于0.5
            {
                cs_40++; // 计数增加
                if(cs_40 == 400) // 如果计数达到400
                {
                    pattern_40_velocity = shaft_velocity; // 记录当前速度
                    pattern++; // 切换到下一个模式
                    cs_40 = 0; // 重置计数
                    PID_init(); // 初始化PID控制器
                }
            }
            else 
                cs_40 = 0; // 如果速度小于0.5，重置计数
            // 将目标电压限制在-2到2之间
        
            break;
        
        case 1051: // 转动旋钮缓慢停止的后续处理
						PID_velocity.P = -0.4;  //0.5
            shaft_velocity = getVelocity(); // 获取轴的速度
            shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter, shaft_velocity); // 使用卡尔曼滤波器处理速度
            static int cs_41=0, cs_41_2=0; // 静态变量，用于计时
            if(abs(shaft_velocity) <= 0.3) // 如果速度绝对值小于等于0.3
            {
                cs_41++; // 计数增加
                if(cs_41 == 200) // 如果计数达到200
                {
									  voltage.q=0;
                    pattern--; // 切换回上一个模式
                    cs_41 = 0; // 重置计数
                    pattern_40_velocity = 0; // 重置记录的速度
                }
            }
            else 
                cs_41 = 0; // 如果速度大于0.3，重置计数
            // 使用速度PID控制器根据记录的速度与实际速度的误差调整目标电压
            voltage.q = PIDoperator(&PID_velocity, (pattern_40_velocity - shaft_velocity)); 
            
            if(cs_41_2++ >= 100) // 每100次循环
            {
                cs_41_2 = 0; // 重置计数
                // 对记录的速度进行衰减
                if(pattern_40_velocity > 0) 
                    pattern_40_velocity -= pattern_40_velocity * 0.07;
                if(pattern_40_velocity < 0) 
                    pattern_40_velocity -= pattern_40_velocity * 0.07;
                if(abs(pattern_40_velocity) <= 0.3) // 如果衰减后的速度小于等于0.3
                {
                    cs_41_2 = 0; // 重置计数
                    pattern--; // 切换回上一个模式
                    pattern_40_velocity = 0; // 重置记录的速度
                }
            }
            break;
                        
        case 1060: // 转动旋钮   不停止
						PID_velocity.P = -0.4;  //0.5
            shaft_velocity = getVelocity(); // 获取轴的速度
            shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter, shaft_velocity); // 使用卡尔曼滤波器处理速度
            voltage.q =-shaft_velocity / 10; // 适当增加一点助力，转起来丝滑一点。
				    voltage.q = _constrain(voltage.q, -2, 2);
            static int cs_50=0; // 静态变量，用于计时
            if(abs(shaft_velocity) >= 0.5) // 如果速度绝对值大于等于0.5
            {
                cs_50++; // 计数增加
                if(cs_50 == 400) // 如果计数达到400
                {
                    PID_init(); // 初始化PID控制器
                    pattern_50_velocity = shaft_velocity * 1.2; // 记录并放大当前速度
                    pattern++; // 切换到下一个模式
                    cs_50 = 0; // 重置计数
                }
            }
            else 
                cs_50 = 0; // 如果速度小于0.5，重置计数

						
            break;
        
        case 1061: // 转动旋钮不停止的后续处理
						PID_velocity.P = -0.4;  //0.5
            shaft_velocity = getVelocity(); // 获取轴的速度
            shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter, shaft_velocity); // 使用卡尔曼滤波器处理速度
            static int cs_51=0; // 静态变量，用于计时
            if(abs(shaft_velocity) <= 0.5) // 如果速度绝对值小于等于0.5
            {
                cs_51++; // 计数增加
                if(cs_51 == 200) // 如果计数达到200
                {
									  voltage.q=0;
                    pattern--; // 切换回上一个模式
                    cs_51 = 0; // 重置计数
                    pattern_50_velocity = 0; // 重置记录的速度
                }
            }
            else 
                cs_51 = 0; // 如果速度大于0.5，重置计数
            // 使用速度PID控制器根据记录的速度与实际速度的误差调整目标电压
            voltage.q = PIDoperator(&PID_velocity, (pattern_50_velocity - shaft_velocity));             
            break;
        
        case 1070: // 阻尼旋钮
            shaft_velocity = getVelocity(); // 获取轴的速度
            shaft_velocity = Kalman_Filter(&kalman_shaft_velocity_Filter, shaft_velocity); // 使用卡尔曼滤波器处理速度
            // 根据速度、阻尼系数和传感器方向计算目标电压，实现阻尼效果
            voltage.q = -shaft_velocity * Damping_Vale * sensor_direction;            
            break;
        
        case 1080: // 多档开关
            // 根据档位开关的编号获取目标角度，计算与实际轴角度的误差，并乘以4得到目标电压
            voltage.q = (get_goal_angle(Gear_Switch_Number) - shaft_angle) * 4;
            break;
        
        case 1090: // 多档开关
            if(1){}; // 空操作，用于占位
            shaft_angle_1090 = shaft_angle; // 记录当前轴角度

            // 将轴角度规范到0到2π之间
            while(shaft_angle_1090 > _2PI) shaft_angle_1090 -= _2PI;
            while(shaft_angle_1090 < 0) shaft_angle_1090 = _2PI + shaft_angle_1090;
            current = getFOCCurrents(electrical_angle); // 获取FOC电流
                
            // 根据档位开关的编号获取第二种目标角度，计算与规范后的轴角度的误差，并乘以4得到目标电压
            voltage.q = (get_goal_angle_two(Gear_Switch_Number) - shaft_angle_1090) * 4;
						
            break;
       case 1100://播放音乐
					for(uint8_t i = 0; i < 200; i++) 
						{
								if(HAL_GPIO_ReadPin(KEYD_GPIO_Port, KEYD_Pin)==0||HAL_GPIO_ReadPin(KEYD_GPIO_Port, KEYD_Pin)==0)break;
								Beep_Play(hetangyeushe_music[i][0]);
								HAL_Delay(hetangyeushe_music[i][1]);
								Beep_Play(0);
								HAL_Delay(hetangyeushe_music[i][2]);
						}							
					break;
               
    }

}
/*
    - `case 0`: 初始化CAN通信并设置发送包，随后切换到模式3000。
    - `case 10`: 简单的位置控制模式，通过PID调整电压以达到目标角度。
    - `case 1010`: 位置控制加上电流限制，防止电流过大。
    - `case 1020`: 位置控制加上阻力反馈，根据速度和目标电流调整电压。
    - `case 1030`: 位置控制、电流限制和速度限制的组合，通过多重PID控制实现精确控制。
    - `case 1040`: 仅速度控制加上电流限制，通过PID调整电压以达到目标速度。
    - `case 1050` 和 `case 1051`: 旋钮控制，缓慢停止电机，通过计时和速度判断切换模式。
    - `case 1060` 和 `case 1061`: 旋钮控制，不停止电机，持续调整目标速度。
    - `case 1070`: 阻尼控制，根据速度和阻尼系数调整电压，实现阻尼效果。
    - `case 1080` 和 `case 1090`: 多档开关控制，根据档位调整目标角度和电压。


    - **PID控制**: 使用PID控制器计算误差并调整目标电压，以实现精确的速度和位置控制。
    - **卡尔曼滤波器**: 用于处理速度信号，减少测量噪声，提高控制精度。
    - **电流和速度限制**: 通过宏定义和条件判断，限制电压在安全范围内，防止过电流和过速度。
*/
		

/******************************************************************************/
/**
 * @brief 设置相电压的函数
 * 
 * 该函数根据给定的Q轴和D轴电压以及电气角度，计算并设置STM32定时器的比较值，以实现相电压的控制。
 * 
 * @param Uq Q轴电压分量
 * @param Ud D轴电压分量
 * @param angle_el 电气角度（弧度）
 
Uout：归一化后的电压幅值，用于后续的占空比计算。
sector：当前电气角度所在的扇区，范围为1到6，分别对应六个120度的扇区。
T0、T1、T2：空间矢量PWM中的基波占空比，用于计算各相的占空比。
Ta、Tb、Tc：ABC三个相的占空比，用于控制三相电机。
 */
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout;               // 输出电压的归一化幅值
    uint32_t sector;          // 当前所在的电气扇区（1到6）
    float T0, T1, T2;         // 空间矢量调制中的基波占空比
    float Ta, Tb, Tc;         // ABC三个相的占空比
    
    // 限制Q轴电压在预定的电压限制范围内
    if(Uq > voltage_limit) Uq = voltage_limit;
    if(Uq < -voltage_limit) Uq = -voltage_limit;
    // 限制D轴电压在预定的电压限制范围内
    if(Ud > voltage_limit) Ud = voltage_limit;
    if(Ud < -voltage_limit) Ud = -voltage_limit;
    
    // 如果D轴电压不为0，说明同时设置了Ud和Uq
    if(Ud) // 只有在设置了Ud和Uq时
    {
        // 计算归一化后的输出电压幅值，使用近似的平方根函数（误差约4%）
        Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
        // 计算并归一化电气角度，确保在0到2π之间
        // 这一步仅在使用近似的sin和cos函数时必要
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
    }
    else
    {
        // 只有Uq可用时，无需使用atan2和sqrt
        Uout = Uq / voltage_power_supply;
        // 计算并归一化电气角度，偏移π/2以确保角度在0到2π之间
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }
    
    // 限制输出电压幅值在0.577以内（相电压最大值的近似值）
    if(Uout > 0.577) Uout = 0.577;
    if(Uout < -0.577) Uout = -0.577;
    
    // 计算当前所在的扇区（1到6）
    sector = (angle_el / _PI_3) + 1;
    // 计算T1和T2，基于当前扇区和电气角度
    T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uout;
    // 计算T0，作为剩余时间
    T0 = 1 - T1 - T2;
    
    // 根据当前扇区设置ABC三个相的占空比
    switch(sector)
    {
        case 1:
            /**
             * 扇区1：
             * A相：T1 + T2 + T0/2
             * B相：T2 + T0/2
             * C相：T0/2
             */
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            /**
             * 扇区2：
             * A相：T1 + T0/2
             * B相：T1 + T2 + T0/2
             * C相：T0/2
             */
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            /**
             * 扇区3：
             * A相：T0/2
             * B相：T1 + T2 + T0/2
             * C相：T2 + T0/2
             */
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            /**
             * 扇区4：
             * A相：T0/2
             * B相：T1 + T0/2
             * C相：T1 + T2 + T0/2
             */
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            /**
             * 扇区5：
             * A相：T2 + T0/2
             * B相：T0/2
             * C相：T1 + T2 + T0/2
             */
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            /**
             * 扇区6：
             * A相：T1 + T2 + T0/2
             * B相：T0/2
             * C相：T1 + T0/2
             */
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            /**
             * 默认情况（可能的错误状态）：
             * 所有相的占空比设为0，系统不输出任何电压
             */
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }
    
    // 将计算得到的占空比乘以PWM周期，设置到定时器的比较寄存器
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, Ta * PWM_Period); // 设置A相的占空比
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, Tb * PWM_Period); // 设置B相的占空比
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, Tc * PWM_Period); // 设置C相的占空比
}
/******************************************************************************/
