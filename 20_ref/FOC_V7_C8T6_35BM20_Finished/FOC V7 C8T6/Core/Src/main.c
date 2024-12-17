/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "MagneticSensor.h" 
#include "FOCMotor.h"
#include "BLDCmotor.h"
#include "FlashStorage.h"
#include "Kalman.h"
#include "pid.h"
#include "InlineCurrentSense.h"

#include "OLED_H.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum
{
	//不响
	T_None=0,
	//低八度  
  T_L1=3822,
  T_L2=3405,
  T_L3=3034,
  T_L4=2863,
  T_L5=2551,
  T_L6=2272,
  T_L7=2052,
	//中八度
  T_M1=1911,
  T_M2=1703,
  T_M3=1517,
  T_M4=1432,
  T_M5=1276,
  T_M6=1136,
  T_M7=1012,
	//高八度
  T_H1=956,
  T_H2=851,
  T_H3=758,
  T_H4=716,
  T_H5=638,
  T_H6=568,
  T_H7=506
};



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VOLT_SUPPLY 12 //供电母线电压
#define MAX_VOLT 5.0f  //限制供电电压(平均值)，最高为VOLT_SUPPLY/√3; 4010电机:6.9V, 2804电机:4V

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float targetVotage = 0;    //当前目标电压
uint8_t beepPlaying = 0;   //当前是否在蜂鸣状态
uint8_t motorID = 1;       //初始电机ID
uint8_t ledBlink = 1;      //led是否处于正常闪烁模式
float speed = 0;           //传感器作差数据所得转速
float filteredAngle = 0;   //经滤波得到的转子角度
uint32_t lastRecvTime = 0; //上次收到CAN数据帧的时间
Kalman angleFilter; //卡尔曼滤波结构体
uint16_t Uart_Send_Flag=1;

float Motor_Target_angle[6];
float Motor_Target_speed[6]={3.14};
float Motor_Target_current[6]={2};//  目标电流，设定为2就是0.2A左右， 用0.2是因为之前OLED显示方便。
float Motor_Feedback_angle[6];
float Motor_Feedback_speed[6];
float Motor_Feedback_current[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//重定义fputc函数
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
    USART1->DR = (uint8_t) ch;      
	return ch;
}
//单片机软件复位
void System_Reset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

/*************CAN通信*************/
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

//CAN外设初始化
void CAN_Init1111()
{

	
	  CAN_FilterTypeDef sFilterConfig;
    CAN_FilterRegTypeDef IDH = {0};
    CAN_FilterRegTypeDef IDL = {0};

#if CAN_ID_TYPE_STD_ENABLE
    IDH.Sub.STID = (CAN_BASE_ID >> 16) & 0xFFFF;		// 标准ID高16位
    IDL.Sub.STID = (CAN_BASE_ID & 0xFFFF);				// 标准ID低16位
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
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

}


//DQCurrent_s current;

// 声明外部变量，这些变量可能在其他文件中定义和更新
extern int pattern; // 模式变量
extern float Target_angle; // 目标角度
extern float Target_speed; // 目标速度
extern float Target_current; // 目标电流
extern int Gear_Switch_Number; // 齿轮切换数
extern float Damping_Vale; // 阻尼值

//发送一个反馈数据帧
void CAN_SendState(float angle, float speed)
{
	CAN_TxHeaderTypeDef header;
	header.StdId = motorID + 0x100;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;	
	header.DLC = 8  ;
	
	uint8_t data[8  ];    //   -0.1
	memcpy(data,&(int32_t){angle*1000},4); 				//角度数据放在前四个字节
	memcpy(&data[4],&(int16_t){speed*100},2); 		//转速数据放在第5-6字节
	memcpy(&data[6],&(int16_t){current.q*100},2); //转速数据放在第5-6字节
//	memcpy(&data[8],&(int16_t){pattern},2);			//CAN 通信只有最多八位数据，要么用CAN——FD 
	
	//printf("%.2f\r\n",angle);
	uint32_t mailbox;
	HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
}

// 定义一个结构体用于存储CAN接收的数据包，包括头部信息和数据负载
typedef struct
{
	CAN_RxHeaderTypeDef hdr; // CAN接收头部信息
	uint8_t payload[8]; // CAN接收的数据负载，最大8字节
}CAN_RxPacketTypeDef;




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
{
	static CAN_RxPacketTypeDef packet;
	
    // CAN数据接收
    if (canHandle->Instance == hcan.Instance)
    {
        if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &packet.hdr, packet.payload) == HAL_OK)		// 获得接收到的数据头和数据
        {

					if(packet.hdr.StdId >= 0x101 && packet.hdr.StdId <= 0x106) //ID=1~4接收0x100数据帧
					{
						Motor_Feedback_angle  [packet.hdr.StdId - 0x101] =   *(int32_t *)&packet.payload[0] / 1000.0f ;
						Motor_Feedback_speed  [packet.hdr.StdId - 0x101] =  (*(int16_t *)&packet.payload[4] );//速度反馈，放大了100倍。
						Motor_Feedback_current[packet.hdr.StdId - 0x101] =  (*(int16_t *)&packet.payload[6] );//速度反馈，放大了100倍。

					}
					
          HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);						// 再次使能FIFO0接收中断
        }
    }
}

/*************FOC*************/

void SimpleFOC_Init()
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); //开启三个PWM通道输出
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	
	MagneticSensor_Init(); //初始化磁传感器
	
	voltage_power_supply=VOLT_SUPPLY; //设定FOC所需参数
	voltage_limit=MAX_VOLT;
	voltage_sensor_align=voltage_limit;
	targetVotage=0;
	
	Motor_init(); //初始化电机信息
	Motor_initFOC(); //初始化FOC参数
	
	motorID = Flash_ReadMotorID(); //从Flash读取电机ID
	motorID = motorID ? motorID : 1;
//	motorID=1;
//	
//	Flash_SaveMotorID(motorID);
//	
//	printf("motorID = %d\r\n",motorID);
}

/*************蜂鸣*************/

//根据蜂鸣周期配置并触发定时器
void Beep_Play(uint16_t period)
{
	if(period!=0)
	{
		__HAL_TIM_SetAutoreload(&htim1,period/2);
		HAL_TIM_Base_Start_IT(&htim1);
		beepPlaying = 1;
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim1);
		beepPlaying = 0;
	}
}

//播放一串音符
void Beep_PlayNotes(uint8_t num, uint16_t notes[][2])
{
	for(uint8_t i=0; i<num; i++)
	{
		Beep_Play(notes[i][0]);
		HAL_Delay(notes[i][1]);
	}
	Beep_Play(0);
}

//蜂鸣中断处理，在定时器中断回调中调用
void Beep_IRQHandler()
{
	static uint8_t flipFlag = 0;
	if(targetVotage == 0)
	{
		setPhaseVoltage(voltage_limit/2, 0, _PI/3 * flipFlag); //使磁场方向在0-PI/3间震荡
		flipFlag = !flipFlag;
	}
}

/*************各个定时任务*************/

//非阻塞按键处理
//void Key_Process()
//{
//	static uint32_t downTime = 0;
//	static uint8_t lastKeyState = 0;
//	uint8_t keyState = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1 : 0;
//	if(keyState && !lastKeyState) //按下
//	{
//		downTime = HAL_GetTick();
//		ledBlink = 0;
//	}
//	else if(keyState && lastKeyState) //按住
//	{
//		uint32_t pressTime = HAL_GetTick() - downTime;
//		if(pressTime < 500*8) //闪8下，每次500ms
//		{
//			if(pressTime%500 < 100)
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//			else
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		}
//		else if(pressTime < 500*12)
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//		else
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //闪8下之后常亮
//	}
//	else if(!keyState && lastKeyState) //松开
//	{
//		uint32_t pressTime = HAL_GetTick() - downTime;
//		if(pressTime > 50 && pressTime <500*8) //闪8下以内松开，设置ID
//		{
//			motorID = pressTime/500 + 1;
//			Flash_SaveMotorID(motorID);
//		}
//		else if(pressTime >= 500*8 && pressTime < 500*12) //闪8下后松开，清空Flash后复位重新校准
//		{
//			Flash_EraseMotorParam();
//			System_Reset();
//		}
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		ledBlink = 1;
//	}
//	lastKeyState = keyState;
//}

//非阻塞LED定时任务，闪烁次数表示电机ID
void Led_Process()
{
	if(!ledBlink) return;
	uint32_t period = 1000 + (100+200)*motorID;
	uint32_t mod = HAL_GetTick() % period;
	if(mod < (100+200)*motorID && mod%(100+200) < 100)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else
	{	
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}

////电机转速计算
//void Motor_SpeedCalcProcess()
//{
//	const uint8_t speedLpfLen = 5;
//	static float speedLpfBuf[speedLpfLen] = {0}; //存放5个filteredAngle
//	
//	float angle = filteredAngle;
//	float curSpeed = (angle-speedLpfBuf[0])*1000/speedLpfLen/_2PI*60;
//	speed = curSpeed;
//	
//	for(uint8_t i=0; i<speedLpfLen-1; i++)
//		speedLpfBuf[i] = speedLpfBuf[i+1];
//	speedLpfBuf[speedLpfLen-1] = angle;
//}

////CAN离线检测，500ms没收到CAN信号则停机
//void Motor_OfflineCheckProcess()
//{
//	static uint8_t isOffline = 0;
//	if(HAL_GetTick() - lastRecvTime > 500)
//	{
//		if(!isOffline)
//		{
//			targetVotage = 0;
//			isOffline = 1;
//		}
//	}
//	else
//		isOffline = 0;
//}


//在滴答定时器中调用，1ms周期，处理各定时任务
void SysTick_UserExec()
{
//	Key_Process();
	Led_Process();
	//Motor_SpeedCalcProcess();
	//Motor_OfflineCheckProcess();
	if(HAL_GetTick()%5)
		CAN_SendState(shaft_angle, shaft_velocity);
//	filteredAngle = Kalman_Filter(&angleFilter, shaft_angle);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
}

uint8_t rx_byte; // 用于存储接收到的一个字节
extern char show_one_flag_2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 仅处理特定的串口中断（如 USART1），避免多串口冲突
    if (huart->Instance == USART1)
    {
        // 静态变量用于状态机
        static char command_buffer[5];         // 存储完整的5字符命令
        static uint8_t buffer_index = 0;       // 当前缓冲区索引
        static enum { WAIT_COMMAND, WAIT_EQUAL, WAIT_DATA } state = WAIT_COMMAND; // 状态机状态

        // 获取接收到的字节
        char received = rx_byte;

        switch(state)
        {
            case WAIT_COMMAND:
                // 检查是否为有效的命令字母（M, C, A, S, Z, G），支持大小写
                if ((received == 'M') || (received == 'm') ||
                    (received == 'C') || (received == 'c') ||
                    (received == 'A') || (received == 'a') ||
                    (received == 'S') || (received == 's') ||
                    (received == 'D') || (received == 'd') ||
                    (received == 'G') || (received == 'g'))
                {
                    command_buffer[0] = received;
                    buffer_index = 1;
                    state = WAIT_EQUAL;
                }
                // 如果不是命令字母，则忽略该字节，继续等待
                break;

            case WAIT_EQUAL:
                if (received == '=')
                {
                    state = WAIT_DATA;
                }
                else
                {
                    // 如果不是 '=', 则重置状态机
                    buffer_index = 0;
                    state = WAIT_COMMAND;
                }
                break;

            case WAIT_DATA:
                // 接收数据部分的三个字符
                if (buffer_index < 4)
                {
                    command_buffer[buffer_index++] = received;
                }

                // 当接收满4个字符（包括命令字母），则处理命令
                if (buffer_index >= 4)
                {
                    command_buffer[4] = '\0'; // 添加字符串结束符，确保安全

                    // 提取命令字母和数据部分
                    char cmd = command_buffer[0];
                    char data_str[4];
                    memcpy(data_str, &command_buffer[1], 3);
                    data_str[3] = '\0'; // 确保字符串结束

                    // 将数据部分转换为整数
                    int value = atoi(data_str);
									
										Uart_Send_Flag = 1 ;// 刷新串口打印内容
									
                    // 根据命令类型更新相应的变量
                    switch(cmd)
                    {
                        case 'M':
                        case 'm':
                            // 更改 pattern 模式变量，按照要求为 1xxx
                            pattern = 1000 + value;
                            break;

                        case 'C':
                        case 'c':
                            // 更改目标电流指令，将整数转换为浮点数 xx.x
                            Motor_Target_current[0] = value ;
                            break;

                        case 'A':
                        case 'a':
                            // 更改目标角度指令，将整数转换为浮点数 xx.x
                            Motor_Target_angle[0] = value / 10.0f;
                            break;

                        case 'S':
                        case 's':
                            // 更改目标速度指令，假设直接赋值为浮点数
                            Motor_Target_speed[0] = (float)value/10.f;
                            break;

                        case 'D':
                        case 'd':
                            // 更改阻尼指令，直接赋值为整数
												Damping_Vale = (float)value/10.f;//只能发送1，所以要/10
                            break;

                        case 'G':
                        case 'g':
                            // 更改多档开关挡位指令，直接赋值为整数
                            Gear_Switch_Number = value;
                            break;

                        default:
                            // 未知命令，忽略
                            break;
                    }
										Target_angle = Motor_Target_angle[0];			//设定目标角度
										Target_current = Motor_Target_current[0];	//设定目标电流
										Target_speed 	= Motor_Target_speed[0];			//设定目标速度
										show_one_flag_2=1;
                    // 处理完一个完整命令后，重置状态机
                    buffer_index = 0;
                    state = WAIT_COMMAND;
                }
                break;

            default:
                // 默认情况下，重置状态机
                buffer_index = 0;
                state = WAIT_COMMAND;
                break;
        }

        // 重新启动接收中断，以便继续接收下一个字节
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

void Uart_Send_Task()
{
	if(Uart_Send_Flag)
	{
		Uart_Send_Flag=0;
		switch(pattern)
		{
			
				case 1000:// 低速高扭矩
					printf("Low speed high torque    <<%4d>> \r\n ",pattern);

					break;
				case 1010:// 位置模式+ 电流限制
					printf("position_loop +  Current_loop    <<%4d>> \r\n ",pattern);
					printf("Target_Angle:%.2f \r\n",Motor_Target_angle[0]);		
					printf("Max_Current: %.2f \r\n",Motor_Target_current[0]/10);					
					break;
				case 1020:// 位置模式 + 阻力反馈 
					printf("position_loop +  Resistance_feedback     <<%4d>> \r\n ",pattern);
					printf("Target_Angle: %.2f \r\n",Motor_Target_angle[0]);		
					printf("Max_Current:  %.2f \r\n",Motor_Target_current[0]/10);	

				break;
			case 1030:// 位置模式 + 电流限制 + 速度限制
					printf("position_loop + Current_loop +  speed_loop    <<%4d>> \r\n ",pattern);
					printf("Target_Angle: %.2f \r\n",Motor_Target_angle[0]);					
					printf("Max_Current:  %.2f \r\n",Motor_Target_current[0]/10);					
					printf("Max_Speed:    %.2f \r\n",Motor_Target_speed[0]);	

			break;				
			case 1040://  速度模式 + 电流限制
					printf("speed_loop +  Current_loop     <<%4d>> \r\n ",pattern);
					printf("Target_Speed: %.2f \r\n",Motor_Target_speed[0]);					
					printf("Max_Current:  %.2f \r\n",Motor_Target_current[0]/10);					

			break;				
			case 1050://转动旋钮   慢慢停止////////////////////////////////////////////////////////////////
					printf("Slow stop knob    <<%4d>> \r\n ",pattern);

			break;
			case 1060://转动旋钮   不停止////////////////////////////////////////////////////////////////
					printf("No stop knob    <<%4d>> \r\n ",pattern);

			break;
			case 1070://阻尼旋钮////////////////////////////////////////////////////////////////
					printf("Resistance knob    <<%4d>> \r\n ",pattern);		
					printf("Damping_Vale: %.2f \r\n",Damping_Vale);		

				break;
			case 1080://多档开关////////////////////////////////////////////////////////////////
					printf("limited Multispeed switch    <<%4d>> \r\n ",pattern);					
					printf("Number: %2d \r\n",Gear_Switch_Number);		

				break;
			case 1090://多档开关////////////////////////////////////////////////////////////////
					printf("unlimited Multispeed switch    <<%4d>> \r\n ",pattern);				
					printf("Number: %2d \r\n",Gear_Switch_Number);		

			
				break;

		}
	}
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	for(int i=0;i<200;i++)OLED_Init_H();
	OLED_Clear();
	OLED_Show_String(3,10,"System Loading");
	
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  // 启动接收中断，每次接收一个字节 
	
	// 初始化内联电流测量，参数包括：采样电阻阻值0.001欧姆，运放增益20倍，A相、B相和C相初始电流设置为0
	InlineCurrentSense(0.001, 20, 0, 0, 0); // SimpleMotor

	// 初始化ADC用于电流测量，并进行偏置电压校准，以确保测量精度
	InlineCurrentSense_Init();

	// 设定电压极限为系统定义的最大电压值
	voltage_limit = MAX_VOLT;

	// 设置直轴（d轴）和交轴（q轴）电流的PID控制器的输出限制为最大电压
	PID_current_q.limit = voltage_limit;
	PID_current_d.limit = voltage_limit;

	// 设置速度PID控制器的输出限制为最大电压，并定义输出增加的最大斜率为50（用于平滑速度变化）
	PID_velocity.limit = voltage_limit;
	PID_velocity.output_ramp = 50;

	// 设置角度P控制器的输出限制为最大电压
	P_angle.limit = voltage_limit;

	// 初始化所有PID控制器
	PID_init();


// 初始化FOC算法，用于电机的矢量控制
	SimpleFOC_Init();
	
//	motorID=1;
//	Flash_SaveMotorID(motorID);
//	printf("motorID = %d\r\n",motorID);

	// 播放开机音效，使用蜂鸣器按顺序播放音调H1, H3, H5，持续时间分别为200ms, 200ms, 500ms
	Beep_PlayNotes(3, (uint16_t[][2]){{T_H1, 200}, {T_H3, 200}, {T_H5, 500}});

	// 初始化卡尔曼滤波器，用于角度估计，设置噪声参数和估计误差
	Kalman_Init(&angleFilter, 0.0005f, 0.1f);

	// 初始化卡尔曼滤波器，用于轴速度估计，设置噪声参数和估计误差
	Kalman_Init(&kalman_shaft_velocity_Filter, 0.0005f, 0.1f);

	// 初始化卡尔曼滤波器，用于交轴电流Iq的估计，设置噪声参数和估计误差
	Kalman_Init(&kalman_current_Iq_Filter, 0.0005f, 0.1f); 

	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);//ENABNLE Motor
	
	OLED_Clear();
	
	KalMan_PramInit(&KalMan_current_Iq);	//把卡尔曼的系数全部跟新一遍，下面调用滤波会用到这些参数

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		
		setTargetVotage(targetVotage); //设定FOC电压
		loopFOC(); //运行FOC算法
		Uart_Send_Task();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

