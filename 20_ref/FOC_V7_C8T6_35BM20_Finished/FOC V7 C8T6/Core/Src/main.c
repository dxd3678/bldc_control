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
	//����
	T_None=0,
	//�Ͱ˶�  
  T_L1=3822,
  T_L2=3405,
  T_L3=3034,
  T_L4=2863,
  T_L5=2551,
  T_L6=2272,
  T_L7=2052,
	//�а˶�
  T_M1=1911,
  T_M2=1703,
  T_M3=1517,
  T_M4=1432,
  T_M5=1276,
  T_M6=1136,
  T_M7=1012,
	//�߰˶�
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
#define VOLT_SUPPLY 12 //����ĸ�ߵ�ѹ
#define MAX_VOLT 5.0f  //���ƹ����ѹ(ƽ��ֵ)�����ΪVOLT_SUPPLY/��3; 4010���:6.9V, 2804���:4V

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float targetVotage = 0;    //��ǰĿ���ѹ
uint8_t beepPlaying = 0;   //��ǰ�Ƿ��ڷ���״̬
uint8_t motorID = 1;       //��ʼ���ID
uint8_t ledBlink = 1;      //led�Ƿ���������˸ģʽ
float speed = 0;           //������������������ת��
float filteredAngle = 0;   //���˲��õ���ת�ӽǶ�
uint32_t lastRecvTime = 0; //�ϴ��յ�CAN����֡��ʱ��
Kalman angleFilter; //�������˲��ṹ��
uint16_t Uart_Send_Flag=1;

float Motor_Target_angle[6];
float Motor_Target_speed[6]={3.14};
float Motor_Target_current[6]={2};//  Ŀ��������趨Ϊ2����0.2A���ң� ��0.2����Ϊ֮ǰOLED��ʾ���㡣
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
//�ض���fputc����
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
    USART1->DR = (uint8_t) ch;      
	return ch;
}
//��Ƭ�������λ
void System_Reset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

/*************CANͨ��*************/
typedef union
{
    __IO uint32_t value;
    struct
    {
        uint8_t REV : 1;			///< [0]    ��δʹ��
        uint8_t RTR : 1;			///< [1]    : RTR������֡��Զ��֡��־λ��
        uint8_t IDE : 1;			///< [2]    : IDE����׼֡����չ֡��־λ��
        uint32_t EXID : 18;			///< [21:3] : �����չ֡ID
        uint16_t STID : 11;			///< [31:22]: ��ű�׼֡ID
    } Sub;
} CAN_FilterRegTypeDef;

#define CAN_BASE_ID 0x100						///< CAN��׼ID�����11λ��Ҳ����0x7FF

#define CAN_FILTER_MODE_MASK_ENABLE 1		///< CAN������ģʽѡ��=0���б�ģʽ  =1������ģʽ

#define CAN_ID_TYPE_STD_ENABLE      1       ///< CAN����ID����ѡ��=1����׼ID��=0����չID

//CAN�����ʼ��
void CAN_Init1111()
{

	
	  CAN_FilterTypeDef sFilterConfig;
    CAN_FilterRegTypeDef IDH = {0};
    CAN_FilterRegTypeDef IDL = {0};

#if CAN_ID_TYPE_STD_ENABLE
    IDH.Sub.STID = (CAN_BASE_ID >> 16) & 0xFFFF;		// ��׼ID��16λ
    IDL.Sub.STID = (CAN_BASE_ID & 0xFFFF);				// ��׼ID��16λ
#else
    IDH.Sub.EXID = (CAN_BASE_ID >> 16) & 0xFFFF;		// ��չID��16λ
    IDL.Sub.EXID = (CAN_BASE_ID & 0xFFFF);				// ��չID��16λ
    IDL.Sub.IDE  = 1;									// ��չ֡��־λ��λ
#endif
    sFilterConfig.FilterBank           = 0;												// ���ù���������
#if CAN_FILTER_MODE_MASK_ENABLE
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;							// ����λģʽ
#else
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;							// �б�ģʽ
#endif
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;							// 32λ��
    sFilterConfig.FilterIdHigh         = IDH.value;										// ��ʶ���Ĵ���һID��ʮ��λ��������չ֡λ
    sFilterConfig.FilterIdLow          = IDL.value;										// ��ʶ���Ĵ���һID��ʮ��λ��������չ֡λ
    sFilterConfig.FilterMaskIdHigh     = IDH.value;										// ��ʶ���Ĵ�����ID��ʮ��λ��������չ֡λ
    sFilterConfig.FilterMaskIdLow      = IDL.value;										// ��ʶ���Ĵ�����ID��ʮ��λ��������չ֡λ
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;									// �������������FIFO0
    sFilterConfig.FilterActivation     = ENABLE;										// ���������
    sFilterConfig.SlaveStartFilterBank = 14;											// ���ô�CAN����ʼ��������ţ�����Ƭ��ֻ��һ��CAN���˴˲�����Ч
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

}


//DQCurrent_s current;

// �����ⲿ��������Щ���������������ļ��ж���͸���
extern int pattern; // ģʽ����
extern float Target_angle; // Ŀ��Ƕ�
extern float Target_speed; // Ŀ���ٶ�
extern float Target_current; // Ŀ�����
extern int Gear_Switch_Number; // �����л���
extern float Damping_Vale; // ����ֵ

//����һ����������֡
void CAN_SendState(float angle, float speed)
{
	CAN_TxHeaderTypeDef header;
	header.StdId = motorID + 0x100;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;	
	header.DLC = 8  ;
	
	uint8_t data[8  ];    //   -0.1
	memcpy(data,&(int32_t){angle*1000},4); 				//�Ƕ����ݷ���ǰ�ĸ��ֽ�
	memcpy(&data[4],&(int16_t){speed*100},2); 		//ת�����ݷ��ڵ�5-6�ֽ�
	memcpy(&data[6],&(int16_t){current.q*100},2); //ת�����ݷ��ڵ�5-6�ֽ�
//	memcpy(&data[8],&(int16_t){pattern},2);			//CAN ͨ��ֻ������λ���ݣ�Ҫô��CAN����FD 
	
	//printf("%.2f\r\n",angle);
	uint32_t mailbox;
	HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
}

// ����һ���ṹ�����ڴ洢CAN���յ����ݰ�������ͷ����Ϣ�����ݸ���
typedef struct
{
	CAN_RxHeaderTypeDef hdr; // CAN����ͷ����Ϣ
	uint8_t payload[8]; // CAN���յ����ݸ��أ����8�ֽ�
}CAN_RxPacketTypeDef;




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
{
	static CAN_RxPacketTypeDef packet;
	
    // CAN���ݽ���
    if (canHandle->Instance == hcan.Instance)
    {
        if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &packet.hdr, packet.payload) == HAL_OK)		// ��ý��յ�������ͷ������
        {

					if(packet.hdr.StdId >= 0x101 && packet.hdr.StdId <= 0x106) //ID=1~4����0x100����֡
					{
						Motor_Feedback_angle  [packet.hdr.StdId - 0x101] =   *(int32_t *)&packet.payload[0] / 1000.0f ;
						Motor_Feedback_speed  [packet.hdr.StdId - 0x101] =  (*(int16_t *)&packet.payload[4] );//�ٶȷ������Ŵ���100����
						Motor_Feedback_current[packet.hdr.StdId - 0x101] =  (*(int16_t *)&packet.payload[6] );//�ٶȷ������Ŵ���100����

					}
					
          HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);						// �ٴ�ʹ��FIFO0�����ж�
        }
    }
}

/*************FOC*************/

void SimpleFOC_Init()
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); //��������PWMͨ�����
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	
	MagneticSensor_Init(); //��ʼ���Ŵ�����
	
	voltage_power_supply=VOLT_SUPPLY; //�趨FOC�������
	voltage_limit=MAX_VOLT;
	voltage_sensor_align=voltage_limit;
	targetVotage=0;
	
	Motor_init(); //��ʼ�������Ϣ
	Motor_initFOC(); //��ʼ��FOC����
	
	motorID = Flash_ReadMotorID(); //��Flash��ȡ���ID
	motorID = motorID ? motorID : 1;
//	motorID=1;
//	
//	Flash_SaveMotorID(motorID);
//	
//	printf("motorID = %d\r\n",motorID);
}

/*************����*************/

//���ݷ����������ò�������ʱ��
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

//����һ������
void Beep_PlayNotes(uint8_t num, uint16_t notes[][2])
{
	for(uint8_t i=0; i<num; i++)
	{
		Beep_Play(notes[i][0]);
		HAL_Delay(notes[i][1]);
	}
	Beep_Play(0);
}

//�����жϴ����ڶ�ʱ���жϻص��е���
void Beep_IRQHandler()
{
	static uint8_t flipFlag = 0;
	if(targetVotage == 0)
	{
		setPhaseVoltage(voltage_limit/2, 0, _PI/3 * flipFlag); //ʹ�ų�������0-PI/3����
		flipFlag = !flipFlag;
	}
}

/*************������ʱ����*************/

//��������������
//void Key_Process()
//{
//	static uint32_t downTime = 0;
//	static uint8_t lastKeyState = 0;
//	uint8_t keyState = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1 : 0;
//	if(keyState && !lastKeyState) //����
//	{
//		downTime = HAL_GetTick();
//		ledBlink = 0;
//	}
//	else if(keyState && lastKeyState) //��ס
//	{
//		uint32_t pressTime = HAL_GetTick() - downTime;
//		if(pressTime < 500*8) //��8�£�ÿ��500ms
//		{
//			if(pressTime%500 < 100)
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//			else
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		}
//		else if(pressTime < 500*12)
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//		else
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //��8��֮����
//	}
//	else if(!keyState && lastKeyState) //�ɿ�
//	{
//		uint32_t pressTime = HAL_GetTick() - downTime;
//		if(pressTime > 50 && pressTime <500*8) //��8�������ɿ�������ID
//		{
//			motorID = pressTime/500 + 1;
//			Flash_SaveMotorID(motorID);
//		}
//		else if(pressTime >= 500*8 && pressTime < 500*12) //��8�º��ɿ������Flash��λ����У׼
//		{
//			Flash_EraseMotorParam();
//			System_Reset();
//		}
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		ledBlink = 1;
//	}
//	lastKeyState = keyState;
//}

//������LED��ʱ������˸������ʾ���ID
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

////���ת�ټ���
//void Motor_SpeedCalcProcess()
//{
//	const uint8_t speedLpfLen = 5;
//	static float speedLpfBuf[speedLpfLen] = {0}; //���5��filteredAngle
//	
//	float angle = filteredAngle;
//	float curSpeed = (angle-speedLpfBuf[0])*1000/speedLpfLen/_2PI*60;
//	speed = curSpeed;
//	
//	for(uint8_t i=0; i<speedLpfLen-1; i++)
//		speedLpfBuf[i] = speedLpfBuf[i+1];
//	speedLpfBuf[speedLpfLen-1] = angle;
//}

////CAN���߼�⣬500msû�յ�CAN�ź���ͣ��
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


//�ڵδ�ʱ���е��ã�1ms���ڣ��������ʱ����
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

uint8_t rx_byte; // ���ڴ洢���յ���һ���ֽ�
extern char show_one_flag_2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // �������ض��Ĵ����жϣ��� USART1��������മ�ڳ�ͻ
    if (huart->Instance == USART1)
    {
        // ��̬��������״̬��
        static char command_buffer[5];         // �洢������5�ַ�����
        static uint8_t buffer_index = 0;       // ��ǰ����������
        static enum { WAIT_COMMAND, WAIT_EQUAL, WAIT_DATA } state = WAIT_COMMAND; // ״̬��״̬

        // ��ȡ���յ����ֽ�
        char received = rx_byte;

        switch(state)
        {
            case WAIT_COMMAND:
                // ����Ƿ�Ϊ��Ч��������ĸ��M, C, A, S, Z, G����֧�ִ�Сд
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
                // �������������ĸ������Ը��ֽڣ������ȴ�
                break;

            case WAIT_EQUAL:
                if (received == '=')
                {
                    state = WAIT_DATA;
                }
                else
                {
                    // ������� '=', ������״̬��
                    buffer_index = 0;
                    state = WAIT_COMMAND;
                }
                break;

            case WAIT_DATA:
                // �������ݲ��ֵ������ַ�
                if (buffer_index < 4)
                {
                    command_buffer[buffer_index++] = received;
                }

                // ��������4���ַ�������������ĸ������������
                if (buffer_index >= 4)
                {
                    command_buffer[4] = '\0'; // ����ַ�����������ȷ����ȫ

                    // ��ȡ������ĸ�����ݲ���
                    char cmd = command_buffer[0];
                    char data_str[4];
                    memcpy(data_str, &command_buffer[1], 3);
                    data_str[3] = '\0'; // ȷ���ַ�������

                    // �����ݲ���ת��Ϊ����
                    int value = atoi(data_str);
									
										Uart_Send_Flag = 1 ;// ˢ�´��ڴ�ӡ����
									
                    // �����������͸�����Ӧ�ı���
                    switch(cmd)
                    {
                        case 'M':
                        case 'm':
                            // ���� pattern ģʽ����������Ҫ��Ϊ 1xxx
                            pattern = 1000 + value;
                            break;

                        case 'C':
                        case 'c':
                            // ����Ŀ�����ָ�������ת��Ϊ������ xx.x
                            Motor_Target_current[0] = value ;
                            break;

                        case 'A':
                        case 'a':
                            // ����Ŀ��Ƕ�ָ�������ת��Ϊ������ xx.x
                            Motor_Target_angle[0] = value / 10.0f;
                            break;

                        case 'S':
                        case 's':
                            // ����Ŀ���ٶ�ָ�����ֱ�Ӹ�ֵΪ������
                            Motor_Target_speed[0] = (float)value/10.f;
                            break;

                        case 'D':
                        case 'd':
                            // ��������ָ�ֱ�Ӹ�ֵΪ����
												Damping_Vale = (float)value/10.f;//ֻ�ܷ���1������Ҫ/10
                            break;

                        case 'G':
                        case 'g':
                            // ���Ķ൵���ص�λָ�ֱ�Ӹ�ֵΪ����
                            Gear_Switch_Number = value;
                            break;

                        default:
                            // δ֪�������
                            break;
                    }
										Target_angle = Motor_Target_angle[0];			//�趨Ŀ��Ƕ�
										Target_current = Motor_Target_current[0];	//�趨Ŀ�����
										Target_speed 	= Motor_Target_speed[0];			//�趨Ŀ���ٶ�
										show_one_flag_2=1;
                    // ������һ���������������״̬��
                    buffer_index = 0;
                    state = WAIT_COMMAND;
                }
                break;

            default:
                // Ĭ������£�����״̬��
                buffer_index = 0;
                state = WAIT_COMMAND;
                break;
        }

        // �������������жϣ��Ա����������һ���ֽ�
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
			
				case 1000:// ���ٸ�Ť��
					printf("Low speed high torque    <<%4d>> \r\n ",pattern);

					break;
				case 1010:// λ��ģʽ+ ��������
					printf("position_loop +  Current_loop    <<%4d>> \r\n ",pattern);
					printf("Target_Angle:%.2f \r\n",Motor_Target_angle[0]);		
					printf("Max_Current: %.2f \r\n",Motor_Target_current[0]/10);					
					break;
				case 1020:// λ��ģʽ + �������� 
					printf("position_loop +  Resistance_feedback     <<%4d>> \r\n ",pattern);
					printf("Target_Angle: %.2f \r\n",Motor_Target_angle[0]);		
					printf("Max_Current:  %.2f \r\n",Motor_Target_current[0]/10);	

				break;
			case 1030:// λ��ģʽ + �������� + �ٶ�����
					printf("position_loop + Current_loop +  speed_loop    <<%4d>> \r\n ",pattern);
					printf("Target_Angle: %.2f \r\n",Motor_Target_angle[0]);					
					printf("Max_Current:  %.2f \r\n",Motor_Target_current[0]/10);					
					printf("Max_Speed:    %.2f \r\n",Motor_Target_speed[0]);	

			break;				
			case 1040://  �ٶ�ģʽ + ��������
					printf("speed_loop +  Current_loop     <<%4d>> \r\n ",pattern);
					printf("Target_Speed: %.2f \r\n",Motor_Target_speed[0]);					
					printf("Max_Current:  %.2f \r\n",Motor_Target_current[0]/10);					

			break;				
			case 1050://ת����ť   ����ֹͣ////////////////////////////////////////////////////////////////
					printf("Slow stop knob    <<%4d>> \r\n ",pattern);

			break;
			case 1060://ת����ť   ��ֹͣ////////////////////////////////////////////////////////////////
					printf("No stop knob    <<%4d>> \r\n ",pattern);

			break;
			case 1070://������ť////////////////////////////////////////////////////////////////
					printf("Resistance knob    <<%4d>> \r\n ",pattern);		
					printf("Damping_Vale: %.2f \r\n",Damping_Vale);		

				break;
			case 1080://�൵����////////////////////////////////////////////////////////////////
					printf("limited Multispeed switch    <<%4d>> \r\n ",pattern);					
					printf("Number: %2d \r\n",Gear_Switch_Number);		

				break;
			case 1090://�൵����////////////////////////////////////////////////////////////////
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
	
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  // ���������жϣ�ÿ�ν���һ���ֽ� 
	
	// ��ʼ������������������������������������ֵ0.001ŷķ���˷�����20����A�ࡢB���C���ʼ��������Ϊ0
	InlineCurrentSense(0.001, 20, 0, 0, 0); // SimpleMotor

	// ��ʼ��ADC���ڵ���������������ƫ�õ�ѹУ׼����ȷ����������
	InlineCurrentSense_Init();

	// �趨��ѹ����Ϊϵͳ���������ѹֵ
	voltage_limit = MAX_VOLT;

	// ����ֱ�ᣨd�ᣩ�ͽ��ᣨq�ᣩ������PID���������������Ϊ����ѹ
	PID_current_q.limit = voltage_limit;
	PID_current_d.limit = voltage_limit;

	// �����ٶ�PID���������������Ϊ����ѹ��������������ӵ����б��Ϊ50������ƽ���ٶȱ仯��
	PID_velocity.limit = voltage_limit;
	PID_velocity.output_ramp = 50;

	// ���ýǶ�P���������������Ϊ����ѹ
	P_angle.limit = voltage_limit;

	// ��ʼ������PID������
	PID_init();


// ��ʼ��FOC�㷨�����ڵ����ʸ������
	SimpleFOC_Init();
	
//	motorID=1;
//	Flash_SaveMotorID(motorID);
//	printf("motorID = %d\r\n",motorID);

	// ���ſ�����Ч��ʹ�÷�������˳�򲥷�����H1, H3, H5������ʱ��ֱ�Ϊ200ms, 200ms, 500ms
	Beep_PlayNotes(3, (uint16_t[][2]){{T_H1, 200}, {T_H3, 200}, {T_H5, 500}});

	// ��ʼ���������˲��������ڽǶȹ��ƣ��������������͹������
	Kalman_Init(&angleFilter, 0.0005f, 0.1f);

	// ��ʼ���������˲������������ٶȹ��ƣ��������������͹������
	Kalman_Init(&kalman_shaft_velocity_Filter, 0.0005f, 0.1f);

	// ��ʼ���������˲��������ڽ������Iq�Ĺ��ƣ��������������͹������
	Kalman_Init(&kalman_current_Iq_Filter, 0.0005f, 0.1f); 

	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);//ENABNLE Motor
	
	OLED_Clear();
	
	KalMan_PramInit(&KalMan_current_Iq);	//�ѿ�������ϵ��ȫ������һ�飬��������˲����õ���Щ����

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		
		setTargetVotage(targetVotage); //�趨FOC��ѹ
		loopFOC(); //����FOC�㷨
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

