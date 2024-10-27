/**
 * @file app_init.c
 * @brief 
 * @author Leo ()
 * @version 1.0
 * @date 2024-10-27
 * 
 * @copyright Copyright (c) 2024  Leo 版权所有，禁止用于商业用途
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2024-10-27 <td>1.0     <td>Leo     <td>内容
 * </table>
 */
#include <stdio.h>
#include "app_init.h"
#include "foc.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

#define PI 3.1415926
uint32_t adc1_val_buf = 0;
uint16_t adc1_val_buf1[4] = {0};
float Theta_add = 0;
uint16_t count = 0;
/* External inputs (root inport signals with default storage) */
extern ExtU rtU;
/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY; 
extern DMA_HandleTypeDef hdma_adc1;
void R3_I_callback(struct __DMA_HandleTypeDef *hdma);
int fputc(int c,FILE *f);

void app_init(void)
{
	rtU.ud = 0;
	rtU.uq = 2;
	rtU.Tpwm = 3200;
	rtU.udc = 12;
	
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc1_val_buf1, 4);
    hdma_adc1.XferCpltCallback = R3_I_callback;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
}

void R3_I_callback(struct __DMA_HandleTypeDef *hdma)
{
	rtU.theta += 0.20;
	Theta_add += 0.01;
	if(rtU.theta >= 2.0f * PI)
	{
		rtU.theta -= 2.0f * PI;
	}
	foc_step();
	htim1.Instance->CCR1 = (uint16_t)rtY.Tcmp3;
	htim1.Instance->CCR2 = (uint16_t)rtY.Tcmp2;
	htim1.Instance->CCR3 = (uint16_t)rtY.Tcmp1;
//	
//	if(count>3000)
//	{
//		count=count;
//	}
//	else 
//	{
//		count++;
//	}
	printf("%d,%d,%d,%.1f\r\n",adc1_val_buf1[1],adc1_val_buf1[2],adc1_val_buf1[3],rtU.theta*100);
//	if(Theta_add > 28* PI)
//			{
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13/GPIO_PIN_14/GPIO_PIN_15,GPIO_PIN_RESET);
//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//			}
}

int fputc(int c,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&c,1,0xfff);
	return c;
}
