
#include "InlineCurrentSense.h"
#include "main.h"
#include "adc.h"
#include "foc_utils.h"
#include "stdio.h"
/************************************************
电流采样底层函数

本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/119220638
创建日期：20210905
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
int pinA,pinB,pinC;
float gain_a,gain_b,gain_c;
float offset_ia,offset_ib,offset_ic;
/******************************************************************************/
void Current_calibrateOffsets(void);


/******************************************************************************/
void InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC)
{
	float volts_to_amps_ratio;
//	
//	pinA = _pinA;
//	pinB = _pinB;
//	pinC = _pinC;
//	
	volts_to_amps_ratio = 1.0 /_shunt_resistor / _gain; // volts to amps
//	
	gain_a =-volts_to_amps_ratio/5;
	gain_b = volts_to_amps_ratio/5;
	gain_c = volts_to_amps_ratio/5;
//	gain_a = 1;
//	gain_b = 1;
//	gain_c = 1;
//	
	printf("gain_a:%.2f,gain_b:%.2f,gain_c:%.2f.\r\n",gain_a,gain_b,gain_c);
}
/******************************************************************************/
void InlineCurrentSense_Init(void)
{
//	configureADCInline(pinA,pinB,pinC);
	Current_calibrateOffsets();   //检测偏置电压，也就是电流0A时的运放输出电压值，理论值=1.65V
}
/******************************************************************************/
// Function finding zero offsets of the ADC

typedef struct{
	float adc_5;
	float adc_6;
} adc_56;

adc_56 ADC_56;
float Temperature;

adc_56 _readADCVoltageInline()
{	
	adc_56 adc_;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,50);
	adc_.adc_6=(float)HAL_ADC_GetValue(&hadc1)*33/40960;
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,50);
	adc_.adc_5=(float)HAL_ADC_GetValue(&hadc1)*33/40960;
	HAL_ADC_Stop (&hadc1);
	
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1,50);
//	Temperature=HAL_ADC_GetValue(&hadc1);
//	HAL_ADC_Stop (&hadc1);

	
  return adc_;
}



void Current_calibrateOffsets(void)
{
	int i;
	offset_ia=0;
	offset_ib=0;
	offset_ic=0;
	// read the adc voltage 1000 times ( arbitrary number )
	for(i=0; i<1000; i++)
	{
		ADC_56=_readADCVoltageInline();
		offset_ia += ADC_56.adc_5;
		offset_ib += ADC_56.adc_6;
		HAL_Delay(1);
	}
	// calculate the mean offsets
	offset_ia = offset_ia/1000;
	offset_ib = offset_ib/1000;
	
	printf("offset_ia:%.4f,offset_ib:%.4f,offset_ic:%.4f.\r\n",offset_ia,offset_ib,offset_ic);
}
/******************************************************************************/
// read all three phase currents (if possible 2 or 3)


PhaseCurrent_s getPhaseCurrents(void)
{
	PhaseCurrent_s current;
	ADC_56=_readADCVoltageInline();
	current.b = (ADC_56.adc_5 - offset_ia)*gain_a;// amps
	current.a = (ADC_56.adc_6 - offset_ib)*gain_b;// amps
	current.c = 0; // amps
	
	return current;
}
/******************************************************************************/





