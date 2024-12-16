#ifndef __OLED_H
#define __OLED_H
//#include "sys.h"
#include "main.h"

//#define OLED_SCK_Pin 	 GPIO_PIN_1
//#define OLED_SDA_Pin 	 GPIO_PIN_0 
//#define OLED_Port    	 GPIOA
//#define OLED_Port_RCC	 RCC_APB2Periph_GPIOA

#define uc unsigned char
#define ui unsigned short int
#define ul unsigned long
#define f8 for(i=0;i<8;i++)
#define f4 for(i=0;i<4;i++)
#define f(k) for(i=0;i<k;i++)



void OLED_Init_H(void);
void OLED_Show_String(uc r,uc c, char *s);
void OLED_Write_AnyWord(uc r,uc c,uc *s,uc z,ul n,...);
void OLED_Clear(void);
void OLED_Set_Pos(uc r,uc c);
void OLED_WR_Byte(uc j,uc k);
//void OLED_Pin_Init(void);
void Calculate(ul k,uc z);
void OLED_Show_Number_xiao(uint8_t r,uint8_t c,float WW);
void OLED_Show_Number(uint8_t r,uint8_t c,uint8_t k,long WW);
void OLED_Show_Number_8x8(uint8_t r,uint8_t c,uint8_t k,long WW);
void Oled_Show_ASCII_8x16(uc r,uc c,uc z,uc n);

extern uc word_16x16[];
extern uc word_num_8x16[];
extern uc word_num_8x8[];

extern char f1,f2;
extern uint8_t zhi[6];
extern uc Data_oled[8][128];
#endif













