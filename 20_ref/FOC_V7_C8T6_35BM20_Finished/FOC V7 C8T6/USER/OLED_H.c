#include "OLED_H.h"

#include "oledfont.h"
//#include "delay.h"
#include "stm32f1xx_hal.h"
#include "main.h"
/*
更新时间：2023-7-18
更新内容：所有的命名都规范化了
					用宏定义来切换屏幕型号
*/
#define OLED_4Pin 
//#define OLED_6Pin 
//#define OLED_7Pin 

#ifdef OLED_6Pin




#endif


#ifdef OLED_4Pin


#define OLED_SCLK_Set HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,GPIO_PIN_SET);
#define OLED_SCLK_Clr HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,GPIO_PIN_RESET);
#define OLED_SDIN_Set HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,OLED_SDA_Pin,GPIO_PIN_SET);
#define OLED_SDIN_Clr HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,OLED_SDA_Pin,GPIO_PIN_RESET);




void Write_IIC_Byte2(unsigned char j)
{
	unsigned char i;
	OLED_SCLK_Clr;
	for(i=0;i<8;i++)		
	{
		
		if(j&0x80)
		{OLED_SDIN_Set;}
		else OLED_SDIN_Clr;
		j=j<<1;
		OLED_SCLK_Set;
		OLED_SCLK_Clr;
	}
}

void OLED_WR_Byte(uc j,uc k)
{	
	OLED_SCLK_Set;
	OLED_SDIN_Set;
	OLED_SDIN_Clr;
	OLED_SCLK_Clr;
	
	Write_IIC_Byte2(0x78);	OLED_SCLK_Set;OLED_SCLK_Clr;
	Write_IIC_Byte2(0x40*k);	OLED_SCLK_Set;OLED_SCLK_Clr;
	Write_IIC_Byte2(j);		OLED_SCLK_Set;OLED_SCLK_Clr;
	
	OLED_SCLK_Set;
	OLED_SDIN_Clr;
	OLED_SDIN_Set;
} 
#endif

void OLED_Set_Pos(uc r,uc c)
{
	OLED_WR_Byte(0xb0+r,0);
	OLED_WR_Byte(((c&0xf0)>>4)|0x10,0);
	OLED_WR_Byte((c&0x0f)|0x01,0); 
}

void OLED_Clear()
{
	uc i,j;
	OLED_Init_H();
	f8
	{
		OLED_Set_Pos(i,0);
		for(j=0;j<128;j++)OLED_WR_Byte(0,1);
	
	}
}
void OLED_Write_AnyWord(uc r,uc c,uc *s,uc z,ul n,...){
	uc i,j,a,b,w=16,e=2;ul *p=&n;	
	if(s==word_num_8x16)w=8;
	if(s==word_num_8x8)w=8,e=1;
	a=w*e;
	z=z*w;
	for(i=0;i<e;i++)
	{
		OLED_Set_Pos(r+i,c);
		b=i*w;
		for(j=0;j<z;j++)
		{
			uc dd;
			dd=s[*(p+j/w)*a+b+j%w];

			OLED_WR_Byte(dd,1);
		}
	}
}
void Oled_Show_ASCII_8x16(uc r,uc c,uc z,uc n){
	uc i,j;unsigned char p=n-32;	
	uc dd;
	for(i=0;i<2;i++)
	{
		OLED_Set_Pos(r+i,c);
		for(j=0;j<8;j++)
		{
			dd=oled_asc2_1608[p*16+i*8+j];
			OLED_WR_Byte(dd,1);
		}
	}
}



void OLED_Show_String(uc r,uc c, char *s){
	uc i;
	for(i=0;s[i]!='\0';i++)
	{
		if(s[i]<0x80)
		{
			Oled_Show_ASCII_8x16(r,c,1,s[i]);
			c+=8;
		}

	}

}

#define OLED_096 
//#define OLED_091 

void OLED_Init_H()
{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); 
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//因为PA15  PB3  PB4这三个是特殊引脚是JTAG下载口之类的。STM32上电默认就是，要把这三个口复位一下，才能做普通IO口用
	
//	OLED_Pin_Init();
	

	OLED_WR_Byte(0xAE,0);//--turn off oled panel
	OLED_WR_Byte(0x00,0);//---set low column address
	OLED_WR_Byte(0x10,0);//---set high column address
	OLED_WR_Byte(0x40,0);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WR_Byte(0x81,0);//--set contrast control register
	OLED_WR_Byte(0xCF,0);// Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1,0);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WR_Byte(0xC8,0);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常r
	OLED_WR_Byte(0xA6,0);//--set normal display
	OLED_WR_Byte(0xA8,0);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f,0);//--1/64 duty
	OLED_WR_Byte(0xD3,0);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00,0);//-not offset
	OLED_WR_Byte(0xd5,0);//--set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(0x80,0);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_Byte(0xD9,0);//--set pre-charge period
	OLED_WR_Byte(0xF1,0);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA,0);//--set com pins hardware configuration
	OLED_WR_Byte(0x12,0);
	OLED_WR_Byte(0xDB,0);//--set vcomh
	OLED_WR_Byte(0x40,0);//Set VCOM Deselect Level
	OLED_WR_Byte(0x20,0);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,0);//
	OLED_WR_Byte(0x8D,0);//--set Charge Pump enable/disable
	OLED_WR_Byte(0x14,0);//--set(0x10) disable
	OLED_WR_Byte(0xA4,0);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6,0);// Disable Inverse Display On (0xa6/a7) 
	OLED_WR_Byte(0xAF,0);


	//OLED_Clear();
}


uint8_t zhi[6];
void calculate(ul k,uc z)
{
	zhi[0]=k/100000%10;
	zhi[1]=k/10000%10;
	zhi[2]=k/1000%10;
	zhi[3]=k/100%10;
	zhi[4]=k/10%10;
	zhi[5]=k/1%10;
	
	if(z==1)
	{
		if(k<99999)zhi[0]=10;
		if(k<9999)zhi[1]=10;
		if(k<999)zhi[2]=10;
		if(k<99)zhi[3]=10;
		if(k<9)zhi[4]=10;
	}
	if(z==2)
	{
		uc i;
		f4 zhi[i]+=48;
	}
}

/*OLED小数显示 显示两位小数*/
void OLED_Show_Number_xiao(uint8_t r,uint8_t c,float number)
{
	uint8_t z;
	number=number*100;
	if(number<0) {z=1;number=-number;}
	else 	  z=0;		
	calculate(number,0);OLED_Write_AnyWord(r,c,word_num_8x16,8-2,z?12:10,zhi[2],zhi[3],11,zhi[4],zhi[5]);		
//	calculate(number,0);OLED_Write_AnyWord(r,c,word_num_8x16,8,z?12:10,zhi[0],zhi[1],zhi[2],zhi[3],11,zhi[4],zhi[5]);		
}

/*OLED数字显示  
uint8_t r	行0-7
uint8_t c	列0-127
uint8_t k	显示几位1-6
long number	显示的值  正负都可以
*/
void OLED_Show_Number(uint8_t r,uint8_t c,uint8_t k,long number)
{
 	uint8_t z;
	if(number<0) {z=1;number=-number;}
	else 	  z=0;		
	calculate(number,0);
	OLED_Write_AnyWord(r,c,word_num_8x16,k+1,z?12:10,zhi[6-k],zhi[7-k],zhi[8-k],zhi[9-k],zhi[10-k],zhi[11-k]);
}

void OLED_Show_Number_8x8(uint8_t r,uint8_t c,uint8_t k,long number)
{
	uint8_t z;
	if(number<0) {z=1;number=-number;}
	else 	  z=0;		
	calculate(number,0);
	OLED_Write_AnyWord(r,c,word_num_8x8,k+1,z?12:10,zhi[6-k],zhi[7-k],zhi[8-k],zhi[9-k],zhi[10-k],zhi[11-k]);
}
