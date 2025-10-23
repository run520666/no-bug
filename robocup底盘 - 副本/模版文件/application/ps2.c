#include "PS2.h"
#include "usart.h"
#include "gpio.h"

/*********************************************************
File:PS2????
Author:pinggai    Version:1.0     Data:2015/05/16
Description: PS2????
**********************************************************/	

//???????    ??  DI->PA0
//                   ??  DO->PA1    CS->PA2     CLK->PA3

u16 Handkey;
u8 Comd[2]={0x01,0x42};	//?????????
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //??????

void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
	{
		;
	}
}

u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//???????


//???????

void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //???????
		}
		else DO_L;

		CLK_H;                        //????
		delay_us(5);
		CLK_L;
		delay_us(5);
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
	delay_us(16);
}

//?????????,0x41=????,0x73=????
//???;0,????
//		  ??,????

u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //????
	PS2_Cmd(Comd[1]);  //????
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//??????
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //????
	PS2_Cmd(Comd[1]);  //????
	for(byte=2;byte<9;byte++)          //??????
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			delay_us(5);;
			CLK_L;
			delay_us(5);;
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(16);
	}
	CS_H;
}

//?????PS2???????,???????
//????????????0, ????1
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //??16???  ???0, ????1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //????????
}

//??????????	 ??0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//???????
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: ??????,
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:??????? 0x00?,???
	   motor2:??????? 0x40~0xFF ???,??? ????
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
  PS2_Cmd(0x01);  //????
	PS2_Cmd(0x42);  //????
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
//short poll
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
//????
void PS2_EnterConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//??????
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  ????????
	PS2_Cmd(0xEE); //Ox03????,???????“MODE”?????
				         //0xEE???????,?????“MODE”?????
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//????
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
//???????
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
//???????
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//??????
	PS2_TurnOnAnalogMode();	//“???”????,???????
	PS2_VibrationMode();	//??????
	PS2_ExitConfing();		//???????
}

int mapJoystickToMotorCurrent127(int joystickValue) {
    // ?????
    int offset = joystickValue - 127;

    // ???????????:-800 ? 800
    int motorCurrent = offset * (800 / 127);  

    return motorCurrent;
}
int mapJoystickToMotorCurrent128(int joystickValue) {
    // ?????
    int offset = joystickValue - 128;

    // ???????????:-800 ? 800
    int motorCurrent = offset * (800 / 128);  

    return motorCurrent;
}
