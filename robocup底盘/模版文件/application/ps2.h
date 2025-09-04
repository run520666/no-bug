#ifndef  __PS2_H
#define  __PS2_H

/*********************************************************
Copyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File:PS2????
Author:pinggai    Version:1.1     Data:2015/10/20
Description: PS2????
             ????:
			 1?????“????”?“????”,?????“??”,????“????”????
			 2???????:???????,??????????????
			                  ???????,??????????
History:  
V1.0: 	2015/05/16
1?????,?????,??????       
**********************************************************/	 
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "gpio.h"

typedef uint16_t u16;
typedef uint8_t u8;

#define DI    HAL_GPIO_ReadPin(DAT_GPIO_Port,DAT_Pin)                      //PA0  ??
                                                                           
#define DO_H  HAL_GPIO_WritePin(CMD_GPIO_Port,CMD_Pin,GPIO_PIN_SET);       //????
#define DO_L  HAL_GPIO_WritePin(CMD_GPIO_Port,CMD_Pin,GPIO_PIN_RESET);     //????
                                                                           
#define CS_H  HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_SET);         //CS??
#define CS_L  HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_RESET);       //CS??
                                                                           
#define CLK_H HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);       //????
#define CLK_L HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);     //????


//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2         9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      26

void delay_us(uint32_t us);

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //???X???
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;

void PS2_Init(void);
u8 PS2_RedLight(void);   //?????????
void PS2_ReadData(void); //?????
void PS2_Cmd(u8 CMD);		  //???????
u8 PS2_DataKey(void);		  //?????
u8 PS2_AnologData(u8 button); //??????????
void PS2_ClearData(void);	  //???????
void PS2_Vibration(u8 motor1, u8 motor2);//????motor1  0xFF?,???,motor2  0x40~0xFF

void PS2_EnterConfing(void);	 //????
void PS2_TurnOnAnalogMode(void); //?????
void PS2_VibrationMode(void);    //????
void PS2_ExitConfing(void);	     //????
void PS2_SetInit(void);		     //?????
int mapJoystickToMotorCurrent128(int joystickValue);
int mapJoystickToMotorCurrent127(int joystickValue);
#endif
