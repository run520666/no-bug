/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mecanum_control.h"  
#include "hwt101.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
// HWT101相关全局变量
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// HWT101相关全局变量
uint8_t g_usart1_receivedata = 0; // 接收数据
    int test = 0;
// 外部变量声明
extern mecanum_control_t mecanum;
//volatile int interrupt_count = 0;  // 全局变量，便于调试
volatile int uart_rx_count = 0;  // 全局变量，便于调试
uint8_t Serial_RxPack[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern uint8_t *tx_buf_ptr;
extern uint16_t tx_buf_len;
extern volatile uint8_t tx_busy;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  /*
static uint8_t S=0,i=0,sum_juge=0;
	
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==SET)
	{//每次使用ReceiveData都会自动清除中断标志位
		uint8_t Rxbyte = USART_ReceiveData(USART1);
		if(S==0&&Serial_RxFlag==0)/////如果已经读到一个数据但还没使用，那么暂时不用管读到的数据
		{
			if(Rxbyte==0x55) 
			{
				S=1;
				sum_juge+=0x55;
			}
		}
		else if(S==1)
		{
			if(Rxbyte==0x53) 
			{
				S=2;
				Serial_RxPack[i]=0x53;
				i++;
				sum_juge+=0x53;
			}else //返回状态0
			{
				S=0;sum_juge=0;i=0;
			}
		}
		else if(S==2)
		{
			if(i!=9)
			{
				Serial_RxPack[i]=Rxbyte;
				sum_juge+=Rxbyte;
				i++;
			}else
			{
				if(Rxbyte==sum_juge)//校验合格，数据正常
				{
					Serial_RxPack[i]=Rxbyte;
					Serial_RxPack[i+1]='\0';
					i=0;S=0;sum_juge=0;
					Serial_RxFlag=1;
				}else				//校验不合格，舍弃数据
				{
					S=0;i=0;sum_juge=0;
				}
			}
		}

	}
*/


//  interrupt_count++;
//   HAL_GPIO_TogglePin(ledG_GPIO_Port, ledG_Pin);
//    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
//  {
//     uart_rx_count++;
//    uint8_t dummy = (uint8_t)(huart1.Instance->DR & 0xFF);  // 读取数据（必须读取）
//    HAL_GPIO_TogglePin(ledB_GPIO_Port, ledB_Pin);          // 闪烁LED
//    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);        // 清除标志
//  }
    /*
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
    {
        if (tx_buf_len > 0)
        {
            USART1
->DR = *tx_buf_ptr++;
            tx_buf_len
--;
        }
        else
        {
            __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
            tx_busy 
= 0;
        }
    }
      */
    
    // static uint8_t S=0, i=0, sum_juge=0;
  
  /*
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
   {

    uint8_t Rxbyte = (uint8_t)(huart1.Instance->DR & 0xFF);
    HAL_GPIO_TogglePin(ledB_GPIO_Port, ledB_Pin);
    uart_rx_count++;

    // 使用您的原始状态机逻辑
    if(S==0 && Serial_RxFlag==0)
    {
      if(Rxbyte==0x55) 
      {
        S=1;
        sum_juge=0x55;
        i=0;
      }
    }
    else if(S==1)
    {
      if(Rxbyte==0x53) 
      {
        S=2;
        Serial_RxPack[i]=0x53;
        i++;
        sum_juge+=0x53;
      }else 
      {
        S=0;sum_juge=0;i=0;
      }
    }
    else if(S==2)
    {
      if(i!=9)
      {
        Serial_RxPack[i]=Rxbyte;
        sum_juge+=Rxbyte;
        i++;
      }else
      {
        if(Rxbyte==sum_juge) // 校验合格，数据正常
        {
          Serial_RxPack[i]=Rxbyte;
          Serial_RxPack[i+1]='\0';
          
          // 解析yaw数据并存储到mecanum结构体
          uint8_t YawL = Serial_RxPack[5];  // 偏航角低字节
          uint8_t YawH = Serial_RxPack[6];  // 偏航角高字节
          
          short yaw_raw = (short)((short)YawH << 8 | YawL);
          fp32 yaw_degree = (fp32)yaw_raw / 32768.0f * 180.0f;
          
          // 转换为0~359度范围（保持您的原始逻辑）
          if(yaw_degree >= -179.0f && yaw_degree <= -1.0f) 
              yaw_degree += 359.0f;
          if(yaw_degree < 0.0f)
              yaw_degree += 360.0f;
              
          // 直接存储到mecanum结构体
          mecanum.current_pos.yaw = yaw_degree;
          
          i=0;S=0;sum_juge=0;
          Serial_RxFlag=1;
        }else // 校验不合格，舍弃数据
        {
          S=0;i=0;sum_juge=0;
        }
      }
    }
    
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
  }  
  */  
   

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    test++;
    hwt101_ReceiveData(g_usart1_receivedata); //调用处理函数
    HAL_UART_Receive_IT(&huart1, &g_usart1_receivedata, 1);//继续进行中断接收

  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
