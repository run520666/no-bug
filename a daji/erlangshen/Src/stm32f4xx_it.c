/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mecanum_control.h"  
#include "hwt101.h"
#include "stp_23l.h"
#include "pid.h"
#include "CAN_receive.h"
#include "communication.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */
// HWT101相关全局变量
uint8_t g_usart7_receivedata = 0; // 接收数据

//激光传感器变量
extern uint8_t uart6_rx_buf[1];  //左激光接收缓冲
extern uint8_t uart8_rx_buf[1];  //右激光接收缓冲

//通信变量
extern uint8_t rxByte;

extern mecanum_control_t mecanum;
extern q_pid speed_pid[4];

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
   while (1)
  {
  }
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
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */

  /* USER CODE END UART7_IRQn 0 */
  HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */

  /* USER CODE END UART7_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */

  /* USER CODE END UART8_IRQn 0 */
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */

  /* USER CODE END UART8_IRQn 1 */
}

/* USER CODE BEGIN 1 */

//定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        if (htim == &htim2)
    {
      
      for (int i = 0; i < 4; i++)
      { 
        speed_pid[i].target = mecanum.wheel_speed[i];
        const motor_measure_t *motor_data = get_chassis_motor_measure_point(i);
        speed_pid_control(&speed_pid[i], speed_pid[i].target, motor_data->speed_rpm);
      }
        CAN_cmd_chassis(speed_pid[0].output,
                        speed_pid[1].output,
                        speed_pid[2].output,
                        speed_pid[3].output);
    }
}

//左激光传感器
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart7)
  {
    hwt101_ReceiveData(g_usart7_receivedata); //调用处理函数，存入了麦轮结构体
    HAL_UART_Receive_IT(&huart7, &g_usart7_receivedata, 1);//继续进行中断接收
  }
  if(huart == &huart6)
  {
    STP23L_RxCallback(uart6_rx_buf[0], 0);  /* 调用 STP-23L 解析函数 */
    HAL_UART_Receive_IT(&huart6, uart6_rx_buf, 1);  /* 重新开启接收中断 */
  }
  if(huart == &huart8)
  {
    STP23L_RxCallback(uart8_rx_buf[0], 1);  /* 调用 STP-23L 解析函数 */
    HAL_UART_Receive_IT(&huart8, uart8_rx_buf, 1);//继续进行中断接收
  }
  if(huart == &huart3){
   // 状态机接收
        if (rxIndex == 0 && rxByte == FRAME_HEAD) {
            // 检测到帧头
            rxBuffer[rxIndex++] = rxByte;
        } else if (rxIndex > 0 && rxIndex < 5) {
            rxBuffer[rxIndex++] = rxByte;
            
            // 接收完整帧（5字节）
            if (rxIndex == 5 && rxByte == FRAME_TAIL) {
                frameReceived = 1;
            }
            
            // 接收错误，重置
            if (rxIndex == 5 && rxByte != FRAME_TAIL) {
                rxIndex = 0;
            }
        } else {
            // 异常，重置
            rxIndex = 0;
        }
        
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart3, &rxByte, 1);
  }

}

/* USER CODE END 1 */
