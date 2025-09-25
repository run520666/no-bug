/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "ps2.h"
#include "pid.h"
#include "usart.h"
#include "mecanum_control.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHEEL_FR 0  // 右前�?
#define WHEEL_FL 1  // 左前�?
#define WHEEL_BL 2  // 左后�?
#define WHEEL_BR 3  // 右后�?
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_TypeDef motor_pid[4];
char tx_buffer[1000];
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;  // 确保你已经在别处定义了这个句�?
extern volatile uint8_t uart_tx_done;
extern void UART1_Send_DMA(uint8_t *buf, uint16_t len);
mecanum_control_t mecanum;

 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART1_Send_DMA(uint8_t *buf, uint16_t len);
void UART1_Send_IT(uint8_t *buf, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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

//float wheel_speed_FR;
//float wheel_speed_FL;
//float wheel_speed_BL;
//float wheel_speed_BR;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim2);
can_filter_init();
  
  // 修改 PID 初始化代码
  for (int i = 0; i < 4; i++) {
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i], PID_Speed, 2000, 500, 10, 10, 4000, 500, 3.7, 0.1, 0.05);    
        // PID模式（位置或速度）、最大输出、积分限幅、死区（绝对值）、控制周期、最大误差、目标值、kp、ki、kd）
        //开始版本kp=2.5，ki=0.1
        //初步调试kp=3.7，ki=0.1，kd=0.05
    motor_pid[i].target = mecanum.wheel_speed[i];
  }
  
  // 初始化麦克纳姆轮控制
  mecanum_init(&mecanum);
  uint32_t last_time = HAL_GetTick();
  uint8_t move_mode = 0; // 移动模式标志

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
    static uint32_t last_time = 0;
    static uint8_t demo_state = 0;
    uint32_t current_time = HAL_GetTick();
    */
    
    // 检测PA0状态

     uint32_t now = HAL_GetTick();
      if (now - last_time > 5000) // 每5秒切换一次
      {
        move_mode = (move_mode + 1) % 4;
        last_time = now;
      }

      switch (move_mode)
      {
        case 0:
          mecanum_move_forward(&mecanum, 250.0f); // 前进
          break;
        case 1:
          mecanum_move_backward(&mecanum, 250.0f); // 后退
          break;
        case 2:
          mecanum_move_left(&mecanum, 250.0f); // 左移
          break;
        case 3:
          mecanum_move_right(&mecanum, 250.0f); // 右移
          break;
      }

    // 更新PID控制
    for (int i = 0; i < 4; i++) 
    {
        motor_pid[i].target = mecanum.wheel_speed[i];
        const motor_measure_t *motor_data = get_chassis_motor_measure_point(i);
        if (motor_data) {
            motor_pid[i].f_cal_pid(&motor_pid[i], motor_data->speed_rpm);
        }
    }
    // 发送电机控制指令
    CAN_cmd_chassis(motor_pid[0].output, motor_pid[1].output, motor_pid[2].output, motor_pid[3].output);
    
    // VOFA绘图数据发送（只在VOFA模式下发送）
    // 发送8个通道：4个实际速度值 + 4个设定速度值
    /*
    float vofa_data[8];
    for (int i = 0; i < 4; i++) {
        vofa_data[i] = (float)get_chassis_motor_measure_point(i)->speed_rpm;     // 实际速度
        vofa_data[i + 4] = (float)motor_pid[i].target;                          // 设定速度
    }
    // 发送浮点数组
    HAL_UART_Transmit(&huart1, (uint8_t*)vofa_data, sizeof(vofa_data), 100);
    // 发送帧尾
    unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart1, tail, 4, 100);
    HAL_Delay(10); // 延迟10毫秒
    */
    
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 定时器中断回调，定时发�?�VOFA数据（float格式，VOFA可直接画图）
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim == &htim2)
//    {
//        float vofa_data[4];
//        for (int i = 0; i < 4; i++) {
//            vofa_data[i] = (float)get_chassis_motor_measure_point(i)->speed_rpm;
//        }
//        // 发�??4个float数据到VOFA+
//        HAL_UART_Transmit(&huart1, (uint8_t*)vofa_data, sizeof(vofa_data), 100);
//    }
//}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{

//if (htim == (&htim2))
//{
//			for(int i=0; i<4; i++)
//{	
//  motor_pid[i].target = 1000;
//		const motor_measure_t *motor_data = get_chassis_motor_measure_point(i);				
//			motor_pid[i].f_cal_pid(&motor_pid[i],motor_data->speed_rpm);    //根据设定值进行PID计算�?
//}
//CAN_cmd_chassis(motor_pid[0].output,motor_pid[1].output,motor_pid[2].output,motor_pid[3].output);
//}
//}


// ? DMA 发�?�完成回�?
//void USART1_IRQHandler(void)
//{
//    /* TXE 空中�? */
//    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
//    {
//        if (tx_buf_len > 0)
//        {
//            USART1->DR = *tx_buf_ptr++;   // 发�?�一个字�?
//            tx_buf_len--;
//        }
//        else
//        {
//            /* 发完了，关闭 TXE 中断，防止一直进中断 */
//            __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
//            tx_busy = 0;
//        }
//    }

//    /* 如有 RX 中断，可在此处理 */
//}

// 定时器中断回调，定时发�?�VOFA数据
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
       char buf[100];
    for (int i = 0; i < 4; i++) {
        int len = snprintf(buf, sizeof(buf), "M%d:%d\r\n", i, get_chassis_motor_measure_point(i)->speed_rpm);
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
   }
    HAL_Delay(500);
    
    }
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */

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
 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
