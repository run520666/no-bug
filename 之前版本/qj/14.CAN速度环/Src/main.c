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

// 麦克纳姆轮控制结构体
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
  
  // 修改 PID 初始化代�?
  for (int i = 0; i < 4; i++) {
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i], PID_Speed, 4000, 500, 10, 0, 4000, 500, 2.5, 0.1, 0);
    motor_pid[i].target = mecanum.wheel_speed[i];
  }
  
  // 初始化麦克纳姆轮控制
  mecanum_init(&mecanum);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 设置所有电机的目标速度为 1000 RPM
    for (int i = 0; i < 4; i++) {
        motor_pid[i].target = 1000; // 固定目标速度
    }

    // 调用 PID 控制函数，计算输出
    for (int i = 0; i < 4; i++) {
        const motor_measure_t *motor_data = get_chassis_motor_measure_point(i);
        motor_pid[i].f_cal_pid(&motor_pid[i], motor_data->speed_rpm);
    }

    // 发送控制命令到电机
    CAN_cmd_chassis(motor_pid[0].output, motor_pid[1].output, motor_pid[2].output, motor_pid[3].output);

    HAL_Delay(10); // 延时 10ms
  }
  /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
void send_motor_speeds_to_vofa(float motor1, float motor2, float motor3, float motor4) {
    char buffer[100];
    int len = snprintf(buffer, sizeof(buffer), "channels: %.2f,%.2f,%.2f,%.2f\n", motor1, motor2, motor3, motor4);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) { // 确保是 TIM2 的中断
        float motor1_speed = get_chassis_motor_measure_point(0)->speed_rpm;
        float motor2_speed = get_chassis_motor_measure_point(1)->speed_rpm;
        float motor3_speed = get_chassis_motor_measure_point(2)->speed_rpm;
        float motor4_speed = get_chassis_motor_measure_point(3)->speed_rpm;

        send_motor_speeds_to_vofa(motor1_speed, motor2_speed, motor3_speed, motor4_speed);
    }
}

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
