/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
#include "usart.h"
#include "mecanum_control.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "hwt101.h"
#include "stp_23l.h"
#include "communication.h"
#include "ttl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*角度设定*/
fp32 t1=0.0f;
fp32 t2=90.0f;
fp32 t3=180.0f;
fp32 t4=-90.0f;

extern UART_HandleTypeDef huart7;  
extern UART_HandleTypeDef huart6;




/* pid部分 */
//速度环
extern q_pid speed_pid[4];
//角度环
extern q_pid angle_pid_s; //直线pid
extern q_pid angle_pid_t; //转向pid

/* 中断接收变量定义 */
//uart6接收缓冲区（左激光）
extern uint8_t uart6_rx_buf[1];
//陀螺仪
extern uint8_t g_usart7_receivedata;
//通信
extern uint8_t rxByte;

//光电标志位
extern uint8_t gd_l;
extern uint8_t gd_r;

/* text */
float vofa_data[3];





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
 

  can_filter_init();

  /* 中断初始化 */
    /* 开启 UART 接收中断 */
  HAL_UART_Receive_IT(&huart3, &rxByte, 1);
  HAL_UART_Receive_IT(&huart7, &g_usart7_receivedata, 1); // 开启串口7接收中断(陀螺仪)
  HAL_UART_Receive_IT(&huart6, uart6_rx_buf, 1); //左侧激光

  //定时器2中断
  HAL_TIM_Base_Start_IT(&htim2);

  /* PID初始化 */
  //速度环初始化
  for (int i = 0; i < 4; i++)
  { 
    speed_pid_init(&speed_pid[i]);
    set_speed_pid(&speed_pid[i], 10.7f, 0.63f, 0.5f, 6000.0f, 600.0f,0.0f);
                            //设置kp,   ki,    kd, 最大输出，最大积分， 死区
                            //初调kp=5.5,ki=0.4,kd=0.2,最大输出2500，积分300，死区10
                            //9.21 kp=10.7,ki=0.63,kd=0.5,最大输出6000，积分600，死区0,效果可以
    speed_pid[i].target = 0.0f; // 初始目标速度为
  }

  //直线角度环和转弯角度环初始化
  angle_controller_init(&angle_pid_s); //角度控制初始化
  angle_controller_init(&angle_pid_t); //角度控制初始化
  set_angle_pid(&angle_pid_s, 200.0f, 0.1f, 0.0f, 6000.0f, 470.0f);
//510*0.6
  set_angle_pid(&angle_pid_t, 60.0f, 0.1f, 0.0f, 6000.0f, 400.0f);
            //设置kp,   ki,   kd,  最大输出，最大积分
	//p50,output1000
	//p60.0f, 0.1f, 0.0f, 1500.0f, 200.0f

  /* 麦克纳姆轮控制初始化 */
  // 初始化麦克纳姆轮控制
  mecanum_init(&mecanum);

  /* 外设初始化 */
  //stp32初始化
  STP23L_Init(&huart6);


  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	HAL_Delay(1000);
//	hwt101_restart(&huart7);
//  set_target_move_to_target(&mecanum,430.0f,0.0f,t1,4000.0f,1,1);
//  HAL_Delay(10);
//  set_target_move_to_target(&mecanum,0.0f,-800.0f*1.03f,t1,4000.0f,1,1); //到达取货站
//  HAL_Delay(10);
//  set_target_move_to_target(&mecanum,0.0f,-340.0f,t1,3000.0f,1,1);
//  HAL_Delay(10);
//  set_target_move_to_target(&mecanum,624.5f,0.0f,t1,3000.0f,1,1); //避障起点
//  HAL_Delay(10);
  move_x(&mecanum,t1,1000.0f);
  HAL_Delay(250);
  move_y(&mecanum,t1,1000.0f);
  HAL_Delay(250);
  set_target_move_to_target(&mecanum,200,0.0f,t1,1000.0f,0,1); //第一个货站
while (1)
{
  UART_SendData(&huart3, gd_l,gd_r,1);
  processReceivedData();
  HAL_Delay(5);
  if(recivedata)
  {
    // 处理接收到的数据
    gd_l=0;
    gd_r=0;
    break;
  }
}



//  set_target_move_to_target(&mecanum,461.0f,0.0f,t1,3000.0f,1,1);  //避障起点
//  HAL_Delay(10);
//  set_target_move_to_target(&mecanum,0.0f,230.0f,t1,3000.0f,1,1);  //左移
//  HAL_Delay(10);
//  set_target_move_to_target(&mecanum,884.0f,0.0f,t1,3000.0f,1,1);
//  HAL_Delay(10);
//  set_target_move_to_target(&mecanum,0.0f,-155.0f+30.0f,t1,3000.0f,1,1);  //右移,开始是225，多了
//  HAL_Delay(10);
//  set_target_move_to_target(&mecanum,126.0f,0.0f,t1,3000.0f,1,1);
//  HAL_Delay(10);
//  move_x(&mecanum,t1,1000.0f);
//  HAL_Delay(250);
//  move_y(&mecanum,t1,1000.0f);
//  HAL_Delay(250);
//  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t1,1000.0f,0,1);  //第二个货站
//	HAL_Delay(100);

//	
//  set_target_move_to_target(&mecanum,810.0f,0.0f,t1,3000.0f,1,1);
//  HAL_Delay(10);
//	
//  move_angle(&mecanum,t2);  //第一次转弯
//  HAL_Delay(500);
//	
//  set_target_move_to_target(&mecanum,69.5f,0.0f,t2,3000.0f,1,1);
//  HAL_Delay(10);
//  move_x(&mecanum,t2,800.0f);
//  HAL_Delay(500);
//  move_y(&mecanum,t2,800.0f);
//  HAL_Delay(500);
//  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t2,800.0f,0,1); //第三个货站
//	HAL_Delay(100);

//	
//  set_target_move_to_target(&mecanum,470.0f,0.0f,t2,3000.0f,0,0);
//  HAL_Delay(10);
//  move_x(&mecanum,t2,800.0f);
//  HAL_Delay(500);
//  move_y(&mecanum,t2,800.0f);
//  HAL_Delay(500);
//  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t2,3000.0f,0,1); //第四个货站
//	HAL_Delay(100);

//	
//  
//  set_target_move_to_target(&mecanum,400.0f,0.0f,t2,3000.0f,1,1);
//  HAL_Delay(10);
//  move_angle(&mecanum,t3);  //第二次转弯
//  HAL_Delay(500);

//  set_target_move_to_target_up(&mecanum,1415.0f+70.0f,0.0f,t3,4000.0f,1,1); //上跷跷板
//  HAL_Delay(10);
//	
//  hwt101_restart(&huart7); //重启hwt101(内部有haldelay)，注意角度
//	
//  set_target_move_to_target(&mecanum,515.0f,0.0f,t1,3000.0f,1,1);
//  HAL_Delay(10);
//	
//  move_angle(&mecanum,t2);  //第三次转弯
//  HAL_Delay(500);
//  
//  set_target_move_to_target(&mecanum,450.5f,0.0f,t2,3000.0f,1,1);
//  move_x(&mecanum,t2,1000.0f);
//  HAL_Delay(500);
//  move_y(&mecanum,t2,1000.0f);
//  HAL_Delay(500);
//  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t2,1000.0f,0,1);//第五个货站
//	HAL_Delay(100);

//	
//  set_target_move_to_target(&mecanum,-740.0f,0.0f,t2,3000.0f,1,1);
//  HAL_Delay(10);

//  move_angle(&mecanum,t1);  //第四次转弯 180度
//  HAL_Delay(500);

//  set_target_move_to_target(&mecanum,253.0f,0.0f,t1,3000.0f,1,1);
//  HAL_Delay(10);
//  move_x(&mecanum,t1,1000.0f);
//  HAL_Delay(500);
//  move_y(&mecanum,t1,1000.0f);
//  HAL_Delay(500);
//  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t1,1000.0f,0,1); //第六个货站
//  HAL_Delay(100);

//	
//  set_target_move_to_target(&mecanum,1420.0f,0.0f,t1,3000.0f,1,1); //回家
//  HAL_Delay(10);
//  set_target_move_to_target(&mecanum,0.0f,460.0f,t1,3000.0f,1,1); //左移
//  HAL_Delay(10);
//  
//	while(1){}







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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
