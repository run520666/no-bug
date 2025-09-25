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
#include "dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHEEL_FR 0  // 右前轮
#define WHEEL_FL 1  // 左前轮
#define WHEEL_BL 2  // 左后轮
#define WHEEL_BR 3  // 右后轮
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
fp32 t1=0.0f;
fp32 t2=90.0f;
fp32 t3=180.0f;
fp32 t4=-90.0f;
PID_TypeDef motor_pid[4];
char tx_buffer[1000];
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;  // 确保你已经在别处定义了这个句�?
extern UART_HandleTypeDef huart6;  // 确保你已经在别处定义了这个句�?

extern volatile uint8_t uart_tx_done;
extern void UART1_Send_DMA(uint8_t *buf, uint16_t len);
extern uint8_t g_usart1_receivedata;
extern q_pid speed_pid[4];
extern q_pid angle_pid_s; //直线pid
extern q_pid angle_pid_t; //转向pid
volatile uint32_t last_interrupt_time = 0; // 上次中断时间
volatile uint8_t gd_mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART1_Send_DMA(uint8_t *buf, uint16_t len);
void UART1_Send_IT(uint8_t *buf, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  //uint8_t move_mode = 0; // 移动模式标志
volatile uint8_t pid_flag = 0; // PID标志
volatile uint8_t stop=1; // 停止标志
//  static uint8_t test_state = 0;  // 0=前进, 1=后退
// static float start_distance = 0.0f;
volatile uint8_t gd_r_test=1;
volatile uint8_t gd_l_test=1;
float vofa_data[8];
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
  MX_DMA_Init();   

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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim2);
can_filter_init();
HAL_UART_Receive_IT(&huart1, &g_usart1_receivedata, 1); // 开启串口1接收中断




  // 修改 PID 初始化代码
//  for (int i = 0; i < 4; i++) {
//    pid_init(&motor_pid[i]);
//    motor_pid[i].f_param_init(&motor_pid[i], PID_Speed, 2000, 300, 10, 10, 2000, 500, 1.5, 0.05, 0.02);    
//        // PID模式（位置或速度）、最大输出、积分限幅、死区（绝对值）、控制周期、最大误差、目标值、kp、ki、kd）
//        //开始版本kp=2.5，ki=0.1
//        //初步调试kp=3.7，ki=0.1，kd=0.05
//        //降低参数防止超调：kp=1.5，ki=0.05，kd=0.02，最大输出=2000，积分限幅=300
//    motor_pid[i].target = mecanum.wheel_speed[i];
//  }
for (int i = 0; i < 4; i++)
{ 
  speed_pid_init(&speed_pid[i]);
  set_speed_pid(&speed_pid[i], 10.7f, 0.63f, 0.5f, 6000.0f, 600.0f,0.0f);
                           //设置kp,   ki,    kd, 最大输出，最大积分， 死区
	                        //初调kp=5.5,ki=0.4,kd=0.2,最大输出2500，积分300，死区10
                          //9.21 kp=10.7,ki=0.63,kd=0.5,最大输出6000，积分600，死区0,效果可以
  speed_pid[i].target = 0.0f; // 初始目标速度为0

}
 

  angle_controller_init(&angle_pid_s); //角度控制初始化
  angle_controller_init(&angle_pid_t); //角度控制初始化
  set_angle_pid(&angle_pid_s, 150.0f, 0.1f, 0.0f, 6000.0f, 400.0f);
  set_angle_pid(&angle_pid_t, 50.0f, 0.1f, 0.0f, 6000.0f, 400.0f);
            //设置kp,   ki,   kd,  最大输出，最大积分
	//p50,output1000
	//p60.0f, 0.1f, 0.0f, 1500.0f, 200.0f
  //set_target_angle(0.0f);
  //设置目标角度

  
  // 初始化麦克纳姆轮控制
  mecanum_init(&mecanum);
  uint32_t last_switch_time = HAL_GetTick();
  HAL_Delay(2500); // 等待系统稳定

//  float speed = 2000.0f;
//  const float speed_step = 500.0f; // 每次提升500


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
    for (int i = 0; i < 4; i++)
  {
    vofa_data[i] = (float)get_chassis_motor_measure_point(i)->speed_rpm;     // 实际速度
    vofa_data[i + 4] = (float)speed_pid[i].target;      
    HAL_UART_Transmit(&huart1, (uint8_t*)vofa_data, sizeof(vofa_data), 100);
    // 发送帧尾
    unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart1, tail, 4, 100);
    HAL_Delay(5); // 延迟10毫秒                    // 设定速度
  }
  uint32_t now = HAL_GetTick();
  if(now - last_switch_time >= 25000)
  {
    if(speed<1500.0f)
    {
    speed += speed_step;
    last_switch_time = now;
    }
    else 
    {
    speed = 2000.0f;
    last_switch_time = now;
   }
 }
  mecanum_move_forward(&mecanum, speed);
  
*/

  
  set_target_move_to_target(&mecanum,430.0f,0.0f,t1,4000.0f,1,1);  //x，y，角度，速度
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,0.0f,-800.0f*1.03f,t1,4000.0f,1,1); //到达取货站
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,0.0f,-340.0f,t1,4000.0f,0,0);
  HAL_Delay(1000);
  set_target_move_to_target(&mecanum,624.5f,0.0f,t1,3000.0f,1,1); //避障起点
  HAL_Delay(500);
  move_x(&mecanum,t1,1000.0f);
  HAL_Delay(500);
  move_y(&mecanum,t1,1000.0f);  
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,230.0f-2.0f,0.0f,t1,800.0f,0,1); //第一个货站
  HAL_Delay(500);

  set_target_move_to_target(&mecanum,461.0f,0.0f,t1,3000.0f,1,1);  //避障起点
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,0.0f,230.0f,t1,3000.0f,1,1);  //左移
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,884.0f,0.0f,t1,3000.0f,1,1);
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,0.0f,-205.0f,t1,3000.0f,1,1);  //右移,开始是225，多了
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,126.0f,0.0f,t1,3000.0f,1,1);
  HAL_Delay(500);
  move_x(&mecanum,t1,1000.0f);
  HAL_Delay(500);
  move_y(&mecanum,t1,1000.0f);
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t1,1000.0f,0,1);  //第二个货站

  HAL_Delay(1000);
  move_angle(&mecanum,t1);
  HAL_Delay(1000);
  set_target_move_to_target(&mecanum,810.0f,0.0f,t1,3000.0f,1,1);
  HAL_Delay(500);
  move_angle(&mecanum,t2);  //第一次转弯
  HAL_Delay(1000);
  set_target_move_to_target(&mecanum,69.5f,0.0f,t2,3000.0f,1,1);
  HAL_Delay(1000);
  move_x(&mecanum,t2,800.0f);
  HAL_Delay(500);
  move_y(&mecanum,t2,800.0f);
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t2,800.0f,0,1); //第三个货站

  HAL_Delay(1000);
  move_angle(&mecanum,t2); 
  HAL_Delay(1000);
  set_target_move_to_target(&mecanum,470.0f,0.0f,t2,3000.0f,0,0);
  HAL_Delay(1000);
  move_x(&mecanum,t2,800.0f);
  HAL_Delay(500);
  move_y(&mecanum,t2,800.0f);
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t2,3000.0f,0,1); //第四个货站

  HAL_Delay(1000);
  move_angle(&mecanum,t2); 
  HAL_Delay(1000);
  set_target_move_to_target(&mecanum,400.0f,0.0f,t2,3000.0f,1,1);
  HAL_Delay(500);
  move_angle(&mecanum,t3);  //第二次转弯
  HAL_Delay(2000);
  set_target_move_to_target(&mecanum,2115.0f,0.0f,t3,3000.0f,1,1);
  HAL_Delay(500);
  move_angle(&mecanum,t4);  //第三次转弯
  HAL_Delay(1000);
  set_target_move_to_target(&mecanum,480.5f,0.0f,t4,3000.0f,1,1);
  move_x(&mecanum,t4,1000.0f);
  HAL_Delay(500);
  move_y(&mecanum,t4,1000.0f);
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t4,1000.0f,0,1);//第五个货站

  HAL_Delay(1000);
  set_target_move_to_target(&mecanum,-740.0f,0.0f,t4,3000.0f,1,1);
  HAL_Delay(500);
  move_angle(&mecanum,t3);  //第四次转弯 180度
  HAL_Delay(2000);
  set_target_move_to_target(&mecanum,253.0f,0.0f,t3,3000.0f,1,1);
  HAL_Delay(500);
  move_x(&mecanum,t3,1000.0f);
  HAL_Delay(500);
  move_y(&mecanum,t3,1000.0f);
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,232.0f-2.0f,0.0f,t3,1000.0f,0,1); //第六个货站
  HAL_Delay(1000);
  set_target_move_to_target(&mecanum,1450.0f,0.0f,t3,3000.0f,1,1);
  HAL_Delay(500);
  set_target_move_to_target(&mecanum,0.0f,340.0f,t3,3000.0f,1,1); //左移
  HAL_Delay(500);
  
while(1)
{
}

    
 /*
		
    
    
    // VOFA绘图数据发送（只在VOFA模式下发送）
    // 发送8个通道：4个实际速度值 + 4个设定速度值
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
/*
volatile uint8_t uart_tx_busy = 0;
float vofa_data[8];
unsigned char vofa_buffer[36]; 
*/
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
CAN_cmd_chassis(speed_pid[0].output,speed_pid[1].output,speed_pid[2].output,speed_pid[3].output);
/*
 if (!uart_tx_busy) {
            // 准备数据
            for (int i = 0; i < 4; i++) {
                vofa_data[i] = (float)get_chassis_motor_measure_point(i)->speed_rpm;
                vofa_data[i + 4] = (float)speed_pid[i].target;
            }
            
            // 复制到发送缓冲区
            memcpy(vofa_buffer, vofa_data, 32);
            vofa_buffer[32] = 0x00;
            vofa_buffer[33] = 0x00;
            vofa_buffer[34] = 0x80;
            vofa_buffer[35] = 0x7f;
            
            // DMA发送
            uart_tx_busy = 1;
            HAL_UART_Transmit_DMA(&huart6, vofa_buffer, 36);

       }
       }

    for (int i = 0; i < 4; i++) 
    {
		
        motor_pid[i].target = mecanum.wheel_speed[i];
        const motor_measure_t *motor_data = get_chassis_motor_measure_point(i);
        
        
            //motor_pid[i].f_cal_pid(&motor_pid[i], motor_data->speed_rpm); //这句有问题，不能放中断回调里
        
    }
    CAN_cmd_chassis(motor_pid[0].output, motor_pid[1].output, motor_pid[2].output, motor_pid[3].output);
    // 不在中断里发送CAN指令，移到主循环
    */
    }

  
}

/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6) {
        uart_tx_busy = 0;
    }
}
*/



   


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
