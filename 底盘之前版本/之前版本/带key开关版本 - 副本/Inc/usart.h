/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
extern volatile uint8_t uart_tx_done;
extern uint8_t  *tx_buf_ptr;
extern uint16_t  tx_buf_len;
extern volatile uint8_t tx_busy;
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UART1_Send_IT(uint8_t *buf, uint16_t len);
void MX_USART1_UART_Init(void);
void UART1_Send_DMA(uint8_t *buf, uint16_t len);
void UART_DMA_Send(const char *str);
void UART_Send_VOFA(float *data, uint8_t channel_count);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

