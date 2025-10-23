#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include "usart.h"

// 帧格式定义
#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55
#define DATA_LENGTH 3

// 外部变量声明
extern uint8_t rxBuffer[5];
extern uint8_t rxIndex;
extern uint8_t rxByte;
extern volatile uint8_t frameReceived;
extern uint8_t recivedata;

// 函数声明
void UART_SendData(UART_HandleTypeDef *huart, uint8_t dataone, uint8_t datatwo, uint8_t datathree);
void processReceivedData(void);

#endif // COMMUNICATION_H
