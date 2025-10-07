#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <Arduino.h>

#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55
#define DATA_LENGTH 3

extern uint8_t rxBuffer[7];
extern uint8_t rxIndex;
extern bool frameReceived;
extern unsigned long lastSendTime;
extern byte left_act;
extern byte right_act;
extern byte send_act;

void uartProtocolInit();
void sendData(uint8_t data);

#endif
