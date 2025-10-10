#line 1 "D:\\GIT\\no-bug\\arduino_q\\uart_protocol.cpp"
#include "uart_protocol.h"

uint8_t rxBuffer[7];
uint8_t rxIndex = 0;
bool frameReceived = false;
unsigned long lastSendTime = 0;
byte left_act = 0;
byte right_act = 0;
byte send_act = 0;

void uartProtocolInit() {
  rxIndex = 0;
  frameReceived = false;
}

void sendData(uint8_t data) {
  uint8_t frame[5];
  frame[0] = FRAME_HEAD;
  frame[1] = DATA_LENGTH;
  frame[2] = data;
  frame[3] = FRAME_HEAD + DATA_LENGTH + data;
  frame[4] = FRAME_TAIL;
  
  Serial.write(frame, 5);
  Serial.print("Sent to STM32: data=");
  Serial.print(data);
}
