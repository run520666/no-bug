#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55

// 接收缓冲区
uint8_t rxBuffer[4];
uint8_t rxIndex = 0;
bool frameReceived = false;

unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Arduino UART Master Ready");
}

void loop() {
  // 每1秒发送数据1到STM32
  if (millis() - lastSendTime >= 1000) {
    sendData(1);  // Arduino发送1
    lastSendTime = millis();
  }
  
  // 接收STM32返回的数据
  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();
    
    // 状态机接收
    if (rxIndex == 0 && inByte == FRAME_HEAD) {
      // 检测到帧头
      rxBuffer[rxIndex++] = inByte;
    } else if (rxIndex > 0 && rxIndex < 4) {
      rxBuffer[rxIndex++] = inByte;
      
      // 接收完整帧
      if (rxIndex == 4 && inByte == FRAME_TAIL) {
        frameReceived = true;
      }
      
      // 接收错误，重置
      if (rxIndex == 4 && inByte != FRAME_TAIL) {
        rxIndex = 0;
        Serial.println("Frame error: wrong tail");
      }
    } else {
      // 异常，重置
      rxIndex = 0;
    }
  }
  
  // 处理接收到的完整帧
  if (frameReceived) {
    // 校验帧头和帧尾
    if (rxBuffer[0] == FRAME_HEAD && rxBuffer[3] == FRAME_TAIL) {
      uint8_t data = rxBuffer[1];
      uint8_t checksum = rxBuffer[2];
      
      // 校验数据
      if (checksum == (FRAME_HEAD + data)) {
        // 数据有效
        Serial.print("Received from STM32: ");
        Serial.println(data);
        
        // 根据收到的数据控制LED
        if (data == 0) {
          digitalWrite(LED_BUILTIN, LOW);
        } else {
          digitalWrite(LED_BUILTIN, HIGH);
        }
      } else {
        Serial.println("Checksum error!");
      }
    }
    
    // 重置接收状态
    rxIndex = 0;
    frameReceived = false;
  }
  
  delay(10);
}

// 发送数据到STM32
void sendData(uint8_t data) {
  uint8_t frame[4];
  frame[0] = FRAME_HEAD;
  frame[1] = data;                    // 数据：1
  frame[2] = FRAME_HEAD + data;       // 校验和
  frame[3] = FRAME_TAIL;
  
  Serial.write(frame, 4);
  Serial.print("Sent to STM32: ");
  Serial.println(data);
}