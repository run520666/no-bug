#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// 包含所有模块头文件
#include "tca9548a.h"
#include "color_sensor_servo.h"
#include "slide_servo.h"
#include "pca9685_servo.h"
#include "gm65_qrcode.h"
#include "uart_protocol.h"

// -------------------------- 状态机管理(保留在ino) --------------------------
// 舵机状态枚举
enum ServoState {
  STATE_IDLE,         // 初始状态,未动作
  STATE_DEPLOYED,     // 已放下
  STATE_RESET         // 已复位
};

// 状态变量
ServoState leftState = STATE_IDLE;
ServoState rightState = STATE_IDLE;
unsigned long leftReleaseTime = 0;
unsigned long rightReleaseTime = 0;
const unsigned long RELEASE_DELAY = 5000;

// 处理状态枚举
enum ProcessingState {
  STATE_WAITING,    // 等待接收信号
  STATE_PROCESSING  // 正在处理信号,忽略新输入
};
ProcessingState currentState = STATE_WAITING;

// -------------------------- 初始化函数 --------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // 初始化I2C总线
  Wire.begin();
  Serial.println(F("I2C bus initialized"));

  // 初始化颜色传感器舵机
  sensorServoInit();
  pushball();
  Serial.println(F("Sensor servo initialized"));

  // 初始化7、8、9号舵机
  zx20s_7ChuShiHua();
  zx20s_8ChuShiHua();
  zx20s_9ChuShiHua();
  Serial.println(F("7,8,9 servos initialized"));

  // 初始化TCA9548A
  Serial.println(F("Initializing TCA9548A..."));

  // 初始化颜色传感器
  if (!colorSensorInit()) {
    Serial.println(F("Color Sensor Initialization Failed!"));
    while (1);
  }
  Serial.println(F("Color Sensors Ready"));

  // 初始化PCA9685
  if (!pca9685Init()) {
    Serial.println(F("PCA9685 Initialization Failed!"));
    while (1);
  }
  Serial.println(F("PCA9685 Ready"));

  // 初始化GM65二维码模块
  gm65Init();
  Serial.println(F("Both GM65 Modules Ready"));

  // 初始化动态映射数组
  initDynamicMapping();

  // 执行自动颜色映射
  Serial.println(F("Starting automatic color mapping..."));
  autoMapColors();
  
  // 映射完成
  isMappingDone = true;
  printDynamicMapping();
  Serial.println(F("Mapping completed! Ready to receive QR commands."));

  // 初始化通信协议
  uartProtocolInit();
  Serial.println(F("Arduino UART Master Ready"));
}

// -------------------------- 主循环函数 --------------------------
void loop() {
  // 只有在等待状态才接收STM32数据
  if (currentState == STATE_WAITING) {
    while (Serial.available() > 0) {
      uint8_t inByte = Serial.read();
      
      // 状态机接收
      if (rxIndex == 0 && inByte == FRAME_HEAD) {
        rxBuffer[rxIndex++] = inByte;
      } else if (rxIndex > 0 && rxIndex < 7) {
        rxBuffer[rxIndex++] = inByte;
        
        if (rxIndex == 7 && inByte == FRAME_TAIL) {
          frameReceived = true;
        }
        
        if (rxIndex == 7 && inByte != FRAME_TAIL) {
          rxIndex = 0;
          Serial.println("Frame error: wrong tail");
        }
      } else {
        rxIndex = 0;
      }
    }
  } else {
    // 处理中状态,清空接收缓冲区
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  // 处理接收到的完整帧
  if (frameReceived) {
    if (rxBuffer[0] == FRAME_HEAD && rxBuffer[6] == FRAME_TAIL) {
      uint8_t length = rxBuffer[1];
      uint8_t dataone = rxBuffer[2];
      uint8_t datatwo = rxBuffer[3];
      uint8_t datathree = rxBuffer[4];
      uint8_t checksum = rxBuffer[5];
      
      uint8_t calc_checksum = FRAME_HEAD + length + dataone + datatwo + datathree;
      if (checksum == calc_checksum && length == 3) {
        Serial.print("Received from STM32: dataone=");
        Serial.print(dataone);
        Serial.print(", datatwo=");
        Serial.print(datatwo);
        Serial.print(", datathree=");
        Serial.println(datathree);

        // 进入处理状态
        currentState = STATE_PROCESSING;
        Serial.println("进入处理状态,忽略新指令...");
        
        if (datathree == 1) {
          // 左侧舵机动作
          if (dataone == 1 && leftState == STATE_IDLE) {
            Serial.println("放下左边舵机...");
            zx20s_8Left();
            zx20s_7Left();
            leftState = STATE_DEPLOYED;
            
            if (isMappingDone) {
              gm65_1.listen();
              String data1 = readGM65Data(gm65_1);
              if (data1.length() > 0) {
                processGM65Data(data1, 1);
                
                if (leftState == STATE_DEPLOYED && millis() - leftReleaseTime >= RELEASE_DELAY) {
                  Serial.println("左侧小球释放后已延迟5秒,复位左侧舵机...");
                  zx20s_8FuWei();
                  zx20s_7FuWei();
                  leftState = STATE_IDLE;  // 改为IDLE以便下次使用
                  sendData(1);
                  currentState = STATE_WAITING;
                  Serial.println("左侧流程结束,等待新指令...");
                  leftReleaseTime = 0;
                }
              }
              delay(SENSOR_READ_DELAY);
            }
          }
          
          // 右侧舵机动作
          if (datatwo == 1 && rightState == STATE_IDLE) {
            Serial.println("放下右边舵机...");
            zx20s_9Right();
            zx20s_7Right();
            rightState = STATE_DEPLOYED;
            
            if (isMappingDone) {
              gm65_2.listen();
              String data2 = readGM65Data(gm65_2);
              if (data2.length() > 0) {
                processGM65Data(data2, 2);
                
                if (rightState == STATE_DEPLOYED && millis() - rightReleaseTime >= RELEASE_DELAY) {
                  Serial.println("右侧小球释放后已延迟5秒,复位右侧舵机...");
                  zx20s_9FuWei();
                  zx20s_7FuWei();
                  rightState = STATE_IDLE;  // 改为IDLE以便下次使用
                  sendData(1);
                  currentState = STATE_WAITING;
                  Serial.println("右侧流程结束,等待新指令...");
                  rightReleaseTime = 0;
                }
              }
            }
          }
        }
      } else {
        Serial.println("Checksum error or wrong length!");
      }
    }
    
    rxIndex = 0;
    frameReceived = false;
  }

  delay(100);
}