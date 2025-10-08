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

// -------------------------- 状态机管理 --------------------------
// 主状态机枚举
enum SystemState {
  STATE_IDLE,           // 空闲状态(默认状态,舵机复位)
  STATE_COLOR_MAPPING,  // 颜色映射状态 (已移至setup)
  STATE_LEFT_QR,        // 左侧二维码识别
  STATE_RIGHT_QR,       // 右侧二维码识别
  STATE_WAIT_DELAY      // 等待延迟后复位
};

SystemState systemState = STATE_IDLE;

// 延迟相关变量
unsigned long operationStartTime = 0;
const unsigned long RELEASE_DELAY = 7000;

// -------------------------- 初始化函数 --------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // 初始化I2C总线
  Wire.begin();
  Serial.println(F("I2C总线初始化完成"));

  // 初始化颜色传感器舵机
  sensorServoInit();
  pushball();
  Serial.println(F("传感器舵机初始化完成"));

  // 初始化7、8、9号舵机
  zx20s_7ChuShiHua();
  zx20s_8ChuShiHua();
  zx20s_9ChuShiHua();
  Serial.println(F("7、8、9号舵机初始化完成"));

  // 初始化TCA9548A
  Serial.println(F("正在初始化TCA9548A..."));

  // 初始化颜色传感器
  if (!colorSensorInit()) {
    Serial.println(F("颜色传感器初始化失败!"));
    while (1);
  }
  Serial.println(F("颜色传感器就绪"));

  // 初始化PCA9685
  if (!pca9685Init()) {
    Serial.println(F("PCA9685初始化失败!"));
    while (1);
  }
  Serial.println(F("PCA9685就绪"));

  // 初始化GM65二维码模块
  gm65Init();
  Serial.println(F("两个GM65模块就绪"));

  // 初始化动态映射数组
  initDynamicMapping();

  /*
  // 执行自动颜色映射
  Serial.println(F("开始自动颜色映射..."));
  autoMapColors();
  isMappingDone = true;
  printDynamicMapping();
  Serial.println(F("颜色映射完成!"));
  */

  // 初始化通信协议
  uartProtocolInit();
  
  // 进入空闲状态
  systemState = STATE_IDLE;
  Serial.println(F("系统就绪 - 等待指令..."));
}

// -------------------------- 主循环函数 --------------------------
void loop() {
  // 状态机主循环
  switch (systemState) {
    
    // ==================== 空闲状态 ====================
    case STATE_IDLE:
      // 确保所有舵机复位
      static bool servosReset = false;
      static bool bufferCleared = false;  // 缓冲区清空标志

      // 每次进入STATE_IDLE时清空一次缓冲区
      if (!bufferCleared) {
        while (Serial.available() > 0) {
          Serial.read();  // 清空缓冲区
        }
        bufferCleared = true;
        Serial.println(F("串口缓冲区已清空"));
      }

      // 复位所有舵机(只执行一次)
      if (!servosReset) {
        zx20s_7FuWei();
        zx20s_8FuWei();
        zx20s_9FuWei();
        servosReset = true;
        Serial.println(F("系统处于空闲状态 - 所有舵机已复位"));
      }
      
      // 接收串口数据
      while (Serial.available() > 0) {
        uint8_t inByte = Serial.read();
        
        if (rxIndex == 0 && inByte == FRAME_HEAD) {
          rxBuffer[rxIndex++] = inByte;
        } else if (rxIndex > 0 && rxIndex < 7) {
          rxBuffer[rxIndex++] = inByte;
          
          if (rxIndex == 7 && inByte == FRAME_TAIL) {
            frameReceived = true;
          }
          
          if (rxIndex == 7 && inByte != FRAME_TAIL) {
            rxIndex = 0;
            Serial.println(F("帧错误: 帧尾不正确"));
          }
        } else {
          rxIndex = 0;
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
            Serial.print(F("收到指令: dataone="));
            Serial.print(dataone);
            Serial.print(F(", datatwo="));
            Serial.print(datatwo);
            Serial.print(F(", datathree="));
            Serial.println(datathree);
            
            // 判断进入哪个状态
            if (datathree == 1) {
             
              // ===== 颜色映射模式 (已移至setup,此处注释) =====
              if (dataone == 0 && datatwo == 0) {
                Serial.println(F("进入颜色映射模式..."));
                systemState = STATE_COLOR_MAPPING;
                servosReset = false;
                bufferCleared = false; 
              }  

              // ===== 左右二维码识别模式 =====
               else if (dataone == 1) {
                // 进入左侧二维码识别
                Serial.println(F("进入左侧二维码识别模式..."));
                systemState = STATE_LEFT_QR;
                servosReset = false;
                bufferCleared = false; 
              } else if (datatwo == 1) {
                // 进入右侧二维码识别
                Serial.println(F("进入右侧二维码识别模式..."));
                systemState = STATE_RIGHT_QR;
                servosReset = false;
                bufferCleared = false; 
              }
            }
          } else {
            Serial.println(F("校验和错误或长度不正确!"));
          }
        }
        
        rxIndex = 0;
        frameReceived = false;
      }
      break;
    
    
    // ==================== 颜色映射状态 (已移至setup) ====================
    case STATE_COLOR_MAPPING:
      Serial.println(F("开始自动颜色映射..."));
      autoMapColors();
      isMappingDone = true;
      printDynamicMapping();
      Serial.println(F("颜色映射完成!"));
      
      // 发送完成信号
      for(int i = 0; i < 3; i++) {
        sendData(1);
        delay(20);  // 确保信号被接收
      }

      Serial.println(F("已向STM32发送完成信号"));
      
      // 返回空闲状态
      systemState = STATE_IDLE;
      delay(500);
      break;
    
    
    // ==================== 左侧二维码识别 ====================
    case STATE_LEFT_QR:
      // 只在第一次进入时放下舵机
      static bool leftServosDeployed = false;
      if (!leftServosDeployed) {
        Serial.println(F("正在放下左侧舵机..."));
        zx20s_8Left();
        zx20s_7Left();
        leftServosDeployed = true;
        delay(1000);  // 等待舵机到位
      }

      if (isMappingDone) {
        Serial.println(F("正在读取左侧二维码..."));
        gm65_1.listen();
        String data1 = readGM65Data(gm65_1);
        
        if (data1.length() > 0) {
          Serial.print(F("左侧二维码数据: "));
          Serial.println(data1);
          processGM65Data(data1, 1);
          
          // 记录操作开始时间
          operationStartTime = millis();
          systemState = STATE_WAIT_DELAY;
          Serial.println(F("小球已释放,等待5秒..."));
        } else {
          Serial.println(F("左侧二维码读取失败..."));
  
        }
      } else {
        Serial.println(F("错误: 未完成颜色映射!"));
      }
      delay(SENSOR_READ_DELAY);
      break;
    
    // ==================== 右侧二维码识别 ====================
    case STATE_RIGHT_QR:
      // 只在第一次进入时放下舵机
      static bool rightServosDeployed = false;
      if (!rightServosDeployed) {
        Serial.println(F("正在放下右侧舵机..."));
        zx20s_9Right();
        zx20s_7Right();
        rightServosDeployed = true;
        delay(1000);  // 等待舵机到位
      }

      if (isMappingDone) {
        Serial.println(F("正在读取右侧二维码..."));
        gm65_2.listen();
        String data2 = readGM65Data(gm65_2);
        
        if (data2.length() > 0) {
          Serial.print(F("右侧二维码数据: "));
          Serial.println(data2);
          processGM65Data(data2, 2);
          
          // 记录操作开始时间
          operationStartTime = millis();
          systemState = STATE_WAIT_DELAY;
          Serial.println(F("小球已释放,等待5秒..."));
        } else {
          Serial.println(F("右侧二维码读取失败"));
        }
      } else {
        Serial.println(F("错误: 未完成颜色映射!"));
      }
      delay(SENSOR_READ_DELAY);
      break;
    
    // ==================== 等待延迟状态 ====================
    case STATE_WAIT_DELAY:
      if (millis() - operationStartTime >= RELEASE_DELAY) {
        Serial.println(F("5秒已到,正在复位舵机..."));
        zx20s_7FuWei();
        zx20s_8FuWei();
        zx20s_9FuWei();
        
        // 发送完成信号
        for(int i = 0; i < 3; i++) {
          sendData(1);
          delay(20);  // 确保信号被接收
        }

        Serial.println(F("操作完成,返回空闲状态"));
        
        // 返回空闲状态
        systemState = STATE_IDLE;
        leftServosDeployed = false;  // 重置标志以便下次进入时重新放下舵机
        rightServosDeployed = false; // 重置标志以便下次进入时重新放下舵机
        operationStartTime = 0;
      }
      break;
  }
  
  delay(10);  // 小延迟避免CPU占用过高
}