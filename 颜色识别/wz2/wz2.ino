#include <Wire.h>
#include "Common.h"
#include "ColorSensor.h"
#include "GM65.h"
#include "PCA9685.h"
#include "ServoControl.h"

// 模块实例化
ColorSensor colorSensor;
GM65 gm65(GM65_RX_PIN, GM65_TX_PIN); // 注意：SoftwareSerial是RX,TX
PCA9685 pca;
ServoControl servoCtrl;

// 颜色角度映射表（索引0-6对应角度0,24,48,72,96,120,144度）
int colorAngleMap[7] = {COLOR_UNKNOWN, COLOR_UNKNOWN, COLOR_UNKNOWN, 
                        COLOR_UNKNOWN, COLOR_UNKNOWN, COLOR_UNKNOWN, COLOR_UNKNOWN};

// 扫描所有角度的小球颜色
void scanAllColors() {
  Serial.println("开始扫描小球颜色...");
  
  // 依次转动到每个角度并识别颜色
  for (int i = 0; i < 7; i++) {
    int angle = PCA_CHANNEL_ANGLES[i];
    Serial.print("扫描角度：");
    Serial.print(angle);
    Serial.print("度...");
    
    // 转动主舵机到目标角度
    servoCtrl.setMainZX20SAngle(angle);
    delay(1000); // 等待稳定
    
    // 读取RGB和HSL数据
    int rgb[3] = {0};
    int hsl[3] = {0};
    if (colorSensor.readRGB(rgb) && colorSensor.readHSL(hsl)) {
      // 识别颜色
      BallColor color = colorSensor.identifyColor(hsl, rgb);
      colorAngleMap[i] = color;
      
      // 打印结果
      Serial.print(" 颜色：");
      switch (color) {
        case COLOR_RED: Serial.print("红色"); break;
        case COLOR_YELLOW: Serial.print("黄色"); break;
        case COLOR_BLUE: Serial.print("蓝色"); break;
        case COLOR_GREEN: Serial.print("绿色"); break;
        case COLOR_WHITE: Serial.print("白色"); break;
        default: Serial.print("未知"); break;
      }
      Serial.println();
    } else {
      colorAngleMap[i] = COLOR_UNKNOWN;
      Serial.println(" 读取失败");
    }
  }
  
  // 回到起始位置
  servoCtrl.setMainZX20SAngle(0);
  Serial.println("扫描完成！");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // 初始化各模块
  Serial.println("初始化颜色传感器...");
  if (!colorSensor.init(TCA_CHANNEL_COLOR_SENSOR)) {
    Serial.println("颜色传感器初始化失败！");
    while (1);
  }
  
  Serial.println("初始化PCA9685...");
  if (!pca.init(TCA_CHANNEL_PCA9685)) {
    Serial.println("PCA9685初始化失败！");
    while (1);
  }
  pca.setPWMFreq(50); // 舵机频率50Hz
  
  Serial.println("初始化舵机控制...");
  if (!servoCtrl.init(pca)) {
    Serial.println("舵机控制初始化失败！");
    while (1);
  }
  
  Serial.println("初始化GM65条码模块...");
  gm65.init();
  
  // 扫描所有颜色
  scanAllColors();
  Serial.println("等待GM65识别二维码...");
}

void loop() {
  // 读取GM65识别结果
  char qrData[32] = {0};
  if (gm65.readQRCode(qrData, sizeof(qrData))) {
    Serial.print("识别到二维码：");
    Serial.println(qrData);
    
    // 解析二维码对应的颜色
    BallColor targetColor = gm65.parseQRCode(qrData);
    if (targetColor == COLOR_UNKNOWN) {
      Serial.println("二维码数据无效(O或未知字符)，不释放小球");
      return;
    }
    
    // 释放对应颜色的小球
    Serial.print("释放颜色：");
    switch (targetColor) {
      case COLOR_RED: Serial.print("红色(SG90)"); break;
      case COLOR_YELLOW: Serial.print("黄色"); break;
      case COLOR_BLUE: Serial.print("蓝色"); break;
      case COLOR_GREEN: Serial.print("绿色"); break;
      case COLOR_WHITE: Serial.print("白色"); break;
      default: break;
    }
    Serial.println();
    
    servoCtrl.releaseBall(targetColor, colorAngleMap);
    delay(1000);
    
    // 释放后联动动作
    Serial.println("执行联动动作(D8->D10)...");
    servoCtrl.releaseLinkAction();
    Serial.println("释放完成！");
  }
  
  delay(500);
}