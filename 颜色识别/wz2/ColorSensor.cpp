#include "ColorSensor.h"
#include "arduino.h"

ColorSensor::ColorSensor() {
  _tcaChannel = TCA_CHANNEL_COLOR_SENSOR;
}

// 切换TCA9548A通道
void ColorSensor::selectTCAChannel(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);
}

// Ping传感器（确保通讯正常）
bool ColorSensor::pingSensor() {
  selectTCAChannel(_tcaChannel);
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(0xAA); // Ping命令
  Wire.endTransmission(0);
  
  Wire.requestFrom(COLOR_SENSOR_ADDR, 1, 1);
  if (Wire.available() > 0) {
    int response = Wire.read();
    return response == 0x66; // 正确响应为0x66
  }
  return false;
}

// 初始化颜色传感器
bool ColorSensor::init(uint8_t tcaChannel) {
  _tcaChannel = tcaChannel;
  Wire.begin();
  selectTCAChannel(_tcaChannel);
  
  // 等待传感器初始化
  uint8_t retry = 0;
  while (!pingSensor() && retry < 5) {
    delay(100);
    retry++;
  }
  return retry < 5;
}

// 读取RGB数据（命令0xD0）
bool ColorSensor::readRGB(int rgb[3]) {
  selectTCAChannel(_tcaChannel);
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(0xD0); // 读取RGB命令
  Wire.endTransmission(0);
  
  Wire.requestFrom(COLOR_SENSOR_ADDR, 3, 1);
  if (Wire.available() >= 3) {
    rgb[0] = Wire.read(); // R
    rgb[1] = Wire.read(); // G
    rgb[2] = Wire.read(); // B
    
    // 暗光补偿
    rgb[0] = constrain(rgb[0] + _darkCompensation, 0, 255);
    rgb[1] = constrain(rgb[1] + _darkCompensation, 0, 255);
    rgb[2] = constrain(rgb[2] + _darkCompensation, 0, 255);
    return true;
  }
  return false;
}

// 读取HSL数据（命令0xD1）
bool ColorSensor::readHSL(int hsl[3]) {
  selectTCAChannel(_tcaChannel);
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(0xD1); // 读取HSL命令
  Wire.endTransmission(0);
  
  Wire.requestFrom(COLOR_SENSOR_ADDR, 3, 1);
  if (Wire.available() >= 3) {
    hsl[0] = Wire.read(); // 色相（0-240）
    hsl[1] = Wire.read(); // 饱和度（0-240）
    hsl[2] = Wire.read(); // 亮度（0-240）
    return true;
  }
  return false;
}

// 颜色识别逻辑
BallColor ColorSensor::identifyColor(int hsl[3], int rgb[3]) {
  int hue = hsl[0];
  int saturation = hsl[1];
  int lightness = hsl[2];
  int r = rgb[0];
  int g = rgb[1];
  int b = rgb[2];

  // 绿色识别
  bool isGreen = (hue >= 50 && hue <= 150)
                 && saturation > 10
                 && lightness < 200
                 && (g > r + 20 && g > b - 20);
  if (isGreen) return COLOR_GREEN;

  // 黄色识别
  bool isYellow = (hue >= 15 && hue <= 70)
                  && saturation > 10
                  && (r > 90 && g > 90 && b < 130);
  if (isYellow) return COLOR_YELLOW;

  // 红色识别
  bool isRed = ((hue >= 0 && hue <= 20) || (hue >= 220 && hue <= 240))
               && saturation > 30
               && (r > 120 && g < 100 && b < 100);
  if (isRed) return COLOR_RED;

  // 蓝色识别
  bool isBlue = (hue >= 120 && hue <= 180)
                && saturation > 20
                && (b > r + 20 && b > g + 20);
  if (isBlue) return COLOR_BLUE;

  // 白色识别
  bool isWhite = (saturation < 30)
                 && (lightness > 80 && lightness < 200)
                 && (abs(r - g) < 40) && (abs(g - b) < 40);
  if (isWhite) return COLOR_WHITE;

  return COLOR_UNKNOWN;
}