#line 1 "D:\\GIT\\no-bug\\arduino_q\\pca9685_servo.cpp"
#include "pca9685_servo.h"
#include "tca9548a.h"

#define TCA_PCA9685_CH 2

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ✅ 记录每个通道的当前角度（默认90度）
static int currentAngles[16] = {90, 90, 90, 90, 90, 90, 90, 90, 
                                 90, 90, 90, 90, 90, 90, 90, 90};

bool pca9685Init() {
  tcaSelect(TCA_PCA9685_CH);
  delay(50);
  
  if (!pca.begin()) {
    return false;
  }
  pca.setPWMFreq(50);
  return true;
}

// ✅ 设置某个通道的初始角度（仅记录，不移动舵机）
void setSG90InitialAngle(int channel, int angle) {
  if (channel < 0 || channel > 15) return;
  
  angle = constrain(angle, 0, 180);
  currentAngles[channel] = angle;
  
  Serial.print(F("通道 "));
  Serial.print(channel);
  Serial.print(F(" 初始角度记录为: "));
  Serial.print(angle);
  Serial.println(F("°"));
}

// ✅ 直接设置某个通道的绝对角度
void setSG90AbsoluteAngle(int channel, int angle) {
  if (channel < 0 || channel > 15) return;
  
  tcaSelect(TCA_PCA9685_CH);
  
  angle = constrain(angle, 0, 180);
  uint16_t pulse = map(angle, 0, 180, SG90_MIN_PULSE, SG90_MAX_PULSE);
  
  Serial.print(F("通道 "));
  Serial.print(channel);
  Serial.print(F(": 设置绝对角度 "));
  Serial.print(angle);
  Serial.print(F("° (脉冲: "));
  Serial.print(pulse);
  Serial.println(F(")"));
  
  pca.setPWM(channel, 0, pulse);
  currentAngles[channel] = angle;
  
  delay(500);
}

// ✅ 基于当前角度的相对控制
void controlSG90Relative(int channel, int relativeAngle) {
  if (channel < 0 || channel > 15) return;
  
  tcaSelect(TCA_PCA9685_CH);
  
  // 从当前角度开始计算目标角度
  int targetAngle = currentAngles[channel] + relativeAngle;
  targetAngle = constrain(targetAngle, 0, 180);
  
  // 转换为脉冲
  uint16_t pulse = map(targetAngle, 0, 180, SG90_MIN_PULSE, SG90_MAX_PULSE);
  
  // 调试输出
  Serial.print(F("通道 "));
  Serial.print(channel);
  Serial.print(F(": "));
  Serial.print(currentAngles[channel]);
  Serial.print(F("° "));
  Serial.print(relativeAngle >= 0 ? "+" : "");
  Serial.print(relativeAngle);
  Serial.print(F("° → "));
  Serial.print(targetAngle);
  Serial.print(F("° (脉冲: "));
  Serial.print(pulse);
  Serial.println(F(")"));
  
  pca.setPWM(channel, 0, pulse);
  
  // 更新当前角度
  currentAngles[channel] = targetAngle;
  
  delay(1000);
}

// ✅ 获取当前角度
int getSG90CurrentAngle(int channel) {
  if (channel < 0 || channel > 15) return 90;
  return currentAngles[channel];
}

// ✅ 复位到90度
void resetSG90(int channel) {
  setSG90AbsoluteAngle(channel, 90);
}