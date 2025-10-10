#line 1 "D:\\GIT\\no-bug\\arduino_q\\pca9685_servo.cpp"
#include "pca9685_servo.h"
#include "tca9548a.h"

#define TCA_PCA9685_CH 2

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

bool pca9685Init() {
  tcaSelect(TCA_PCA9685_CH);
  delay(50);
  
  if (!pca.begin()) {
    return false;
  }
  pca.setPWMFreq(50);
  return true;
}

uint16_t relativeAngleToPulseSG90(int relativeAngle) {
  // 1. 初始角度：SG90_HOME_PULSE 对应的是 90°（中间位置）
  int home_angle = 90;  // 明确初始角度为90°（与SG90_HOME_PULSE匹配）
  // 2. 计算目标角度（初始角度 + 相对角度，限制在0-180°）
  int target_angle = home_angle + relativeAngle;
  target_angle = constrain(target_angle, 0, 180);  // 防止舵机超量程损坏
  // 3. 将目标角度转换为脉冲值（0°→150，180°→600）
  uint16_t target_pulse = map(target_angle, 0, 180, SG90_MIN_PULSE, SG90_MAX_PULSE);
  // 调试：打印角度和脉冲值，验证是否正确
  Serial.print("Target Angle: ");
  Serial.print(target_angle);
  Serial.print(" → Pulse: ");
  Serial.println(target_pulse);
  return target_pulse;
}

void controlSG90Relative(int channel, int relativeAngle) {
  tcaSelect(TCA_PCA9685_CH);
  uint16_t pulse = relativeAngleToPulseSG90(relativeAngle);
  pca.setPWM(channel, 0, pulse);
  delay(1000);
}

void resetSG90(int channel) {
  controlSG90Relative(channel, 0);
}
