#include "PCA9685.h"
#include "arduino.h"

PCA9685::PCA9685() {
  _tcaChannel = TCA_CHANNEL_PCA9685;
}

// 切换TCA9548A通道
void PCA9685::selectTCAChannel(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);
}

// 写入PCA9685寄存器
void PCA9685::writeReg(uint8_t reg, uint8_t value) {
  selectTCAChannel(_tcaChannel);
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  delay(5);
}

// 读取PCA9685寄存器
uint8_t PCA9685::readReg(uint8_t reg) {
  selectTCAChannel(_tcaChannel);
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.endTransmission(0);
  
  Wire.requestFrom(PCA9685_ADDR, 1, 1);
  if (Wire.available() > 0) {
    return Wire.read();
  }
  return 0;
}

// 初始化PCA9685
bool PCA9685::init(uint8_t tcaChannel) {
  _tcaChannel = tcaChannel;
  Wire.begin();
  selectTCAChannel(_tcaChannel);
  
  // 复位PCA9685
  writeReg(PCA9685_MODE1, 0x00);
  delay(10);
  
  // 检查是否通讯正常
  uint8_t mode1 = readReg(PCA9685_MODE1);
  return mode1 == 0x00;
}

// 设置PWM频率（舵机常用50Hz）
void PCA9685::setPWMFreq(float freq) {
  // 计算预分频值
  float prescaleval = 25000000.0 / (4096.0 * freq) - 1.0;
  uint8_t prescale = floor(prescaleval + 0.5);
  
  // 进入睡眠模式以修改预分频
  uint8_t oldmode = readReg(PCA9685_MODE1);
  uint8_t newmode = (oldmode & 0x7F) | 0x10; // 睡眠模式
  writeReg(PCA9685_MODE1, newmode);
  writeReg(PCA9685_PRESCALE, prescale);
  writeReg(PCA9685_MODE1, oldmode);
  delay(5);
  writeReg(PCA9685_MODE1, oldmode | 0x80); // 自动增量
}

// 角度转PWM值（0-180度对应500-2500us）
uint16_t PCA9685::angleToPWM(uint8_t angle) {
  angle = constrain(angle, 0, 180);
  // 50Hz下，一个周期20000us，4096个刻度
  uint16_t pwm = map(angle, 0, 180, 102, 512); // 500us->102, 2500us->512
  return pwm;
}

// 控制舵机角度
void PCA9685::setServoAngle(uint8_t channel, uint8_t angle) {
  if (channel >= 16) return; // PCA9685共16通道
  
  uint16_t pwm = angleToPWM(angle);
  uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
  
  selectTCAChannel(_tcaChannel);
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);       // LEDx_ON_L
  Wire.write(0x00);      // ON低字节
  Wire.write(0x00);      // ON高字节
  Wire.write(pwm & 0xFF); // OFF低字节
  Wire.write(pwm >> 8);  // OFF高字节
  Wire.endTransmission();
}