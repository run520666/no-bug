#ifndef GM65_H
#define GM65_H

#include <SoftwareSerial.h>
#include "Common.h"

class GM65 {
public:
  GM65(uint8_t rxPin, uint8_t txPin);
  ~GM65() {};
  
  // 初始化GM65
  void init();
  // 读取识别到的二维码数据
  bool readQRCode(char *data, uint8_t maxLen);
  // 根据二维码数据解析目标颜色
  BallColor parseQRCode(const char *data);

private:
  SoftwareSerial _gm65Serial;
  uint8_t _rxPin;
  uint8_t _txPin;
  // 清除串口缓存
  void clearBuffer();
};

#endif