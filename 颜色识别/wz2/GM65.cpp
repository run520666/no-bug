#include "GM65.h"
#include "arduino.h"

GM65::GM65(uint8_t rxPin, uint8_t txPin) 
  : _gm65Serial(rxPin, txPin), _rxPin(rxPin), _txPin(txPin) {
}

// 初始化GM65（默认串口参数：9600, 8N1）
void GM65::init() {
  _gm65Serial.begin(9600);
  clearBuffer();
  delay(100); // 等待模块启动
}

// 清除串口缓存
void GM65::clearBuffer() {
  while (_gm65Serial.available() > 0) {
    _gm65Serial.read();
    delay(1);
  }
}

// 读取二维码数据（返回是否成功，数据存入data）
bool GM65::readQRCode(char *data, uint8_t maxLen) {
  if (!_gm65Serial.available()) return false;
  
  clearBuffer();
  uint8_t len = 0;
  unsigned long startTime = millis();
  
  // 读取数据（超时1000ms）
  while (millis() - startTime < 1000 && len < maxLen - 1) {
    if (_gm65Serial.available()) {
      data[len++] = _gm65Serial.read();
      startTime = millis(); // 重置超时
    }
    delay(1);
  }
  data[len] = '\0'; // 字符串结束符
  
  // 过滤无效数据（仅保留非空数据）
  return len > 0 && data[0] != '\0';
}

// 解析二维码数据（R-释放SG90, Y-红, B-蓝, G-绿, W-白, O-不释放）
BallColor GM65::parseQRCode(const char *data) {
  if (strlen(data) == 0) return COLOR_UNKNOWN;
  
  switch (toupper(data[0])) {
    case 'Y': return COLOR_YELLOW;
    case 'B': return COLOR_BLUE;
    case 'G': return COLOR_GREEN;
    case 'W': return COLOR_WHITE;
    case 'R': return COLOR_RED; // R对应释放SG90（此处用RED占位，后续单独处理）
    case 'O': return COLOR_UNKNOWN; // O不释放
    default: return COLOR_UNKNOWN;
  }
}