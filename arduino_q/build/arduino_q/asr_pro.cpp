#line 1 "D:\\GIT\\no-bug\\arduino_q\\asr_pro.cpp"
#include "asr_pro.h"
#include <SoftwareSerial.h>

// 定义ASR Pro模块的引脚
#define ASR_PRO_RX 4
#define ASR_PRO_TX 5

// 创建SoftwareSerial实例
SoftwareSerial asr_pro(ASR_PRO_RX, ASR_PRO_TX);

// 初始化函数
void asrInit() 
{
  asr_pro.begin(115200);
  Serial.println(F("ASR Pro模块初始化完成"));
}

// 发送数据函数
void sendToAsr(String data) 
{
  if (data.length() > 0) {
    // 直接将传入的字符串发送出去
    asr_pro.print(data);
    
    Serial.print(F("已发送到ASR Pro: "));
    Serial.println(data);
  }
}