
#ifndef ASR_PRO_H
#define ASR_PRO_H

#include <Arduino.h>

// 初始化ASR Pro模块
void asrInit();

// 发送数据到ASR Pro模块
void sendToAsr(String data);

#endif