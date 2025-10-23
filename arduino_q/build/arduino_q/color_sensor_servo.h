#line 1 "D:\\GIT\\no-bug\\arduino_q\\color_sensor_servo.h"
#ifndef COLOR_SENSOR_SERVO_H
#define COLOR_SENSOR_SERVO_H

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// 颜色传感器配置
#define COLOR_SENSOR_ADDR 0x48
#define CMD_READ_RGB 0xD0
#define CMD_READ_HSL 0xD1
#define CMD_PING 0xAA
#define PING_RESPONSE 0x66
#define SENSOR_READ_DELAY 50

// 传感器舵机配置
#define SENSOR_SERVO_PIN 6

// ✅ 270度舵机参数
#define SERVO_270_MIN_PULSE 500   // 0度对应的脉宽(微秒)
#define SERVO_270_MAX_PULSE 2500  // 270度对应的脉宽(微秒)
#define SERVO_270_MIN_ANGLE 0     // 最小角度
#define SERVO_270_MAX_ANGLE 270   // 最大角度

extern Servo sensorServo;

// 角度通道映射结构
struct AngleChannelMap {
  int angle;
  int channel1;
  int channel2;
};

extern AngleChannelMap angleMaps[];
extern const int ANGLE_MAP_COUNT;

// 颜色二维码映射结构
struct ColorQrMap {
  const char* colorName;
  char qrCodeChar;
};

extern ColorQrMap colorQrMap[];
extern const int COLOR_QR_COUNT;

// 颜色通道映射数组
extern const char* channelToColor[10];
extern int colorToChannels[5][2];
extern int colorChannelCount[5];
extern bool isMappingDone;

// 函数声明
void sensorServoInit();
void pushball();
bool colorSensorPing(int sensorNum);
bool colorSensorInit();
void readRGB(int rgb[3], int sensorNum);
void readHSL(int hsl[3], int sensorNum);
const char* detectColorSensor1();
const char* detectColorSensor2();
const char* detectColor(int sensorNum);
void initDynamicMapping();
void updateColorToChannels(const char* color, int channel);
void printDynamicMapping();
int getChannelsByColor(const char* color, int channels[]);
void autoMapColors();

// ✅ 270度舵机控制函数
void servo270Write(int angle);

#endif