#ifndef COMMON_H
#define COMMON_H

// 颜色定义
enum BallColor {
  COLOR_UNKNOWN,
  COLOR_RED,
  COLOR_YELLOW,
  COLOR_BLUE,
  COLOR_GREEN,
  COLOR_WHITE
};

// PCA9685通道映射（角度->通道）
const int ANGLE_TO_PCA_CHANNEL[7] = {0, 1, 2, 3, 4, 5, 6};
// 对应角度值
const int PCA_CHANNEL_ANGLES[7] = {0, 24, 48, 72, 96, 120, 144};

// 舵机引脚定义
const int ZX20S_MAIN_PIN = 9;    // 主ZX20S舵机（带动颜色传感器）
const int ZX20S_D8_PIN = 8;      // 辅助ZX20S舵机1
const int ZX20S_D10_PIN = 10;    // 辅助ZX20S舵机2

// GM65串口引脚
const int GM65_TX_PIN = 2;
const int GM65_RX_PIN = 3;

// TCA9548A相关定义
const int TCA9548A_ADDR = 0x70;
const int TCA_CHANNEL_COLOR_SENSOR = 0;
const int TCA_CHANNEL_PCA9685 = 1;

// 舵机角度参数
const int SG90_RELEASE_ANGLE = 100;  // SG90释放角度（100度）
const int ZX20S_RELEASE_ANGLE = 100; // ZX20S释放角度（逆时针100度）

#endif