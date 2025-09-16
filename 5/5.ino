#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

// 硬件引脚定义
const int GM65_RX_PIN = 10;
const int GM65_TX_PIN = 11;
const int TCA9548A_ADDR = 0x70;
const int COLOR_SENSOR_ADDR = 0x48;
const int PCA9685_ADDR = 0x40;
const int PCA_RESET_PIN = 5;

// 软件串口
SoftwareSerial gm65Serial(GM65_RX_PIN, GM65_TX_PIN);

// 舵机参数
constexpr int SG90_INIT_ANGLE = 0;
constexpr int SG90_RELEASE_ANGLE = 95;
constexpr int SG90_9PIN_ANGLE = 100;
constexpr int SG90_8PIN_ANGLE = 45;
constexpr int SG90_PULSE_INIT = 550;
constexpr int SG90_PULSE_RELEASE = SG90_PULSE_INIT + (2500 - SG90_PULSE_INIT) * SG90_RELEASE_ANGLE / 180;
constexpr int SG90_PULSE_MIN = SG90_PULSE_INIT;
constexpr int SG90_PULSE_MAX = 2500;

// 系统状态和变量
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);
char ball_color[10];
int current_hole = 0;
uint8_t system_state = 0;  // 位标志: bit0=system_ready, bit1=color_scan_complete, bit2=servo_initialized, bit3=first_qrcode_received

// 函数声明
bool tcaSelect(uint8_t channel);
void readRGBData(int rgb[3]);
void readHSLData(int hsl[3]);
char getColorFromHSL(int hsl[3], int rgb[3]);
void pcaReset();
void initSG90Servos();
void moveSG90Servo(int channel, int angle);
void releaseBallsByColor(char color);
void identifyCurrentBall();
void resetColorScan();
char readGM65Command();
void processSerialCommands();

// TCA9548A通道选择
bool tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  delay(50);
  return true;
}

// 颜色传感器功能
void readRGBData(int rgb[3]) {
  if (!tcaSelect(0)) return;
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(0xD0);
  Wire.endTransmission(0);
  Wire.requestFrom(COLOR_SENSOR_ADDR, 3, 1);
  int i = 0;
  while (Wire.available() && i < 3) {
    rgb[i++] = Wire.read();
  }
  delay(50);
}

void readHSLData(int hsl[3]) {
  if (!tcaSelect(0)) return;
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(1);
  Wire.endTransmission(0);
  Wire.requestFrom(COLOR_SENSOR_ADDR, 3, 1);
  int i = 0;
  while (Wire.available() && i < 3) {
    hsl[i++] = Wire.read();
  }
  delay(50);
}

// 改进的颜色识别算法，解决黄色误判为白色的问题
char getColorFromHSL(int hsl[3], int rgb[3]) {
  int hue = hsl[0];
  int saturation = hsl[1];
  int lightness = hsl[2];

  int r = rgb[0];
  int g = rgb[1];
  int b = rgb[2];

  // 调整补偿值，避免过度曝光影响判断
  int compensationValue = (lightness > 180) ? 10 : 30;
  r = constrain(r + compensationValue, 0, 255);
  g = constrain(g + compensationValue, 0, 255);
  b = constrain(b + compensationValue, 0, 255);

  // 先检测黄色，提高优先级
  // 黄色的特点：红和绿的值都较高，蓝色值较低，色相在20-60之间
  if (hue >= 20 && hue <= 60 && saturation > 20) {
    if (r > 100 && g > 100 && b < 120 && abs(r - g) < 50) {
      // 增加黄色的亮度范围，避免强光下误判
      if (lightness > 50 && lightness < 220) {
        return 'y'; // 黄色优先识别
      }
    }
  }

  // 检测过曝光情况
  bool isOverExposed = (r > 230 && g > 230) || (r > 230 && b > 230) || (g > 230 && b > 230);
  if (isOverExposed) {
    // 过曝光下的白色判断：饱和度低，三原色接近
    if (saturation < 30 && abs(r - g) < 30 && abs(g - b) < 30) {
      return 'w';
    }
    // 过曝光下的黄色判断
    else if (hue >= 20 && hue <= 60 && r > g - 20 && r < g + 20 && b < 180) {
      return 'y';
    }
  }

  // 红色判断
  if (((hue >= 0 && hue <= 20) || (hue >= 220 && hue <= 240)) && 
      saturation > 30 && r > 120 && g < 100 && b < 100) {
    return 'r';
  }
  
  // 蓝色判断
  else if ((hue >= 120 && hue <= 180) && saturation > 20 && 
           b > r + 20 && b > g + 20) {
    return 'b';
  }
  
  // 绿色判断
  else if ((hue >= 50 && hue <= 150) && saturation > 10 && 
           lightness < 200 && g > r + 20 && g > b - 20) {
    return 'g';
  }
  
  // 白色判断：提高饱和度阈值，避免将低饱和黄色误判为白色
  else if (saturation < 25 && lightness > 80 && lightness < 220 && 
           abs(r - g) < 40 && abs(g - b) < 40) {
    // 确保白色的蓝色值不会太低，避免与黄色混淆
    if (b > 100) {
      return 'w';
    }
  }

  return 'u'; // 未知颜色
}

// PCA9685硬件复位
void pcaReset() {
  #ifdef PCA_RESET_PIN
    pinMode(PCA_RESET_PIN, OUTPUT);
    digitalWrite(PCA_RESET_PIN, LOW);
    delay(200);
    digitalWrite(PCA_RESET_PIN, HIGH);
    delay(300);
  #else
    if (!tcaSelect(1)) return;
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(0x00);
    Wire.write(0x80);
    Wire.endTransmission();
    delay(300);
  #endif
}

// 舵机初始化
void initSG90Servos() {
  if (!tcaSelect(1)) return;
  pca.begin();
  pca.setPWMFreq(50);
  delay(200);

  for (int i = 0; i < 10; i++) {
    pca.setPWM(i, 0, SG90_PULSE_INIT);
    delay(50);
  }
  system_state |= (1 << 2);  // 设置servo_initialized位
}

// 舵机移动函数
void moveSG90Servo(int channel, int angle) {
  // 检查所有必要条件
  if (!(system_state & (1 << 0)) || !(system_state & (1 << 1)) || 
      !(system_state & (1 << 2)) || !(system_state & (1 << 3)) ||
      channel < 0 || channel > 9) {
    if (tcaSelect(1)) {
      pca.setPWM(channel, 0, SG90_PULSE_INIT);
    }
    return;
  }

  int targetPulse;
  if (angle == SG90_INIT_ANGLE) {
    targetPulse = SG90_PULSE_INIT;
  } else {
    targetPulse = map(angle, 0, 180, SG90_PULSE_MIN, SG90_PULSE_MAX);
    targetPulse = constrain(targetPulse, SG90_PULSE_MIN, SG90_PULSE_MAX);
  }

  if (tcaSelect(1)) {
    pca.setPWM(channel, 0, targetPulse);
    delay(600);
  }
}

// 小球释放函数
void releaseBallsByColor(char color) {
  if (color != 'r' && color != 'y' && color != 'b' && color != 'g' && color != 'w') {
    return;
  }
  if (!(system_state & (1 << 0)) || !(system_state & (1 << 1)) || !(system_state & (1 << 2))) {
    return;
  }

  system_state |= (1 << 3);  // 设置first_qrcode_received位
  
  int release_count = 0;
  for (int i = 0; i < 10; i++) {
    if (ball_color[i] == color && release_count < 2) {
      moveSG90Servo(i, SG90_RELEASE_ANGLE);
      release_count++;
      delay(300);
    }
  }

  if (release_count > 0) {
    moveSG90Servo(9, SG90_9PIN_ANGLE);
    moveSG90Servo(8, SG90_8PIN_ANGLE);
    delay(1000);

    for (int i = 0; i < 10; i++) {
      moveSG90Servo(i, SG90_INIT_ANGLE);
    }
  }
}

// 颜色识别流程
void identifyCurrentBall() {
  if (current_hole >= 10) {
    system_state |= (1 << 1);  // 设置color_scan_complete位
    Serial.println("所有球孔已识别完成!");
    return;
  }
  
  int hsl[3], rgb[3];
  readHSLData(hsl);
  readRGBData(rgb);
  ball_color[current_hole] = getColorFromHSL(hsl, rgb);
  
  Serial.print("球孔 ");
  Serial.print(current_hole);
  Serial.print(" 颜色识别为: ");
  Serial.println(ball_color[current_hole]);
  
  current_hole++;
  
  if (current_hole < 10) {
    Serial.print("请转动传感器到第 ");
    Serial.print(current_hole);
    Serial.println(" 号球孔位置，按任意键继续识别");
  } else {
    Serial.println("所有球孔识别完成！等待二维码指令释放小球...");
    system_state |= (1 << 1);  // 设置color_scan_complete位
  }
}

void resetColorScan() {
  current_hole = 0;
  system_state &= ~(1 << 1);  // 清除color_scan_complete位
  Serial.println("颜色识别已重置");
  Serial.println("请转动传感器到第 0 号球孔位置，按任意键开始识别");
}

// GM65模块处理
char readGM65Command() {
  if (!(system_state & (1 << 0)) || !(system_state & (1 << 1))) {
    while (gm65Serial.available() > 0) {
      gm65Serial.read();
    }
    return 'n';
  }
  
  if (gm65Serial.available() > 0) {
    delay(50);
    String cmd = gm65Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 1) {
      char c = cmd.charAt(0);
      if (c == 'r' || c == 'y' || c == 'b' || c == 'g' || c == 'w' || c == 'o') {
        Serial.print("收到二维码指令: ");
        Serial.println(c);
        return c;
      }
    }
    Serial.println("收到无效二维码指令，忽略！");
    return 'n';
  }
  return 'n';
}

// 串口指令处理
void processSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available() > 0) {
      Serial.read();
    }
    
    if (cmd == 'r' || cmd == 'R') {
      resetColorScan();
    } else {
      identifyCurrentBall();
    }
  }
}

// 系统初始化
void setup() {
  Serial.begin(9600);
  while (!Serial) {};
  
  Serial.println("====================系统启动====================");
  Serial.println("启动阶段舵机保持初始位置，不执行任何动作...");

  Wire.begin();
  delay(200);

  pcaReset();
  initSG90Servos();
  delay(500);

  if (tcaSelect(0)) {
    Serial.println("颜色传感器就绪，等待手动识别...");
  }

  gm65Serial.begin(9600);
  delay(500);
  while (gm65Serial.available() > 0) {
    gm65Serial.read();
  }
  Serial.println("GM65二维码模块就绪，等待颜色识别完成...");

  resetColorScan();

  system_state |= (1 << 0);  // 设置system_ready位
  Serial.println("====================系统就绪====================");
}

// 主循环
void loop() {
  processSerialCommands();
  
  if ((system_state & (1 << 0)) && (system_state & (1 << 1))) {
    char gm65_cmd = readGM65Command();
    if (gm65_cmd != 'n') {
      releaseBallsByColor(gm65_cmd);
    }
  }

  delay(100);
}
