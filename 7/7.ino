#include <Wire.h>                  // I2C通信库
#include <Adafruit_PWMServoDriver.h>// PCA PCA9685舵机驱动库
#include <SoftwareSerial.h>         // 软件串口库，用于GM65二维码模块
#include <Servo.h>                 // 舵机控制库

// -------------------------- 硬件参数配置 --------------------------
// TCA9548A I2C多路复用器配置
#define TCA9548A_ADDR 0x70         // TCA9548A默认I2C地址
#define TCA_COLOR_SENSOR_CH 0      // 颜色传感器连接到TCA的通道0
#define TCA_PCA9685_CH 1           // PCA9685舵机驱动板连接到TCA的通道1

// PCA9685舵机驱动板配置（控制10个SG90舵机，对应10个球孔）
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);  // PCA9685默认地址
#define SG90_MIN_PULSE 150         // SG90舵机最小脉冲宽度（对应0°）
#define SG90_MAX_PULSE 600         // SG90舵机最大脉冲宽度（对应180°）
#define SG90_RELATIVE_ANGLE 90     // 释放小球时相对初始位置转动的角度

// GM65二维码模块配置（使用软件串口）
#define GM65_1_RX 10               // 第一个GM65的RX引脚连接到Arduino 10号引脚
#define GM65_1_TX 11               // 第一个GM65的TX引脚连接到Arduino 11号引脚
#define GM65_2_RX 12               // 第二个GM65的RX引脚连接到Arduino 12号引脚
#define GM65_2_TX 13               // 第二个GM65的TX引脚连接到Arduino 13号引脚
SoftwareSerial gm65_1(GM65_1_RX, GM65_1_TX);  // 创建第一个GM65的软件串口对象
SoftwareSerial gm65_2(GM65_2_RX, GM65_2_TX);  // 创建第二个GM65的软件串口对象
#define GM65_BAUD 9600             // GM65二维码模块默认波特率

// 舵机对象与引脚定义
Servo zx20s_7;         // ZX20S舵机，连接到7号引脚
Servo zx20s_8;         // ZX20S舵机，连接到8号引脚
Servo zx20s_9;         // ZX20S舵机，连接到9号引脚
Servo mg90s_sensor;    // MG90S舵机，用于带动颜色传感器转动，连接到6号引脚
#define ZX20S_PIN_7 7  // ZX20S舵机引脚定义
#define ZX20S_PIN_8 8
#define ZX20S_PIN_9 9
#define MG90S_PIN_6 6  // MG90S舵机引脚定义

// ZX20S舵机相对转动角度定义
#define ZX20S_REL_100 100          // 相对初始位置转动100度
#define ZX20S_REL_45_CW 45         // 相对初始位置顺时针转45度
#define ZX20S_REL_45_CCW -45       // 相对初始位置逆时针转45度（负值表示）

// 颜色识别传感器配置
#define COLOR_SENSOR_ADDR 0x4F     // 颜色传感器I2C地址
#define CMD_READ_RGB 0xD0          // 读取RGB数据命令
#define CMD_READ_HSL 0xD1          // 读取HSL数据命令
#define CMD_PING 0xAA              // Ping命令（检测传感器是否连接）
#define PING_RESPONSE 0x66         // Ping成功响应值

// 颜色-二维码字符映射表（内存优化：使用const char*替代String）
struct ColorQrMap {
  const char* colorName;  // 颜色名称（与识别结果对应）
  char qrCodeChar;        // 二维码中对应的字符（r=红,y=黄,b=蓝,g=绿,w=白）
};
ColorQrMap colorQrMap[] = {
  {"Red",    'r'},    // 红色对应二维码字符'r'
  {"Yellow", 'y'},    // 黄色对应二维码字符'y'
  {"Blue",   'b'},    // 蓝色对应二维码字符'b'
  {"Green",  'g'},    // 绿色对应二维码字符'g'
  {"White",  'w'}     // 白色对应二维码字符'w'
};
const int COLOR_QR_COUNT = sizeof(colorQrMap) / sizeof(colorQrMap[0]);  // 计算颜色数量

// 动态映射核心数据结构（内存优化：减少内存占用）
// 通道→颜色映射：索引=PCA通道号（0-9），值=对应小球颜色
const char* channelToColor[10] = {"Unknown", "Unknown", "Unknown", "Unknown", "Unknown", 
                                 "Unknown", "Unknown", "Unknown", "Unknown", "Unknown"};
// 颜色→通道映射：索引0-4对应5种颜色，存储该颜色对应的2个通道号
int colorToChannels[5][2];
// 记录每种颜色已找到的通道数量（最多2个）
int colorChannelCount[5];

// 全局状态变量
int scanIndex = 0;               // 当前扫描的球孔序号（0-9）
bool isScanning = true;          // 是否正在扫描小球颜色（初始为true）
bool isMappingDone = false;      // 颜色-通道映射是否完成（初始为false）

// 舵机初始位置存储
int sg90HomePositions[10];       // SG90舵机初始位置（脉冲值）
int zx20sHomePositions[3];       // ZX20S舵机初始位置（角度）
bool servosCalibrated = false;   // 舵机是否已校准（初始为false）

// -------------------------- 函数声明（避免函数调用顺序问题） --------------------------
void initDynamicMapping();
void tcaSelect(uint8_t channel);
bool colorSensorPing();
bool colorSensorInit();
void readRGB(int rgb[3]);
void readHSL(int hsl[3]);
const char* detectColor();
void calibrateSG90Servos();
uint16_t relativeAngleToPulseSG90(int channel, int relativeAngle);
void controlSG90Relative(int channel, int relativeAngle);
void resetSG90(int channel);
void resetAllSG90();
void calibrateZX20SServos();
void controlZX20SRelative(int servoIndex, int relativeAngle);
void resetZX20S(int servoIndex);
void resetAllZX20S();
void zx20sActionForGM65_1();
void zx20sActionForGM65_2();
void initSensorServo();
void moveSensorToBall(int ballIndex);
void scanSingleBall();
void updateColorToChannels(const char* color, int channel);
void printDynamicMapping();
int getChannelsByColor(const char* color, int channels[]);
String readGM65Data(SoftwareSerial &gm65);
void processGM65Data(String data, bool isGM65_1);

// -------------------------- 初始化函数 --------------------------
void setup() {
  Serial.begin(115200);          // 初始化调试串口，波特率115200
  delay(1000);                   // 等待系统稳定

  // 初始化TCA9548A多路复用器
  Wire.begin();                  // 启动I2C通信
  tcaSelect(TCA_COLOR_SENSOR_CH);// 先选中颜色传感器通道
  Serial.println("Initializing TCA9548A...");

  // 初始化颜色传感器（失败则停机）
  if (!colorSensorInit()) {
    Serial.println("Color Sensor Initialization Failed!");
    while (1);  // 初始化失败，程序停滞
  }
  Serial.println("Color Sensor Ready");

  // 初始化PCA9685舵机驱动板并校准SG90舵机
  tcaSelect(TCA_PCA9685_CH);     // 选中PCA9685通道
  pca.begin();                   // 初始化PCA9685
  pca.setPWMFreq(50);            // 设置舵机频率为50Hz（标准舵机频率）
  calibrateSG90Servos();         // 校准SG90舵机初始位置
  Serial.println("PCA9685 Ready");

  // 初始化GM65二维码模块
  gm65_1.begin(GM65_BAUD);       // 启动第一个GM65的软件串口
  gm65_2.begin(GM65_BAUD);       // 启动第二个GM65的软件串口
  gm65_1.setTimeout(100);        // 设置读取超时时间100ms
  gm65_2.setTimeout(100);
  Serial.println("GM65 Modules Ready");

  // 初始化所有舵机（ZX20S + MG90S）
  zx20s_7.attach(ZX20S_PIN_7);   // 绑定ZX20S舵机到对应引脚
  zx20s_8.attach(ZX20S_PIN_8);
  zx20s_9.attach(ZX20S_PIN_9);
  calibrateZX20SServos();        // 校准ZX20S舵机初始位置
  initSensorServo();             // 初始化MG90S传感器舵机
  Serial.println("All Servos Ready");

  // 初始化动态映射数组
  initDynamicMapping();

  // 启动小球颜色扫描
  Serial.println("Start Scanning Balls (0-9)...");
}

// -------------------------- 核心功能函数 --------------------------

/**
 * 初始化动态映射数组（清空初始值）
 */
void initDynamicMapping() {
  // 初始化“颜色→通道”映射（置为-1表示未分配）
  for (int c = 0; c < 5; c++) {
    colorChannelCount[c] = 0;    // 重置计数
    colorToChannels[c][0] = -1;  // 第一个通道
    colorToChannels[c][1] = -1;  // 第二个通道
  }
}

/**
 * 选择TCA9548A的指定通道（0-7）
 * @param channel 要选择的通道号（0-7）
 */
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;       // 通道号超出范围则返回
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);      // 对应通道置1，其他置0（二进制位操作）
  Wire.endTransmission();
  delay(10);                     // 切换通道后等待稳定
}

/**
 * 检测颜色传感器是否在线（发送Ping命令）
 * @return 传感器在线返回true，否则返回false
 */
bool colorSensorPing() {
  tcaSelect(TCA_COLOR_SENSOR_CH);  // 选中颜色传感器通道
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CMD_PING);            // 发送Ping命令
  Wire.endTransmission(0);         // 不发送停止位

  // 读取响应
  Wire.requestFrom(COLOR_SENSOR_ADDR, 1);
  if (Wire.available()) {
    return Wire.read() == PING_RESPONSE;  // 检查响应是否正确
  }
  return false;
}

/**
 * 初始化颜色传感器（多次尝试连接）
 * @return 初始化成功返回true，否则返回false
 */
bool colorSensorInit() {
  // 最多尝试5次连接
  for (int i = 0; i < 5; i++) {
    if (colorSensorPing()) {
      return true;  // 连接成功
    }
    delay(500);     // 间隔500ms重试
  }
  return false;     // 多次尝试失败
}

/**
 * 读取颜色传感器的RGB数据
 * @param rgb 存储RGB数据的数组（长度3）
 */
void readRGB(int rgb[3]) {
  tcaSelect(TCA_COLOR_SENSOR_CH);
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CMD_READ_RGB);        // 发送读取RGB命令
  Wire.endTransmission(0);

  // 读取3个字节的RGB数据
  Wire.requestFrom(COLOR_SENSOR_ADDR, 3);
  int i = 0;
  while (Wire.available() && i < 3) {
    rgb[i++] = Wire.read();
  }
}

/**
 * 读取颜色传感器的HSL数据
 * @param hsl 存储HSL数据的数组（长度3）
 */
void readHSL(int hsl[3]) {
  tcaSelect(TCA_COLOR_SENSOR_CH);
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CMD_READ_HSL);        // 发送读取HSL命令
  Wire.endTransmission(0);

  // 读取3个字节的HSL数据
  Wire.requestFrom(COLOR_SENSOR_ADDR, 3);
  int i = 0;
  while (Wire.available() && i < 3) {
    hsl[i++] = Wire.read();
  }
}

/**
 * 识别颜色（基于RGB和HSL数据）
 * @return 识别到的颜色名称（字符串指针）
 */
const char* detectColor() {
  int rgb[3] = {0};  // 局部变量存储RGB数据（内存优化：避免全局变量）
  int hsl[3] = {0};  // 局部变量存储HSL数据
  readRGB(rgb);      // 读取RGB值
  readHSL(hsl);      // 读取HSL值

  // 暗光补偿（提升亮度，避免环境光过暗导致识别错误）
  int r = constrain(rgb[0] + 50, 0, 255);  // 红色分量（限制在0-255）
  int g = constrain(rgb[1] + 50, 0, 255);  // 绿色分量
  int b = constrain(rgb[2] + 50, 0, 255);  // 蓝色分量

  // 优化判断顺序：先判断容易识别的颜色（如白色）
  // 判断白色：低饱和度，三通道值接近
  if (hsl[1] < 30 && hsl[2] > 80 && hsl[2] < 200 && 
      abs(r - g) < 40 && abs(g - b) < 40) {
    return "White";
  }
  // 判断红色：色相在0-20或220-240，高饱和度，红色分量高
  else if ((hsl[0] >= 0 && hsl[0] <= 20) || (hsl[0] >= 220 && hsl[0] <= 240)) {
    if (hsl[1] > 30 && r > 120 && g < 100 && b < 100) {
      return "Red";
    }
  }
  // 判断黄色：色相在20-60，红绿色分量较高
  else if (hsl[0] >= 20 && hsl[0] <= 60) {
    if (hsl[1] > 30 && hsl[2] > 80 && hsl[2] < 200 && 
        r > g + 10 && r > 100 && g > 80 && b < 100) {
      return "Yellow";
    }
  }
  // 判断蓝色：色相在120-180，蓝色分量高
  else if (hsl[0] >= 120 && hsl[0] <= 180) {
    if (hsl[1] > 20 && b > r + 20 && b > g + 20) {
      return "Blue";
    }
  }
  // 判断绿色：色相在50-150，绿色分量高
  else if (hsl[0] >= 50 && hsl[0] <= 150) {
    if (hsl[1] > 10 && hsl[2] < 200 && g > r + 20 && g > b - 20) {
      return "Green";
    }
  }
  
  return "Unknown";  // 未识别到颜色
}

/**
 * 校准SG90舵机（通过串口交互设置初始位置）
 */
void calibrateSG90Servos() {
  Serial.println("Calibrating SG90 servos. Please set all servos to their home positions.");
  Serial.println("Calibration steps:");
  Serial.println("1. For each servo (0-9), adjust to its home position");
  Serial.println("2. Press any key to save position and move to next servo");
  
  tcaSelect(TCA_PCA9685_CH);  // 选中PCA9685通道
  
  for (int ch = 0; ch < 10; ch++) {
    Serial.print("Calibrating SG90 servo channel ");
    Serial.println(ch);
    
    // 先移动到中间位置作为起点
    int midPulse = (SG90_MIN_PULSE + SG90_MAX_PULSE) / 2;
    pca.setPWM(ch, 0, midPulse);
    delay(1000);  // 等待舵机到位
    
    // 等待用户调整并按任意键确认
    while (!Serial.available());
    Serial.read();  // 读取按键（不处理具体值）
    
    // 保存当前脉冲值作为初始位置
    sg90HomePositions[ch] = midPulse;
    Serial.print("Saved home position for channel ");
    Serial.println(ch);
  }
  
  Serial.println("SG90 servo calibration complete");
  servosCalibrated = true;  // 标记舵机已校准
}

/**
 * 将相对角度转换为SG90舵机的脉冲宽度
 * @param channel 舵机通道号
 * @param relativeAngle 相对初始位置的角度
 * @return 对应的脉冲宽度
 */
uint16_t relativeAngleToPulseSG90(int channel, int relativeAngle) {
  // 将初始位置脉冲值转换为角度（0-180）
  int angle = map(sg90HomePositions[channel], SG90_MIN_PULSE, SG90_MAX_PULSE, 0, 180);
  angle += relativeAngle;  // 加上相对角度
  angle = constrain(angle, 0, 180);  // 限制在舵机可动范围
  // 将角度转换回脉冲宽度
  return map(angle, 0, 180, SG90_MIN_PULSE, SG90_MAX_PULSE);
}

/**
 * 相对初始位置控制SG90舵机
 * @param channel 舵机通道号
 * @param relativeAngle 相对初始位置的角度
 */
void controlSG90Relative(int channel, int relativeAngle) {
  if (!servosCalibrated) {  // 未校准则提示错误
    Serial.println("Error: Servos not calibrated!");
    return;
  }
  
  tcaSelect(TCA_PCA9685_CH);  // 选中PCA9685通道
  uint16_t pulse = relativeAngleToPulseSG90(channel, relativeAngle);
  pca.setPWM(channel, 0, pulse);  // 设置舵机脉冲
  delay(500);  // 等待舵机转动到位
}

/**
 * 将SG90舵机复位到初始位置
 * @param channel 舵机通道号
 */
void resetSG90(int channel) {
  controlSG90Relative(channel, 0);  // 相对角度0即回到初始位置
}

/**
 * 将所有SG90舵机复位到初始位置
 */
void resetAllSG90() {
  for (int ch = 0; ch < 10; ch++) {
    resetSG90(ch);
  }
}

/**
 * 校准ZX20S舵机（通过串口交互设置初始位置）
 */
void calibrateZX20SServos() {
  Serial.println("Calibrating ZX20S servos. Please set all servos to their home positions.");
  Serial.println("Calibration steps:");
  Serial.println("1. For each servo (7,8,9), adjust to its home position");
  Serial.println("2. Press any key to save position and move to next servo");
  
  // 舵机索引：0=zx20s_7，1=zx20s_8，2=zx20s_9
  for (int i = 0; i < 3; i++) {
    Serial.print("Calibrating ZX20S servo on pin ");
    Serial.println(i == 0 ? ZX20S_PIN_7 : (i == 1 ? ZX20S_PIN_8 : ZX20S_PIN_9));
    
    // 先移动到中间位置作为起点
    int midAngle = 90;
    if (i == 0) zx20s_7.write(midAngle);
    else if (i == 1) zx20s_8.write(midAngle);
    else zx20s_9.write(midAngle);
    delay(1000);  // 等待舵机到位
    
    // 等待用户调整并按任意键确认
    while (!Serial.available());
    Serial.read();  // 读取按键（不处理具体值）
    
    // 保存当前角度作为初始位置
    zx20sHomePositions[i] = midAngle;
    Serial.print("Saved home position for ZX20S servo ");
    Serial.println(i);
  }
  
  Serial.println("ZX20S servo calibration complete");
}

/**
 * 通用舵机控制函数（合并重复代码，内存优化）
 * @param servo 舵机对象引用
 * @param homePos 初始位置角度
 * @param relativeAngle 相对初始位置的角度
 */
void controlServo(Servo &servo, int homePos, int relativeAngle) {
  int targetAngle = homePos + relativeAngle;  // 计算目标角度
  targetAngle = constrain(targetAngle, 0, 180);  // 限制在舵机可动范围
  servo.write(targetAngle);  // 控制舵机转动
  delay(500);  // 等待舵机到位
}

/**
 * 相对初始位置控制ZX20S舵机
 * @param servoIndex 舵机索引（0=7号引脚，1=8号引脚，2=9号引脚）
 * @param relativeAngle 相对初始位置的角度
 */
void controlZX20SRelative(int servoIndex, int relativeAngle) {
  if (!servosCalibrated) {  // 未校准则提示错误
    Serial.println("Error: Servos not calibrated!");
    return;
  }
  
  // 根据索引控制对应舵机（使用通用舵机控制函数）
  if (servoIndex == 0) controlServo(zx20s_7, zx20sHomePositions[0], relativeAngle);
  else if (servoIndex == 1) controlServo(zx20s_8, zx20sHomePositions[1], relativeAngle);
  else if (servoIndex == 2) controlServo(zx20s_9, zx20sHomePositions[2], relativeAngle);
}

/**
 * 将ZX20S舵机复位到初始位置
 * @param servoIndex 舵机索引
 */
void resetZX20S(int servoIndex) {
  controlZX20SRelative(servoIndex, 0);  // 相对角度0即回到初始位置
}

/**
 * 将所有ZX20S舵机复位到初始位置
 */
void resetAllZX20S() {
  for (int i = 0; i < 3; i++) {
    resetZX20S(i);
  }
}

/**
 * GM65_1触发的ZX20S舵机动作序列
 */
void zx20sActionForGM65_1() {
  Serial.println("Execute ZX20S Action for GM65_1");
  controlZX20SRelative(1, ZX20S_REL_100);  // 8号舵机转100°
  delay(1000);
  controlZX20SRelative(0, ZX20S_REL_45_CW);  // 7号舵机顺时针45°
  delay(1000);
  resetAllZX20S();  // 复位所有ZX20S舵机
}

/**
 * GM65_2触发的ZX20S舵机动作序列
 */
void zx20sActionForGM65_2() {
  Serial.println("Execute ZX20S Action for GM65_2");
  controlZX20SRelative(2, ZX20S_REL_100);  // 9号舵机转100°
  delay(1000);
  controlZX20SRelative(1, ZX20S_REL_45_CCW);  // 8号舵机逆时针45°
  delay(1000);
  resetAllZX20S();  // 复位所有ZX20S舵机
}

/**
 * 初始化MG90S传感器舵机（带动颜色传感器转动）
 */
void initSensorServo() {
  mg90s_sensor.attach(MG90S_PIN_6);  // 绑定舵机到6号引脚
  mg90s_sensor.write(0);  // 初始位置转到0°（对应球孔0）
  delay(1000);  // 等待舵机到位
  Serial.println("MG90S Sensor Servo Initialized");
}

/**
 * 移动颜色传感器到指定球孔位置
 * @param ballIndex 球孔序号（0-9）
 */
void moveSensorToBall(int ballIndex) {
  // 计算目标角度：10个球孔均匀分布在360°圆盘上，每个对应36°
  int targetAngle = ballIndex * 36;
  targetAngle = constrain(targetAngle, 0, 360);  // 限制角度范围
  
  Serial.print("Move color sensor to Ball ");
  Serial.print(ballIndex);
  Serial.print(" (Angle: ");
  Serial.print(targetAngle);
  Serial.println("°)");
  
  mg90s_sensor.write(targetAngle);  // 控制舵机转动到目标角度
  delay(1000);  // 等待舵机到位并稳定
}

/**
 * 扫描单个球孔的颜色并更新动态映射
 */
void scanSingleBall() {
  if (scanIndex >= 10) {  // 10个球孔已扫描完成
    isScanning = false;       // 停止扫描
    isMappingDone = true;     // 标记映射完成
    printDynamicMapping();    // 打印最终映射关系
    Serial.println("All Balls Scanned! Dynamic Mapping Completed.");
    return;
  }

  // 步骤1：移动传感器到当前扫描的球孔
  moveSensorToBall(scanIndex);

  // 步骤2：识别颜色（最多重试3次，直到识别成功）
  const char* ballColor = "Unknown";
  int retry = 3;
  while (retry-- > 0 && strcmp(ballColor, "Unknown") == 0) {
    ballColor = detectColor();
    delay(300);  // 间隔300ms重试
  }

  // 步骤3：更新动态映射关系
  int currentChannel = scanIndex;  // 球孔序号=PCA通道号
  updateColorToChannels(ballColor, currentChannel);  // 更新“颜色→通道”映射

  // 步骤4：打印扫描结果
  Serial.print("Scanned Ball ");
  Serial.print(scanIndex);
  Serial.print(" (Channel ");
  Serial.print(currentChannel);
  Serial.print("): Color = ");
  Serial.println(ballColor);

  // 步骤5：进入下一个球孔扫描
  scanIndex++;
  Serial.println("Auto Scan Next Ball After 1 Second...");
  delay(1000);  // 间隔1秒扫描下一个
}

/**
 * 更新“颜色→通道”映射（确保每种颜色最多对应2个通道）
 * @param color 颜色名称
 * @param channel 通道号
 */
void updateColorToChannels(const char* color, int channel) {
  // 遍历颜色映射表，找到匹配的颜色
  for (int c = 0; c < 5; c++) {
    if (strcmp(colorQrMap[c].colorName, color) == 0) {  // 字符串比较
      if (colorChannelCount[c] < 2) {  // 每种颜色最多2个通道
        colorToChannels[c][colorChannelCount[c]] = channel;
        colorChannelCount[c]++;  // 计数+1
      }
      break;  // 找到后退出循环
    }
  }
}

/**
 * 打印动态映射关系（调试用）
 */
void printDynamicMapping() {
  Serial.println("\n=== Dynamic Color-Channel Mapping ===");
  // 打印“通道→颜色”映射
  Serial.println("1. Channel → Color:");
  for (int ch = 0; ch < 10; ch++) {
    Serial.print("  Channel ");
    Serial.print(ch);
    Serial.print(": ");
    Serial.println(channelToColor[ch]);
  }
  
  // 打印“颜色→通道”映射
  Serial.println("\n2. Color → Channels:");
  for (int c = 0; c < 5; c++) {
    Serial.print("  ");
    Serial.print(colorQrMap[c].colorName);
    Serial.print(": ");
    for (int i = 0; i < 2; i++) {
      if (colorToChannels[c][i] != -1) {  // 只打印已分配的通道
        Serial.print(colorToChannels[c][i]);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
  Serial.println("======================================");
}

/**
 * 根据颜色获取对应的舵机通道
 * @param color 颜色名称
 * @param channels 存储通道号的数组
 * @return 通道数量（1或2）
 */
int getChannelsByColor(const char* color, int channels[]) {
  int count = 0;
  // 遍历颜色映射表，找到匹配的颜色
  for (int c = 0; c < 5; c++) {
    if (strcmp(colorQrMap[c].colorName, color) == 0) {
      // 收集该颜色对应的所有通道
      for (int i = 0; i < 2; i++) {
        if (colorToChannels[c][i] != -1) {
          channels[count++] = colorToChannels[c][i];
        }
      }
      break;  // 找到后退出循环
    }
  }
  return count;  // 返回通道数量
}

/**
 * 读取GM65二维码模块的数据
 * @param gm65 软件串口对象引用
 * @return 读取到的字符串数据
 */
String readGM65Data(SoftwareSerial &gm65) {
  String data = "";
  while (gm65.available()) {  // 读取所有可用数据
    char c = gm65.read();
    if (c != '\n' && c != '\r') {  // 过滤换行和回车符
      data += c;
    }
  }
  return data;
}

/**
 * 处理GM65读取到的二维码数据
 * @param data 二维码数据
 * @param isGM65_1 是否为第一个GM65模块
 */
void processGM65Data(String data, bool isGM65_1) {
  if (!isMappingDone) {  // 映射未完成时不处理指令
    Serial.println("Error: Dynamic Mapping Not Completed! Skip QR Code Processing.");
    return;
  }

  if (data.length() == 0) return;  // 空数据则返回
  Serial.print("GM65 ");
  Serial.print(isGM65_1 ? "1" : "2");
  Serial.print(" Read: ");
  Serial.println(data);

  // 步骤1：提取二维码中的颜色字符（r/y/b/g/w）
  char targetQrChar = ' ';
  for (int i = 0; i < data.length(); i++) {
    char c = tolower(data[i]);  // 转为小写统一处理
    for (int q = 0; q < COLOR_QR_COUNT; q++) {
      if (c == colorQrMap[q].qrCodeChar) {
        targetQrChar = c;
        break;
      }
    }
    if (targetQrChar != ' ') break;  // 找到后退出循环
  }

  // 步骤2：根据颜色字符找到对应的颜色名称
  const char* targetColor = "Unknown";
  for (int q = 0; q < COLOR_QR_COUNT; q++) {
    if (colorQrMap[q].qrCodeChar == targetQrChar) {
      targetColor = colorQrMap[q].colorName;
      break;
    }
  }

  // 步骤3：根据颜色名称找到对应的舵机通道
  int targetChannels[2];
  int channelCount = getChannelsByColor(targetColor, targetChannels);
  if (channelCount == 0) {  // 未找到对应通道
    Serial.println("Error: No Channels Found for Target Color!");
    return;
  }

  // 步骤4：控制对应通道的舵机释放小球
  Serial.print("Release All ");
  Serial.print(targetColor);
  Serial.print(" Balls (Channels: ");
  for (int i = 0; i < channelCount; i++) {
    Serial.print(targetChannels[i]);
    if (i < channelCount - 1) Serial.print(", ");
    // 释放小球：相对初始位置转动90°
    controlSG90Relative(targetChannels[i], SG90_RELATIVE_ANGLE);
    delay(500);
    resetSG90(targetChannels[i]);  // 复位舵机
  }
  Serial.println(")");

  // 步骤5：执行对应的ZX20S动作
  if (isGM65_1) {
    zx20sActionForGM65_1();
  } else {
    zx20sActionForGM65_2();
  }
}

// -------------------------- 主循环函数 --------------------------
void loop() {
  // 第一步：优先完成所有小球的颜色扫描和动态映射
  if (isScanning) {
    scanSingleBall();
    return;
  }

  // 第二步：映射完成后，持续监听GM65二维码指令
  if (gm65_1.available() > 0) {  // 第一个GM65有数据
    String data1 = readGM65Data(gm65_1);
    processGM65Data(data1, true);
  }

  if (gm65_2.available() > 0) {  // 第二个GM65有数据
    String data2 = readGM65Data(gm65_2);
    processGM65Data(data2, false);
  }

  delay(100);  // 降低循环频率，避免占用过多资源
}