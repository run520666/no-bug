#include <Wire.h>                  // I2C通信库
#include <Adafruit_PWMServoDriver.h>// PCA9685舵机驱动库
#include <SoftwareSerial.h>         // 软件串口库，用于GM65二维码模块
#include <Servo.h>                  // 舵机控制库，用于控制颜色传感器和7/8/9号舵机转动

// -------------------------- 硬件参数配置 --------------------------
// TCA9548A I2C多路复用器配置
#define TCA9548A_ADDR 0x70         // TCA9548A默认I2C地址
#define TCA_COLOR_SENSOR1_CH 0     // 第一个颜色传感器连接到TCA的通道0
#define TCA_COLOR_SENSOR2_CH 1     // 第二个颜色传感器连接到TCA的通道1
#define TCA_PCA9685_CH 2           // PCA9685舵机驱动板连接到TCA的通道2

// 控制颜色传感器转动的舵机配置
Servo sensorServo;                 // 创建舵机对象
#define SENSOR_SERVO_PIN 6         // 控制颜色识别传感器的舵机连接到6号引脚

// 7、8、9号引脚舵机配置
Servo zx20s_7;         // 翻转舵机，连接到7号引脚
Servo zx20s_8;         // 左舵机，连接到8号引脚
Servo zx20s_9;         // 右舵机，连接到9号引脚
#define ZX20S_PIN_7 7  // 翻转舵机引脚定义
#define ZX20S_PIN_8 8  // 左舵机引脚定义
#define ZX20S_PIN_9 9  // 右舵机引脚定义

//stm32通信
#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55

// 接收缓冲区
uint8_t rxBuffer[4];
uint8_t rxIndex = 0;
bool frameReceived = false;

unsigned long lastSendTime = 0;

// 翻转舵机相对转动角度定义
int pos = 0;
int zx20s_7_initialAngle = 90;      // 初始角度
int zx20s_7_moveAngle = 20;         // 转动角度
int zx20s_7_currentAngle = 90;      // 当前角度

// 左舵机相对转动角度定义
int zx20s_8_initialAngle = 180;     // 初始角度
int zx20s_8_moveAngle = 65;         // 转动角度
int zx20s_8_currentAngle = 180;     // 当前角度

// 右舵机相对转动角度定义
int zx20s_9_initialAngle = 35;      // 初始角度
int zx20s_9_moveAngle = 115;        // 转动角度
int zx20s_9_currentAngle = 35;      // 当前角度

int direction = 0;                  // 货舱方向 0左1右



// PCA9685舵机驱动板配置（控制10个SG90舵机）
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);  // PCA9685默认地址
#define SG90_MIN_PULSE 150         // SG90舵机最小脉冲宽度（对应0°）
#define SG90_MAX_PULSE 600         // SG90舵机最大脉冲宽度（对应180°）
#define SG90_HOME_PULSE 375        // SG90舵机默认初始位置脉冲值（中间位置）
#define SG90_RELATIVE_ANGLE -90    // 释放小球时相对初始位置转动的角度

// GM65二维码模块配置
#define GM65_1_RX 10               // 第一个GM65的RX引脚连接到Arduino 10号引脚
#define GM65_1_TX 11               // 第一个GM65的TX引脚连接到Arduino 11号引脚
#define GM65_2_RX 12               // 第二个GM65的RX引脚连接到Arduino 12号引脚
#define GM65_2_TX 13               // 第二个GM65的TX引脚连接到Arduino 13号引脚
SoftwareSerial gm65_1(GM65_1_RX, GM65_1_TX);  // 创建第一个GM65的软件串口对象
SoftwareSerial gm65_2(GM65_2_RX, GM65_2_TX);  // 创建第二个GM65的软件串口对象
#define GM65_BAUD 9600             // GM65默认波特率
#define GM65_TIMEOUT 100           // 二维码模块读取超时时间(毫秒)

// 颜色识别传感器配置
#define COLOR_SENSOR_ADDR 0x48     // 颜色传感器I2C地址
#define CMD_READ_RGB 0xD0          // 读取RGB数据命令
#define CMD_READ_HSL 0xD1          // 读取HSL数据命令
#define CMD_PING 0xAA              // Ping命令
#define PING_RESPONSE 0x66         // Ping成功响应值
#define SENSOR_READ_DELAY 50       // 两个传感器读取之间的延迟(毫秒)，解决冲突

// 传感器角度与通道映射配置
struct AngleChannelMap {
  int angle;         // 舵机角度
  int channel1;      // 第一个颜色传感器对应的PCA通道
  int channel2;      // 第二个颜色传感器对应的PCA通道
};
AngleChannelMap angleMaps[] = {
  {5,    0, 5},     // 0度时，传感器1对应通道0，传感器2对应通道5
  {47,   1, 6},     // 36度时，传感器1对应通道1，传感器2对应通道6
  {95,   2, 7},     // 80度时，传感器1对应通道2，传感器2对应通道7
  {135,  3, 8},     // 118度时，传感器1对应通道3，传感器2对应通道8
  {175,  4, 9}      // 180度时，传感器1对应通道4，传感器2对应通道9
};
const int ANGLE_MAP_COUNT = sizeof(angleMaps) / sizeof(angleMaps[0]);

// 颜色-二维码字符映射表
struct ColorQrMap {
  const char* colorName;  // 颜色名称
  char qrCodeChar;        // 二维码中对应的字符
};
ColorQrMap colorQrMap[] = {
  {"Red",    'r'},    // 红色对应'r'
  {"Yellow", 'y'},    // 黄色对应'y'
  {"Blue",   'b'},    // 蓝色对应'b'
  {"Green",  'g'},    // 绿色对应'g'
  {"White",  'w'}     // 白色对应'w'
};
const int COLOR_QR_COUNT = sizeof(colorQrMap) / sizeof(colorQrMap[0]);

// 颜色-通道映射数据结构
const char* channelToColor[10] = {"Unknown", "Unknown", "Unknown", "Unknown", "Unknown", 
                                 "Unknown", "Unknown", "Unknown", "Unknown", "Unknown"};
int colorToChannels[5][2];  // 颜色→通道映射
int colorChannelCount[5];   // 每种颜色已找到的通道数量

// 全局状态变量
bool isMappingDone = false;  // 颜色-通道映射是否完成

// -------------------------- 函数声明 --------------------------
void initDynamicMapping();
void tcaSelect(uint8_t channel);
bool colorSensorPing(int sensorNum);
bool colorSensorInit();
void readRGB(int rgb[3], int sensorNum);
void readHSL(int hsl[3], int sensorNum);
const char* detectColorSensor1();  // 传感器1颜色识别
const char* detectColorSensor2();  // 传感器2颜色识别
const char* detectColor(int sensorNum);  // 统一接口
uint16_t relativeAngleToPulseSG90(int relativeAngle);
void controlSG90Relative(int channel, int relativeAngle);
void resetSG90(int channel);
void updateColorToChannels(const char* color, int channel);
void printDynamicMapping();
int getChannelsByColor(const char* color, int channels[]);
String readGM65Data(SoftwareSerial &gm65);
void processGM65Data(String data, int gm65Num);
void autoMapColors();

// 7、8、9号舵机控制函数声明
void zx20s_7ChuShiHua();     // 翻转舵机初始化函数
void zx20s_7Left();          // 向左转动函数
void zx20s_7Right();         // 向右转动函数
void zx20s_7FuWei();         // 复位函数
void zx20s_8ChuShiHua();     // 左舵机初始化函数
void zx20s_8Left();          // 左舵机放下滑道函数
void zx20s_8FuWei();         // 左舵机复位函数
void zx20s_9ChuShiHua();     // 右舵机初始化函数
void zx20s_9Right();         // 右舵机放下滑道函数
void zx20s_9FuWei();         // 右舵机复位函数

// -------------------------- 初始化函数 --------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  //根据stm32实际接口设置
  //  pinMode(LED_BUILTIN, OUTPUT);
  //Serial.println("Arduino UART Master Ready");

  // 初始化颜色传感器舵机
  sensorServo.attach(SENSOR_SERVO_PIN);
  Serial.println("Sensor servo initialized");

  // 初始化7、8、9号舵机
  zx20s_7ChuShiHua();
  zx20s_8ChuShiHua();
  zx20s_9ChuShiHua();
  Serial.println("7,8,9 servos initialized");

  // 初始化I2C和TCA9548A
  Wire.begin();
  Serial.println("Initializing TCA9548A...");

  // 初始化颜色传感器
  if (!colorSensorInit()) {
    Serial.println("Color Sensor Initialization Failed!");
    while (1);
  }
  Serial.println("Color Sensors Ready");

  // 初始化PCA9685
  tcaSelect(TCA_PCA9685_CH);
  pca.begin();
  pca.setPWMFreq(50);
  Serial.println("PCA9685 Ready");

  // 初始化GM65二维码模块
  gm65_1.begin(GM65_BAUD);
  gm65_2.begin(GM65_BAUD);
  Serial.println("Both GM65 Modules Ready");

  // 初始化动态映射数组
  initDynamicMapping();

  // 执行自动颜色映射
  Serial.println("Starting automatic color mapping...");
  autoMapColors();
  
  // 映射完成
  isMappingDone = true;
  printDynamicMapping();
  Serial.println("Mapping completed! Ready to receive QR commands.");
}

// -------------------------- 核心功能函数 --------------------------

void initDynamicMapping() {
  for (int c = 0; c < 5; c++) {
    colorChannelCount[c] = 0;
    colorToChannels[c][0] = -1;
    colorToChannels[c][1] = -1;
  }
}

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);  // 等待通道切换完成
}

bool colorSensorPing(int sensorNum) {
  // 选择对应的传感器通道
  tcaSelect(sensorNum == 1 ? TCA_COLOR_SENSOR1_CH : TCA_COLOR_SENSOR2_CH);
  
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CMD_PING);
  Wire.endTransmission(0);

  Wire.requestFrom(COLOR_SENSOR_ADDR, 1);
  if (Wire.available()) {
    return Wire.read() == PING_RESPONSE;
  }
  return false;
}

bool colorSensorInit() 
{
  // 检查两个传感器，加入延迟避免冲突
  bool sensor1Ready = false;
  bool sensor2Ready = false;
  
  for (int i = 0; i < 5; i++) {
    if (!sensor1Ready) {
      sensor1Ready = colorSensorPing(1);
      delay(SENSOR_READ_DELAY);  // 传感器1检测后延迟
    }
    if (!sensor2Ready) {
      sensor2Ready = colorSensorPing(2);
      delay(SENSOR_READ_DELAY);  // 传感器2检测后延迟
    }
    
    if (sensor1Ready && sensor2Ready) break;
    delay(100);
  }
  
  Serial.print("Sensor 1 ready: "); Serial.println(sensor1Ready ? "Yes" : "No");
  Serial.print("Sensor 2 ready: "); Serial.println(sensor2Ready ? "Yes" : "No");
  
  return sensor1Ready && sensor2Ready;
}

void readRGB(int rgb[3], int sensorNum) {
  tcaSelect(sensorNum == 1 ? TCA_COLOR_SENSOR1_CH : TCA_COLOR_SENSOR2_CH);
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CMD_READ_RGB);
  Wire.endTransmission(0);

  Wire.requestFrom(COLOR_SENSOR_ADDR, 3);
  int i = 0;
  while (Wire.available() && i < 3) {
    rgb[i++] = Wire.read();
  }
  delay(10);  // 读取后短暂延迟
}

void readHSL(int hsl[3], int sensorNum) {
  tcaSelect(sensorNum == 1 ? TCA_COLOR_SENSOR1_CH : TCA_COLOR_SENSOR2_CH);
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CMD_READ_HSL);
  Wire.endTransmission(0);

  Wire.requestFrom(COLOR_SENSOR_ADDR, 3);
  int i = 0;
  while (Wire.available() && i < 3) {
    hsl[i++] = Wire.read();
  }
  delay(10);  // 读取后短暂延迟
}

// 传感器1的颜色识别（独立阈值）
const char* detectColorSensor1() {
  int rgb[3] = {0};
  int hsl[3] = {0};
  int r_sum = 0, g_sum = 0, b_sum = 0;
  int h_sum = 0, s_sum = 0, l_sum = 0;
  int SAMPLE_COUNT = 3;
  int r = 0, g = 0, b = 0;
  // 三次采样求平均
  for (int i = 0; i < SAMPLE_COUNT; i++) {
  readRGB(rgb, 1);
  readHSL(hsl, 1);
   // 调试输出，包含传感器编号
   r = rgb[0];
   g = rgb[1];
   b = rgb[2];
  Serial.print("Sensor 1 | RGB: ");
  Serial.print(r); Serial.print(",");
  Serial.print(g); Serial.print(",");
  Serial.print(b); Serial.print(" | HSL: ");
  Serial.print(hsl[0]); Serial.print(",");
  Serial.print(hsl[1]); Serial.print(",");
  Serial.println(hsl[2]);
  // 传感器1的暗光补偿参数
   r = constrain(rgb[0] + 0, 0, 255);
   g = constrain(rgb[1] + 0, 0, 255);
   b = constrain(rgb[2] + 0, 0, 255);
    r_sum += r;
    g_sum += g;
    b_sum += b;
    h_sum += hsl[0];
    s_sum += hsl[1];
    l_sum += hsl[2];
    delay(10);  // 每次采样间隔，可按需要调整
  }

  // 求平均
  r = r_sum / SAMPLE_COUNT;
  g = g_sum / SAMPLE_COUNT;
  b = b_sum / SAMPLE_COUNT;
  hsl[0] = h_sum / SAMPLE_COUNT;
  hsl[1] = s_sum / SAMPLE_COUNT;
  hsl[2] = l_sum / SAMPLE_COUNT;
  
 // 调试输出，包含传感器编号
  Serial.print("Sensor 1 | RGB: ");
  Serial.print(r); Serial.print(",");
  Serial.print(g); Serial.print(",");
  Serial.print(b); Serial.print(" | HSL: ");
  Serial.print(hsl[0]); Serial.print(",");
  Serial.print(hsl[1]); Serial.print(",");
  Serial.println(hsl[2]);

// 1. 判断黄色（传感器1的阈值）
  if (
    r >= 200 && r <= 255 &&
    g >= 170 && g <= 255 &&
    b >= 0 && b <= 255 &&
    hsl[0] >= 170 && hsl[0] <= 225 &&
    hsl[1] >= 0 && hsl[1] <= 230 &&
    hsl[2] >= 130 && hsl[2] <= 245 &&
    hsl[0] < hsl[1]) {
   return "Yellow";
   }

   // HSL 判断（Hue≈60°, S高, L中高）
   //else if (hsl[0] >= 0 && hsl[0] <= 200 &&
    //hsl[1] >= 0 && hsl[1] <= 200 &&
    //hsl[2] >= 120 && hsl[2] <= 245) {
      //return "Yellow";
    //}

  // 2. 判断白色（传感器1的阈值）
  //if (hsl[1] < 190 && hsl[2] > 200 &&
      //r > 190 && g > 200 &&
      //abs(r - g) < 35 && abs(g - b) < 60) {
   // return "White";
  //}
  else if (r >=190 && r <=255 && 
      g >=190 && g <=255 && 
      b >=190 && b <=255 &&
      hsl[0] >=0 && hsl[0] <=225 &&
      hsl[1] >=0 && hsl[1] <=220 &&
      hsl[2] >=180 && hsl[2] <=240) {
        return "White";
      }
  
  // 3. 判断绿色（传感器1的阈值）
  else if (r >= 45 && r <= 255 &&
    g >= 150 && g <= 255 &&
    b >= 120 && b <= 255 &&
    hsl[0] >= 110 && hsl[0] <= 130 &&
    hsl[1] >= 75 && hsl[1] <= 145 &&
    hsl[2] >= 100 && hsl[2] <= 165) {
   return "Green";
}

// HSL 判断（补充判断条件）
  //else if (hsl[0] >= 115 && hsl[0] <= 130 &&
    //hsl[1] >= 80 && hsl[1] <= 145 &&
    //hsl[2] >= 120 && hsl[2] <= 160) {
      //return "Green";
    //}
  
  // 4. 判断红色（传感器1的阈值）
  else if (r >= 150 && r <= 255 &&
    g >= 0 && g <= 90 &&
    b >= 0 && b <= 120 &&
    hsl[0] >= 0 && hsl[0] <= 50 &&
    hsl[1] >= 80 && hsl[1] <= 230 &&
    hsl[2] >= 70 && hsl[2] <= 120){
    return "Red";
    }

  //else if (hsl[0] >= 0 && hsl[0] <= 10 &&
    //hsl[1] >= 200 && hsl[1] <= 240 &&
    //hsl[2] >= 100 && hsl[2] <= 130) {
     //return "Red";
    //}
  
  // 5. 判断蓝色（传感器1的阈值）
  else if(r >= 0 && r <= 60 &&
      g >= 20 && g <= 100 &&
      b >= 40 && b <= 150 &&
      hsl[0] >= 130 && hsl[0] <= 180 &&
      hsl[1] >= 20 && hsl[1] <= 245 &&
      hsl[2] >= 30 && hsl[2] <= 80) {
        return "Blue";
          }
  
  return "Unknown";
}


// 传感器2的颜色识别（独立阈值）
const char* detectColorSensor2() {
  int rgb[3] = {0};
  int hsl[3] = {0};
  int r_sum = 0, g_sum = 0, b_sum = 0;
  int h_sum = 0, s_sum = 0, l_sum = 0;
  int SAMPLE_COUNT = 3;
  int r = 0, g = 0, b = 0;
  // 三次采样求平均
  for (int i = 0; i < SAMPLE_COUNT; i++){
  readRGB(rgb, 2);
  readHSL(hsl, 2);

// 调试输出，包含传感器编号
   r = rgb[0];
   g = rgb[1];
   b = rgb[2];
  Serial.print("Sensor 2 | RGB: ");
  Serial.print(r); Serial.print(",");
  Serial.print(g); Serial.print(",");
  Serial.print(b); Serial.print(" | HSL: ");
  Serial.print(hsl[0]); Serial.print(",");
  Serial.print(hsl[1]); Serial.print(",");
  Serial.println(hsl[2]);

  // 传感器2的暗光补偿参数（可以与传感器1不同）
  int r = constrain(rgb[0] + 0, 0, 255);
  int g = constrain(rgb[1] + 0, 0, 255);
  int b = constrain(rgb[2] + 0, 0, 255);

  r_sum += r;
    g_sum += g;
    b_sum += b;
    h_sum += hsl[0];
    s_sum += hsl[1];
    l_sum += hsl[2];
    delay(10);  // 每次采样间隔，可按需要调整
  }

  // 求平均
  r = r_sum / SAMPLE_COUNT;
  g = g_sum / SAMPLE_COUNT;
  b = b_sum / SAMPLE_COUNT;
  hsl[0] = h_sum / SAMPLE_COUNT;
  hsl[1] = s_sum / SAMPLE_COUNT;
  hsl[2] = l_sum / SAMPLE_COUNT;

  Serial.print("Sensor 2 | RGB: ");
  Serial.print(r); Serial.print(",");
  Serial.print(g); Serial.print(",");
  Serial.print(b); Serial.print(" | HSL: ");
  Serial.print(hsl[0]); Serial.print(",");
  Serial.print(hsl[1]); Serial.print(",");
  Serial.println(hsl[2]);

// 1. 判断黄色（传感器2的阈值）
  if (r >= 200 && r <= 255 &&
    g >= 200 && g <= 255 &&
    b >= 0 && b <= 255 &&
    hsl[0] >= 150 && hsl[0] <= 215 &&
    hsl[1] >= 140 && hsl[1] <= 230 &&
    hsl[2] >= 120 && hsl[2] <= 245) {
   return "Yellow";
   }
   // HSL 判断（Hue≈60°, S高, L中高）
   //else if (hsl[0] >= 0 && hsl[0] <= 75 &&
    //hsl[1] >= 0 && hsl[1] <= 100 &&
   // hsl[2] >= 120 && hsl[2] <= 245) {
     // return "Yellow";
    //}

// 2. 优先判断白色（传感器2的阈值，可以与传感器1不同）
  //if (hsl[1] < 35 && hsl[2] > 80 &&
      //abs(r - g) < 5 && abs(g - b) < 10) {
    //return "White";
  //}
  else if (r >=190 && r <=255 && 
      g >=190 && g <=255 && b >=190 && b <=255 &&
      hsl[0] >=0 && hsl[0] <=230 &&
      hsl[1] >=0 && hsl[1] <=200 &&
      hsl[2] >=200 && hsl[2] <=240) {
        return "White";
    }
  
// 3. 判断绿色（传感器2的阈值）
  else if (r >= 60 && r <= 150 &&
    g >= 160 && g <= 255 &&
    b >= 80 && b <= 255 &&
    hsl[0] >= 110 && hsl[0] <= 130 &&
    hsl[1] >= 55 && hsl[1] <= 225 &&
    hsl[2] >= 100 && hsl[2] <= 186) {
  return "Green";
   }

// HSL 判断（作为补充判断条件）
  //else if (hsl[0] >= 120 && hsl[0] <= 130 &&
    //hsl[1] >= 55 && hsl[1] <= 160 &&
    //hsl[2] >= 130 && hsl[2] <= 165) {
      //return "Green";
    //}
  
  
  // 4. 判断红色（传感器2的阈值）
   else if (r >= 20 && r <= 255 &&
    g >= 0 && g <= 230 &&
    b >= 0 && b <= 255 &&
    hsl[0] >= 0 && hsl[0] <= 240 &&
    hsl[1] >= 70 && hsl[1] <= 240 &&
    hsl[2] >= 85 && hsl[2] <= 190){
    return "Red";
    }
  //else if ((hsl[0] >= 0 && hsl[0] <= 30) || (hsl[0] >= 210 && hsl[0] <= 255)) {
    //if (hsl[1] > 20 &&
        //r > g + 20 && r > b + 20) {
      //return "Red";
    //}
  //}
  
  // 5. 判断蓝色（传感器2的阈值）
  else if(r >= 50 && r <= 145 &&
          g >= 110 && g <= 170 &&
          b >= 150 && b <= 220) {
            return "Blue";
          }
  else if(hsl[0] >= 130 && hsl[0] <= 180 &&
          hsl[1] >= 80 && hsl[1] <= 245 &&
          hsl[2] >= 30 && hsl[2] <= 110) {
            return "Blue";
          }
  //else if (hsl[0] >= 130 && hsl[0] <= 170) {
    //if (hsl[1] > 50 && 
        //b > 5 && b < 160 &&
        //abs(g - b) > 40 ) {
      //return "Blue";
    //}
  //}
  
  return "Unknown";
}

// 统一的颜色检测接口
const char* detectColor(int sensorNum) {
  if (sensorNum == 1) {
    return detectColorSensor1();
  } else {
    return detectColorSensor2();
  }
}
    
uint16_t relativeAngleToPulseSG90(int relativeAngle) {
  int angle = map(SG90_HOME_PULSE, SG90_MIN_PULSE, SG90_MAX_PULSE, 0, 180);
  angle += relativeAngle;
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SG90_MIN_PULSE, SG90_MAX_PULSE);
}

void controlSG90Relative(int channel, int relativeAngle) {
  tcaSelect(TCA_PCA9685_CH);
  uint16_t pulse = relativeAngleToPulseSG90(relativeAngle);
  pca.setPWM(channel, 0, pulse);
  delay(100);
}

void resetSG90(int channel) {
  controlSG90Relative(channel, 0);
}

void updateColorToChannels(const char* color, int channel) {
  channelToColor[channel] = color;
  
  for (int c = 0; c < 5; c++) {
    if (strcmp(colorQrMap[c].colorName, color) == 0) {
      if (colorChannelCount[c] < 2) {
        colorToChannels[c][colorChannelCount[c]] = channel;
        colorChannelCount[c]++;
      }
      break;
    }
  }
}

void printDynamicMapping() {
  Serial.println("\n=== Color-Channel Mapping ===");
  Serial.println("Channel → Color:");
  for (int ch = 0; ch < 10; ch++) {
    Serial.print("  Channel ");
    Serial.print(ch);
    Serial.print(": ");
    Serial.println(channelToColor[ch]);
  }
  
  Serial.println("\nColor → Channels:");
  for (int c = 0; c < 5; c++) {
    Serial.print("  ");
    Serial.print(colorQrMap[c].colorName);
    Serial.print(": ");
    for (int i = 0; i < 2; i++) {
      if (colorToChannels[c][i] != -1) {
        Serial.print(colorToChannels[c][i]);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
  Serial.println("============================");
}

int getChannelsByColor(const char* color, int channels[]) {
  int count = 0;
  for (int c = 0; c < 5; c++) {
    if (strcmp(colorQrMap[c].colorName, color) == 0) {
      for (int i = 0; i < 2; i++) {
        if (colorToChannels[c][i] != -1) {
          channels[count++] = colorToChannels[c][i];
        }
      }
      break;
    }
  }
  return count;
}

// 修改后的GM65数据读取函数
String readGM65Data(SoftwareSerial &gm65) {
  String data = "";
  unsigned long startTime = millis();
  
  // 等待数据到来，超时返回空字符串
  while (!gm65.available() && millis() - startTime < GM65_TIMEOUT) {
    delay(1);
  }
  
  // 读取所有可用数据
  while (gm65.available()) {
    data += (char)gm65.read();
    delay(2); // 等待下一个字节
  }
  
  return data;
}

void processGM65Data(String data, int gm65Num) {
  if (!isMappingDone) {
    Serial.println("Error: Mapping Not Completed!");
    return;
  }

  if (data.length() == 0) return;
  Serial.print("GM65 ");
  Serial.print(gm65Num);
  Serial.print(" Read: ");
  Serial.println(data);

  // 提取颜色字符
  char targetQrChar = ' ';
  for (int i = 0; i < data.length(); i++) {
    char c = tolower(data[i]);
    for (int q = 0; q < COLOR_QR_COUNT; q++) {
      if (c == colorQrMap[q].qrCodeChar) {
        targetQrChar = c;
        break;
      }
    }
    if (targetQrChar != ' ') break;
  }

  // 找到对应颜色
  const char* targetColor = "Unknown";
  for (int q = 0; q < COLOR_QR_COUNT; q++) {
    if (colorQrMap[q].qrCodeChar == targetQrChar) {
      targetColor = colorQrMap[q].colorName;
      break;
    }
  }

  // 找到对应通道并释放小球
  int targetChannels[2];
  int channelCount = getChannelsByColor(targetColor, targetChannels);
  if (channelCount == 0) {
    Serial.println("Error: No Channels Found!");
    return;
  }

  Serial.print("Release All ");
  Serial.print(targetColor);
  Serial.print(" Balls (Channels: ");
  for (int i = 0; i < channelCount; i++) {
    Serial.print(targetChannels[i]);
    if (i < channelCount - 1) Serial.print(", ");
    controlSG90Relative(targetChannels[i], SG90_RELATIVE_ANGLE);
    delay(500);
    resetSG90(targetChannels[i]);
  }
  Serial.println(")");

  // 根据不同GM65执行不同的舵机动作序列
  if (gm65Num == 1) {
    // 第一个GM65: 8号舵机先转动，再7号舵机转动
    Serial.println("Processing with GM65 1 - activating servo 8 then 7");
    zx20s_8Left();
    delay(500);
    zx20s_7Left();
    delay(500);
    
    // 短暂延迟后复位
    delay(1000);
    zx20s_7FuWei();
    delay(300);
    zx20s_8FuWei();
  } 
  else if (gm65Num == 2) {
    // 第二个GM65: 9号舵机先转动，再7号舵机转动
    Serial.println("Processing with GM65 2 - activating servo 9 then 7");
    zx20s_9Right();
    delay(500);
    zx20s_7Right();
    delay(500);
    
    // 短暂延迟后复位
    delay(1000);
    zx20s_7FuWei();
    delay(300);
    zx20s_9FuWei();
  }
}

// 自动映射颜色到通道
void autoMapColors() {
  // 首先将舵机复位到0度
  sensorServo.write(0);
  delay(1500);
  
  // 按预设角度依次转动并读取两个传感器的颜色
  for (int i = 0; i < ANGLE_MAP_COUNT; i++) {
    int angle = angleMaps[i].angle;
    int channel1 = angleMaps[i].channel1;
    int channel2 = angleMaps[i].channel2;
    
    Serial.print("\nMoving to angle: ");
    Serial.println(angle);
    
    // 转动到指定角度
    sensorServo.write(angle);
    delay(1500);  // 等待舵机稳定
    
    // 读取第一个传感器颜色并映射到对应通道
    const char* color1 = detectColorSensor1();  // 直接调用传感器1的检测函数
    updateColorToChannels(color1, channel1);
    Serial.print("Sensor 1 detected ");
    Serial.print(color1);
    Serial.print(" for channel ");
    Serial.println(channel1);
    
    // 添加延迟，确保两个传感器不会同时通信
    delay(SENSOR_READ_DELAY);
    
    // 读取第二个传感器颜色并映射到对应通道
    const char* color2 = detectColorSensor2();  // 直接调用传感器2的检测函数
    updateColorToChannels(color2, channel2);
    Serial.print("Sensor 2 detected ");
    Serial.print(color2);
    Serial.print(" for channel ");
    Serial.println(channel2);
  }
  
  // 回到初始位置
  sensorServo.write(0);
  delay(1000);
}

// -------------------------- 7、8、9号舵机控制函数 --------------------------

void zx20s_7ChuShiHua() {
  zx20s_7.attach(ZX20S_PIN_7);
  zx20s_7.write(zx20s_7_initialAngle);
  zx20s_7_currentAngle = zx20s_7_initialAngle;
  delay(1000);
  Serial.println("翻转舵机初始化完成");
}
// 翻转舵机向左转动函数
void zx20s_7Left() {
  zx20s_7_currentAngle = zx20s_7_currentAngle + zx20s_7_moveAngle;
  if (zx20s_7_currentAngle > 180) zx20s_7_currentAngle = 180;  // 限制最大角度
  zx20s_7.write(zx20s_7_currentAngle);
  
  /*zx20s_7_currentAngle = zx20s_7_currentAngle - zx20s_7_moveAngle;
  if (zx20s_7_currentAngle < 0) zx20s_7_currentAngle = 0;      // 限制最小角度
  zx20s_7.write(zx20s_7_currentAngle);
  Serial.println("左翻");*/
}
// 翻转舵机向右转动函数
void zx20s_7Right() {
  zx20s_7_currentAngle = zx20s_7_currentAngle - zx20s_7_moveAngle;
  if (zx20s_7_currentAngle < 0) zx20s_7_currentAngle = 0;      // 限制最小角度
  zx20s_7.write(zx20s_7_currentAngle);
  
  /*zx20s_7_currentAngle = zx20s_7_currentAngle + zx20s_7_moveAngle;
  if (zx20s_7_currentAngle > 180) zx20s_7_currentAngle = 180;  // 限制最大角度
  zx20s_7.write(zx20s_7_currentAngle);*/
  Serial.println("右翻 ");
}
// 翻转舵机复位函数
void zx20s_7FuWei() {
  zx20s_7_currentAngle = zx20s_7_initialAngle;
  zx20s_7.write(zx20s_7_currentAngle);
  Serial.println("复位初始角度");
}

// 左舵机初始化函数
void zx20s_8ChuShiHua() {
  zx20s_8.attach(ZX20S_PIN_8);
  zx20s_8.write(zx20s_8_initialAngle);
  zx20s_8_currentAngle = zx20s_8_initialAngle;
  delay(1000);
  Serial.println("左舵机初始化完成");
}
// 左滑道放下函数
void zx20s_8Left() {
  if (zx20s_8_currentAngle < 0) zx20s_8_currentAngle = 0;      // 限制最小角度
  zx20s_8_currentAngle = zx20s_8_currentAngle - zx20s_8_moveAngle;
   
  zx20s_8.write(zx20s_8_currentAngle);
  for (pos = 180; pos >= zx20s_8_currentAngle; pos -= 1) { // goes from 180 degrees to 0 degrees
    // in steps of 1 degree
    zx20s_8.write(pos);             // tell servo to go to position in variable 'pos'
    delay(10);
  }
  Serial.println("左滑道已放下");
}
// 左舵机复位函数
void zx20s_8FuWei() {
  zx20s_8_currentAngle = zx20s_8_initialAngle;
  zx20s_8.write(zx20s_8_currentAngle);
  Serial.println("复位初始角度");
}
// 右舵机初始化函数
void zx20s_9ChuShiHua() {
  zx20s_9.attach(ZX20S_PIN_9);
  zx20s_9.write(zx20s_9_initialAngle);
  zx20s_9_currentAngle = zx20s_9_initialAngle;
  delay(1000);
  Serial.println("右舵机初始化完成");
}
// 右滑道放下函数
void zx20s_9Right() {

  zx20s_9_currentAngle = zx20s_9_currentAngle + zx20s_9_moveAngle;
  if (zx20s_9_currentAngle > 180) zx20s_9_currentAngle = 180;  // 限制最大角度
  zx20s_9.write(zx20s_9_currentAngle); //初始化为35度
 for (pos = 0; pos <= zx20s_9_currentAngle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    zx20s_9.write(pos);             // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  Serial.println("右滑道已放下");
}
// 右舵机复位函数
void zx20s_9FuWei() {
  zx20s_9_currentAngle = zx20s_9_initialAngle;
  zx20s_9.write(zx20s_9_currentAngle);
  Serial.println("复位初始角度zx20s_9_currentAngle");
}

// -------------------------- 主循环函数 --------------------------
void loop() {
   //通信测试，后期需有确认和停止发送
   // 每1秒发送数据1到STM32
  if (millis() - lastSendTime >= 1000) {
    sendData(1);  // Arduino发送1
    lastSendTime = millis();
  }
  
  // 接收STM32返回的数据
  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();
    
    // 状态机接收
    if (rxIndex == 0 && inByte == FRAME_HEAD) {
      // 检测到帧头
      rxBuffer[rxIndex++] = inByte;
    } else if (rxIndex > 0 && rxIndex < 4) {
      //之后rxIndex要改成5位，date有两个
      rxBuffer[rxIndex++] = inByte;
      
      // 接收完整帧
      if (rxIndex == 4 && inByte == FRAME_TAIL) {
        //5
        frameReceived = true;
      }
      
      // 接收错误，重置
      if (rxIndex == 4 && inByte != FRAME_TAIL) {
        rxIndex = 0;
        Serial.println("Frame error: wrong tail");
      }
    } else {
      // 异常，重置
      rxIndex = 0;
    }
  }
  
  // 处理接收到的完整帧
  if (frameReceived) {
    // 校验帧头和帧尾
    if (rxBuffer[0] == FRAME_HEAD && rxBuffer[3] == FRAME_TAIL) {
      uint8_t data = rxBuffer[1];//lflag=rxBuffer[1],rflag= rxBuffer[2]
      uint8_t checksum = rxBuffer[2];//checksum = rxBuffer[3];
      
      // 校验数据
      if (checksum == (FRAME_HEAD + data)) {
        // 数据有效
        Serial.print("Received from STM32: ");
        Serial.println(data);//lflag rflag
        
        // 根据收到的数据控制LED（实际控制判断逻辑，滑道舵机）
        if (data == 0) {
          digitalWrite(LED_BUILTIN, LOW);/
        } else {
          digitalWrite(LED_BUILTIN, HIGH);
        }
      } else {
        Serial.println("Checksum error!");
      }
    }
    
    // 重置接收状态
    rxIndex = 0;
    frameReceived = false;
  }
  
  delay(10);

  // 映射完成后处理二维码指令
  if (isMappingDone) {
    // 切换到第一个GM65并读取数据
    gm65_1.listen();
    String data1 = readGM65Data(gm65_1);
    if (data1.length() > 0) {
      processGM65Data(data1, 1);
    }
    
    // 添加延迟避免连续读取冲突
    delay(SENSOR_READ_DELAY);
    
    // 切换到第二个GM65并读取数据
    gm65_2.listen();
    String data2 = readGM65Data(gm65_2);
    if (data2.length() > 0) {
      processGM65Data(data2, 2);
    }
  }

  delay(100);
}

// 发送数据到STM32
void sendData(uint8_t data) {
  uint8_t frame[4];
  frame[0] = FRAME_HEAD;
  frame[1] = data;                    // 数据：1
  frame[2] = FRAME_HEAD + data;       // 校验和
  frame[3] = FRAME_TAIL;
  
  Serial.write(frame, 4);
  Serial.print("Sent to STM32: ");
  Serial.println(data);
}
    