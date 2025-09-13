#include <Wire.h>
const int SENSOR_ADDR_1 = 0x48;
const int SENSOR_ADDR_2 = 0x49;

char Ping(int addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xaa);
  Wire.endTransmission(0);
  Wire.requestFrom(addr, 1, 1);
  unsigned char device = Wire.read();
  if (device == 0x66) {
    Serial.print("发现设备: 0x");
    Serial.println(addr, HEX);
    return 0;
  } else {
    Serial.print("没有发现设备: 0x");
    Serial.println(addr, HEX);
    return 1;
  }
}

void Error(int addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xde);
  Wire.endTransmission(0);
  Wire.requestFrom(addr, 1, 1);
  unsigned char error = Wire.read();
  Serial.print("[0x"); Serial.print(addr, HEX); Serial.print("] ");
  if (!error) {
    Serial.println("没有错误");
  } else {
    if (error & 0b00010000) Serial.println("R过曝");
    if (error & 0b00001000) Serial.println("G过曝");
    if (error & 0b00000100) Serial.println("B过曝");
    if (error & 0b00000010) Serial.println("按键短路");
    if (error & 0b00000001) Serial.println("对管过曝");
  }
}

void Soft_reset(int addr) {
  Serial.print("软件复位: 0x");
  Serial.println(addr, HEX);
  Wire.beginTransmission(addr);
  Wire.write(0xc0);
  Wire.endTransmission();
}

void Get_Version(int addr) {
  Serial.print("[0x"); Serial.print(addr, HEX); Serial.print("] 当前版本V");
  char version_h, version_l;
  Wire.beginTransmission(addr);
  Wire.write(0xc1);
  Wire.endTransmission();
  Wire.requestFrom(addr, 1, 1);
  unsigned char version = Wire.read();
  version_l = version & 0x0F;
  version_h = (version >> 4) & 0x0F;
  Serial.print(version_h, DEC);
  Serial.print(".");
  Serial.print(version_l, DEC);
  Serial.println();
}

void change_IIC_Address(unsigned char address) {
  Serial.print("更改地址：");
  Serial.println(address, HEX);
  Wire.beginTransmission(SENSOR_ADDR_1);
  Wire.write(0xAD);
  Wire.write(0xAD);
  Wire.write(address);
  Wire.endTransmission();
}

void Generalcall_readdress() {
  Serial.println("广播");
  unsigned char readdress[8] = {0xB8, 0xD0, 0xCE, 0xAA, 0xBF, 0xC6, 0xBC, 0xBC};
  Wire.beginTransmission(0x00);
  Wire.write(readdress, 8);
  Wire.endTransmission();
}

void Read_RGB(int addr) {
  Serial.print("[0x"); Serial.print(addr, HEX); Serial.print("] RGB: ");
  Wire.beginTransmission(addr);
  Wire.write(0xD0);
  Wire.endTransmission(1);
  Wire.requestFrom(addr, 3, 1);
  for (int i = 0; i < 3; i++) {
    unsigned char data = Wire.read();
    Serial.print(data, DEC);
    Serial.print("  ");
  }
  Serial.println();
}

void Read_HSL(int addr) {
  Serial.print("[0x"); Serial.print(addr, HEX); Serial.print("] HSL: ");
  Wire.beginTransmission(addr);
  Wire.write(0xD1);
  Wire.endTransmission(1);
  Wire.requestFrom(addr, 3, 1);
  for (int i = 0; i < 3; i++) {
    unsigned char data = Wire.read();
    Serial.print(data, DEC);
    Serial.print("  ");
  }
  Serial.println();
}

void iic_scaner() {
  int nDevices = 0;
  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C 设备地址在 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");
      ++nDevices;
      break;
    } else if (error == 4) {
      Serial.print("错误地址 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
}

// 颜色识别函数（针对绿色、黄色优化）
void RecognizeColor(int addr) {
  unsigned char rgb[3], hsl[3];
  // 读取RGB和HSL数据
  Wire.beginTransmission(addr);
  Wire.write(0xD0);
  Wire.endTransmission(1);
  Wire.requestFrom(addr, 3, 1);
  for (int i = 0; i < 3; i++) rgb[i] = Wire.read();

  Wire.beginTransmission(addr);
  Wire.write(0xD1);
  Wire.endTransmission(1);
  Wire.requestFrom(addr, 3, 1);
  for (int i = 0; i < 3; i++) hsl[i] = Wire.read();

  // 打印调试数据
  Serial.print("[0x"); Serial.print(addr, HEX); Serial.print("] 原始RGB: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(rgb[i]);
    Serial.print(" ");
  }
  Serial.print(" | HSL: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(hsl[i]);
    Serial.print(" ");
  }
  Serial.print(" | ");

  // 暗光补偿（保持不变）
  const int darkCompensation = 25;
  int r = constrain(rgb[0] + darkCompensation, 0, 255);
  int g = constrain(rgb[1] + darkCompensation, 0, 255);
  int b = constrain(rgb[2] + darkCompensation, 0, 255);

  // 提取HSL分量
  int hue = hsl[0];       // 色相（0-240）
  int saturation = hsl[1];// 饱和度（0-240）
  int lightness = hsl[2]; // 亮度（0-240）

   // 1. 绿色识别优化（扩大色相+调整通道差+放宽亮度）
  bool isGreen = (hue >= 50 && hue <= 150)  // 色相范围扩大到150
                 && saturation > 10         // 饱和度要求降低到10
                 && lightness < 200         // 亮度上限放宽到200
                 && (g > r + 20 && g > b - 20); // 通道差调整

  // 2. 黄色识别（保持原逻辑或微调）
  bool isYellow = (hue >= 15 && hue <= 70) 
                  && saturation > 10 
                  && (r > 90 && g > 90 && b < 130);

  // 其他颜色识别（保持不变，或同步放宽阈值）
  bool isRed = ((hue >= 0 && hue <= 20) || (hue >= 220 && hue <= 240)) 
               && saturation > 30 
               && (r > 120 && g < 100 && b < 100);

  bool isBlue = (hue >= 120 && hue <= 180) 
                && saturation > 20 
                && (b > r + 20 && b > g + 20);

  bool isWhite = (saturation < 30) 
               && (lightness > 80 && lightness < 200) // 限制亮度上限
               && (abs(r - g) < 40) && (abs(g - b) < 40); // 确保RGB接近

  // 输出结果
  Serial.print("识别结果: ");
  if (isRed) Serial.println("红色");
  else if (isYellow) Serial.println("黄色");
  else if (isBlue) Serial.println("蓝色");
  else if (isGreen) Serial.println("绿色");
  else if (isWhite) Serial.println("白色");
  else Serial.println("未知颜色");
}

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  Serial.begin(115200);
  Generalcall_readdress();
  // 对两个传感器分别复位、检测、获取版本
  Soft_reset(SENSOR_ADDR_1);
  Soft_reset(SENSOR_ADDR_2);
  while (Ping(SENSOR_ADDR_1));
  while (Ping(SENSOR_ADDR_2));
  Get_Version(SENSOR_ADDR_1);
  Get_Version(SENSOR_ADDR_2);
}

void task() {
  // 对两个传感器分别执行主要操作
  while (Ping(SENSOR_ADDR_1));
  while (Ping(SENSOR_ADDR_2));
  Get_Version(SENSOR_ADDR_1);
  Get_Version(SENSOR_ADDR_2);
  Error(SENSOR_ADDR_1);
  Error(SENSOR_ADDR_2);
  Read_HSL(SENSOR_ADDR_1);
  Read_HSL(SENSOR_ADDR_2);
  Read_RGB(SENSOR_ADDR_1);
  Read_RGB(SENSOR_ADDR_2);
  RecognizeColor(SENSOR_ADDR_1);
  RecognizeColor(SENSOR_ADDR_2);
}

void loop() {
  task();
  delay(1000);
}