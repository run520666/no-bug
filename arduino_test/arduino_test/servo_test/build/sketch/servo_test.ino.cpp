#include <Arduino.h>
#line 1 "D:\\GIT\\no-bug\\arduino_test\\arduino_test\\servo_test\\servo_test.ino"
#include <Servo.h>

// 舵机引脚定义
#define ZX20S_PIN_7 7
#define ZX20S_PIN_8 8
#define ZX20S_PIN_9 9

// 舵机对象
Servo zx20s_7;
Servo zx20s_8;
Servo zx20s_9;

// ✅ 修改这个值来选择要测试的舵机 (7, 8, 或 9)
int selectedServo = 9;  // 改为 8 或 9 来测试其他舵机

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // 根据选择初始化对应舵机
  if (selectedServo == 7) {
    zx20s_7.attach(ZX20S_PIN_7);
    Serial.println(F("=== 舵机7测试模式 ==="));
  } else if (selectedServo == 8) {
    zx20s_8.attach(ZX20S_PIN_8);
    Serial.println(F("=== 舵机8测试模式 ==="));
  } else if (selectedServo == 9) {
    zx20s_9.attach(ZX20S_PIN_9);
    Serial.println(F("=== 舵机9测试模式 ==="));
  }
  
  // ...existing code...
  Serial.println(F("请在串口监视器中输入角度值 (0-180):"));
  Serial.print(F("当前测试的舵机编号: "));
  Serial.println(selectedServo);
  Serial.println();
}

void loop() {
  // 检查串口输入
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // 去除空格和换行符
    
    // 将输入转换为整数
    int angle = input.toInt();
    
    // 检查角度范围
    if (angle >= 0 && angle <= 180) {
      // 控制对应的舵机
      if (selectedServo == 7) {
        zx20s_7.write(angle);
        Serial.print(F("舵机7 转到: "));
      } else if (selectedServo == 8) {
        zx20s_8.write(angle);
        Serial.print(F("舵机8 转到: "));
      } else if (selectedServo == 9) {
        zx20s_9.write(angle);
        Serial.print(F("舵机9 转到: "));
      }
      
      Serial.print(angle);
      Serial.println(F("度"));
      
    } else {
      Serial.println(F("❌ 角度超出范围! 请输入 0-180 之间的数值"));
    }
    
    Serial.println(F("请输入下一个角度值:"));
  }
}
