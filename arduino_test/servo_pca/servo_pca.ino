#include <Servo.h>

// ==================== 可修改的定义 ====================
#define SERVO_PIN 9           // 舵机连接的引脚号（修改这里控制不同引脚）
#define INITIAL_ANGLE 90      // 初始绝对角度（修改这里设置初始角度）
// =====================================================

Servo myServo;
int currentAngle = INITIAL_ANGLE;

void setup() {
  Serial.begin(115200);
  
  // 初始化舵机
  myServo.attach(SERVO_PIN);
  myServo.write(INITIAL_ANGLE);
  delay(500);
  
  Serial.println(F("舵机测试程序"));
  Serial.print(F("引脚: "));
  Serial.println(SERVO_PIN);
  Serial.print(F("初始角度: "));
  Serial.println(INITIAL_ANGLE);
  Serial.println(F("输入相对角度 (例: +10 或 -20)"));
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      int relativeAngle = input.toInt();
      
      // 计算目标角度
      int targetAngle = currentAngle + relativeAngle;
      targetAngle = constrain(targetAngle, 0, 180);
      
      // 移动舵机
      Serial.print(currentAngle);
      Serial.print(F("° "));
      Serial.print(relativeAngle >= 0 ? "+" : "");
      Serial.print(relativeAngle);
      Serial.print(F("° → "));
      Serial.println(targetAngle);
      
      myServo.write(targetAngle);
      currentAngle = targetAngle;
    }
  }
}