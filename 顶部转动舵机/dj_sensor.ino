#include <Servo.h>                 // 舵机控制库
Servo mg90s_sensor;    // MG90S舵机，用于带动颜色传感器转动，连接到6号引脚
#define MG90S_PIN_6 6  // MG90S舵机引脚定义

void initSensorServo();

void setup() {
  Serial.begin(9600);
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

  void rotateServo360() {
    for (int i = 0; i < 10; i++) {
      int angle = -180 + i * 36; // 从-180°开始，每步加36°
      mg90s_sensor.write(angle);
      Serial.print("Current Angle: ");
      Serial.println(angle);
      delay(500); // 停500ms
    }
  }

  rotateServo360();
}

void loop() {
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

}
