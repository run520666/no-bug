/*
 * i2c_comm.c
 *
 * 用于STM32底盘与Arduino UNO上位机之间通过I2C通信的示例代码。
 * STM32作为I2C从机，Arduino作为主机。
 */
#include "main.h"
#include "i2c_comm.h"
#include <string.h>

#define I2C_SLAVE_ADDR 0x08 // STM32作为I2C从机地址

uint8_t i2c_rx_buffer[32];
uint8_t i2c_tx_buffer[32];

// 接收回调（由HAL库I2C中断调用）
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 这里可以处理收到的数据，例如解析指令
    // i2c_rx_buffer中为主机发来的数据
    // ...
    // 重新启动接收
    HAL_I2C_Slave_Receive_IT(hi2c, i2c_rx_buffer, sizeof(i2c_rx_buffer));
}

// 发送回调（由HAL库I2C中断调用）
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 发送完成后可做处理
}

// 初始化I2C通信（在主程序初始化时调用）
void I2C_Comm_Init(I2C_HandleTypeDef *hi2c)
{
    // 启动I2C从机接收
    HAL_I2C_Slave_Receive_IT(hi2c, i2c_rx_buffer, sizeof(i2c_rx_buffer));
}

// 示例：准备要发送给主机的数据
void I2C_Prepare_Tx_Data(const uint8_t *data, uint16_t len)
{
    if(len > sizeof(i2c_tx_buffer)) len = sizeof(i2c_tx_buffer);
    memcpy(i2c_tx_buffer, data, len);
}

// 示例：主循环中可根据需要调用，准备好数据后等待主机读取
// Arduino主机端可用Wire.requestFrom()读取

/*
 * Arduino UNO主机端示例代码：
 *
 * #include <Wire.h>
 * void setup() {
 *   Wire.begin(); // 作为主机
 *   Serial.begin(9600);
 * }
 * void loop() {
 *   Wire.requestFrom(0x08, 8); // 读取8字节
 *   while(Wire.available()) {
 *     char c = Wire.read();
 *     Serial.print(c, HEX); Serial.print(" ");
 *   }
 *   delay(500);
 *   // 发送数据给STM32
 *   Wire.beginTransmission(0x08);
 *   Wire.write("CMD123");
 *   Wire.endTransmission();
 *   delay(500);
 * }
 */
