#include "stm32f4xx_hal.h"
#include "main.h"
#include "i2c.h"
#include <string.h>

/**
 * @file i2c_stm32_arduino.c
 * @brief STM32作为I2C主机与Arduino UNO（从机）通信示例
 *
 * Arduino UNO默认I2C地址为0x08（可根据实际修改）
 *
 * 连接方式：
 *   STM32 SCL <-> Arduino A5
 *   STM32 SDA <-> Arduino A4
 *   GND互连
 */

#define ARDUINO_I2C_ADDR 0x08

extern I2C_HandleTypeDef hi2c1; // 假设使用I2C1

// 发送数据到Arduino
HAL_StatusTypeDef STM32_I2C_Send(uint8_t *data, uint16_t size)
{
    return HAL_I2C_Master_Transmit(&hi2c1, ARDUINO_I2C_ADDR << 1, data, size, 100);
}

// 从Arduino接收数据
HAL_StatusTypeDef STM32_I2C_Receive(uint8_t *data, uint16_t size)
{
    return HAL_I2C_Master_Receive(&hi2c1, ARDUINO_I2C_ADDR << 1, data, size, 100);
}

// 示例：发送字符串并接收应答
void I2C_Arduino_Example(void)
{
    uint8_t tx_buf[] = "Hello Arduino!";
    uint8_t rx_buf[32] = {0};

    if (STM32_I2C_Send(tx_buf, strlen((char*)tx_buf)) == HAL_OK)
    {
        HAL_Delay(10); // 等待Arduino处理
        if (STM32_I2C_Receive(rx_buf, 8) == HAL_OK)
        {
            // 处理接收到的数据
        }
    }
}

// 在主循环或定时器中调用 I2C_Arduino_Example()
