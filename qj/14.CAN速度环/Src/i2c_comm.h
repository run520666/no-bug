/*
 * i2c_comm.h
 *
 * I2C通信模块头文件，声明相关接口
 */
#ifndef __I2C_COMM_H
#define __I2C_COMM_H

#include "stm32f4xx_hal.h"

void I2C_Comm_Init(I2C_HandleTypeDef *hi2c);
void I2C_Prepare_Tx_Data(const uint8_t *data, uint16_t len);

#endif // __I2C_COMM_H
