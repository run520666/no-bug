#ifndef STP_23L_H
#define STP_23L_H

#include "stm32f4xx_hal.h"

#define HEADER 0x57
#define DEVICE_ADDRESS 0x10
#define PACK_GET_DISTANCE 0x02
#define CHUNK_OFFSET 0x00

typedef struct
{
    uint16_t distance;
    uint16_t noise;
    uint32_t peak;
    uint8_t confidence;
    uint32_t intg;
    int16_t reftof;
} LidarPointTypedef;

// 统一的初始化和回调函数
void STP23L_Init(UART_HandleTypeDef *huart, uint8_t lr);
void STP23L_RxCallback(uint8_t temp_data, uint8_t lr);

// 全局距离变量
extern uint16_t stp23l_distance_left;
extern uint16_t stp23l_distance_right;

#endif /* STP_23L_H */
