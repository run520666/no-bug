#ifndef __STP_23L_H__
#define __STP_23L_H__

#include "stm32f4xx_hal.h"
#include "string.h"

#define HEADER 0xAA
#define DEVICE_ADDRESS 0x00
#define CHUNK_OFFSET 0x00
#define PACK_GET_DISTANCE 0x02

typedef struct {
    int16_t distance;       /* 距离数据：单位 mm */
    uint16_t noise;
    uint32_t peak;
    uint8_t confidence;
    uint32_t intg;
    int16_t reftof;
} LidarPointTypedef;

extern uint16_t stp23l_distance;  /* 当前距离值 */

extern void STP23L_Init(UART_HandleTypeDef *huart);
extern void STP23L_RxCallback(uint8_t data);

#endif //__STP_23L_H__
