#ifndef __HWT101_H
#define __HWT101_H

#include "main.h"

void hwt101_ReceiveData(uint8_t RxData); //接收数据处理
extern float yaw; // 航向角->mecanum.current_pos.yaw

#endif

