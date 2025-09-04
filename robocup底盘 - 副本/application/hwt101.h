#ifndef __HWT101_H
#define __HWT101_H

#include "main.h"
#include "mecanum_control.h"

void hwt101_ReceiveData(uint8_t RxData); //接收数据处理
extern float yaw; // 航向角->mecanum.current_pos.yaw
extern void set_yaw(float yaw);
extern mecanum_control_t mecanum;
#endif

