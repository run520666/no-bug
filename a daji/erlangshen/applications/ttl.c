#include "ttl.h"
#include "main.h"
#include "mecanum_control.h"
#include "pid.h"


extern float vofa_data[3];
extern UART_HandleTypeDef huart6; 
extern mecanum_control_t mecanum;
extern q_pid angle_pid_s; //直线pid
extern q_pid angle_pid_t; //转向pid

void ttl_test(UART_HandleTypeDef *huart)
{
	  vofa_data[0] = (float)mecanum.current_pos.yaw;          // 当前yaw角度
    vofa_data[1] = (float)angle_pid_s.target;               // 直线角度PID目标角度
    vofa_data[2] = (float)angle_pid_t.target;               // 转向角度PID目标角度
    HAL_UART_Transmit(huart, (uint8_t*)vofa_data, sizeof(vofa_data), 100);
    // 发送帧尾
    unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(huart, tail, 4, 100);
}
