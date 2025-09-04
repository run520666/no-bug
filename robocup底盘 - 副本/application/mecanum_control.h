/**
  ****************************(C) COPYRIGHT 2023 YOUR_NAME****************************
  * @file       mecanum_control.c/h
  * @brief      麦克纳姆轮底盘控制模型实现，实现四轮麦轮运动学解算
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-07-21      YOUR_NAME       1. 创建
  *
  @verbatim
  ==============================================================================
  麦轮运动学模型：
  前右轮 = vx + vy + vw  (M1)
  前左轮 = vx - vy - vw  (M2)
  后左轮 = vx + vy - vw  (M3)
  后右轮 = vx - vy + vw  (M4)
  
  其中vx是前后速度，vy是左右速度，vw是旋转速度
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 YOUR_NAME****************************
  */

#ifndef MECANUM_CONTROL_H
#define MECANUM_CONTROL_H

#include "struct_typedef.h"

// 位置结构体
typedef struct
{
    fp32 distance; // 距离
    fp32 yaw;   // 角度
} position_t; //麦轮结构体中名字为 current_pos和 target_pos


// 麦轮底盘控制结构体
typedef struct
{
    // 输入参数
    fp32 vx;      // 前后方向速度，前为正
    fp32 vy;      // 左右方向速度，左为正
    fp32 vw;      // 旋转速度，逆时针为正
    
    // 输出参数 - 四个轮子的目标速度
    fp32 wheel_speed[4];  // 依次为：前右(M1)、前左(M2)、后左(M3)、后右(M4)

    // 编码器值
    uint16_t last_ecd[4];
    fp32 total_wheel_distance[4]; //累计轮子转动的距离

    // 速度限制
    fp32 max_wheel_speed; // 轮子最大速度限制

    position_t current_pos; //包含距离，角度
    position_t target_pos;
} mecanum_control_t;
extern mecanum_control_t mecanum;

// 麦轮底盘初始化
extern void mecanum_init(mecanum_control_t *mecanum_control);

// 麦轮底盘速度解算，根据vx, vy, vw计算四个轮子的速度
extern void mecanum_calculate_wheel_speed(mecanum_control_t *mecanum_control);

// 限制轮速在最大值范围内
extern void mecanum_limit_wheel_speed(mecanum_control_t *mecanum_control);

// 基本运动控制函数（单位转速，注意36减速比）
extern void mecanum_stop(mecanum_control_t *mecanum_control); //停止
extern void mecanum_move_forward(mecanum_control_t *mecanum_control, fp32 speed); //前进
extern void mecanum_move_backward(mecanum_control_t *mecanum_control, fp32 speed); //后退
extern void mecanum_move_left(mecanum_control_t *mecanum_control, fp32 speed); //左移
extern void mecanum_move_right(mecanum_control_t *mecanum_control, fp32 speed); //右移
extern void mecanum_rotate_left(mecanum_control_t *mecanum_control, fp32 speed); //左转
extern void mecanum_rotate_right(mecanum_control_t *mecanum_control, fp32 speed); //右转
#endif // MECANUM_CONTROL_H
