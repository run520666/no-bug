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
#include "pid.h"
#include "encoder_position.h"

// 导航状态枚举
typedef enum
{
    NAV_IDLE,        // 空闲状态
    NAV_MOVING,      // 移动到目标状态
    NAV_ARRIVED,     // 已到达目标
    NAV_ERROR        // 错误状态
} nav_state_t;

// 导航控制结构体
typedef struct
{
    // 目标位置
    fp32 target_x;     // 目标X坐标 (mm)
    fp32 target_y;     // 目标Y坐标 (mm)
    
    // 控制参数
    fp32 max_speed;         // 最大速度 (mm/s)
    fp32 arrival_threshold; // 到达阈值 (mm)
    fp32 kp_distance;       // 距离比例系数
    fp32 kp_angle;          // 角度比例系数
    
    // 状态
    nav_state_t state;      // 导航状态
    uint8_t enable;         // 导航使能
    
} navigation_control_t;

// 麦轮底盘控制结构体
typedef struct
{
    // 速度控制参数（内环）
    fp32 vx;      // 前后方向速度，前为正
    fp32 vy;      // 左右方向速度，左为正
    fp32 vw;      // 旋转速度，逆时针为正
    
    // 输出参数 - 四个轮子的目标速度
    fp32 wheel_speed[4];  // 依次为：前右(M1)、前左(M2)、后左(M3)、后右(M4)
    
    // 速度限制
    fp32 max_wheel_speed; // 轮子最大速度限制
    
    // 导航控制
    navigation_control_t nav;     // 导航控制结构体
    encoder_position_t *encoder;  // 编码器位置反馈指针
    
} mecanum_control_t;

// 麦轮底盘初始化
extern void mecanum_init(mecanum_control_t *mecanum_control);

// 设置编码器位置反馈
extern void mecanum_set_encoder(mecanum_control_t *mecanum_control, encoder_position_t *encoder);

// 麦轮底盘速度解算，根据vx, vy, vw计算四个轮子的速度
extern void mecanum_calculate_wheel_speed(mecanum_control_t *mecanum_control);

// 限制轮速在最大值范围内
extern void mecanum_limit_wheel_speed(mecanum_control_t *mecanum_control);

// 基本运动控制函数
extern void mecanum_stop(mecanum_control_t *mecanum_control);
extern void mecanum_move_forward(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_move_backward(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_move_left(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_move_right(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_rotate_left(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_rotate_right(mecanum_control_t *mecanum_control, fp32 speed);

// 坐标导航控制函数
extern void mecanum_nav_init(mecanum_control_t *mecanum_control);
extern void mecanum_nav_set_target(mecanum_control_t *mecanum_control, fp32 target_x, fp32 target_y);
extern void mecanum_nav_enable(mecanum_control_t *mecanum_control, uint8_t enable);
extern void mecanum_nav_update(mecanum_control_t *mecanum_control);
extern nav_state_t mecanum_nav_get_state(mecanum_control_t *mecanum_control);
extern fp32 mecanum_nav_get_distance_to_target(mecanum_control_t *mecanum_control);

// 简化的坐标导航函数（推荐在main中使用）
extern void mecanum_goto_position(mecanum_control_t *mecanum_control, fp32 x, fp32 y);
extern uint8_t mecanum_is_arrived(mecanum_control_t *mecanum_control);
extern void mecanum_stop_navigation(mecanum_control_t *mecanum_control);

#endif // MECANUM_CONTROL_H
