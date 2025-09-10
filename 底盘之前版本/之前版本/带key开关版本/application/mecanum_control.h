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

#define  WHEEL_C 0.2985f      //95mm直径，0.2985m周长
#define  ENCODER_RES 8192.0f // 编码器分辨率（每圈8192脉冲）

// 位置结构体
typedef struct
{
    fp32 distance; // 累计距离（米）
    fp32 angle;   // 角度（弧度）
} position_t;

// 导航状态枚举
typedef enum
{
    NAV_IDLE,      // 空闲状态
    NAV_ROTATING,  // 旋转状态
    NAV_MOVING,    // 直线移动状态
    NAV_ARRIVED    // 已到达目标
} nav_state_t;

// 麦轮底盘控制结构体
typedef struct
{
    // 输入参数
    fp32 vx;      // 前后方向速度，前为正
    fp32 vy;      // 左右方向速度，左为正
    fp32 vw;      // 旋转速度，逆时针为正
    
    // 输出参数 - 四个轮子的目标速度
    fp32 wheel_speed[4];  // 依次为：前右(M1)、前左(M2)、后左(M3)、后右(M4)
    uint16_t last_ecd[4];  // 上次编码器值

    // 速度限制
    fp32 max_wheel_speed; // 轮子最大速度限制

    // 导航相关参数
    position_t current_pos;  // 当前位置
    position_t target_pos;   // 目标位置
    nav_state_t nav_state;   // 导航状态
    
    // 导航参数
    fp32 position_tolerance;  // 位置容差
    fp32 angle_tolerance;     // 角度容差
    fp32 default_speed;       // 默认速度
    
} mecanum_control_t;

// 麦轮底盘初始化
extern void mecanum_init(mecanum_control_t *mecanum_control);

// 麦轮底盘速度解算，根据vx, vy, vw计算四个轮子的速度
extern void mecanum_calculate_wheel_speed(mecanum_control_t *mecanum_control);

// 限制轮速在最大值范围内
extern void mecanum_limit_wheel_speed(mecanum_control_t *mecanum_control);

//编码器计数
extern void mecanum_xy_distance(mecanum_control_t *mecanum_control);

// 基本运动控制函数
extern void mecanum_stop(mecanum_control_t *mecanum_control);
extern void mecanum_move_forward(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_move_backward(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_move_left(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_move_right(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_rotate_left(mecanum_control_t *mecanum_control, fp32 speed);
extern void mecanum_rotate_right(mecanum_control_t *mecanum_control, fp32 speed);

// 导航相关函数
extern void mecanum_set_target(mecanum_control_t *mecanum_control, fp32 x, fp32 y);
extern void mecanum_update_position(mecanum_control_t *mecanum_control, fp32 x, fp32 y, fp32 angle);
extern void mecanum_navigate_step(mecanum_control_t *mecanum_control);

#endif // MECANUM_CONTROL_H
