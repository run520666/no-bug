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

#include "mecanum_control.h"
#include "chassis_config.h"
#include <math.h>
#include <stdlib.h>  // 为NULL定义添加
#include <stdio.h>   // 为printf函数添加

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
  * @brief          麦轮底盘初始化
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @retval         none
  */
void mecanum_init(mecanum_control_t *mecanum_control)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    
    // 初始化输入参数
    mecanum_control->vx = 0.0f;
    mecanum_control->vy = 0.0f;
    mecanum_control->vw = 0.0f;
    
    // 初始化输出参数
    for (uint8_t i = 0; i < 4; i++)
    {
        mecanum_control->wheel_speed[i] = 0.0f;
    }
    
    // 设置默认最大速度限制 (可根据实际情况调整)
    mecanum_control->max_wheel_speed = 8000.0f;  // rpm
    
    // 初始化编码器指针
    mecanum_control->encoder = NULL;
    
    // 初始化导航控制
    mecanum_nav_init(mecanum_control);
}

/**
  * @brief          麦轮底盘速度解算
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @retval         none
  */
void mecanum_calculate_wheel_speed(mecanum_control_t *mecanum_control)
{
    if (mecanum_control == NULL)
    {
        return;
    }

    // 获取输入速度
    fp32 vx = mecanum_control->vx;
    fp32 vy = mecanum_control->vy;
    fp32 vw = mecanum_control->vw;

    // 运动学解算 - 根据实际测试调整
    mecanum_control->wheel_speed[0] = vx - vy + vw; // 前右轮 (东北↗)
    mecanum_control->wheel_speed[1] = vx + vy - vw; // 前左轮 (西北↖)
    mecanum_control->wheel_speed[2] = vx - vy - vw; // 后左轮 (西南↙) - 修正vy符号
    mecanum_control->wheel_speed[3] = vx + vy - vw; // 后右轮 (东南↘) - 修正vy符号
    
    // 速度限制
    mecanum_limit_wheel_speed(mecanum_control);
}

/**
  * @brief          限制轮速在最大值范围内
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @retval         none
  */
void mecanum_limit_wheel_speed(mecanum_control_t *mecanum_control)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    
    fp32 max_speed = 0.0f;
    
    // 找出四个轮子中速度最大的绝对值
    for (uint8_t i = 0; i < 4; i++)
    {
        fp32 abs_speed = fabsf(mecanum_control->wheel_speed[i]);
        if (abs_speed > max_speed)
        {
            max_speed = abs_speed;
        }
    }
    
    // 如果最大速度超过限制，则等比例缩小所有轮子的速度
    if (max_speed > mecanum_control->max_wheel_speed)
    {
        fp32 scale = mecanum_control->max_wheel_speed / max_speed;
        for (uint8_t i = 0; i < 4; i++)
        {
            mecanum_control->wheel_speed[i] *= scale;
        }
    }
}

/**
  * @brief          停止机器人
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @retval         none
  */
void mecanum_stop(mecanum_control_t *mecanum_control)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    
    mecanum_control->vx = 0.0f;
    mecanum_control->vy = 0.0f;
    mecanum_control->vw = 0.0f;
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          前进
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      speed: 速度值(rpm)
  * @retval         none
  */
void mecanum_move_forward(mecanum_control_t *mecanum_control, fp32 speed)
{
    if (mecanum_control == NULL)
    {
        return;
    }

    mecanum_control->vx = -speed; // 反转方向
    mecanum_control->vy = 0.0f;
    mecanum_control->vw = 0.0f;
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          后退
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      speed: 速度值(rpm)
  * @retval         none
  */
void mecanum_move_backward(mecanum_control_t *mecanum_control, fp32 speed)
{
    if (mecanum_control == NULL)
    {
        return;
    }

    mecanum_control->vx = speed;
    mecanum_control->vy = 0.0f;
    mecanum_control->vw = 0.0f;
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          左移
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      speed: 速度值(rpm)
  * @retval         none
  */
void mecanum_move_left(mecanum_control_t *mecanum_control, fp32 speed)
{
    if (mecanum_control == NULL)
    {
        return;
    }

    mecanum_control->vx = 0.0f;
    mecanum_control->vy = speed;
    mecanum_control->vw = 0.0f;
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          右移
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      speed: 速度值(rpm)
  * @retval         none
  */
void mecanum_move_right(mecanum_control_t *mecanum_control, fp32 speed)
{
    if (mecanum_control == NULL)
    {
        return;
    }

    mecanum_control->vx = 0.0f;
    mecanum_control->vy = -speed; // 反转方向
    mecanum_control->vw = 0.0f;
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          左转
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      speed: 速度值(rpm)
  * @retval         none
  */
void mecanum_rotate_left(mecanum_control_t *mecanum_control, fp32 speed)
{
    if (mecanum_control == NULL)
    {
        return;
    }

    mecanum_control->vx = 0.0f;
    mecanum_control->vy = 0.0f;
    mecanum_control->vw = -speed; // 反转方向
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          右转
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      speed: 速度值(rpm)
  * @retval         none
  */
void mecanum_rotate_right(mecanum_control_t *mecanum_control, fp32 speed)
{
    if (mecanum_control == NULL)
    {
        return;
    }

    mecanum_control->vx = 0.0f;
    mecanum_control->vy = 0.0f;
    mecanum_control->vw = speed; // 反转方向
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          设置编码器位置反馈
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      encoder: 编码器位置结构体指针
  * @retval         none
  */
void mecanum_set_encoder(mecanum_control_t *mecanum_control, encoder_position_t *encoder)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    
    mecanum_control->encoder = encoder;
}

/**
  * @brief  初始化导航控制参数
  * @param  mecanum_control: 麦轮控制结构体指针
  * @retval None
  */
void mecanum_nav_init(mecanum_control_t *mecanum_control)
{
    // 初始化导航参数
    mecanum_control->nav.target_x = 0.0f;
    mecanum_control->nav.target_y = 0.0f;
    mecanum_control->nav.max_speed = 500.0f;         // 默认最大速度500mm/s
    mecanum_control->nav.arrival_threshold = 20.0f;   // 默认到达阈值20mm
    mecanum_control->nav.kp_distance = 1.0f;         // 距离比例系数
    mecanum_control->nav.kp_angle = 5.0f;            // 角度比例系数
    mecanum_control->nav.state = NAV_IDLE;
    mecanum_control->nav.enable = 0;
}

/**
  * @brief  设置导航目标位置
  * @param  mecanum_control: 麦轮控制结构体指针
  * @param  target_x: 目标X坐标(mm)
  * @param  target_y: 目标Y坐标(mm)
  * @retval None
  */
void mecanum_nav_set_target(mecanum_control_t *mecanum_control, fp32 target_x, fp32 target_y)
{
    mecanum_control->nav.target_x = target_x;
    mecanum_control->nav.target_y = target_y;
    
    // 设置目标后自动使能导航
    mecanum_control->nav.enable = 1;
    mecanum_control->nav.state = NAV_MOVING;
}

/**
  * @brief  使能/禁用导航功能
  * @param  mecanum_control: 麦轮控制结构体指针
  * @param  enable: 1-使能, 0-禁用
  * @retval None
  */
void mecanum_nav_enable(mecanum_control_t *mecanum_control, uint8_t enable)
{
    mecanum_control->nav.enable = enable;
    
    if (!enable)
    {
        mecanum_control->nav.state = NAV_IDLE;
        mecanum_stop(mecanum_control);  // 禁用时停止运动
    }
}

/**
  * @brief  更新导航控制状态（需要在主循环中定期调用）
  * @param  mecanum_control: 麦轮控制结构体指针
  * @retval None
  */
void mecanum_nav_update(mecanum_control_t *mecanum_control)
{
    fp32 dx, dy, distance, target_angle, angle_diff;
    fp32 current_x, current_y, current_angle;
    
    // 检查使能状态和编码器有效性
    if (!mecanum_control->nav.enable || mecanum_control->encoder == NULL)
    {
        return;
    }
    
    // 获取当前位置信息
    current_x = mecanum_control->encoder->x;
    current_y = mecanum_control->encoder->y;
    current_angle = mecanum_control->encoder->angle;
    
    // 计算与目标点的距离和角度
    dx = mecanum_control->nav.target_x - current_x;
    dy = mecanum_control->nav.target_y - current_y;
    distance = sqrtf(dx * dx + dy * dy);
    
    // 检查是否到达目标点
    if (distance <= mecanum_control->nav.arrival_threshold)
    {
        mecanum_control->nav.state = NAV_ARRIVED;
        mecanum_stop(mecanum_control);
        return;
    }
    
    // 计算目标角度
    target_angle = atan2f(dy, dx) * 180.0f / PI;  // 转换为角度制
    
    // 计算角度差值并规范化到[-180, 180]范围
    angle_diff = target_angle - current_angle;
    while (angle_diff > 180.0f) angle_diff -= 360.0f;
    while (angle_diff < -180.0f) angle_diff += 360.0f;
    
    // 根据距离和角度差计算速度
    if (fabsf(angle_diff) > 90.0f)  // 如果角度差过大，先停止运动再旋转
    {
        mecanum_stop(mecanum_control);
        mecanum_control->vw = angle_diff * mecanum_control->nav.kp_angle;
    }
    else  // 正常行进
    {
        // 前进速度与距离成正比
        fp32 forward_speed = distance * mecanum_control->nav.kp_distance;
        
        // 限制最大速度
        if (forward_speed > mecanum_control->nav.max_speed)
        {
            forward_speed = mecanum_control->nav.max_speed;
        }
        
        // 分解为X和Y方向速度
        mecanum_control->vx = forward_speed * cosf(angle_diff * PI / 180.0f);
        mecanum_control->vy = forward_speed * sinf(angle_diff * PI / 180.0f);
        
        // 旋转速度与角度差成正比
        mecanum_control->vw = angle_diff * mecanum_control->nav.kp_angle;
    }
    
    // 更新轮速
    mecanum_calculate_wheel_speed(mecanum_control);
    mecanum_limit_wheel_speed(mecanum_control);
}

/**
  * @brief  获取导航状态
  * @param  mecanum_control: 麦轮控制结构体指针
  * @retval 当前导航状态
  */
nav_state_t mecanum_nav_get_state(mecanum_control_t *mecanum_control)
{
    return mecanum_control->nav.state;
}

/**
  * @brief  获取到目标点的距离
  * @param  mecanum_control: 麦轮控制结构体指针
  * @retval 到目标点的距离(mm)
  */
fp32 mecanum_nav_get_distance_to_target(mecanum_control_t *mecanum_control)
{
    fp32 dx, dy;
    
    if (mecanum_control->encoder == NULL)
    {
        return 0.0f;
    }
    
    dx = mecanum_control->nav.target_x - mecanum_control->encoder->x;
    dy = mecanum_control->nav.target_y - mecanum_control->encoder->y;
    
    return sqrtf(dx * dx + dy * dy);
}

/**
  * @brief  简化的坐标导航函数 - 移动到指定位置
  * @param  mecanum_control: 麦轮控制结构体指针
  * @param  x: 目标X坐标(mm)
  * @param  y: 目标Y坐标(mm)
  * @retval None
  */
void mecanum_goto_position(mecanum_control_t *mecanum_control, fp32 x, fp32 y)
{
    mecanum_nav_set_target(mecanum_control, x, y);
}

/**
  * @brief  检查是否到达目标位置
  * @param  mecanum_control: 麦轮控制结构体指针
  * @retval 1-已到达, 0-未到达
  */
uint8_t mecanum_is_arrived(mecanum_control_t *mecanum_control)
{
    return (mecanum_control->nav.state == NAV_ARRIVED) ? 1 : 0;
}

/**
  * @brief  停止导航
  * @param  mecanum_control: 麦轮控制结构体指针
  * @retval None
  */
void mecanum_stop_navigation(mecanum_control_t *mecanum_control)
{
    mecanum_nav_enable(mecanum_control, 0);
}
