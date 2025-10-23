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
#include <math.h>
#include <stdlib.h>  // 为NULL定义添加

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
    
    // 初始化导航参数
    mecanum_control->current_pos.x = 0.0f;
    mecanum_control->current_pos.y = 0.0f;
    mecanum_control->current_pos.angle = 0.0f;
    
    mecanum_control->target_pos.x = 0.0f;
    mecanum_control->target_pos.y = 0.0f;
    mecanum_control->target_pos.angle = 0.0f;
    
    mecanum_control->nav_state = NAV_IDLE;
    
    // 设置导航参数
    mecanum_control->position_tolerance = 0.05f;  // 位置容差(米)
    mecanum_control->angle_tolerance = 0.05f;     // 角度容差(弧度)
    mecanum_control->default_speed = 1000.0f;     // 默认移动速度
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

    // 运动学解算并调整方向
    mecanum_control->wheel_speed[0] = +(vx + vy + vw); // 前右轮
    mecanum_control->wheel_speed[1] = -(vx - vy - vw); // 前左轮
    mecanum_control->wheel_speed[2] = -(vx + vy - vw); // 后左轮
    mecanum_control->wheel_speed[3] = +(vx - vy + vw); // 后右轮
    
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
    
    mecanum_control->vx = speed;
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
    
    mecanum_control->vx = -speed;
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
    mecanum_control->vy = -speed;
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
    mecanum_control->vw = speed;
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
    mecanum_control->vw = -speed;
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          设置导航目标点
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      x: 目标X坐标
  * @param[in]      y: 目标Y坐标
  * @retval         none
  */
void mecanum_set_target(mecanum_control_t *mecanum_control, fp32 x, fp32 y)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    
    mecanum_control->target_pos.x = x;
    mecanum_control->target_pos.y = y;
    mecanum_control->nav_state = NAV_ROTATING; // 开始导航，先旋转对准目标点
}

/**
  * @brief          更新机器人位置
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      x: 当前X坐标
  * @param[in]      y: 当前Y坐标
  * @param[in]      angle: 当前朝向角度
  * @retval         none
  * @note           这个函数将来应该由编码器和陀螺仪等传感器数据更新
  */
void mecanum_update_position(mecanum_control_t *mecanum_control, fp32 x, fp32 y, fp32 angle)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    
    mecanum_control->current_pos.x = x;
    mecanum_control->current_pos.y = y;
    mecanum_control->current_pos.angle = angle;
}

/**
  * @brief          获取当前位置到目标点的距离
  * @param[in]      mecanum_control: 麦轮底盘控制结构体指针
  * @retval         距离值
  */
fp32 mecanum_get_distance_to_target(mecanum_control_t *mecanum_control)
{
    if (mecanum_control == NULL)
    {
        return 0.0f;
    }
    
    fp32 dx = mecanum_control->target_pos.x - mecanum_control->current_pos.x;
    fp32 dy = mecanum_control->target_pos.y - mecanum_control->current_pos.y;
    
    return sqrtf(dx*dx + dy*dy);
}

/**
  * @brief          获取当前位置到目标点的角度
  * @param[in]      mecanum_control: 麦轮底盘控制结构体指针
  * @retval         角度值(弧度)
  */
fp32 mecanum_get_angle_to_target(mecanum_control_t *mecanum_control)
{
    if (mecanum_control == NULL)
    {
        return 0.0f;
    }
    
    fp32 dx = mecanum_control->target_pos.x - mecanum_control->current_pos.x;
    fp32 dy = mecanum_control->target_pos.y - mecanum_control->current_pos.y;
    
    return atan2f(dy, dx);
}

/**
  * @brief          导航步进函数，执行一步导航操作
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @retval         none
  * @note           这个函数需要周期性调用，以进行导航
  */
void mecanum_navigate_step(mecanum_control_t *mecanum_control)
{
    if (mecanum_control == NULL || mecanum_control->nav_state == NAV_IDLE)
    {
        return;
    }
    
    // 获取到目标点的距离和角度
    fp32 distance = mecanum_get_distance_to_target(mecanum_control);
    fp32 target_angle = mecanum_get_angle_to_target(mecanum_control);
    
    // 计算角度差
    fp32 angle_diff = target_angle - mecanum_control->current_pos.angle;
    
    // 规范化角度到[-PI, PI]
    while (angle_diff > 3.14159f)
        angle_diff -= 6.28318f;
    while (angle_diff < -3.14159f)
        angle_diff += 6.28318f;
    
    switch (mecanum_control->nav_state)
    {
        case NAV_ROTATING:
            // 旋转阶段 - 转向目标点方向
            if (fabsf(angle_diff) < mecanum_control->angle_tolerance)
            {
                // 已经对准目标方向，切换到移动阶段
                mecanum_stop(mecanum_control);
                mecanum_control->nav_state = NAV_MOVING;
            }
            else if (angle_diff > 0)
            {
                // 需要左转
                mecanum_rotate_left(mecanum_control, mecanum_control->default_speed / 2);
            }
            else
            {
                // 需要右转
                mecanum_rotate_right(mecanum_control, mecanum_control->default_speed / 2);
            }
            break;
            
        case NAV_MOVING:
            // 移动阶段 - 直线移动到目标点
            if (distance < mecanum_control->position_tolerance)
            {
                // 已经到达目标点
                mecanum_stop(mecanum_control);
                mecanum_control->nav_state = NAV_ARRIVED;
            }
            else
            {
                // 继续前进
                mecanum_move_forward(mecanum_control, mecanum_control->default_speed);
                
                // 如果方向偏差较大，重新调整方向
                if (fabsf(angle_diff) > mecanum_control->angle_tolerance * 2)
                {
                    mecanum_stop(mecanum_control);
                    mecanum_control->nav_state = NAV_ROTATING;
                }
            }
            break;
            
        case NAV_ARRIVED:
            // 已到达目标点
            mecanum_stop(mecanum_control);
            break;
            
        default:
            break;
    }
}
