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
#include "CAN_receive.h"
#include <math.h>
#include <stdlib.h>  // 为NULL定义添加
mecanum_control_t mecanum = {0};

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
    mecanum_control->max_wheel_speed = 2000.0f;  // rpm
    
    // 初始化导航参数
    mecanum_control->current_pos.distance = 0.0f;
    mecanum_control->current_pos.angle = 0.0f;

    mecanum_control->target_pos.distance = 0.0f;
    mecanum_control->target_pos.angle = 0.0f;
}

/**
  * @brief          麦轮底盘速度解算
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @retval         none
  */
void mecanum_calculate_wheel_speed(mecanum_control_t *mecanum_control)
//速度为转速，还要经过36减速比
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
    mecanum_control->wheel_speed[0] = - vx - vy - vw; // 前右轮
    mecanum_control->wheel_speed[1] =   vx - vy - vw; // 前左轮
    mecanum_control->wheel_speed[2] =   vx + vy - vw; // 后左轮
    mecanum_control->wheel_speed[3] = - vx + vy - vw; // 后右轮
    
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
//速度为转速，还要经过36减速比
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


