/**
  ****************************(C) COPYRIGHT 2023 YOUR_NAME****************************
  * @file       encoder_position.c/h
  * @brief      编码器位置反馈模块，基于四个轮子编码器计算当前位置
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-07-21      YOUR_NAME       1. 创建
  *
  @verbatim
  ==============================================================================
  编码器位置计算原理：
  1. 根据编码器增量计算轮子转动距离
  2. 使用麦轮逆运动学计算机器人运动
  3. 积分得到机器人在坐标系中的位置
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 YOUR_NAME****************************
  */

#include "encoder_position.h"
#include "chassis_config.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
  * @brief          编码器位置模块初始化
  * @param[out]     encoder_pos: 编码器位置结构体指针
  * @retval         none
  */
void encoder_position_init(encoder_position_t *encoder_pos)
{
    if (encoder_pos == NULL)
    {
        return;
    }
    
    // 初始化位置
    encoder_pos->x = 0.0f;
    encoder_pos->y = 0.0f;
    encoder_pos->angle = 0.0f;
    
    // 初始化编码器记录
    for (uint8_t i = 0; i < 4; i++)
    {
        encoder_pos->last_encoder[i] = 0;
    }
    
    // 设置机械参数（从配置文件读取）
    encoder_pos->wheel_radius = WHEEL_RADIUS_MM;              // 轮子半径47.5mm (直径95mm)
    encoder_pos->wheel_base_x = CHASSIS_WHEEL_BASE_X_MM;      // 前后轮距离（需要实际测量）
    encoder_pos->wheel_base_y = CHASSIS_WHEEL_BASE_Y_MM;      // 左右轮距离（需要实际测量）
    encoder_pos->encoder_resolution = MOTOR_ENCODER_RESOLUTION; // 电机编码器分辨率
    encoder_pos->reduction_ratio = MOTOR_REDUCTION_RATIO;     // 电机减速比
}

/**
  * @brief          更新位置信息
  * @param[in,out]  encoder_pos: 编码器位置结构体指针
  * @param[in]      encoder_data: 四个轮子的编码器数据
  * @retval         none
  */
void encoder_position_update(encoder_position_t *encoder_pos, const int32_t encoder_data[4])
{
    if (encoder_pos == NULL || encoder_data == NULL)
    {
        return;
    }
    
    // 计算编码器增量
    int32_t delta_encoder[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        delta_encoder[i] = encoder_data[i] - encoder_pos->last_encoder[i];
        encoder_pos->last_encoder[i] = encoder_data[i];
    }
    
    // 将编码器增量转换为轮子转动距离（mm）
    fp32 wheel_distance[4];
    fp32 distance_per_pulse = (2.0f * M_PI * encoder_pos->wheel_radius) / 
                             (encoder_pos->encoder_resolution * encoder_pos->reduction_ratio);
    
    for (uint8_t i = 0; i < 4; i++)
    {
        wheel_distance[i] = delta_encoder[i] * distance_per_pulse;
    }
    
    // 麦轮逆运动学：从轮子速度反推机器人运动
    // 参考麦轮运动学模型：
    // 前右轮 = vx + vy + vw  (M1)
    // 前左轮 = vx - vy - vw  (M2)  
    // 后左轮 = vx + vy - vw  (M3)
    // 后右轮 = vx - vy + vw  (M4)
    
    // 考虑实际的轮子安装方向（与mecanum_control.c保持一致）
    fp32 adjusted_distance[4];
    adjusted_distance[0] = +wheel_distance[0];  // 前右轮
    adjusted_distance[1] = -wheel_distance[1];  // 前左轮
    adjusted_distance[2] = -wheel_distance[2];  // 后左轮  
    adjusted_distance[3] = +wheel_distance[3];  // 后右轮
    
    // 逆运动学解算
    fp32 dx = (adjusted_distance[0] + adjusted_distance[1] + adjusted_distance[2] + adjusted_distance[3]) / 4.0f;
    fp32 dy = (adjusted_distance[0] - adjusted_distance[1] + adjusted_distance[2] - adjusted_distance[3]) / 4.0f;
    fp32 dw = (adjusted_distance[0] - adjusted_distance[1] - adjusted_distance[2] + adjusted_distance[3]) / 
              (4.0f * (encoder_pos->wheel_base_x + encoder_pos->wheel_base_y) / 2.0f);
    
    // 更新角度
    encoder_pos->angle += dw;
    
    // 角度标准化到 [-π, π]
    while (encoder_pos->angle > M_PI)
    {
        encoder_pos->angle -= 2.0f * M_PI;
    }
    while (encoder_pos->angle < -M_PI)
    {
        encoder_pos->angle += 2.0f * M_PI;
    }
    
    // 将机器人坐标系下的运动转换到全局坐标系
    fp32 cos_angle = cosf(encoder_pos->angle);
    fp32 sin_angle = sinf(encoder_pos->angle);
    
    fp32 global_dx = dx * cos_angle - dy * sin_angle;
    fp32 global_dy = dx * sin_angle + dy * cos_angle;
    
    // 更新位置
    encoder_pos->x += global_dx;
    encoder_pos->y += global_dy;
}

/**
  * @brief          重置位置为原点
  * @param[out]     encoder_pos: 编码器位置结构体指针
  * @retval         none
  */
void encoder_position_reset(encoder_position_t *encoder_pos)
{
    if (encoder_pos == NULL)
    {
        return;
    }
    
    encoder_pos->x = 0.0f;
    encoder_pos->y = 0.0f;
    encoder_pos->angle = 0.0f;
}

/**
  * @brief          设置当前位置
  * @param[out]     encoder_pos: 编码器位置结构体指针
  * @param[in]      x: X坐标
  * @param[in]      y: Y坐标  
  * @param[in]      angle: 角度
  * @retval         none
  */
void encoder_position_set(encoder_position_t *encoder_pos, fp32 x, fp32 y, fp32 angle)
{
    if (encoder_pos == NULL)
    {
        return;
    }
    
    encoder_pos->x = x;
    encoder_pos->y = y;
    encoder_pos->angle = angle;
}

/**
  * @brief          获取当前位置
  * @param[in]      encoder_pos: 编码器位置结构体指针
  * @param[out]     x: X坐标指针
  * @param[out]     y: Y坐标指针
  * @param[out]     angle: 角度指针
  * @retval         none
  */
void encoder_position_get(encoder_position_t *encoder_pos, fp32 *x, fp32 *y, fp32 *angle)
{
    if (encoder_pos == NULL)
    {
        return;
    }
    
    if (x != NULL) *x = encoder_pos->x;
    if (y != NULL) *y = encoder_pos->y;
    if (angle != NULL) *angle = encoder_pos->angle;
}

/**
  * @brief          计算距离目标点的距离
  * @param[in]      encoder_pos: 编码器位置结构体指针
  * @param[in]      target_x: 目标X坐标
  * @param[in]      target_y: 目标Y坐标
  * @retval         距离值(mm)
  */
fp32 encoder_position_distance_to_target(encoder_position_t *encoder_pos, fp32 target_x, fp32 target_y)
{
    if (encoder_pos == NULL)
    {
        return 0.0f;
    }
    
    fp32 dx = target_x - encoder_pos->x;
    fp32 dy = target_y - encoder_pos->y;
    
    return sqrtf(dx * dx + dy * dy);
}
