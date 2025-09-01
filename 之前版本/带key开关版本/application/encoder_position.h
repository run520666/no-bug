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
  编码器位置计算：
  根据四个轮子的编码器数据，计算机器人在坐标系中的位置(x,y)和角度
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 YOUR_NAME****************************
  */

#ifndef ENCODER_POSITION_H
#define ENCODER_POSITION_H

#include "struct_typedef.h"

// 编码器位置计算结构体
typedef struct
{
    // 当前位置
    fp32 x;           // X坐标 (mm)
    fp32 y;           // Y坐标 (mm)
    fp32 angle;       // 角度 (弧度)
    
    // 上次编码器值（用于增量计算）
    int32_t last_encoder[4];  // 四个轮子的上次编码器值
    
    // 机械参数
    fp32 wheel_radius;        // 轮子半径 (mm)
    fp32 wheel_base_x;        // 轮距X方向 (mm) - 前后轮距离
    fp32 wheel_base_y;        // 轮距Y方向 (mm) - 左右轮距离
    fp32 encoder_resolution;  // 编码器分辨率 (脉冲/圈)
    fp32 reduction_ratio;     // 减速比
    
} encoder_position_t;

// 编码器位置模块初始化
extern void encoder_position_init(encoder_position_t *encoder_pos);

// 更新位置信息（根据编码器数据）
extern void encoder_position_update(encoder_position_t *encoder_pos, const int32_t encoder_data[4]);

// 重置位置为原点
extern void encoder_position_reset(encoder_position_t *encoder_pos);

// 设置当前位置
extern void encoder_position_set(encoder_position_t *encoder_pos, fp32 x, fp32 y, fp32 angle);

// 获取当前位置
extern void encoder_position_get(encoder_position_t *encoder_pos, fp32 *x, fp32 *y, fp32 *angle);

// 计算距离目标点的距离
extern fp32 encoder_position_distance_to_target(encoder_position_t *encoder_pos, fp32 target_x, fp32 target_y);

#endif // ENCODER_POSITION_H
