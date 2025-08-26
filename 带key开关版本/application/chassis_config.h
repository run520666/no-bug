/**
  ****************************(C) COPYRIGHT 2023 YOUR_NAME****************************
  * @file       chassis_config.h
  * @brief      底盘机械参数配置文件
  * @note       已根据实际底盘测量数据设置参数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-07-21      YOUR_NAME       1. 创建
  *  V1.1.0     2023-07-21      YOUR_NAME       2. 更新实际测量参数
  *
  @verbatim
  ==============================================================================
  底盘参数已确认：
  - 轮子直径：95mm
  - 电机型号：M2006 (减速比1:36)
  - 前后轮距：205mm
  - 左右轮距：225mm
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 YOUR_NAME****************************
  */

#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

// ========================================
// 轮子参数（已确认）
// ========================================
#define WHEEL_DIAMETER_MM       95.0f           // 轮子直径 95mm（已确认）
#define WHEEL_RADIUS_MM         (WHEEL_DIAMETER_MM / 2.0f)  // 轮子半径 47.5mm

// ========================================
// 电机参数（已确认：M2006）
// ========================================
// M2006电机参数：
#define MOTOR_ENCODER_RESOLUTION    8192.0f     // M2006编码器分辨率 8192  
#define MOTOR_REDUCTION_RATIO       36.0f       // M2006减速比 1:36

// 如果使用M3508电机：
// #define MOTOR_ENCODER_RESOLUTION    8192.0f     // M3508编码器分辨率 8192
// #define MOTOR_REDUCTION_RATIO       19.0f       // M3508减速比 1:19

// ========================================
// 底盘几何参数（已实际测量）
// ========================================
#define CHASSIS_WHEEL_BASE_X_MM     205.0f      // 前后轮距离205mm（已测量）
#define CHASSIS_WHEEL_BASE_Y_MM     225.0f      // 左右轮距离225mm（已测量）

// ========================================
// 导航控制参数（可调）
// ========================================
#define NAV_MAX_SPEED_RPM           1000.0f     // 导航最大速度 (rpm)
#define NAV_ARRIVAL_THRESHOLD_MM    50.0f       // 到达阈值 (mm)
#define NAV_KP_DISTANCE             5.0f        // 距离比例系数
#define NAV_KP_ANGLE                500.0f      // 角度比例系数

// ========================================
// 参数总结
// ========================================
/*
✅ 已确认的底盘参数：
- 轮子直径：95mm (半径47.5mm)
- 电机型号：M2006
- 编码器分辨率：8192
- 减速比：1:36
- 前后轮距：205mm
- 左右轮距：225mm

🎯 导航系统现在可以使用以下函数进行精确定位：
- goto_coordinate(x, y) : 导航到指定坐标(mm)
- mecanum_is_arrived()  : 检查是否到达目标
- 串口命令："goto x y" : 通过串口控制导航
*/

#endif // CHASSIS_CONFIG_H
