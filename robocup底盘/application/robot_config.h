#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H



//M2006 参数
#define M2006_ENCODER_RES       8192.0f         // C620编码器分辨率 (8192 线)
#define M2006_GEAR_RATIO        36.0f        // C620 减速比

//麦轮 参数
#define WHEEL_DIAMETER          0.095f          // 轮子直径 95mm (转换为米)
#define WHEEL_CIRCUMFERENCE     (3.14159f * WHEEL_DIAMETER)  // 轮子周长 ≈ 0.298m

//编码器处理
#define MAX_ECD_DELTA           4096            // 最大合理编码器变化量

#endif /* ROBOT_CONFIG_H */
