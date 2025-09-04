// 在mecanum_control.h中扩展结构体
typedef struct
{
    // 位置信息
    position_t current_pos;
    position_t target_pos;
    
    // 🎯 关键：分层速度控制
    struct {
        fp32 nav_vx, nav_vy;     // 导航层期望速度（位置环输出）
        fp32 stab_wz;            // 稳定层期望角速度（角度环输出）
        fp32 final_vx, final_vy, final_wz; // 最终合成速度
    } velocity;
    
    fp32 wheel_speed[4];
    fp32 max_speed;
    uint8_t is_moving;
} mecanum_control_t;
// 位置控制器
typedef struct
{
    _pid pid_x;          // X方向位置PID
    _pid pid_y;          // Y方向位置PID  
    _pid pid_yaw;        // 角度稳定PID
    
    fp32 max_linear_vel; // 最大线速度限制
    fp32 max_angular_vel;// 最大角速度限制
} position_controller_t;

position_controller_t pos_ctrl;
// 🎯 核心函数：解决速度冲突的关键
void mecanum_position_control_task(void)
{
    // 步骤1：位置环计算导航速度
    pos_ctrl.pid_x.target_val = mecanum.target_pos.x;
    pos_ctrl.pid_y.target_val = mecanum.target_pos.y;
    
    fp32 nav_vx = location_pid_realize(&pos_ctrl.pid_x, mecanum.current_pos.x);
    fp32 nav_vy = location_pid_realize(&pos_ctrl.pid_y, mecanum.current_pos.y);
    
    // 步骤2：角度环计算稳定角速度
    pos_ctrl.pid_yaw.target_val = mecanum.target_pos.yaw;
    fp32 stab_wz = turn_angle_pid_realize(&pos_ctrl.pid_yaw, mecanum.current_pos.yaw);
    
    // 步骤3：🎯 速度合成（解决冲突的关键）
    // 导航速度 + 角度稳定 = 最终控制速度
    mecanum.velocity.final_vx = nav_vx;     // 位置环主导X方向
    mecanum.velocity.final_vy = nav_vy;     // 位置环主导Y方向  
    mecanum.velocity.final_wz = stab_wz;    // 角度环主导旋转
    
    // 步骤4：速度限幅
    limit_velocity(&mecanum.velocity.final_vx, &mecanum.velocity.final_vy, 
                   &mecanum.velocity.final_wz);
    
    // 步骤5：麦轮逆运动学解算
    mecanum_inverse_kinematics(mecanum.velocity.final_vx, 
                              mecanum.velocity.final_vy,
                              mecanum.velocity.final_wz);
}

// 🎯 麦轮逆运动学解算
void mecanum_inverse_kinematics(fp32 vx, fp32 vy, fp32 wz)
{
    // 麦轮几何参数
    fp32 L = 0.3f;  // 轴距
    fp32 W = 0.3f;  // 轮距
    fp32 R = 0.05f; // 轮半径
    
    // 🎯 关键：麦轮运动学公式（统一处理三个速度分量）
    mecanum.wheel_speed[0] = (vx - vy - (L+W)*wz) / R;  // 右前轮
    mecanum.wheel_speed[1] = (vx + vy + (L+W)*wz) / R;  // 左前轮
    mecanum.wheel_speed[2] = (vx - vy + (L+W)*wz) / R;  // 左后轮  
    mecanum.wheel_speed[3] = (vx + vy - (L+W)*wz) / R;  // 右后轮
    
    // 转换为电机RPM
    for(int i = 0; i < 4; i++) {
        mecanum.wheel_speed[i] *= 60.0f / (2.0f * 3.14159f) * 19.0f; // 减速比
    }
}
// 🎯 智能速度限幅：保持运动方向不变
void limit_velocity(fp32 *vx, fp32 *vy, fp32 *wz)
{
    fp32 max_linear = 1500.0f;   // 最大线速度 mm/s
    fp32 max_angular = 800.0f;   // 最大角速度 deg/s
    
    // 线速度限幅（保持方向）
    fp32 linear_magnitude = sqrtf((*vx)*(*vx) + (*vy)*(*vy));
    if(linear_magnitude > max_linear) {
        fp32 scale = max_linear / linear_magnitude;
        *vx *= scale;
        *vy *= scale;
    }
    
    // 角速度限幅
    if(*wz > max_angular) *wz = max_angular;
    else if(*wz < -max_angular) *wz = -max_angular;
}
void position_controller_init(void)
{
    // 🎯 参考F103工程的参数设置
    
    // X方向位置环（类似location_pid）
    pos_ctrl.pid_x.Kp = 0.8f;   // 比F103稍大（麦轮响应更快）
    pos_ctrl.pid_x.Ki = 0.0f;
    pos_ctrl.pid_x.Kd = 0.1f;
    
    // Y方向位置环  
    pos_ctrl.pid_y.Kp = 0.8f;
    pos_ctrl.pid_y.Ki = 0.0f;
    pos_ctrl.pid_y.Kd = 0.1f;
    
    // 角度稳定环（类似turn_angle_pid）
    pos_ctrl.pid_yaw.Kp = 2.0f; // 比F103稍大
    pos_ctrl.pid_yaw.Ki = 0.1f;
    pos_ctrl.pid_yaw.Kd = 0.2f;
    
    // 速度限制
    pos_ctrl.max_linear_vel = 1500.0f;
    pos_ctrl.max_angular_vel = 800.0f;
}
// 🎯 主控制函数：输入坐标、角度、速度
void mecanum_move_to_target(fp32 target_x, fp32 target_y, fp32 target_angle, fp32 speed)
{
    // 步骤1：计算期望的移动方向
    fp32 dx = target_x - mecanum.current_pos.x;
    fp32 dy = target_y - mecanum.current_pos.y;
    fp32 distance = sqrtf(dx*dx + dy*dy);
    
    // 步骤2：计算期望速度方向（归一化）
    if(distance > 10.0f) {  // 距离大于1cm才移动
        mecanum.target_vx = (dx / distance) * speed;  // X方向速度
        mecanum.target_vy = (dy / distance) * speed;  // Y方向速度
    } else {
        mecanum.target_vx = 0.0f;  // 到达目标，停止
        mecanum.target_vy = 0.0f;
    }
    
    // 步骤3：角度环计算（保持车头指向X正方向）
    angle_ctrl.target_angle = target_angle;  // 通常为0°
    mecanum.target_wz = turn_angle_pid_realize(&angle_ctrl.pid_yaw, mecanum.current_pos.yaw);
    
    // 步骤4：麦轮逆运动学解算
    mecanum_inverse_kinematics(mecanum.target_vx, mecanum.target_vy, mecanum.target_wz);
}

// 🎯 麦轮逆运动学解算
void mecanum_inverse_kinematics(fp32 vx, fp32 vy, fp32 wz)
{
    // 麦轮几何参数（根据您的实际底盘调整）
    fp32 L = 150.0f;  // 前后轮距 mm
    fp32 W = 150.0f;  // 左右轮距 mm  
    fp32 R = 50.0f;   // 轮半径 mm
    
    // 麦轮运动学公式（速度单位：mm/s → 转换为轮子角速度 rad/s）
    fp32 wz_rad = wz * 3.14159f / 180.0f;  // 角度转弧度
    
    mecanum.wheel_speed[0] = (vx - vy - (L+W)*wz_rad) / R;  // 右前轮
    mecanum.wheel_speed[1] = (vx + vy + (L+W)*wz_rad) / R;  // 左前轮
    mecanum.wheel_speed[2] = (vx - vy + (L+W)*wz_rad) / R;  // 左后轮  
    mecanum.wheel_speed[3] = (vx + vy - (L+W)*wz_rad) / R;  // 右后轮
    
    // 转换为电机RPM
    for(int i = 0; i < 4; i++) {
        mecanum.wheel_speed[i] *= 60.0f / (2.0f * 3.14159f) * 19.0f; // 减速比19:1
    }
}

// 角度控制器初始化
void angle_controller_init(void)
{
    // 角度环PID参数（保持车头指向X正方向）
    angle_ctrl.pid_yaw.Kp = 2.0f;
    angle_ctrl.pid_yaw.Ki = 0.1f;
    angle_ctrl.pid_yaw.Kd = 0.2f;
    angle_ctrl.pid_yaw.target_val = 0.0f;  // 目标角度0°
    angle_ctrl.pid_yaw.actual_val = 0.0f;
    angle_ctrl.pid_yaw.err = 0.0f;
    angle_ctrl.pid_yaw.err_last = 0.0f;
    angle_ctrl.pid_yaw.integral = 0.0f;
}