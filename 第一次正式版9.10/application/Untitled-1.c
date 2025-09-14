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
// 在mecanum_control.c中添加新函数

/**
  * @brief          根据目标坐标移动并保持角度
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      target_x: 目标X坐标 (mm)
  * @param[in]      target_y: 目标Y坐标 (mm)
  * @param[in]      target_angle: 目标角度 (-180到+180度)
  * @param[in]      speed: 移动速度 (rpm)
  * @retval         none
  */
void mecanum_move_to_coordinate(mecanum_control_t *mecanum_control, 
                               fp32 target_x, fp32 target_y, 
                               fp32 target_angle, fp32 speed)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    
    // 🎯 计算当前位置到目标位置的距离和方向
    fp32 current_x = mecanum_control->current_pos.distance * cosf(mecanum_control->current_pos.yaw * 3.14159f / 180.0f);
    fp32 current_y = mecanum_control->current_pos.distance * sinf(mecanum_control->current_pos.yaw * 3.14159f / 180.0f);
    
    fp32 dx = target_x - current_x;  // X方向距离差
    fp32 dy = target_y - current_y;  // Y方向距离差
    fp32 distance = sqrtf(dx*dx + dy*dy);  // 到目标点的直线距离
    
    // 🎯 如果距离大于阈值才移动
    if(distance > 50.0f) {  // 距离大于5cm才移动
        // 计算归一化的移动方向
        fp32 direction_x = dx / distance;
        fp32 direction_y = dy / distance;
        
        // 设置线速度（朝向目标点）
        mecanum_control->vx = direction_x * speed;
        mecanum_control->vy = direction_y * speed;
    } else {
        // 到达目标位置，停止移动
        mecanum_control->vx = 0.0f;
        mecanum_control->vy = 0.0f;
    }
    
    // 🎯 设置目标角度并计算角度环输出
    set_target_angle(target_angle);
    mecanum_control->vw = angle_control_calculate(mecanum_control->current_pos.yaw);
    
    // 执行麦轮解算
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          检查是否到达目标位置
  * @param[in]      mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      target_x: 目标X坐标 (mm)
  * @param[in]      target_y: 目标Y坐标 (mm)
  * @param[in]      tolerance: 位置容差 (mm)
  * @retval         1: 已到达, 0: 未到达
  */
uint8_t mecanum_is_position_reached(mecanum_control_t *mecanum_control, 
                                   fp32 target_x, fp32 target_y, fp32 tolerance)
{
    if (mecanum_control == NULL)
    {
        return 0;
    }
    
    // 计算当前位置
    fp32 current_x = mecanum_control->current_pos.distance * cosf(mecanum_control->current_pos.yaw * 3.14159f / 180.0f);
    fp32 current_y = mecanum_control->current_pos.distance * sinf(mecanum_control->current_pos.yaw * 3.14159f / 180.0f);
    
    // 计算距离
    fp32 dx = target_x - current_x;
    fp32 dy = target_y - current_y;
    fp32 distance = sqrtf(dx*dx + dy*dy);
    
    return (distance < tolerance);
}

/**
  * @brief          前进并保持角度
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      speed: 速度值(rpm)
  * @param[in]      target_angle: 目标角度 (-180到+180度)
  * @retval         none
  */
void mecanum_move_forward_with_angle_hold(mecanum_control_t *mecanum_control, 
                                         fp32 speed, fp32 target_angle)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    
    // 设置线速度
    mecanum_control->vx = speed;
    mecanum_control->vy = 0.0f;
    
    // 🎯 角度环计算vw
    set_target_angle(target_angle);
    mecanum_control->vw = angle_control_calculate(mecanum_control->current_pos.yaw);
    
    // 执行麦轮解算
    mecanum_calculate_wheel_speed(mecanum_control);
}

/**
  * @brief          侧移并保持角度
  * @param[out]     mecanum_control: 麦轮底盘控制结构体指针
  * @param[in]      speed: 速度值(rpm)
  * @param[in]      target_angle: 目标角度 (-180到+180度)
  * @retval         none
  */
void mecanum_move_sideways_with_angle_hold(mecanum_control_t *mecanum_control, 
                                          fp32 speed, fp32 target_angle)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    
    // 设置线速度
    mecanum_control->vx = 0.0f;
    mecanum_control->vy = speed;
    
    // 🎯 角度环计算vw
    set_target_angle(target_angle);
    mecanum_control->vw = angle_control_calculate(mecanum_control->current_pos.yaw);
    
    // 执行麦轮解算
    mecanum_calculate_wheel_speed(mecanum_control);
}

void move_xy(mecanum_control_t *mecanum_control, fp32 angle, fp32 speed)
{ 
    static uint8_t set = 0;
    static fp32 vx = 0.0f;
    static uint32_t last_check_time = 0;
    static uint8_t obstacle_count = 0;  // 连续检测计数
    
    if(!set) 
    {
        // 初始化设置...
        vx = speed;
        set = 1;
        obstacle_count = 0;
    }
    
    while(1)
    { 
        uint32_t current_time = HAL_GetTick();
        
        // 🚨 每50ms检测一次，提供去抖效果
        if (current_time - last_check_time > 50)
        {
            last_check_time = current_time;
            
            if (HAL_GPIO_ReadPin(gd_l_far_GPIO_Port, gd_l_far_Pin) == GPIO_PIN_RESET || 
                HAL_GPIO_ReadPin(gd_r_far_GPIO_Port, gd_r_far_Pin) == GPIO_PIN_RESET)
            {
                obstacle_count++;
                if (obstacle_count >= 2) // 连续2次检测到障碍物才停止
                {
                    mecanum_stop(mecanum_control);
                    mecanum_calculate_wheel_speed(mecanum_control);
                    set = 0;
                    vx = 0.0f;
                    break;
                }
            }
            else
            {
                obstacle_count = 0; // 重置计数
            }
        }
        
        // 正常运动逻辑
        mecanum_control->vx = vx;
        mecanum_control->vy = 0.0f;
        mecanum_control->vw = angle_controller();
        mecanum_calculate_wheel_speed(mecanum_control);
        
        HAL_Delay(20);
    }
}