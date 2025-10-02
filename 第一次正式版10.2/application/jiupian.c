void move_pure_y_direction(mecanum_control_t *mecanum_control, fp32 target_y, fp32 speed)
{
    static uint8_t set = 0;
    
    if(!set) {
        mecanum_control->target_pos.x = mecanum_control->current_pos.x; // X保持当前值
        mecanum_control->target_pos.y = target_y;
        mecanum_control->target_pos.yaw = mecanum_control->current_pos.yaw;
        set = 1;
    }
    
    while(1)
    {
        fp32 dy = mecanum_control->target_pos.y - mecanum_control->current_pos.y;
        fp32 distance_to_target = fabsf(dy);
        
        // 🎯 X方向位置纠偏（小幅度纠正）
        fp32 x_error = mecanum_control->target_pos.x - mecanum_control->current_pos.x;
        fp32 vx_correction = x_error * 10.0f; // 降低纠偏系数
        
        // 🎯 Y方向主运动
        fp32 vy_main = 0.0f;
        if(dy > 20.0f) {
            vy_main = speed;
        } else if(dy < -20.0f) {
            vy_main = -speed;
        }
        
        // 🎯 速度归一化，保持总速度为speed
        fp32 total_speed = sqrtf(vx_correction * vx_correction + vy_main * vy_main);
        
        if(total_speed > speed && total_speed > 0.1f) {
            // 如果总速度超过设定速度，进行归一化
            fp32 scale = speed / total_speed;
            mecanum_control->vx = vx_correction * scale;
            mecanum_control->vy = vy_main * scale;
        } else {
            mecanum_control->vx = vx_correction;
            mecanum_control->vy = vy_main;
        }
        
        // 角度控制保持不变
        set_target_angle(mecanum_control->target_pos.yaw);
        mecanum_control->vw = angle_controller();
        
        mecanum_calculate_wheel_speed(mecanum_control);
        
        // ... PID控制代码 ...
        
        if(distance_to_target <= 20.0f) {
            set = 0;
            break;
        }
        
        HAL_Delay(1);
    }
}
void move_pure_y_direction(mecanum_control_t *mecanum_control, fp32 target_y, fp32 speed)
{
    static uint8_t set = 0;
    
    if(!set) {
        mecanum_control->target_pos.x = mecanum_control->current_pos.x;
        mecanum_control->target_pos.y = target_y;
        mecanum_control->target_pos.yaw = mecanum_control->current_pos.yaw;
        set = 1;
    }
    
    while(1)
    {
        fp32 dy = mecanum_control->target_pos.y - mecanum_control->current_pos.y;
        fp32 distance_to_target = fabsf(dy);
        
        // 🎯 X方向位置纠偏，但限制最大纠偏速度
        fp32 x_error = mecanum_control->target_pos.x - mecanum_control->current_pos.x;
        fp32 max_correction_speed = speed * 0.3f; // 纠偏速度不超过主速度的30%
        
        fp32 vx_correction = x_error * 5.0f; // 降低纠偏系数
        
        // 限制纠偏速度
        if(vx_correction > max_correction_speed) {
            vx_correction = max_correction_speed;
        } else if(vx_correction < -max_correction_speed) {
            vx_correction = -max_correction_speed;
        }
        
        mecanum_control->vx = vx_correction;
        
        // 🎯 Y方向主运动，保持原始速度
        if(dy > 20.0f) {
            mecanum_control->vy = speed;
        } else if(dy < -20.0f) {
            mecanum_control->vy = -speed;
        } else {
            mecanum_control->vy = 0.0f;
        }
        
        // 角度控制
        set_target_angle(mecanum_control->target_pos.yaw);
        mecanum_control->vw = angle_controller();
        
        mecanum_calculate_wheel_speed(mecanum_control);
        
        // ... 其余代码 ...
    }
}