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
#include "main.h"
#include <math.h>
#include <stdlib.h>  // 为NULL定义添加
#include "pid.h"
mecanum_control_t mecanum = {0};
extern volatile uint8_t stop;

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
    mecanum_control->current_pos.x = 0.0f;
    mecanum_control->current_pos.y = 0.0f;
    mecanum_control->current_pos.yaw = 0.0f;

    mecanum_control->target_pos.x = 0.0f;
    mecanum_control->target_pos.y = 0.0f;
    mecanum_control->target_pos.distance = 0.0f;
    mecanum_control->target_pos.yaw = 0.0f;
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

/*设置目标位置，角度，速度
  传到麦轮结算的vx，vy，vw
*/

void mecanum_move_to_target(mecanum_control_t *mecanum_control, fp32 x, fp32 y, fp32 angle, fp32 speed)
{
    if (mecanum_control == NULL)
    {
        return;
    }
    stop=0;
    static uint8_t set=0;
    fp32 dx, dy, target_distance;
    
    //利用set让这个目标只设置一次
    if (!set) {
        mecanum_control->target_pos.x = x;
        mecanum_control->target_pos.y = y;
         // 计算当前位置到目标位置的距离和角度
        dx = mecanum_control->target_pos.x - mecanum_control->current_pos.x;
        dy = mecanum_control->target_pos.y - mecanum_control->current_pos.y;
        target_distance = sqrtf(dx*dx + dy*dy); //勾股定理算目标距离
        mecanum_control->target_pos.distance = target_distance; //更新目标距离到target_pos结构体中
        set = 1; // 标记目标已设置
    
    // 设置线速度和旋转速度
  
        fp32 direction_x = dx / target_distance;  // 算cos角
        fp32 direction_y = dy / target_distance;
        
        mecanum_control->vx = speed * direction_x;  // X方向速度=总速度*cos角
        mecanum_control->vy = speed * direction_y;  // Y方向速度=总速度*sin角
       

        mecanum_control->vx = 0.0f;
        mecanum_control->vy = 0.0f;
    
    // 角度环计算vw
    set_target_angle(mecanum_control->target_pos.yaw); // 设置目标角度为结构体保存的angle
    mecanum_control->vw = angle_controller(); 

    // 执行麦轮解算
    mecanum_calculate_wheel_speed(mecanum_control);
    }
}

void move_to_target(mecanum_control_t *mecanum_control, fp32 x, fp32 y, fp32 angle, fp32 speed)
{
    if (mecanum_control == NULL)
    {
        return;
    }

    fp32 distance_tolerance = 100.0f;  // 位置容差 mm
    fp32 angle_tolerance = 2.0f;      // 角度容差 1度

    // 🎯 循环直到到达目标
    while (1)
    {
        // 第一步：执行mecanum_move_to_target
        if(pid_flag)
        {
         // 仅在pid_flag被定时器中断置1时执行
        mecanum_move_to_target(mecanum_control, x, y, angle, speed);

         for (int i = 0; i < 4; i++) 
            {
            const motor_measure_t *motor_data = get_chassis_motor_measure_point(i);
            if (motor_data != NULL) 
                {
                // 设置电机目标转速为计算出的轮速
                motor_pid[i].target = mecanum_control->wheel_speed[i];
                // 执行PID计算
                motor_pid[i].f_cal_pid(&motor_pid[i], motor_data->speed_rpm);
                }
            }
            CAN_cmd_chassis(motor_pid[0].output, motor_pid[1].output, motor_pid[2].output, motor_pid[3].output);
            pid_flag = 0; // 清除标志位
        }

        // 计算角度误差
        fp32 angle_error = angle - mecanum_control->current_pos.yaw;
        // 标准化角度误差到 -180~+180 度
        while(angle_error > 180.0f) angle_error -= 360.0f;
        while(angle_error <= -180.0f) angle_error += 360.0f;

        //  检查是否到达目标(距离在can断中处理了）
        if (mecanum_control->current_pos.distance <= distance_tolerance && fabsf(angle_error) <= angle_tolerance)
        {
            // 到达目标，停止电机
            mecanum_stop(mecanum_control);
              for(int stop_count = 0; stop_count < 5; stop_count++) {
                CAN_cmd_chassis(0, 0, 0, 0);
                HAL_Delay(20);
            }
            //  更新target_pos为current_pos
            mecanum_control->target_pos.x = mecanum_control->current_pos.x;
            mecanum_control->target_pos.y = mecanum_control->current_pos.y;
            mecanum_control->target_pos.yaw = mecanum_control->current_pos.yaw;
            mecanum_control->target_pos.distance = 0.0f;  // 距离归零
            mecanum_control->current_pos.distance = 0.0f;  // 当前位置距离也归零
            
            break;  // 跳出循环
        }

       // 等待下一个PID周期，在此前pid_flag会被定时器中断置1，目前电机是停转的，会保持禁止
       // 到下次定时器启动 pid_flag变为1，跳出空循环，继续执行下一次PID计算
       HAL_Delay(1); // 避免死循环占用CPU
        
      
    }
}

void set_target_move_to_target(mecanum_control_t *mecanum_control, fp32 x, fp32 y, fp32 angle, fp32 speed)
{

    static uint8_t set=0;
    static fp32 vx=0.0f;
    static fp32 vy=0.0f;

    fp32 distance;

    //fp32 vx_global = 0.0f;
    //fp32 vy_global = 0.0f;
    if(!set)
    {
        mecanum_control->reset = 1;
        mecanum_control->current_pos.distance = 0.0f; //重置当前位置到目标距离
         mecanum_control->current_pos.x = 0.0f;
        mecanum_control->current_pos.y = 0.0f;

        mecanum_control->target_pos.x = x;
        mecanum_control->target_pos.y = y;
        mecanum_control->target_pos.yaw = angle;
        set_target_angle(mecanum_control->target_pos.yaw);

        fp32 dx = x;
        fp32 dy = y;
        distance = sqrtf(dx*dx + dy*dy);
        mecanum_control->target_pos.distance=distance;

         if(distance > 0.1f) {
            // 🎯 这是移动方向，不是机器人朝向
            vx = speed * (dx / distance);  // X方向移动速度
            vy = speed * (dy / distance);  // Y方向移动速度
        } else {
            vx = 0.0f;
            vy = 0.0f;
        }
         set = 1;
    }
    while(1)
    {
    mecanum_control->vx = vx;
    mecanum_control->vy = vy;

    
    /*
    fp32 dx = mecanum_control->target_pos.x - mecanum_control->current_pos.x;
    fp32 dy = mecanum_control->target_pos.y - mecanum_control->current_pos.y;
    fp32 current_distance = sqrtf(dx*dx + dy*dy);
    */



    /*
    fp32 target_angle_global = atan2f(dy, dx) * 180.0f / 3.1415f;
        

    //vx_global = speed * cosf(target_angle_global * 3.1415f / 180.0f);
   // vy_global = speed * sinf(target_angle_global * 3.1415f / 180.0f);

    fp32 yaw_rad = mecanum_control->current_pos.yaw * 3.1415f / 180.0f;
    mecanum_control->vx =  speed * cosf(target_angle_global * 3.1415f / 180.0f);
    mecanum_control->vy =  speed * sinf(target_angle_global * 3.1415f / 180.0f);
    */

   // mecanum_control->vx = vx;
   // mecanum_control->vy = vy;
    mecanum_control->vw = angle_controller();

    mecanum_calculate_wheel_speed(mecanum_control);

    fp32 distance_tolerance = 5.0f;  // 位置容差 5mm
    fp32 angle_tolerance = 1.0f;       // 角度容差 1度

        /*
        fp32 current_dx = mecanum_control->target_pos.x - mecanum_control->current_pos.x;
        fp32 current_dy = mecanum_control->target_pos.y - mecanum_control->current_pos.y;
        fp32 current_distance = sqrtf(current_dx*current_dx + current_dy*current_dy);
        mecanum_control->current_pos.current_distance = current_distance; //更新当前位置到目标距离
        */

        // 计算角度误差
    fp32 angle_error = mecanum_control->target_pos.yaw - mecanum_control->current_pos.yaw;
    while(angle_error > 180.0f) angle_error -= 360.0f;
    while(angle_error < -180.0f) angle_error += 360.0f;

        // 🎯 检查是否到达目标
    if (mecanum_control->current_pos.distance>= mecanum_control->target_pos.distance- distance_tolerance && fabsf(angle_error) <= angle_tolerance)
        {
            // 到达目标位置，停止移动
             mecanum_stop(mecanum_control);
            mecanum_control->reset = 1;
            stop=1; //全局停止
            //mecanum_control->target_pos.x = 0.0f;
            //mecanum_control->target_pos.y = 0.0f;
            mecanum_control->target_pos.yaw = mecanum_control->current_pos.yaw;
            mecanum_control->target_pos.distance = 0.0f;

            mecanum_control->vx = 0.0f;
            mecanum_control->vy = 0.0f;
            mecanum_control->vw = 0.0f;
              mecanum_calculate_wheel_speed(mecanum_control);
           /* 
           for (int i = 0; i < 4; i++) 
            {
                motor_pid[i].target = 0.0f;      // 强制设置为0
                motor_pid[i].iout = 0.0f;        //  清除积分输出
                motor_pid[i].last_err = 0.0f;    //  清除上次误差
                motor_pid[i].err = 0.0f;         // 清除当前误差
                motor_pid[i].output = 0.0f;      // 清除输出
            }
            
            
            //  5. 设置角度控制器目标为当前角度
            angle_pid.err = 0.0f;              // 清除当前误差
            angle_pid.last_err = 0.0f;         // 清除上次误差
            angle_pid.integral = 0.0f;         // 清除积分项
            angle_pid.p_output = 0.0f;         // 清除P输出
            angle_pid.i_output = 0.0f;         // 清除I输出
            angle_pid.d_output = 0.0f;         // 清除D输出
            angle_pid.output = 0.0f;           // 清除总输出
    
            set_target_angle(mecanum_control->current_pos.yaw);
            */
            
            // 6. 停止机器人
            mecanum_stop(mecanum_control);
            
            // 🎯 7. 发送多次停止命令确保停止
            for(int stop_count = 0; stop_count < 5; stop_count++) {
                CAN_cmd_chassis(0, 0, 0, 0);
                set_target_angle(mecanum_control->current_pos.yaw);
                HAL_Delay(10);
            }
            
            // 8. 重置set标志
            vx=0.0f;
            vy=0.0f;
            set = 0;
            break; // 跳出while循环，函数结束
        }
        
        HAL_Delay(5); // 避免死循环占用CPU
    } 

}
void stop_all(void)
{
    if(pid_flag)
    {
    //  强制设置所有电机PID target为0
        for (int i = 0; i < 4; i++) 
        {
         motor_pid[i].target = 0.0f;  // 强制为0
        }        
    //  发送停止命令
    CAN_cmd_chassis(0, 0, 0, 0);  // 直接发送0
    pid_flag = 0;
    }
                

}







   


