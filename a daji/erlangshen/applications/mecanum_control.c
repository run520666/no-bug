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
#include <stdint.h>  // 为uint32_t类型添加
#include "pid.h"
#include "robot_config.h"
#include "struct_typedef.h"

mecanum_control_t mecanum = {0};
extern q_pid angle_pid_s; //直线pid
extern q_pid angle_pid_t; //转向pid

uint8_t gd_l=0;
uint8_t gd_r=0;
extern float vofa_data[3];
extern UART_HandleTypeDef huart6; 
extern UART_HandleTypeDef huart3;

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
    set_target_angle(&angle_pid_s, mecanum_control->target_pos.yaw); // 设置目标角度为结构体保存的angle
    mecanum_control->vw = angle_controller(&angle_pid_s);

    // 执行麦轮解算
    mecanum_calculate_wheel_speed(mecanum_control);
    }
}


// 设置单段的x/y坐标，相对坐标系
// vx vy仅根据距离设置一次，vw由角度环算出
//接受 麦轮结构体指针 x坐标 y坐标 角度 速度
void set_target_move_to_target(mecanum_control_t *mecanum_control, fp32 x, fp32 y, fp32 angle, fp32 speed,uint8_t act_control,uint8_t stop_control)

{

    static uint8_t set=0;
    static fp32 vx=0.0f;
    static fp32 vy=0.0f;
    static fp32 act_p=0.2f; //启动加速比例（相对总距离）
    static fp32 stop_p=0.2f; //停止减速比例（相对总距离）
    static fp32 direction_x, direction_y;
    static uint32_t start_time=0;
    fp32 a =speed/1.0f; //加速度 500mm/s²
    fp32 current_speed;

    fp32 distance;

    static fp32 act_distance;
    static fp32 stop_distance;
    

    if(!set) //这部分设置仅一次。1.重置位置（把坐标系定在当前位置）2.设置目标位置，角度 3.计算距离 4.计算速度
    {
        //重置当前位置为0，x,y,distance=0
        mecanum_control->reset = 1;
        mecanum_control->current_pos.distance = 0.0f; //重置当前位置到目标距离
        mecanum_control->current_pos.x = 0.0f;
        mecanum_control->current_pos.y = 0.0f;

        //设置目标x，y，angle
        mecanum_control->target_pos.x = x;
        mecanum_control->target_pos.y = y;
        mecanum_control->target_pos.yaw = angle;
        set_target_angle(&angle_pid_s, mecanum_control->target_pos.yaw); //目标角度标准化


        //计算距离->target_pos.distance(仅一次)
        fp32 dx = x;
        fp32 dy = y;

        distance = sqrtf(dx*dx + dy*dy);
        mecanum_control->target_pos.distance=distance;
        // 防止除零错误
        if (distance > 0.001f) {
            direction_x = dx / distance;  // 算cos角
            direction_y = dy / distance;
        } else {
            direction_x = 0.0f;
            direction_y = 0.0f;
        }

        act_distance = distance * act_p; //加速距离
        stop_distance = distance * stop_p; //减速距离

        start_time = HAL_GetTick(); //记录开始时间

        set = 1;
     }
         

    while(1)
    {
    


    
    mecanum_control->a=a;
     fp32 real_time=(HAL_GetTick()-start_time)/1000.0f; //实际加速时间

    // 优化加速控制逻辑
    if(act_control==1&& real_time<speed/a) 
    {
        
        //加速控制逻辑
       current_speed=a* real_time;
    
       if(current_speed>=speed){
           current_speed=speed;
       }
       mecanum_control->current_speed=current_speed;
       mecanum_control->real_time=real_time;
    }
    
    // 优化减速控制逻辑
    else if(stop_control && distance - mecanum_control->current_pos.distance < stop_distance) 
    {
        //减速控制逻辑
        if(speed>1000.0f)
        {
        if(current_speed>1000.0f) //最低速度100mm/s
        {
             current_speed = 0.5f*speed*(distance- mecanum_control->current_pos.distance) / stop_distance; //匀减速运动公式 v=v0-at
        }
        if(current_speed<=1000.0f){
            current_speed=1000.0f; //最低速度1000mm/s
        }
    }
       if(speed<=1000.0f) //如果总速度小于1000mm/s
    {
        if(current_speed>200.0f) //最低速度200mm/s
        {
             current_speed = 0.8f*speed*(distance- mecanum_control->current_pos.distance) / stop_distance; //匀减速运动公式 v=v0-at
        }
        if(current_speed<=200.0f){
            current_speed=200.0f; //最低速度200mm/s
        }
    }
       
       mecanum_control->current_speed=current_speed;
    }

    else 
    {
        current_speed = speed; //保持匀速
    }

    vx=current_speed * direction_x;
    vy=current_speed * direction_y;

    mecanum_control->vx = vx;
    mecanum_control->vy = vy;
    mecanum_control->vw = angle_controller(&angle_pid_s); //直线pid

    //执行麦轮解算算四轮速度
    mecanum_calculate_wheel_speed(mecanum_control);

    //设置容差
    fp32 distance_tolerance = 3.0f;  // 位置容差 3mm
    fp32 angle_tolerance = 8.0f;       // 角度容差 8度
    //计算角度误差
    fp32 angle_error = mecanum_control->target_pos.yaw - mecanum_control->current_pos.yaw;
    while(angle_error > 180.0f) angle_error -= 360.0f;
    while(angle_error < -180.0f) angle_error += 360.0f;
   

        //检查是否到达目标
    if (mecanum_control->current_pos.distance>= mecanum_control->target_pos.distance- distance_tolerance && fabsf(angle_error) <= angle_tolerance)
    //如果当前距离大于>=目标距离-容差 并且角度误差<=容差停车
        {
            // 到达目标位置，停止移动
            //
             mecanum_stop(mecanum_control);
            mecanum_control->reset = 1;

            //target重置
            mecanum_control->target_pos.x = 0.0f;
            mecanum_control->target_pos.y = 0.0f;
            mecanum_control->target_pos.yaw = mecanum_control->current_pos.yaw;
            mecanum_control->target_pos.distance = 0.0f;

            //速度重置为0
            mecanum_control->vx = 0.0f;
            mecanum_control->vy = 0.0f;
            mecanum_control->vw = 0.0f;
              mecanum_calculate_wheel_speed(mecanum_control);
            // 停止机器人
            mecanum_stop(mecanum_control);

            //重置set标志，让下次设置依然只生效一次
            vx=0.0f;
            vy=0.0f;
            set = 0;
            break; // 跳出while循环，函数结束
        }  
        HAL_Delay(5); // 避免死循环占用CPU
    }
} 

/*
// 到达目标前一段距离，确定x坐标
// 记录左侧/右侧有障碍物
void move_x(mecanum_control_t *mecanum_control, fp32 angle, fp32 speed)
{ 
    static uint8_t set=0;
    fp32 vx=0.0f;
    static uint32_t last_time=0;
    uint8_t jg_conter=0;
    uint8_t gd_conter=0;

    if(!set) //这部分设置仅一次。1.重置位置（把坐标系定在当前位置）2.设置目标位置，角度 3.计算距离 4.计算速度
    {
        //重置位置
        mecanum_control->current_pos.distance = 0.0f; //重置当前位置到目标距离
        mecanum_control->current_pos.x = 0.0f;
        mecanum_control->current_pos.y = 0.0f;
        //设置目标角度
        mecanum_control->target_pos.yaw = angle;
        set_target_angle(&angle_pid_s, mecanum_control->target_pos.yaw); //目标角度标准化

        vx=speed; //设置vx
        set = 1;
    }
    while(1)
    {
        vx=speed;
        // 计算速度 vw由速度环算出
        mecanum_control->vx = vx;
        mecanum_control->vy = 0.0f;
        mecanum_control->vw = 0.0f;
        mecanum_calculate_wheel_speed(mecanum_control);

        uint32_t current_time = HAL_GetTick();


        //前进停车逻辑，以及左侧/右侧障碍物记录
        if(current_time -last_time >= 25)
        {
            last_time = current_time;
             if(HAL_GPIO_ReadPin(gd_l_far_GPIO_Port, gd_l_far_Pin)==GPIO_PIN_RESET) //左侧光电（阈值较远）检测到障碍
            {
                gd_conter++;
                if(gd_conter>=2) //消抖，在25ms内检测到2次
                {
                    gd_conter=0; //重置计数
                    gd_l=1;
                }
            }

            if(HAL_GPIO_ReadPin(gd_r_far_GPIO_Port, gd_r_far_Pin)==GPIO_PIN_RESET) //右侧光电（阈值较远）检测到障碍
            {
                gd_conter++;
                if(gd_conter>=2)
                {
                    gd_conter=0;
                    gd_r=1;      
                }
            }

            if( mecanum_control->stp_distance_l <= 400 || mecanum_control->stp_distance_r <= 400) //前方有障碍物停车
            {   
                jg_conter++;
                if(jg_conter>=2) //消抖，在25ms内检测到2次
                {
                    jg_conter=0; //重置计数
                    mecanum_stop(mecanum_control);
                    vx=0.0f;
                    set = 0;
                    break;
                }
            }

           
        }

    }
}
*/
void move_x(mecanum_control_t *mecanum_control, fp32 angle, fp32 speed)
{ 
    static uint8_t set=0;
    static fp32 vx=0.0f;
    static uint32_t last_time=0;
    uint8_t conter=0;
    uint32_t current_time;
    
    if(!set) //这部分设置仅一次。1.重置位置（把坐标系定在当前位置）2.设置目标位置，角度 3.计算距离 4.计算速度
    {
        //重置位置
        mecanum_control->current_pos.distance = 0.0f; //重置当前位置到目标距离
        mecanum_control->current_pos.x = 0.0f;
        mecanum_control->current_pos.y = 0.0f;
        //设置目标角度
        mecanum_control->target_pos.yaw = angle;
        set_target_angle(&angle_pid_s, mecanum_control->target_pos.yaw); //目标角度标准化

        vx=speed; //设置vx
        set = 1;
    }
    while(1)
    {
        // 计算速度 vw由速度环算出
        mecanum_control->vx = vx;
        mecanum_control->vy = 0.0f;
        mecanum_control->vw = angle_controller(&angle_pid_s);
        mecanum_calculate_wheel_speed(mecanum_control);

        current_time = HAL_GetTick();
        if(current_time -last_time >= 50)
        {
            last_time = current_time;

            if(mecanum_control->stp_distance_l >= 30 && 
                mecanum_control->stp_distance_l <= 350 && 
                (HAL_GPIO_ReadPin(gd_l_far_GPIO_Port, gd_l_far_Pin)==GPIO_PIN_RESET)  )
            {
                conter++;
                if(conter>=2)
                {
                   conter = 0;
                   set=0;
                   gd_l=1;
                   vx=0.0f;
                   mecanum_stop(mecanum_control);

                     break;
                }
            }

               if(mecanum_control->stp_distance_r >= 30 &&
                 mecanum_control->stp_distance_r <= 350 && 
                 (HAL_GPIO_ReadPin(gd_r_far_GPIO_Port, gd_r_far_Pin)==GPIO_PIN_RESET) )
            {
               conter++;
               if(conter>=2)
               {
                conter = 0;
                set=0;
                gd_r=1;
                vx=0.0f;
                mecanum_stop(mecanum_control);
                break;
               }
            }
            //右侧光电（阈值较远）检测到障碍
  

        }
        HAL_Delay(3); // 避免死循环占用CPU
        
    }
}


// 确定x坐标后左/右移动确定y坐标
void move_y(mecanum_control_t *mecanum_control, fp32 angle, fp32 speed)
{ 
    static uint8_t set = 0;
    static uint32_t last_time = 0;
    static uint8_t jg_conter = 0;
    uint32_t current_time;

    if(!set) //这部分设置仅一次。1.重置位置（把坐标系定在当前位置）2.设置目标位置，角度
    {
        //重置位置
        mecanum_control->current_pos.distance = 0.0f; //重置当前位置到目标距离
        mecanum_control->current_pos.x = 0.0f;
        mecanum_control->current_pos.y = 0.0f;
        //设置目标角度
        mecanum_control->target_pos.yaw = angle;
        set_target_angle(&angle_pid_s, mecanum_control->target_pos.yaw); //目标角度标准化

        jg_conter = 0;
        set = 1;
    }
    
    while(1)
    {
        // 计算vx，vw速度（vy根据情况确定）
        mecanum_control->vx = 0.0f;
        mecanum_control->vw = angle_controller(&angle_pid_s);

        current_time = HAL_GetTick();

        //左移或右移的逻辑
        if(gd_l == 1) //左侧有障碍物情况
        {
            if(current_time - last_time >= 25) //25ms检测一次
            {
                last_time = current_time;

                //停车条件
                if(mecanum_control->stp_distance_l <= 230 && mecanum_control->stp_distance_l >= 190)
                {
                    jg_conter++;
                    if(jg_conter >= 2) //消抖
                    {
                        mecanum_stop(mecanum_control);
                        
                        set = 0;
                        jg_conter = 0;
                        return; //退出函数
                    }
                }

                //右走
                else if(mecanum_control->stp_distance_l < 190)
                {
                    jg_conter++;
                    if(jg_conter >= 2) //消抖
                    {
                        jg_conter = 0;
                        mecanum_control->vx = 0.0f;
                        mecanum_control->vy = -speed; //左侧近，往右走
                        mecanum_control->vw = angle_controller(&angle_pid_s);
                        mecanum_calculate_wheel_speed(mecanum_control);
                    }
                }

                //左走
                else if(mecanum_control->stp_distance_l > 230)
                {
                    jg_conter++;
                    if(jg_conter >= 2) //消抖
                    {
                        jg_conter = 0;
                        mecanum_control->vx = 0.0f;
                        mecanum_control->vy = speed; //左侧远，往左走
                        mecanum_control->vw = angle_controller(&angle_pid_s);
                        mecanum_calculate_wheel_speed(mecanum_control);
                    }
                }

            }
        }
        else if(gd_r == 1) //右侧有障碍物情况
        {
            if(current_time - last_time >= 25)
            {
                last_time = current_time;
                
                //停车条件
                if(mecanum_control->stp_distance_r <= 230 && mecanum_control->stp_distance_r >= 190)
                {
                    jg_conter++;
                    if(jg_conter >= 2)
                    {
                        mecanum_stop(mecanum_control);
                    
                        set = 0;
                        jg_conter = 0;
                        return; //退出函数
                    }
                }
                //左走
                else if(mecanum_control->stp_distance_r < 190) //右侧近，往左走
                {
                    if(jg_conter >= 2)
                    {
                        jg_conter = 0;
                        mecanum_control->vx = 0.0f;
                        mecanum_control->vy = speed;
                        mecanum_control->vw = angle_controller(&angle_pid_s);
                        mecanum_calculate_wheel_speed(mecanum_control);
                    }
                }
                    //右走
                    else if(mecanum_control->stp_distance_r > 230) //右侧远，往右走
                    {
                        jg_conter = 0;
                        mecanum_control->vx = 0.0f;
                        mecanum_control->vy = -speed;
                        mecanum_control->vw = angle_controller(&angle_pid_s);
                        mecanum_calculate_wheel_speed(mecanum_control);
                }
            }
        }
        
        HAL_Delay(5); // 避免死循环占用CPU
    }
}

//调整角度

void move_angle(mecanum_control_t *mecanum_control, fp32 angle)
{ 
    static uint8_t set=0;
     if(!set) //这部分设置仅一次。1.重置位置（把坐标系定在当前位置）2.设置目标位置，角度 3.计算距离 4.计算速度
    {
        //重置位置
        mecanum_control->current_pos.distance = 0.0f; //重置当前位置到目标距离
        mecanum_control->current_pos.x = 0.0f;
        mecanum_control->current_pos.y = 0.0f;
        //设置目标角度
        mecanum_control->target_pos.yaw = angle;
        set_target_angle(&angle_pid_t, mecanum_control->target_pos.yaw); //目标角度标准化

        set = 1;
    }
    while(1)
    {
        // 计算速度
        mecanum_control->vx = 0.0f;
        mecanum_control->vy = 0.0f;
        mecanum_control->vw = angle_controller(&angle_pid_t);
        mecanum_calculate_wheel_speed(mecanum_control);

        // 计算角度误差并标准化到 -180~+180 度
        fp32 angle_error = mecanum_control->target_pos.yaw - mecanum_control->current_pos.yaw;
        while(angle_error > 180.0f) angle_error -= 360.0f;
        while(angle_error < -180.0f) angle_error += 360.0f;

    vofa_data[0] = (float)mecanum.current_pos.yaw;          // 当前yaw角度
    vofa_data[1] = (float)angle_pid_s.target;               // 直线角度PID目标角度
    vofa_data[2] = (float)angle_pid_t.target;               // 转向角度PID目标角度
    HAL_UART_Transmit(&huart6, (uint8_t*)vofa_data, sizeof(vofa_data), 100);
    // 发送帧尾
    unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart6, tail, 4, 100);

        //检查是否到达目标
        if (fabsf(angle_error) <= 0.2f) //之后可能调整为output输出小于某值
        {
            // 到达目标位置，停止移动
            mecanum_stop(mecanum_control);
           

            //target重置
            mecanum_control->target_pos.x = 0.0f;
            mecanum_control->target_pos.y = 0.0f;
            mecanum_control->target_pos.yaw = mecanum_control->current_pos.yaw;
            mecanum_control->target_pos.distance = 0.0f;

            //速度重置为0
            mecanum_control->vx = 0.0f;
            mecanum_control->vy = 0.0f;
            mecanum_control->vw = 0.0f;
            mecanum_calculate_wheel_speed(mecanum_control);
           
            //重置set标志，让下次设置依然只生效一次
            set = 0;
            break; // 跳出while循环，函数结束
        }

    }
}

void set_target_move_to_target_up(mecanum_control_t *mecanum_control, fp32 x, fp32 y, fp32 angle, fp32 speed,uint8_t act_control,uint8_t stop_control)

{
    static uint8_t set=0;
    static fp32 vx=0.0f;
    static fp32 vy=0.0f;
    static fp32 act_p=0.2f; //启动加速比例（相对总距离）
    static fp32 stop_p=0.2f; //停止减速比例（相对总距离）
    static fp32 direction_x, direction_y;
    static uint32_t start_time=0;
    fp32 a =speed/1.0f; //加速度 500mm/s²
    fp32 current_speed;

    fp32 distance;

    static fp32 act_distance;
    static fp32 stop_distance;
    

    if(!set) //这部分设置仅一次。1.重置位置（把坐标系定在当前位置）2.设置目标位置，角度 3.计算距离 4.计算速度
    {
        //重置当前位置为0，x,y,distance=0
        mecanum_control->reset = 1;
        mecanum_control->current_pos.distance = 0.0f; //重置当前位置到目标距离
        mecanum_control->current_pos.x = 0.0f;
        mecanum_control->current_pos.y = 0.0f;

        //设置目标x，y，angle
        mecanum_control->target_pos.x = x;
        mecanum_control->target_pos.y = y;
        mecanum_control->target_pos.yaw = angle;
        set_target_angle(&angle_pid_s, mecanum_control->target_pos.yaw); //目标角度标准化


        //计算距离->target_pos.distance(仅一次)
        fp32 dx = x;
        fp32 dy = y;

        distance = sqrtf(dx*dx + dy*dy);
        mecanum_control->target_pos.distance=distance;
        // 防止除零错误
        if (distance > 0.001f) {
            direction_x = dx / distance;  // 算cos角
            direction_y = dy / distance;
        } else {
            direction_x = 0.0f;
            direction_y = 0.0f;
        }

        act_distance = distance * act_p; //加速距离
        stop_distance = distance * stop_p; //减速距离

        start_time = HAL_GetTick(); //记录开始时间

        set = 1;
     }
         

    while(1)
    {
    
    mecanum_control->a=a;
     fp32 real_time=(HAL_GetTick()-start_time)/1000.0f; //实际加速时间

    // 优化加速控制逻辑
    if(act_control==1&& real_time<speed/a) 
    {
        
        //加速控制逻辑
       current_speed=a* real_time;
    
       if(current_speed>=speed){
           current_speed=speed;
       }
       mecanum_control->current_speed=current_speed;
       mecanum_control->real_time=real_time;
    }
    
    // 优化减速控制逻辑
    else if(stop_control && distance - mecanum_control->current_pos.distance < stop_distance) 
    {
        //减速控制逻辑
        if(speed>1000.0f)
        {
        if(current_speed>1000.0f) //最低速度100mm/s
        {
             current_speed = 0.8f*speed*(distance- mecanum_control->current_pos.distance) / stop_distance; //匀减速运动公式 v=v0-at
        }
        if(current_speed<=1000.0f){
            current_speed=1000.0f; //最低速度1000mm/s
        }
    }
       if(speed<=1000.0f) //如果总速度小于1000mm/s
    {
        if(current_speed>200.0f) //最低速度200mm/s
        {
             current_speed = 0.8f*speed*(distance- mecanum_control->current_pos.distance) / stop_distance; //匀减速运动公式 v=v0-at
        }
        if(current_speed<=200.0f){
            current_speed=200.0f; //最低速度200mm/s
        }
    }
       
       mecanum_control->current_speed=current_speed;
    }

    else 
    {
        current_speed = speed; //保持匀速
    }
    vx=current_speed * direction_x;
    vy=current_speed * direction_y;

    mecanum_control->vx = vx;
    mecanum_control->vy = vy;
    mecanum_control->vw = 0.0f; //不转向

    //执行麦轮解算算四轮速度
    mecanum_calculate_wheel_speed(mecanum_control);

    //设置容差
    fp32 distance_tolerance = 3.0f;  // 位置容差 3mm

    vofa_data[0] = (float)mecanum.current_pos.yaw;          // 当前yaw角度
    vofa_data[1] = (float)angle_pid_s.target;               // 直线角度PID目标角度
    vofa_data[2] = (float)angle_pid_t.target;               // 转向角度PID目标角度
    HAL_UART_Transmit(&huart6, (uint8_t*)vofa_data, sizeof(vofa_data), 100);
    // 发送帧尾
    unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart6, tail, 4, 100);

   

        //检查是否到达目标
    if (mecanum_control->current_pos.distance>= mecanum_control->target_pos.distance- distance_tolerance)
    //如果当前距离大于>=目标距离-容差 并且角度误差<=容差停车
        {
            // 到达目标位置，停止移动
            //
             mecanum_stop(mecanum_control);
            mecanum_control->reset = 1;
            

            //target重置
            mecanum_control->target_pos.x = 0.0f;
            mecanum_control->target_pos.y = 0.0f;
            mecanum_control->target_pos.yaw = mecanum_control->current_pos.yaw;
            mecanum_control->target_pos.distance = 0.0f;

            //速度重置为0
            mecanum_control->vx = 0.0f;
            mecanum_control->vy = 0.0f;
            mecanum_control->vw = 0.0f;
              mecanum_calculate_wheel_speed(mecanum_control);
            // 停止机器人
            mecanum_stop(mecanum_control);

            //重置set标志，让下次设置依然只生效一次
            vx=0.0f;
            vy=0.0f;
            set = 0;
            break; // 跳出while循环，函数结束
        }  
        HAL_Delay(5); // 避免死循环占用CPU
    }
} 



















   


