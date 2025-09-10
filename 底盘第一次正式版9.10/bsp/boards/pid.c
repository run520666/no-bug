/**
  ******************************************************************************
  * @file    pid.c
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   ??PID?????????PID?????????????????????????
  ******************************************************************************
  * @attention ?????????????
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "mecanum_control.h"
#include "math.h"

extern mecanum_control_t mecanum;
q_pid angle_pid;
q_pid speed_pid[4];


#define ABS(x)		((x>0)? x: -x)   // ??????

// PID???????
PID_TypeDef pid_pitch,pid_pithch_speed,pid_roll,pid_roll_speed,pid_yaw_speed;
extern int isMove;

// PID???????
static void pid_param_init
(
		PID_TypeDef * pid, 
		PID_ID   id,
		uint16_t maxout,
		uint16_t intergral_limit,
		float deadband,
		uint16_t period,
		int16_t  max_err,   // ????
		int16_t  target,

		float 	kp, 
		float 	ki, 
		float 	kd
)

{
	pid->id = id;		
	
	pid->ControlPeriod = period;             // ????(??)
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

// PID??????
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

// PID??????
static float pid_calculate(PID_TypeDef* pid, float measure)//, int16_t target)
{
//	uint32_t time,lasttime;
	
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
	pid->measure = measure;
  //	pid->target = target;
		
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	
	pid->err = pid->target - pid->measure;
	
	// ??????????
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		

		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		// ??????????
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		// PID????
		pid->output = pid->pout + pid->iout + pid->dout;
		

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  // ????
		if(pid->output > pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
		// 死区补偿：输出在0但绝对值超过100时，强制输出±30
		if(pid->output > 0 && pid->output < 100)
			pid->output = 30;
		else if(pid->output < 0 && pid->output > -100)
			pid->output = -30;
	}


	return pid->output;
}

// PID初始化
void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}

//新写速度环

void speed_pid_init(q_pid *speed_pid)
{
	speed_pid->target = 0.0f;      // ??????0?
    speed_pid->kp = 0.0f;                // ????
    speed_pid->ki = 0.0f;                // ????
    speed_pid->kd = 0.0f;                // ????

    speed_pid->last_err = 0.0f;        // ????
    speed_pid->integral = 0.0f;          // ????
    speed_pid->max_output = 0.0f;      // ???????
    speed_pid->max_integral = 0.0f;    // ????
    speed_pid->enable = 1;               // ????
}

void set_speed_pid(q_pid *speed_pid,float kp, float ki, float kd,float max_out,float max_i,float deadband)
{ 
	speed_pid->kp = kp;
	speed_pid->ki = ki;
	speed_pid->kd = kd;
	speed_pid->max_output = max_out;
	speed_pid->max_integral = max_i;
	speed_pid->deadband = deadband;
}

void speed_pid_control(q_pid *speed_pid, float target_speed, float current_speed)
{
    if(!speed_pid->enable) {
        speed_pid->output = 0.0f;
		return; 
    }

	speed_pid->target = target_speed;
    speed_pid->current = current_speed;

	 // 计算误差
    speed_pid->err = speed_pid->target - speed_pid->current;

	if(fabsf(speed_pid->err) < speed_pid->deadband) {
         // 在死区内，保持上次输出
		 return;
    }

	// P项计算
    speed_pid->p_output = speed_pid->kp * speed_pid->err;
    
    // I项计算
    speed_pid->integral += speed_pid->err;
    
    // 积分限幅
    if(speed_pid->integral > speed_pid->max_integral) {
        speed_pid->integral = speed_pid->max_integral;
    } else if(speed_pid->integral < -speed_pid->max_integral) {
        speed_pid->integral = -speed_pid->max_integral;
    }
    // I项计算
    speed_pid->i_output = speed_pid->ki * speed_pid->integral;

	// D项计算
    speed_pid->d_output = speed_pid->kd * (speed_pid->err - speed_pid->last_err);
    
    // 总输出
    speed_pid->output = speed_pid->p_output + speed_pid->i_output + speed_pid->d_output;
    
    // 输出限幅
    if(speed_pid->output > speed_pid->max_output) {
        speed_pid->output = speed_pid->max_output;
    } else if(speed_pid->output < -speed_pid->max_output) {
        speed_pid->output = -speed_pid->max_output;
    }
    
    // 保存当前误差为上次误差
    speed_pid->last_err = speed_pid->err;
}





//设置误差-180~180
float set_error(float target, float current)
{
	float error = target - current;
	 while(error > 180.0f) {
        error -= 360.0f;
    }
    while(error < -180.0f) {
        error += 360.0f;
    }
    return error;
}

//设置目标角度-180~180
void set_target_angle(float angle)
{
    // 将角度限制在-180到180之间
    while(angle > 180.0f) angle -= 360.0f;
    while(angle <= -180.0f) angle += 360.0f;

    angle_pid.target = angle;
}

//pid初始化
void angle_controller_init(void)
{
    angle_pid.target = 0.0f;      // ??????0?
    angle_pid.kp = 0.0f;                // ????
    angle_pid.ki = 0.0f;                // ????
    angle_pid.kd = 0.0f;                // ????

    angle_pid.last_err = 0.0f;        // ????
    angle_pid.integral = 0.0f;          // ????
    angle_pid.max_output = 0.0f;      // ???????
    angle_pid.max_integral = 0.0f;    // ????
    angle_pid.enable = 1;               // ????
}

//设置PID参数
void set_angle_pid(float kp, float ki, float kd,float max_out,float max_i) //设置kp,ki,kd,输出限幅，积分限幅
{
	angle_pid.kp = kp;
	angle_pid.ki = ki;
	angle_pid.kd = kd;
	angle_pid.max_output = max_out;
	angle_pid.max_integral = max_i;
}
float angle_controller(void)
{ 
	if(!angle_pid.enable) {
        return 0.0f;
    } //如果未使能，返回0
	float current_yaw = mecanum.current_pos.yaw; //获取yaw角
	angle_pid.current = current_yaw;  // 记录当前yaw角
    angle_pid.err = set_error(angle_pid.target, current_yaw); //计算当前误差

	float p_output = angle_pid.kp * angle_pid.err; //比例项
	angle_pid.p_output=p_output;

	angle_pid.integral += angle_pid.err; //积分
    if(angle_pid.integral > angle_pid.max_integral) //积分限幅
	{
        angle_pid.integral = angle_pid.max_integral;
    } 
	else if(angle_pid.integral < -angle_pid.max_integral)
	 {
        angle_pid.integral = -angle_pid.max_integral;
    }

	float i_output = angle_pid.ki * angle_pid.integral; //积分项
	angle_pid.i_output=i_output;
    float d_output = angle_pid.kd * (angle_pid.err - angle_pid.last_err); //微分项
	angle_pid.d_output=d_output;

    float output = p_output + i_output + d_output; //总输出

    if(output > angle_pid.max_output) // 输出限幅
	{
        output = angle_pid.max_output;
    } else if(output < -angle_pid.max_output)
	 {
        output = -angle_pid.max_output;
    }
    
    angle_pid.output = output;  //  保存输出到结构体
    angle_pid.last_err = angle_pid.err;  //  保存当前误差到上次误差


    return output;
}













