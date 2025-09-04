/**
  ******************************************************************************
  * @file    pid.c
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   通用PID控制器实现
  *          支持比例(P)、积分(I)、微分(D)控制
  *          包含死区处理、积分限幅、输出限幅等功能
  *          每个PID结构体都要先进行函数指针初始化，然后进行参数初始化
  ******************************************************************************
  * @attention 
  * 功能特点:
  * 1. 支持死区控制，避免小误差时的震荡
  * 2. 积分限幅，防止积分饱和
  * 3. 输出限幅，保护执行机构
  * 4. 死区补偿，改善小信号响应
  * 5. 面向对象设计，通过函数指针调用
  *
  * 使用方法:
  * 1. 先调用pid_init()初始化函数指针
  * 2. 调用f_param_init()设置PID参数
  * 3. 在控制循环中调用f_cal_pid()计算输出
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "pid.h"

/* Private defines -----------------------------------------------------------*/
#define ABS(x)		((x>0)? x: -x)   // 绝对值宏定义

/* Global variables ----------------------------------------------------------*/
// PID控制器实例，用于不同的控制回路
PID_TypeDef pid_pitch,          // 俯仰角PID控制器
            pid_pithch_speed,   // 俯仰角速度PID控制器
            pid_roll,           // 横滚角PID控制器
            pid_roll_speed,     // 横滚角速度PID控制器
            pid_yaw_speed;      // 偏航角速度PID控制器

extern int isMove;  // 外部移动状态标志

/* Private function prototypes -----------------------------------------------*/
static void pid_param_init(PID_TypeDef * pid, PID_ID id, uint16_t maxout,
                          uint16_t intergral_limit, float deadband, uint16_t period,
                          int16_t max_err, int16_t target, float kp, float ki, float kd);
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd);
static float pid_calculate(PID_TypeDef* pid, float measure);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  PID参数初始化函数  
 * @param  pid: PID结构体指针
 * @param  id: PID控制器ID标识
 * @param  maxout: PID输出最大值限制
 * @param  intergral_limit: 积分项限制值，防止积分饱和
 * @param  deadband: 死区范围，小于此值的误差将被忽略
 * @param  period: 控制周期(毫秒)
 * @param  max_err: 允许的最大误差值
 * @param  target: 目标设定值
 * @param  kp: 比例系数
 * @param  ki: 积分系数  
 * @param  kd: 微分系数
 * @retval None
 */
static void pid_param_init(
	PID_TypeDef * pid, 
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	float deadband,
	uint16_t period,
	int16_t  max_err,
	int16_t  target,
	float 	kp, 
	float 	ki, 
	float 	kd)
{
	pid->id = id;		
	
	// 设置控制周期和死区参数
	pid->ControlPeriod = period;
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	
	// 设置PID控制参数
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	// 初始化历史数据
	pid->measure = 0.0f;
	pid->last_measure = 0.0f;
	pid->err = 0.0f;
	pid->last_err = 0.0f;
	
	// 初始化积分项和输出
	pid->integral = 0.0f;
	pid->pout = 0.0f;
	pid->iout = 0.0f;
	pid->dout = 0.0f;
	pid->out = 0.0f;
}

/**
 * @brief  PID参数重设函数
 * @param  pid: PID结构体指针  
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval None
 */
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	// 重设PID参数
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	// 清零历史数据，防止干扰
	pid->measure = 0.0f;
	pid->last_measure = 0.0f;
	pid->err = 0.0f;
	pid->last_err = 0.0f;
	
	// 清零积分项和输出
	pid->integral = 0.0f;
	pid->pout = 0.0f;
	pid->iout = 0.0f;
	pid->dout = 0.0f;
	pid->out = 0.0f;
}

/**
 * @brief  PID控制计算函数
 * @param  pid: PID结构体指针
 * @param  measure: 当前测量值(反馈值)
 * @retval 返回PID控制器输出值
 */
static float pid_calculate(PID_TypeDef* pid, float measure)
{
	// 更新测量值
	pid->last_measure = pid->measure;
	pid->measure = measure;
	
	// 计算当前误差
	pid->last_err = pid->err;
	pid->err = pid->target - pid->measure;
	
	// 死区处理：误差小于死区范围时，误差置零
	if(ABS(pid->err) > pid->DeadBand)
	{
		// 比例项计算
		pid->pout = pid->kp * pid->err;
		
		// 积分项计算，带积分限幅
		pid->integral += pid->err;
		// 积分限幅，防止积分饱和
		if(pid->integral > pid->IntegralLimit)
			pid->integral = pid->IntegralLimit;
		else if(pid->integral < -pid->IntegralLimit)
			pid->integral = -pid->IntegralLimit;
		
		pid->iout = pid->ki * pid->integral;
		
		// 微分项计算
		pid->dout = pid->kd * (pid->err - pid->last_err);
		
		// PID输出计算
		pid->out = pid->pout + pid->iout + pid->dout;
		
		// 输出限幅
		if(pid->out > pid->MaxOutput)
			pid->out = pid->MaxOutput;
		else if(pid->out < -(pid->MaxOutput))
			pid->out = -(pid->MaxOutput);
	}
	else
	{
		// 在死区内，输出为0
		pid->err = 0;
		pid->out = 0;
	}
	
	return pid->out;
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  PID结构体初始化函数
 * @param  pid: PID结构体指针
 * @retval None
 * @note   此函数初始化PID结构体的函数指针，必须在使用PID前调用
 */
void pid_init(PID_TypeDef * pid)
{
	// 初始化函数指针
	pid->f_param_init = pid_param_init;  // 参数初始化函数
	pid->f_pid_reset = pid_reset;        // 参数重设函数  
	pid->f_cal_pid = pid_calculate;      // PID计算函数
}
