/**
  ******************************************************************************
  * @file		 pid.h
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef _PID_H
#define _PID_H

#include "stdint.h"

typedef enum
{

	PID_Position,
	PID_Speed
	
}PID_ID;

typedef struct _PID_TypeDef
{
	PID_ID id;
	
	float target;							//目标值（由麦轮公式给定）
	float measure;						//测量值（反馈值）
	float err;							//误差
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;

	float   last_err;      		//上次误差

	float pout;
	float iout;
	float dout;

	float output;						//输出
	float last_output;			//上次输出

	float MaxOutput;				//最大输出
	float IntegralLimit;		//积分限幅
	float DeadBand;			  //死区
	float ControlPeriod;		//控制周期
	float  Max_Err;					//最大误差

					  uint32_t thistime;
					uint32_t lasttime;
						uint8_t dtime;

	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID参数初始化
				   PID_ID id,
				   uint16_t maxOutput,
				   uint16_t integralLimit,
				   float deadband,
				   uint16_t controlPeriod,
					int16_t max_err,     
					int16_t  target,
				   float kp,
				   float ki,
				   float kd);

	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid参数重置
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //pid计算
}PID_TypeDef;

typedef struct
{
	float target; //目标
	float current; //实际
	float err; //误差
	float last_err; //上次误差
	float kp,ki,kd; //PID参数
	float integral; //积分值
	
	float p_output;
	float i_output;
	float d_output;
	
	float max_output;
	float max_integral;
	
	float output;
	uint8_t enable;
}q_pid;
extern q_pid angle_pid;

void pid_init(PID_TypeDef* pid);
#endif

//extern PID_TypeDef pid_pitch;    
extern PID_TypeDef motor_pid[4];
extern void pid_init(PID_TypeDef* pid); //pid初始化
void set_target_angle(float angle); //设置目标角度
extern void set_angle_pid(float kp, float ki, float kd,float max_out,float max_i); //设置kp,ki,kd,输出限幅，积分限幅
extern float angle_controller(void); //角度控制
extern void angle_controller_init(void); //角度控制初始化

