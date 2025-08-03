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
	
	float target;							//目标值
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;
	
	float   measure;					//测量值
	float   err;							//误差
	float   last_err;      		//上次误差
	
	float pout;
	float iout;
	float dout;
	
	float output;						//本次输出
	float last_output;			//上次输出
	
	float MaxOutput;				//输出限幅
	float IntegralLimit;		//积分限幅
	float DeadBand;			  //死区（绝对值）
	float ControlPeriod;		//控制周期
	float  Max_Err;					//最大误差
	
					  uint32_t thistime;
					uint32_t lasttime;
						uint8_t dtime;	
	
	void (*f_param_init)(struct _PID_TypeDef *pid,  // PID参数初始化
				   PID_ID id,                      // PID模式（位置式或速度式）
				   uint16_t maxOutput,            // 最大输出限幅
				   uint16_t integralLimit,        // 积分限幅
				   float deadband,                // 死区（绝对值）
				   uint16_t controlPeriod,        // 控制周期（单位：ms）
					int16_t max_err,               // 最大误差
					int16_t  target,                // 目标值
				   float kp,                      // 比例系数
				   float ki,                      // 积分系数
				   float kd);                     // 微分系数
				   
	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid三个参数修改
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //pid计算
}PID_TypeDef;

void pid_init(PID_TypeDef* pid);
#endif

//extern PID_TypeDef pid_pitch;    
extern PID_TypeDef motor_pid[4];
