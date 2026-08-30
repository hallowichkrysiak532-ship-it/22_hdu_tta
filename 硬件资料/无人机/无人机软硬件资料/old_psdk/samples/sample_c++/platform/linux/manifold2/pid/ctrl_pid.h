#ifndef __CTRL_PID_H_
#define	__CTRL_PID_H_

#include "public_math.h"

typedef struct tag_ctrl_pid_t
{

	
	float kp;	//比例参数
	float ki;	//积分参数
	float kd;	//微分参数
	
	float dt;	//间隔时间
	
	float cut_off_freq;		//截止频率
	lowpass_filter_1p_t filter_deriv;	//微分滤波器
	
	float input;	//输入误差量
	float deriv;	//输入微分量 经过滤波
	float p_out;	//比例输出
	float i_out;	//积分输出
	float d_out;	//微分输出
	
	float out;		//总输出
	
	float input_limit;		//输入误差限制
	float imax_limit;		//积分限幅
	
	unsigned char reset_I_flag:1;	//积分复位标志
	unsigned char allow_add_I_flag:1;	//允许增加积分标志
}ctrl_pid_t;





void SetControlKp(float kp, ctrl_pid_t *pid);	//设定比例系数
void SetControlKi(float ki, ctrl_pid_t *pid);	//设定积分系数
void SetControlKd(float kd, ctrl_pid_t *pid);	//设定微分系数
void SetControlCutOffFreq(float freq, ctrl_pid_t *pid);	//设定微分输入滤波器截止频率
void SetInputLimit(float limit, ctrl_pid_t *pid);	//设定输入限幅
void SetImaxLimit(float imax, ctrl_pid_t *pid);		//设定积分限幅
void ResetControl_I(ctrl_pid_t *pid);				//复位积分
float Pid_Control(float error, float dt, ctrl_pid_t *pid);	//调用PID控制器





#endif
