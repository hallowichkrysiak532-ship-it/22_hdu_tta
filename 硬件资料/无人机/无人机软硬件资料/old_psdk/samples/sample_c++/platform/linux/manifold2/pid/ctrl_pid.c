#include "ctrl_pid.h"


void SetControlKp(float kp, ctrl_pid_t *pid)
{
	pid->kp = kp;
}

void SetControlKi(float ki, ctrl_pid_t *pid)
{
	pid->ki = ki;
}

void SetControlKd(float kd, ctrl_pid_t *pid)
{
	pid->kd = kd;
}
void SetControlCutOffFreq(float freq, ctrl_pid_t *pid)
{
	pid->cut_off_freq = freq;
	pid->filter_deriv.not_first_input = 0;
}

void SetInputLimit(float limit, ctrl_pid_t *pid)
{
	pid->input_limit = limit;
}

void SetImaxLimit(float imax, ctrl_pid_t *pid)
{
	pid->imax_limit = imax;
}


void ResetControl_I(ctrl_pid_t *pid)
{
	pid->reset_I_flag = 1;
}

unsigned int pid_err_nan = 0;
float Pid_Control(float error, float dt, ctrl_pid_t *pid)
{
	float deriv;
	
	pid->dt = dt;
	
	error = constrain_float(error, -pid->input_limit, +pid->input_limit);
	deriv = (error - pid->input)/pid->dt;
	pid->deriv = lowpass_filter_1p_fliter(&pid->filter_deriv, deriv, pid->cut_off_freq, pid->dt);	//微分滤波
	
	pid->input = error;
	
	if(fpclassify(pid->deriv)==FP_NAN_2||fpclassify(pid->deriv)==FP_INFINITE_2)
	{
		pid->deriv = 0;
		pid->filter_deriv.output = 0;
		pid_err_nan++;
	}
	if(fpclassify(pid->input)==FP_NAN_2||fpclassify(pid->input)==FP_INFINITE_2)
	{
		pid->input = 0;
		pid_err_nan++;
	}
	

	
	pid->p_out = pid->input * pid->kp;		//计算比例量
	pid->d_out = pid->deriv * pid->kd;		//计算微分量
	
	if(pid->reset_I_flag == 1)		//清除积分
	{
		pid->reset_I_flag = 0;
		pid->i_out = 0;
	}
	else
	{
		// 特定情况下，只允许积分减小
		if (!pid->allow_add_I_flag || ((pid->i_out > 0 && pid->input < 0) || (pid->i_out < 0 && pid->input > 0))) 
		{
//			error = constrain_float(error, -pid->input_limit, +pid->input_limit);
			if(!is_zero(pid->ki) && !is_zero(pid->dt)) 
			{
				pid->i_out += (pid->input * pid->ki) * pid->dt;
				pid->i_out = constrain_float(pid->i_out, -pid->imax_limit, +pid->imax_limit);
			}
		}
	}
	
	pid->out = 	pid->p_out+
				pid->i_out+ 
				pid->d_out;
	
	
	return pid->out;
}


