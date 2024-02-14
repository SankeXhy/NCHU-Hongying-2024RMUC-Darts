#include "pid.h"

pid_struct_t M3508_2_PID_S;
pid_struct_t M3508_1_PID_S;
pid_struct_t M3508_1_PID_P;
pid_struct_t M3508_2_PID_P;
pid_struct_t M2006_1_PID_S;

void pid_init(pid_struct_t *pid,float kp,float ki,float kd,float i_max,float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}


float pid_calc(pid_struct_t *pid, float ref, float fdb)
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;

  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  pid->i_out=LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

  pid->output = pid->p_out + pid->i_out + pid->d_out;
  pid->output=LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}

float LIMIT_MIN_MAX(float in,float min,float max)
{
	if(in<=min)
	{
		in=min;
	}
	else if(in>=max)
	{
		in=max;
	}
	return in;
}

void M3508_PID_init()
{
	pid_init(&M3508_1_PID_S,1.8,1.6,0.8, 13000, 15000);//P,I,D
//	pid_init(&M3508_2_PID_P,2,0,0.6, 1000, 1000);
	pid_init(&M3508_2_PID_S,1.8,1.6,0.8, 13000, 15000);//P,I,D
	pid_init(&M2006_1_PID_S,1.3,0.11,0.15, 6000, 8500);//P,I,D
}
