#include "main.h"
#include "stm32f4xx.h"

typedef struct _pid_struct_t
{
  float kp;//比例
  float ki;//积分
  float kd;//微分
  float i_max;//积分限幅
  float out_max;//输出限幅

  float ref;      // target value目标值
  float fdb;      // feedback value反馈值
  float err[2];   // error and last error差值

  float p_out;//比例输出
  float i_out;//积分输出
  float d_out;//微分输出
  float output;//pid总输出
}pid_struct_t;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

float pid_calc(pid_struct_t *pid, float ref, float fdb);
float LIMIT_MIN_MAX(float in,float min,float max);
