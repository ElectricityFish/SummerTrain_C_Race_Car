#include "PID.h"


void PID_Update(PID_t *p)			// 一般PID函数
{
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
	}
	else
	{
		p->ErrorInt = 0;
	}
	
	if(p->ErrorInt>=p->OutMax/2)p->ErrorInt=p->OutMax/2.f;	// 积分限幅
	if(p->ErrorInt<=p->OutMin/2)p->ErrorInt=p->OutMin/2.f;
	
	
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
	+ p->Kd * (p->Error0 - p->Error1);				
		   
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
	
	p->Actual1=p->Actual;
}

static float servo_pid_clampf(float value, float min_value, float max_value)
{
	if(value < min_value)
	{
		return min_value;
	}
	if(value > max_value)
	{
		return max_value;
	}
	return value;
}

void servo_pid_up_date(Servo_PID_t *p)
{
	float abs_error;
	float error_ratio;
	float kp_min;
	float kp_max;

	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;

	// 菜单误设 KpMax < KpMin 时，以 KpMin 为上限，避免出现负的动态增益区间。
	kp_min = p->KpMin;
	kp_max = p->KpMax;
	if(kp_max < kp_min)
	{
		kp_max = kp_min;
	}

	abs_error = (p->Error0 >= 0.0f) ? p->Error0 : -p->Error0;
	if(p->ErrorFull > 0.0f)
	{
		error_ratio = abs_error / p->ErrorFull;
	}
	else
	{
		error_ratio = 1.0f;
	}
	error_ratio = servo_pid_clampf(error_ratio, 0.0f, 1.0f);
	p->KpNow = kp_min + (kp_max - kp_min) * error_ratio * error_ratio;

	if(p->Ki != 0.0f)
	{
		p->ErrorInt += p->Error0;
	}
	else
	{
		p->ErrorInt = 0.0f;
	}
	p->ErrorInt = servo_pid_clampf(p->ErrorInt, p->OutMin / 2.0f, p->OutMax / 2.0f);

	p->Out = p->KpNow * p->Error0
		+ p->Ki * p->ErrorInt
		+ p->Kd * (p->Error0 - p->Error1);
	p->Out = servo_pid_clampf(p->Out, p->OutMin, p->OutMax);
}






