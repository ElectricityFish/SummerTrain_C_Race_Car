#include "PID.h"
#include <string.h>


Servo_PID_t servo_pid;
Yaw_Rate_PID_t yaw_rate_pid;


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

static float servo_pid_absf(float value)
{
	return (value >= 0.0f) ? value : -value;
}

void PID_init(void)
{
	memset(&servo_pid, 0, sizeof(servo_pid));
	memset(&yaw_rate_pid, 0, sizeof(yaw_rate_pid));

	// 图像外环：图像误差（像素）转换为目标横摆角速度（°/s）。
	servo_pid.KpMin = 1.5f;
	servo_pid.KpMax = 5.00f;
	servo_pid.ErrorFull = 35.0f;
	servo_pid.OutMax = 300.0f;
	servo_pid.OutMin = -300.0f;
	servo_pid.KpNow = servo_pid.KpMin;

	// 横摆角速度内环：初始仅使用 P，避免在尚未验证方向时引入积分或加速度噪声。
	yaw_rate_pid.Kp = 0.24f;
	yaw_rate_pid.Ki = 0.0f;
	yaw_rate_pid.Kd = 0.0f;
	// 静止采样中心约为 +0.24 °/s，左转为正、右转为负。
	yaw_rate_pid.GyroBias = 0.24f;
	yaw_rate_pid.FilterAlpha = 0.25f;
	yaw_rate_pid.Deadband = 0.30f;
	// 最终安装行程仍由 servomotor_set_angle() 裁剪，此处不重复施加机械限幅。
	yaw_rate_pid.OutMax = 55.0f;
	yaw_rate_pid.OutMin = -55.0f;
}

void servo_pid_reset(void)
{
	servo_pid.Actual = 0.0f;
	servo_pid.Out = 0.0f;
	servo_pid.Error0 = 0.0f;
	servo_pid.KpNow = servo_pid.KpMin;
}

void yaw_rate_pid_reset(void)
{
	yaw_rate_pid.Target = 0.0f;
	yaw_rate_pid.Actual = 0.0f;
	yaw_rate_pid.Out = 0.0f;
	yaw_rate_pid.Error0 = 0.0f;
	yaw_rate_pid.Error1 = 0.0f;
	yaw_rate_pid.ErrorInt = 0.0f;
}

void servo_pid_up_date(Servo_PID_t *p)
{
	float abs_error;
	float error_ratio;
	float kp_min;
	float kp_max;

	p->Error0 = p->Target - p->Actual;

	// 菜单误设 KpMax < KpMin 时，以 KpMin 为上限，避免出现负的动态增益区间。
	kp_min = p->KpMin;
	kp_max = p->KpMax;
	if(kp_max < kp_min)
	{
		kp_max = kp_min;
	}

	abs_error = servo_pid_absf(p->Error0);
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
	p->Out = p->KpNow * p->Error0;
	p->Out = servo_pid_clampf(p->Out, p->OutMin, p->OutMax);
}

void yaw_rate_pid_up_date(Yaw_Rate_PID_t *p)
{
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	if(p->Ki != 0.0f)
	{
		p->ErrorInt += p->Error0;
	}
	else
	{
		p->ErrorInt = 0.0f;
	}
	p->ErrorInt = servo_pid_clampf(p->ErrorInt, p->OutMin / 2.0f, p->OutMax / 2.0f);

	p->Out = p->Kp * p->Error0
		+ p->Ki * p->ErrorInt
		+ p->Kd * (p->Error0 - p->Error1);
	p->Out = servo_pid_clampf(p->Out, p->OutMin, p->OutMax);
}






