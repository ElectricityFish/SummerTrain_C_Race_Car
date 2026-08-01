#include "ServoMotor.h"

volatile float servomotor_control_angle_command = SERVOMOTOR_CONTROL_CENTER_ANGLE;

//把角度换算成逐飞PWM库需要的万分比占空比
static uint32 servomotor_angle_to_duty(float angle)
{
	float pulse_us_float;
	uint32 pulse_us;
	uint32 period_us;

	if(angle < SERVOMOTOR_MIN_ANGLE)
	{
		angle = SERVOMOTOR_MIN_ANGLE;
	}
	if(angle > SERVOMOTOR_MAX_ANGLE)
	{
		angle = SERVOMOTOR_MAX_ANGLE;
	}

	//以90度和1520us为中点分段换算，便于以后分别标定左右机械极限
	if(angle <= SERVOMOTOR_MID_ANGLE)
	{
		pulse_us_float = SERVOMOTOR_MIN_PULSE_US
			+ (angle - SERVOMOTOR_MIN_ANGLE)
			* (SERVOMOTOR_MID_PULSE_US - SERVOMOTOR_MIN_PULSE_US)
			/ (SERVOMOTOR_MID_ANGLE - SERVOMOTOR_MIN_ANGLE);
	}
	else
	{
		pulse_us_float = SERVOMOTOR_MID_PULSE_US
			+ (angle - SERVOMOTOR_MID_ANGLE)
			* (SERVOMOTOR_MAX_PULSE_US - SERVOMOTOR_MID_PULSE_US)
			/ (SERVOMOTOR_MAX_ANGLE - SERVOMOTOR_MID_ANGLE);
	}
	pulse_us = (uint32)(pulse_us_float + 0.5f);

	period_us = 1000000U / SERVOMOTOR_PWM_FREQ;
	return (pulse_us * PWM_DUTY_MAX + period_us / 2U) / period_us;
}

//把上层逻辑角度限制在实车安全范围内，再加上7.5度机械安装偏移
//对应关系：62.5 -> 实际70度，90 -> 实际97.5度，117.5 -> 实际125度
static float servomotor_calibrate_angle(float control_angle)
{
	if(control_angle < SERVOMOTOR_CONTROL_RIGHT_MAX_ANGLE)
	{
		control_angle = SERVOMOTOR_CONTROL_RIGHT_MAX_ANGLE;
	}
	if(control_angle > SERVOMOTOR_CONTROL_LEFT_MAX_ANGLE)
	{
		control_angle = SERVOMOTOR_CONTROL_LEFT_MAX_ANGLE;
	}

	return control_angle + SERVOMOTOR_ANGLE_OFFSET;
}

//舵机初始化：A15输出PWM，上层以90度作为统一的直行中值
void servomotor_init(void)
{
	servomotor_control_angle_command = SERVOMOTOR_CONTROL_CENTER_ANGLE;
	pwm_init(
		SERVOMOTOR_PWM_PIN,
		SERVOMOTOR_PWM_FREQ,
		servomotor_angle_to_duty(
			servomotor_calibrate_angle(SERVOMOTOR_CONTROL_CENTER_ANGLE)));
}

//控制车辆转向；输入使用上层逻辑角度，底层自动完成限幅和7.5度偏移校正
void servomotor_set_angle(float angle)
{
	float calibrated_angle = servomotor_calibrate_angle(angle);
	servomotor_control_angle_command = calibrated_angle - SERVOMOTOR_ANGLE_OFFSET;

	pwm_set_duty(
		SERVOMOTOR_PWM_PIN,
		servomotor_angle_to_duty(calibrated_angle));
}

// 将 PWM 占空比清零，停止向舵机发送控制脉冲。
void servomotor_disable(void)
{
	pwm_set_duty(SERVOMOTOR_PWM_PIN, 0U);
}
