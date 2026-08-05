#include "SpeedControl.h"

#include <stddef.h>

#define SPEED_CONTROL_DEFAULT_OUT_MAX       (6000)
#define SPEED_CONTROL_OUT_ABS_MAX           (6000)
#define SPEED_CONTROL_INTEGRAL_ABS_MAX      (100000.0f)
#define SPEED_CONTROL_KP_GAIN_SCALE         (10.0f)

volatile Speed_PID_t speed_left_pid;
volatile Speed_PID_t speed_right_pid;
volatile int16 speed_running_target_pulse;

volatile uint8 speed_debug_enabled;
volatile uint8 speed_debug_run;
volatile uint8 speed_debug_left_enabled;
volatile uint8 speed_debug_right_enabled;

static volatile uint8 speed_control_closed_loop_enabled;

static float speed_control_limit_float(float value, float lower, float upper)
{
	if(value < lower)
	{
		return lower;
	}
	if(value > upper)
	{
		return upper;
	}
	return value;
}

static int16 speed_control_limit_out_max(int16 out_max)
{
	if(out_max < 0)
	{
		return 0;
	}
	if(out_max > SPEED_CONTROL_OUT_ABS_MAX)
	{
		return SPEED_CONTROL_OUT_ABS_MAX;
	}
	return out_max;
}

static int16 speed_control_float_to_int16(float value)
{
	return (value >= 0.0f) ? (int16)(value + 0.5f) : (int16)(value - 0.5f);
}

static int16 speed_control_limit_target(int16 target)
{
	if(target < -SPEED_CONTROL_TARGET_ABS_MAX)
	{
		return -SPEED_CONTROL_TARGET_ABS_MAX;
	}
	if(target > SPEED_CONTROL_TARGET_ABS_MAX)
	{
		return SPEED_CONTROL_TARGET_ABS_MAX;
	}
	return target;
}

static void speed_control_reset_pid_output(volatile Speed_PID_t *pid)
{
	if(pid == NULL)
	{
		return;
	}

	pid->Error = 0.0f;
	pid->ErrorInt = 0.0f;
	pid->POut = 0.0f;
	pid->IOut = 0.0f;
	pid->Out = 0;
}

static void speed_control_update_pid(volatile Speed_PID_t *pid)
{
	float proposed_error_int;
	float output;
	int16 out_max;

	if(pid == NULL)
	{
		return;
	}

	out_max = speed_control_limit_out_max(pid->OutMax);
	pid->OutMax = out_max;
	pid->Error = (float)pid->TargetPulse - (float)pid->ActualPulse;
	pid->POut = SPEED_CONTROL_KP_GAIN_SCALE * pid->Kp * pid->Error;

	if(pid->Ki == 0.0f)
	{
		pid->ErrorInt = 0.0f;
	}
	else
	{
		proposed_error_int = speed_control_limit_float(
			pid->ErrorInt + pid->Error,
			-SPEED_CONTROL_INTEGRAL_ABS_MAX,
			SPEED_CONTROL_INTEGRAL_ABS_MAX);
		output = pid->POut + pid->Ki * proposed_error_int;

		// 误差仍在推动输出饱和时暂停积分，避免恢复后产生反向冲击。
		if(!((output > (float)out_max && pid->Error > 0.0f)
			|| (output < -(float)out_max && pid->Error < 0.0f)))
		{
			pid->ErrorInt = proposed_error_int;
		}
	}

	pid->IOut = pid->Ki * pid->ErrorInt;
	output = speed_control_limit_float(
		pid->POut + pid->IOut,
		-(float)out_max,
		(float)out_max);
	pid->Out = speed_control_float_to_int16(output);
}

void speed_control_init(void)
{
	speed_left_pid.TargetPulse = 0;
	speed_left_pid.ActualPulse = 0;
	speed_left_pid.Kp = 4.0f;
	speed_left_pid.Ki = 0.15f;
	speed_left_pid.OutMax = SPEED_CONTROL_DEFAULT_OUT_MAX;
	speed_control_reset_pid_output(&speed_left_pid);

	speed_right_pid.TargetPulse = 0;
	speed_right_pid.ActualPulse = 0;
	speed_right_pid.Kp = 4.05f;
	speed_right_pid.Ki = 0.15f;
	speed_right_pid.OutMax = SPEED_CONTROL_DEFAULT_OUT_MAX;
	speed_control_reset_pid_output(&speed_right_pid);

	speed_running_target_pulse = 350;
	speed_debug_enabled = 0U;
	speed_debug_run = 0U;
	speed_debug_left_enabled = 0U;
	speed_debug_right_enabled = 0U;
	speed_control_closed_loop_enabled = 0U;
}

bool speed_control_debug_is_active(void)
{
	return (speed_debug_enabled != 0U)
		&& (speed_debug_run != 0U)
		&& ((speed_debug_left_enabled != 0U) || (speed_debug_right_enabled != 0U));
}

void speed_control_reset_pid(void)
{
	speed_control_reset_pid_output(&speed_left_pid);
	speed_control_reset_pid_output(&speed_right_pid);
}

void speed_control_set_closed_loop_enabled(bool enabled)
{
	speed_control_closed_loop_enabled = enabled ? 1U : 0U;
	speed_control_reset_pid();
}

void speed_control_set_closed_loop_target(int16 left_target, int16 right_target)
{
	speed_left_pid.TargetPulse = speed_control_limit_target(left_target);
	speed_right_pid.TargetPulse = speed_control_limit_target(right_target);
}

bool speed_control_closed_loop_is_active(void)
{
	return (speed_control_closed_loop_enabled != 0U);
}

void speed_control_10ms_task(int16 left_pulse, int16 right_pulse)
{
	speed_left_pid.ActualPulse = left_pulse;
	speed_right_pid.ActualPulse = right_pulse;

	// IDLE 单轮调试优先于零速保持闭环。
	if(speed_control_debug_is_active())
	{
		if(speed_debug_left_enabled != 0U)
		{
			speed_control_update_pid(&speed_left_pid);
		}
		else
		{
			speed_control_reset_pid_output(&speed_left_pid);
		}

		if(speed_debug_right_enabled != 0U)
		{
			speed_control_update_pid(&speed_right_pid);
		}
		else
		{
			speed_control_reset_pid_output(&speed_right_pid);
		}
		return;
	}

	if(speed_control_closed_loop_is_active())
	{
		speed_control_update_pid(&speed_left_pid);
		speed_control_update_pid(&speed_right_pid);
		return;
	}

	speed_control_reset_pid_output(&speed_left_pid);
	speed_control_reset_pid_output(&speed_right_pid);
}

void speed_control_get_closed_loop_duty(int16 *left_duty, int16 *right_duty)
{
	if(left_duty == NULL || right_duty == NULL)
	{
		return;
	}

	if(!speed_control_closed_loop_is_active())
	{
		*left_duty = 0;
		*right_duty = 0;
		return;
	}

	*left_duty = speed_left_pid.Out;
	*right_duty = speed_right_pid.Out;
}

void speed_control_debug_get_duty(int16 *left_duty, int16 *right_duty)
{
	if(left_duty == NULL || right_duty == NULL)
	{
		return;
	}

	if(!speed_control_debug_is_active())
	{
		*left_duty = 0;
		*right_duty = 0;
		return;
	}

	*left_duty = (speed_debug_left_enabled != 0U) ? speed_left_pid.Out : 0;
	*right_duty = (speed_debug_right_enabled != 0U) ? speed_right_pid.Out : 0;
}

void speed_control_debug_stop(void)
{
	speed_debug_run = 0U;
	speed_control_reset_pid();
}
