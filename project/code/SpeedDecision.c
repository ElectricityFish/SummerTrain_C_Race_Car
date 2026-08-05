#include "SpeedDecision.h"

#include "SpeedControl.h"

#define STEERING_DIFFERENTIAL_DEADBAND_MAX   (55.0f)
#define STEERING_DIFFERENTIAL_GAIN_MAX       (1.00f)
#define STEERING_DIFFERENTIAL_RATIO_MAX      (0.80f)
#define STEERING_DIFFERENTIAL_OUTER_SCALE_MAX (1.00f)

volatile uint8 steering_differential_enabled;
volatile float steering_differential_deadband;
volatile float steering_differential_gain;
volatile float steering_differential_max_ratio;
volatile float steering_differential_outer_scale;

volatile int16 speed_decision_base_target_pulse;
volatile uint8 speed_decision_differential_active;
volatile float speed_decision_steering_demand;
volatile float speed_decision_effective_demand;
volatile float speed_decision_differential_ratio;
volatile float speed_decision_inner_reduce_pulse;
volatile float speed_decision_outer_plus_pulse;
volatile int16 speed_decision_left_target_pulse;
volatile int16 speed_decision_right_target_pulse;

static float speed_decision_absf(float value)
{
	return (value >= 0.0f) ? value : -value;
}

static float speed_decision_limitf(float value, float lower, float upper)
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

static int16 speed_decision_limit_base_target(int16 target)
{
	if(target < 0)
	{
		return 0;
	}
	if(target > SPEED_CONTROL_RUN_TARGET_MAX)
	{
		return SPEED_CONTROL_RUN_TARGET_MAX;
	}
	return target;
}

static int16 speed_decision_float_to_int16(float value)
{
	return (value >= 0.0f) ? (int16)(value + 0.5f) : (int16)(value - 0.5f);
}

void speed_decision_init(void)
{
	steering_differential_enabled = 1U;
	steering_differential_deadband = 3.0f;
	steering_differential_gain = 0.235f;
	steering_differential_max_ratio = 0.80f;
	steering_differential_outer_scale = 0.45f;
	speed_decision_stop();
}

void speed_decision_apply(int16 base_target, float signed_steering_demand)
{
	float base_target_float;
	float deadband;
	float gain;
	float max_ratio;
	float outer_scale;
	float scaled_demand;
	float inner_target;
	float outer_target;
	float left_target;
	float right_target;

	speed_decision_base_target_pulse = speed_decision_limit_base_target(base_target);
	base_target_float = (float)speed_decision_base_target_pulse;
	speed_decision_steering_demand = signed_steering_demand;
	speed_decision_differential_active = 0U;
	speed_decision_effective_demand = 0.0f;
	speed_decision_differential_ratio = 0.0f;
	speed_decision_inner_reduce_pulse = 0.0f;
	speed_decision_outer_plus_pulse = 0.0f;
	left_target = base_target_float;
	right_target = base_target_float;

	deadband = speed_decision_limitf(
		steering_differential_deadband,
		0.0f,
		STEERING_DIFFERENTIAL_DEADBAND_MAX);
	gain = speed_decision_limitf(
		steering_differential_gain,
		0.0f,
		STEERING_DIFFERENTIAL_GAIN_MAX);
	max_ratio = speed_decision_limitf(
		steering_differential_max_ratio,
		0.0f,
		STEERING_DIFFERENTIAL_RATIO_MAX);
	outer_scale = speed_decision_limitf(
		steering_differential_outer_scale,
		0.0f,
		STEERING_DIFFERENTIAL_OUTER_SCALE_MAX);
	steering_differential_deadband = deadband;
	steering_differential_gain = gain;
	steering_differential_max_ratio = max_ratio;
	steering_differential_outer_scale = outer_scale;

	speed_decision_effective_demand = speed_decision_absf(signed_steering_demand)
		- deadband;
	if(speed_decision_effective_demand < 0.0f)
	{
		speed_decision_effective_demand = 0.0f;
	}

	if((steering_differential_enabled != 0U)
		&& (speed_decision_base_target_pulse > 0)
		&& (speed_decision_effective_demand > 0.0f)
		&& (gain > 0.0f)
		&& (max_ratio > 0.0f))
	{
		scaled_demand = gain * speed_decision_effective_demand;
		speed_decision_differential_ratio = scaled_demand
			/ (2.0f + scaled_demand);
		speed_decision_differential_ratio = speed_decision_limitf(
			speed_decision_differential_ratio,
			0.0f,
			max_ratio);
		inner_target = base_target_float
			* (1.0f - speed_decision_differential_ratio);
		speed_decision_inner_reduce_pulse = base_target_float - inner_target;
		speed_decision_outer_plus_pulse = speed_decision_inner_reduce_pulse
			* outer_scale;
		outer_target = base_target_float + speed_decision_outer_plus_pulse;

		// 外轮加速量按内轮减速量的 OuterScale 比例追加，0 可退回只减内轮。
		// 正转向需求表示左转，左轮为内轮；负转向需求表示右转，右轮为内轮。
		if(signed_steering_demand > 0.0f)
		{
			left_target = inner_target;
			right_target = outer_target;
		}
		else
		{
			left_target = outer_target;
			right_target = inner_target;
		}
		speed_decision_differential_active = 1U;
	}

	left_target = speed_decision_limitf(
		left_target,
		0.0f,
		(float)SPEED_CONTROL_TARGET_ABS_MAX);
	right_target = speed_decision_limitf(
		right_target,
		0.0f,
		(float)SPEED_CONTROL_TARGET_ABS_MAX);
	speed_decision_left_target_pulse = speed_decision_float_to_int16(left_target);
	speed_decision_right_target_pulse = speed_decision_float_to_int16(right_target);

	speed_control_set_closed_loop_target(
		speed_decision_left_target_pulse,
		speed_decision_right_target_pulse);
}

void speed_decision_stop(void)
{
	speed_decision_base_target_pulse = 0;
	speed_decision_differential_active = 0U;
	speed_decision_steering_demand = 0.0f;
	speed_decision_effective_demand = 0.0f;
	speed_decision_differential_ratio = 0.0f;
	speed_decision_inner_reduce_pulse = 0.0f;
	speed_decision_outer_plus_pulse = 0.0f;
	speed_decision_left_target_pulse = 0;
	speed_decision_right_target_pulse = 0;
}
