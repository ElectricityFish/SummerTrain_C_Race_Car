#include "SpeedDecision.h"

#include "SpeedControl.h"

#define STEERING_DIFFERENTIAL_DEADBAND_MAX   (55.0f)
#define STEERING_DIFFERENTIAL_GAIN_MAX       (1.00f)
#define STEERING_DIFFERENTIAL_RATIO_MAX      (0.80f)
#define STEERING_DIFFERENTIAL_OUTER_SCALE_MAX (1.00f)

#define SPEED_PLAN_BOOST_MAX                 (130)
#define SPEED_PLAN_PROSPECT_START            (80.0f)
#define SPEED_PLAN_PROSPECT_FULL             (96.0f)
#define SPEED_PLAN_TURN_START                (3.0f)
#define SPEED_PLAN_TURN_FULL                 (20.0f)
#define SPEED_PLAN_ACCEL_STEP                (2)
#define SPEED_PLAN_DECEL_STEP                (10)

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

volatile int16 speed_plan_vision_target;
volatile int16 speed_plan_turn_cap;
volatile int16 speed_plan_raw_target;
volatile int16 speed_plan_final_target;

static uint8 speed_plan_valid;

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
	steering_differential_gain = 0.24f;
	steering_differential_max_ratio = 0.8f;
	steering_differential_outer_scale = 0.2f;
	speed_decision_stop();
}

void speed_decision_start(int16 base_target)
{
	base_target = speed_decision_limit_base_target(base_target);
	speed_plan_vision_target = base_target;
	speed_plan_turn_cap = base_target;
	speed_plan_raw_target = base_target;
	speed_plan_final_target = base_target;
	speed_plan_valid = 1U;
}

void speed_decision_update(
	int16 base_target,
	float filtered_prospect,
	float signed_steering_demand)
{
	int16 plan_max;
	int16 vision_target;
	int16 turn_cap;
	int16 raw_target;
	float normalized;
	float smooth;
	float turn_ratio;
	float steering_abs;

	base_target = speed_decision_limit_base_target(base_target);
	if(base_target <= 0)
	{
		speed_plan_vision_target = 0;
		speed_plan_turn_cap = 0;
		speed_plan_raw_target = 0;
		speed_plan_final_target = 0;
		speed_plan_valid = 1U;
		speed_decision_apply(0, signed_steering_demand);
		return;
	}

	plan_max = base_target + SPEED_PLAN_BOOST_MAX;
	if(plan_max > SPEED_CONTROL_RUN_TARGET_MAX)
	{
		plan_max = SPEED_CONTROL_RUN_TARGET_MAX;
	}

	normalized = (filtered_prospect - SPEED_PLAN_PROSPECT_START)
		/ (SPEED_PLAN_PROSPECT_FULL - SPEED_PLAN_PROSPECT_START);
	normalized = speed_decision_limitf(normalized, 0.0f, 1.0f);
	smooth = normalized * normalized * (3.0f - 2.0f * normalized);
	vision_target = speed_decision_float_to_int16(
		(float)base_target + (float)(plan_max - base_target) * smooth);

	steering_abs = speed_decision_absf(signed_steering_demand);
	turn_ratio = (steering_abs - SPEED_PLAN_TURN_START)
		/ (SPEED_PLAN_TURN_FULL - SPEED_PLAN_TURN_START);
	turn_ratio = speed_decision_limitf(turn_ratio, 0.0f, 1.0f);
	turn_cap = speed_decision_float_to_int16(
		(float)plan_max - (float)(plan_max - base_target) * turn_ratio);
	raw_target = (vision_target < turn_cap) ? vision_target : turn_cap;

	if(speed_plan_valid == 0U)
	{
		speed_plan_final_target = base_target;
		speed_plan_valid = 1U;
	}

	if(raw_target > speed_plan_final_target)
	{
		int16 difference = raw_target - speed_plan_final_target;
		speed_plan_final_target += (difference > SPEED_PLAN_ACCEL_STEP)
			? SPEED_PLAN_ACCEL_STEP
			: difference;
	}
	else if(raw_target < speed_plan_final_target)
	{
		int16 difference = speed_plan_final_target - raw_target;
		speed_plan_final_target -= (difference > SPEED_PLAN_DECEL_STEP)
			? SPEED_PLAN_DECEL_STEP
			: difference;
	}

	speed_plan_vision_target = vision_target;
	speed_plan_turn_cap = turn_cap;
	speed_plan_raw_target = raw_target;
	speed_decision_apply(speed_plan_final_target, signed_steering_demand);
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
	speed_plan_vision_target = 0;
	speed_plan_turn_cap = 0;
	speed_plan_raw_target = 0;
	speed_plan_final_target = 0;
	speed_plan_valid = 0U;
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
