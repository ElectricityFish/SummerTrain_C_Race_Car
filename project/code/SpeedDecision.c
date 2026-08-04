#include "SpeedDecision.h"

#include "ServoMotor.h"
#include "SpeedControl.h"

#define ACKERMANN_WHEELBASE_MM             (200.0f)
#define ACKERMANN_REAR_TRACK_MM            (154.75f)
#define ACKERMANN_GAIN_MIN                 (0.0f)
#define ACKERMANN_GAIN_MAX                 (1.20f)
#define ACKERMANN_DEG_TO_RAD               (0.0174532925f)

typedef struct
{
	float servo_delta_deg;
	float wheel_delta_deg;
} Ackermann_Steer_Table_t;

// 实车标定：输入为经过舵机安全限幅后的逻辑角度减去90度，
// 输出为根据左右前轮实测角计算得到的等效前轮转角。
static const Ackermann_Steer_Table_t ackermann_steer_table[] =
{
	{-25.00f, -32.93f}, {-23.75f, -32.77f}, {-20.00f, -25.45f},
	{-15.00f, -19.49f}, {-10.00f, -14.94f}, { -5.00f,  -5.36f},
	{  0.00f,   0.00f}, {  5.00f,   4.75f}, { 10.00f,  10.38f},
	{ 15.00f,  18.00f}, { 20.00f,  25.69f}, { 23.75f,  27.05f},
	{ 25.00f,  33.59f}
};

#define ACKERMANN_STEER_TABLE_COUNT \
	((uint8)(sizeof(ackermann_steer_table) / sizeof(ackermann_steer_table[0])))

volatile uint8 ackermann_enabled;
volatile float ackermann_gain;
volatile int16 speed_decision_base_target_pulse;
volatile float ackermann_servo_delta_deg;
volatile float ackermann_wheel_delta_deg;
volatile float ackermann_differential_ratio;

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

static float speed_decision_maxf(float first, float second)
{
	return (first > second) ? first : second;
}

static int16 speed_decision_limit_base_target(int16 target)
{
	if(target < 0)
	{
		return 0;
	}
	if(target > SPEED_CONTROL_TARGET_ABS_MAX)
	{
		return SPEED_CONTROL_TARGET_ABS_MAX;
	}
	return target;
}

static int16 speed_decision_float_to_int16(float value)
{
	return (value >= 0.0f) ? (int16)(value + 0.5f) : (int16)(value - 0.5f);
}

static float speed_decision_lookup_wheel_delta(float servo_delta_deg)
{
	uint8 index;
	float ratio;

	if(servo_delta_deg <= ackermann_steer_table[0].servo_delta_deg)
	{
		return ackermann_steer_table[0].wheel_delta_deg;
	}

	for(index = 0U; index < (ACKERMANN_STEER_TABLE_COUNT - 1U); index++)
	{
		if(servo_delta_deg <= ackermann_steer_table[index + 1U].servo_delta_deg)
		{
			ratio = (servo_delta_deg - ackermann_steer_table[index].servo_delta_deg)
				/ (ackermann_steer_table[index + 1U].servo_delta_deg
					- ackermann_steer_table[index].servo_delta_deg);
			return ackermann_steer_table[index].wheel_delta_deg
				+ ratio * (ackermann_steer_table[index + 1U].wheel_delta_deg
					- ackermann_steer_table[index].wheel_delta_deg);
		}
	}

	return ackermann_steer_table[ACKERMANN_STEER_TABLE_COUNT - 1U].wheel_delta_deg;
}

static float speed_decision_tan_taylor(float angle_rad)
{
	float angle_square = angle_rad * angle_rad;

	// 最大等效轮角约34度，保留五次项时误差远小于1%。
	return angle_rad * (1.0f + angle_square / 3.0f
		+ 2.0f * angle_square * angle_square / 15.0f);
}

void speed_decision_init(void)
{
	ackermann_enabled = 1U;
	ackermann_gain = 0.95f;
	speed_decision_stop();
}

void speed_decision_apply(int16 base_target)
{
	float wheel_angle_rad;
	float gain;
	float scale;
	float left_target;
	float right_target;

	speed_decision_base_target_pulse = speed_decision_limit_base_target(base_target);
	ackermann_servo_delta_deg = servomotor_control_angle_command
		- SERVOMOTOR_CONTROL_CENTER_ANGLE;
	ackermann_wheel_delta_deg = speed_decision_lookup_wheel_delta(
		ackermann_servo_delta_deg);

	if((ackermann_enabled == 0U) || (speed_decision_base_target_pulse == 0))
	{
		ackermann_differential_ratio = 0.0f;
		speed_control_set_closed_loop_target(
			speed_decision_base_target_pulse,
			speed_decision_base_target_pulse);
		return;
	}

	gain = speed_decision_limitf(ackermann_gain,
		ACKERMANN_GAIN_MIN, ACKERMANN_GAIN_MAX);
	wheel_angle_rad = ackermann_wheel_delta_deg * ACKERMANN_DEG_TO_RAD;
	ackermann_differential_ratio = gain * ACKERMANN_REAR_TRACK_MM
		/ (2.0f * ACKERMANN_WHEELBASE_MM)
		* speed_decision_tan_taylor(wheel_angle_rad);

	// RunTarget 是外轮上限。等比例缩放后保持几何内外轮比，且任一轮都不超出基础目标。
	scale = speed_decision_maxf(1.0f,
		speed_decision_absf(1.0f - ackermann_differential_ratio));
	scale = speed_decision_maxf(scale,
		speed_decision_absf(1.0f + ackermann_differential_ratio));
	left_target = (float)speed_decision_base_target_pulse
		* (1.0f - ackermann_differential_ratio) / scale;
	right_target = (float)speed_decision_base_target_pulse
		* (1.0f + ackermann_differential_ratio) / scale;

	speed_control_set_closed_loop_target(
		speed_decision_float_to_int16(left_target),
		speed_decision_float_to_int16(right_target));
}

void speed_decision_stop(void)
{
	speed_decision_base_target_pulse = 0;
	ackermann_servo_delta_deg = 0.0f;
	ackermann_wheel_delta_deg = 0.0f;
	ackermann_differential_ratio = 0.0f;
}
