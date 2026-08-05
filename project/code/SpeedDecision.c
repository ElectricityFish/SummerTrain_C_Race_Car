#include "SpeedDecision.h"

#include "ServoMotor.h"
#include "SpeedControl.h"

#define ACKERMANN_WHEELBASE_MM             (200.0f)
#define ACKERMANN_REAR_TRACK_MM            (154.75f)
#define ACKERMANN_GAIN                     (1.0f)
#define ACKERMANN_DEG_TO_RAD               (0.0174532925f)
#define DIFFERENTIAL_ERROR_DEADZONE_PX     (10.0f)
#define EMPIRICAL_ERROR_FULL_PX             (15.0f)
#define EMPIRICAL_REDUCE_RATIO_MAX         (2.00f)
#define EMPIRICAL_PLUS_RATIO_MAX           (0.30f)
#define DIFFERENTIAL_OUTER_MAX_RATIO_MIN   (1.00f)
#define DIFFERENTIAL_OUTER_MAX_RATIO_MAX   (2.00f)

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
volatile uint8 empirical_differential_enabled;
volatile float empirical_reduce_max_ratio;
volatile float empirical_plus_max_ratio;
volatile float differential_inner_min_ratio;
volatile float differential_outer_max_ratio;
volatile int16 speed_decision_base_target_pulse;
volatile uint8 speed_decision_differential_active;
volatile float ackermann_servo_delta_deg;
volatile float ackermann_wheel_delta_deg;
volatile float ackermann_differential_ratio;
volatile float ackermann_pixel_error_abs;
volatile float empirical_error_ratio;
volatile float empirical_inner_reduce_pulse;
volatile float empirical_outer_plus_pulse;
volatile float speed_decision_left_raw_target_pulse;
volatile float speed_decision_right_raw_target_pulse;
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

static float speed_decision_get_error_ratio(float error_abs)
{
	float transition_ratio;

	if(error_abs <= DIFFERENTIAL_ERROR_DEADZONE_PX)
	{
		return 0.0f;
	}
	if(EMPIRICAL_ERROR_FULL_PX <= DIFFERENTIAL_ERROR_DEADZONE_PX)
	{
		return 1.0f;
	}

	transition_ratio = speed_decision_limitf(
		(error_abs - DIFFERENTIAL_ERROR_DEADZONE_PX)
			/ (EMPIRICAL_ERROR_FULL_PX - DIFFERENTIAL_ERROR_DEADZONE_PX),
		0.0f,
		1.0f);

	// Smoothstep：15 和 20 像素两端斜率均为 0，减小差速介入/满幅时的突变。
	return transition_ratio * transition_ratio * (3.0f - 2.0f * transition_ratio);
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
	ackermann_enabled = 0U;				//先将阿克曼差速关闭
	empirical_differential_enabled = 1U;
	empirical_reduce_max_ratio = 0.9f;
	empirical_plus_max_ratio = 0.20f;
	differential_inner_min_ratio = -1.00f;
	differential_outer_max_ratio = 1.30f;
	speed_decision_stop();
}

void speed_decision_apply(int16 base_target, float steering_error_px)
{
	float base_target_float;
	float wheel_angle_rad;
	float empirical_reduce_ratio;
	float empirical_plus_ratio;
	float inner_min_ratio;
	float inner_min_target;
	float outer_max_ratio;
	float outer_max_target;
	float scale;
	float left_raw_target;
	float right_raw_target;
	float left_target;
	float right_target;

	speed_decision_base_target_pulse = speed_decision_limit_base_target(base_target);
	ackermann_servo_delta_deg = servomotor_control_angle_command
		- SERVOMOTOR_CONTROL_CENTER_ANGLE;
	ackermann_wheel_delta_deg = speed_decision_lookup_wheel_delta(
		ackermann_servo_delta_deg);
	ackermann_pixel_error_abs = speed_decision_absf(steering_error_px);
	base_target_float = (float)speed_decision_base_target_pulse;

	ackermann_differential_ratio = 0.0f;
	empirical_error_ratio = 0.0f;
	empirical_inner_reduce_pulse = 0.0f;
	empirical_outer_plus_pulse = 0.0f;
	speed_decision_differential_active = 0U;

	// 实测直道范围内同时关闭阿克曼和经验差速，直接给左右轮相同目标。
	if((speed_decision_base_target_pulse == 0)
		|| (ackermann_pixel_error_abs <= DIFFERENTIAL_ERROR_DEADZONE_PX)
		|| ((ackermann_enabled == 0U)
			&& (empirical_differential_enabled == 0U)))
	{
		speed_decision_left_raw_target_pulse = base_target_float;
		speed_decision_right_raw_target_pulse = base_target_float;
		speed_decision_left_target_pulse = speed_decision_base_target_pulse;
		speed_decision_right_target_pulse = speed_decision_base_target_pulse;
		speed_control_set_closed_loop_target(
			speed_decision_left_target_pulse,
			speed_decision_right_target_pulse);
		return;
	}

	speed_decision_differential_active = 1U;
	// 15 像素以内完全关闭；15~20 像素使用 Smoothstep 平滑介入，
	// 误差达到 20 像素及以上时经验差速完全生效。
	empirical_error_ratio = speed_decision_get_error_ratio(
		ackermann_pixel_error_abs);
	if(ackermann_enabled != 0U)
	{
		wheel_angle_rad = ackermann_wheel_delta_deg * ACKERMANN_DEG_TO_RAD;
		ackermann_differential_ratio = ACKERMANN_GAIN
			* ACKERMANN_REAR_TRACK_MM
			/ (2.0f * ACKERMANN_WHEELBASE_MM)
			* speed_decision_tan_taylor(wheel_angle_rad);
	}

	// 先保留未经 RunTarget 上限归一化的阿克曼目标，经验层在其上追加线性残差。
	left_raw_target = base_target_float
		* (1.0f - ackermann_differential_ratio);
	right_raw_target = base_target_float
		* (1.0f + ackermann_differential_ratio);

	if(empirical_differential_enabled != 0U)
	{
		empirical_reduce_ratio = speed_decision_limitf(
			empirical_reduce_max_ratio,
			0.0f,
			EMPIRICAL_REDUCE_RATIO_MAX);
		empirical_plus_ratio = speed_decision_limitf(
			empirical_plus_max_ratio,
			0.0f,
			EMPIRICAL_PLUS_RATIO_MAX);
		empirical_inner_reduce_pulse = base_target_float
			* empirical_reduce_ratio
			* empirical_error_ratio;
		empirical_outer_plus_pulse = base_target_float
			* empirical_plus_ratio
			* empirical_error_ratio;

		// Error = Target - Actual：正误差需要左转，左轮是内轮；
		// 负误差需要右转，右轮是内轮。经验差速不再依赖舵机命令或轮角。
		if(steering_error_px > 0.0f)
		{
			left_raw_target -= empirical_inner_reduce_pulse;
			right_raw_target += empirical_outer_plus_pulse;
		}
		else if(steering_error_px < 0.0f)
		{
			right_raw_target -= empirical_inner_reduce_pulse;
			left_raw_target += empirical_outer_plus_pulse;
		}
	}

	speed_decision_left_raw_target_pulse = left_raw_target;
	speed_decision_right_raw_target_pulse = right_raw_target;

	// RunTarget 是分配前的基准速度。外轮允许加速到 RunTarget * OutMax；
	// 只有组合目标超过该上限时才等比例缩放，避免破坏内外轮比例。
	left_target = left_raw_target;
	right_target = right_raw_target;
	outer_max_ratio = speed_decision_limitf(
		differential_outer_max_ratio,
		DIFFERENTIAL_OUTER_MAX_RATIO_MIN,
		DIFFERENTIAL_OUTER_MAX_RATIO_MAX);
	outer_max_target = base_target_float * outer_max_ratio;
	scale = speed_decision_maxf(1.0f, left_target / outer_max_target);
	scale = speed_decision_maxf(scale, right_target / outer_max_target);
	left_target /= scale;
	right_target /= scale;

	// 最终负下限只约束当前转向的内轮；外轮仍保持非负。
	// InnerMin=-1 时，内轮最低允许反转到 -RunTarget。
	inner_min_ratio = speed_decision_limitf(
		differential_inner_min_ratio,
		-1.0f,
		1.0f);
	inner_min_target = base_target_float * inner_min_ratio;
	if(steering_error_px > 0.0f)
	{
		left_target = speed_decision_limitf(
			left_target, inner_min_target, outer_max_target);
		right_target = speed_decision_limitf(
			right_target, 0.0f, outer_max_target);
	}
	else if(steering_error_px < 0.0f)
	{
		right_target = speed_decision_limitf(
			right_target, inner_min_target, outer_max_target);
		left_target = speed_decision_limitf(
			left_target, 0.0f, outer_max_target);
	}
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
	ackermann_servo_delta_deg = 0.0f;
	ackermann_wheel_delta_deg = 0.0f;
	ackermann_differential_ratio = 0.0f;
	ackermann_pixel_error_abs = 0.0f;
	empirical_error_ratio = 0.0f;
	empirical_inner_reduce_pulse = 0.0f;
	empirical_outer_plus_pulse = 0.0f;
	speed_decision_left_raw_target_pulse = 0.0f;
	speed_decision_right_raw_target_pulse = 0.0f;
	speed_decision_left_target_pulse = 0;
	speed_decision_right_target_pulse = 0;
}
