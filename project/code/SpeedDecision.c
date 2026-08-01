#include "SpeedDecision.h"

#include "ServoMotor.h"
#include "SpeedControl.h"

#define ACKERMANN_WHEELBASE_MM             (200.0f)
#define ACKERMANN_REAR_TRACK_MM            (54.75f)
#define ACKERMANN_GAIN_MIN                 (0.0f)
#define ACKERMANN_GAIN_MAX                 (1.20f)
#define ACKERMANN_DEG_TO_RAD               (0.0174532925f)

typedef struct
{
    float servo_delta_deg;
    float wheel_delta_deg;
} Ackermann_Steer_Table_t;

// 实车标定：输入是“经过舵机限幅后的上层逻辑角度 - 90 度”。
// 前轮等效角由左右轮实测角计算而得。-5 度点采用已确认的右轮 -7.1 度数据。
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

static int8 speed_decision_last_nonzero_direction;

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

static int16 speed_decision_limit_target(int16 target)
{
    if(target < -SPEED_DECISION_RUNNING_BASE_TARGET_PULSE)
    {
        return -SPEED_DECISION_RUNNING_BASE_TARGET_PULSE;
    }
    if(target > SPEED_DECISION_RUNNING_BASE_TARGET_PULSE)
    {
        return SPEED_DECISION_RUNNING_BASE_TARGET_PULSE;
    }
    return target;
}

static int16 speed_decision_float_to_int16(float value)
{
    if(value >= 0.0f)
    {
        return (int16)(value + 0.5f);
    }
    return (int16)(value - 0.5f);
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

    // 到本车最大等效轮角约 34 度时，保留五次项的误差仍远小于 1%。
    return angle_rad * (1.0f + angle_square / 3.0f
        + 2.0f * angle_square * angle_square / 15.0f);
}

static void speed_decision_apply_targets(void)
{
    float wheel_angle_rad;
    float gain;
    float differential_ratio;
    float scale;
    float left_target;
    float right_target;
    int16 base_target = speed_decision_base_target_pulse;

    ackermann_servo_delta_deg = servomotor_control_angle_command
        - SERVOMOTOR_CONTROL_CENTER_ANGLE;
    ackermann_wheel_delta_deg = speed_decision_lookup_wheel_delta(ackermann_servo_delta_deg);

    if((ackermann_enabled == 0U) || (base_target == 0))
    {
        speed_control_set_closed_loop_target(base_target, base_target);
        return;
    }

    gain = speed_decision_limitf(ackermann_gain, ACKERMANN_GAIN_MIN, ACKERMANN_GAIN_MAX);
    wheel_angle_rad = ackermann_wheel_delta_deg * ACKERMANN_DEG_TO_RAD;
    differential_ratio = gain * ACKERMANN_REAR_TRACK_MM
        / (2.0f * ACKERMANN_WHEELBASE_MM)
        * speed_decision_tan_taylor(wheel_angle_rad);

    // 外侧轮不超过基础目标的绝对值，正转与倒车均保持正确的内外轮速度比例。
    scale = speed_decision_maxf(1.0f, speed_decision_absf(1.0f - differential_ratio));
    scale = speed_decision_maxf(scale, speed_decision_absf(1.0f + differential_ratio));
    left_target = (float)base_target * (1.0f - differential_ratio) / scale;
    right_target = (float)base_target * (1.0f + differential_ratio) / scale;
    speed_control_set_closed_loop_target(
        speed_decision_float_to_int16(left_target),
        speed_decision_float_to_int16(right_target));
}

void speed_decision_init(void)
{
    ackermann_enabled = 1U;
    ackermann_gain = 0.5f;
    speed_decision_base_target_pulse = 0;
    ackermann_servo_delta_deg = 0.0f;
    ackermann_wheel_delta_deg = 0.0f;
    speed_decision_last_nonzero_direction = 0;
}

void speed_decision_set_base_target(int16 base_target)
{
    int8 next_direction = 0;

    speed_decision_base_target_pulse = speed_decision_limit_target(base_target);
    if(speed_decision_base_target_pulse > 0)
    {
        next_direction = 1;
    }
    else if(speed_decision_base_target_pulse < 0)
    {
        next_direction = -1;
    }

    if((next_direction != 0)
        && (speed_decision_last_nonzero_direction != 0)
        && (next_direction != speed_decision_last_nonzero_direction))
    {
        // PLAY 正反转跨零时不保留正向或反向的积分，避免电机反向突跳。
        speed_control_reset_pid();
    }

    if(next_direction != 0)
    {
        speed_decision_last_nonzero_direction = next_direction;
    }

    if(speed_control_closed_loop_is_active())
    {
        speed_decision_apply_targets();
    }
}

void speed_decision_stop(void)
{
    speed_decision_base_target_pulse = 0;
    speed_decision_last_nonzero_direction = 0;
    ackermann_servo_delta_deg = 0.0f;
    ackermann_wheel_delta_deg = 0.0f;
}

void speed_decision_10ms_task(void)
{
    if(speed_control_closed_loop_is_active())
    {
        speed_decision_apply_targets();
    }
}
