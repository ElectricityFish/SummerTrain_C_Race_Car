#include "SpeedPlanner.h"

#include "Image_Process.h"
#include "ServoMotor.h"
#include "SpeedDecision.h"

#define SPEED_PLANNER_NEAR_ROW                 (96U)
#define SPEED_PLANNER_MIDDLE_ROW               (72U)
#define SPEED_PLANNER_FAR_ROW                  (48U)
#define SPEED_PLANNER_TARGET_MAX               SPEED_DECISION_RUNNING_BASE_TARGET_PULSE

volatile Speed_Planner_Config_t speed_planner_config;
volatile uint16 speed_planner_bend_px;
volatile float speed_planner_curve;
volatile int16 speed_planner_raw_target_pulse;
volatile int16 speed_planner_target_pulse;

static volatile uint8 speed_planner_running;

static float speed_planner_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int16 speed_planner_absi16(int16 value)
{
    return (value >= 0) ? value : (int16)-value;
}

static float speed_planner_limitf(float value, float lower, float upper)
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

static int16 speed_planner_limit_target(int16 target)
{
    if(target < 0)
    {
        return 0;
    }
    if(target > SPEED_PLANNER_TARGET_MAX)
    {
        return SPEED_PLANNER_TARGET_MAX;
    }
    return target;
}

static int16 speed_planner_float_to_int16(float value)
{
    return (int16)(value + 0.5f);
}

static int16 speed_planner_get_safe_target(void)
{
    return speed_planner_limit_target(speed_planner_config.safe_target);
}

static int16 speed_planner_get_straight_target(void)
{
    return speed_planner_limit_target(speed_planner_config.straight_target);
}

static int16 speed_planner_limit_step(int16 step)
{
    return (step > 0) ? step : 1;
}

void speed_planner_init(void)
{
    speed_planner_config.enabled = 1U;
    speed_planner_config.start_target = 200;
    speed_planner_config.straight_target = 220;
    speed_planner_config.curve_min_target = 180;
    speed_planner_config.safe_target = 180;
    speed_planner_config.bend_full_px = 28.0f;
    speed_planner_config.steer_full_deg = 20.0f;
    speed_planner_config.steer_weight = 0.30f;
    speed_planner_config.curve_filter_current = 45U;
    speed_planner_config.up_step_per_10ms = 1;
    speed_planner_config.down_step_per_10ms = 4;
    speed_planner_config.image_safe_age_ms = 120U;
    speed_planner_config.image_stop_age_ms = 300U;

    speed_planner_bend_px = 0U;
    speed_planner_curve = 0.0f;
    speed_planner_raw_target_pulse = 0;
    speed_planner_target_pulse = 0;
    speed_planner_running = 0U;
}

void speed_planner_start(void)
{
    int16 start_target = speed_planner_limit_target(speed_planner_config.start_target);

    speed_planner_running = 1U;
    speed_planner_bend_px = 0U;
    speed_planner_curve = 0.0f;
    speed_planner_raw_target_pulse = start_target;
    speed_planner_target_pulse = start_target;
    speed_decision_set_base_target(start_target);
}

void speed_planner_stop(void)
{
    speed_planner_running = 0U;
    speed_planner_bend_px = 0U;
    speed_planner_curve = 0.0f;
    speed_planner_raw_target_pulse = 0;
    speed_planner_target_pulse = 0;
}

void speed_planner_update_from_image(void)
{
    int16 bend;
    float bend_full;
    float steer_full;
    float curve_raw;
    float filter_current;
    float curve_min;
    float target;

    if(speed_planner_running == 0U)
    {
        return;
    }

    // 三点二阶差分能消除整体平移，更接近赛道本身的弯曲程度。
    bend = (int16)image_mid_line[SPEED_PLANNER_NEAR_ROW]
        - 2 * (int16)image_mid_line[SPEED_PLANNER_MIDDLE_ROW]
        + (int16)image_mid_line[SPEED_PLANNER_FAR_ROW];
    speed_planner_bend_px = (uint16)speed_planner_absi16(bend);

    if(speed_planner_config.enabled == 0U)
    {
        speed_planner_curve = 0.0f;
        speed_planner_raw_target_pulse = speed_planner_get_straight_target();
        return;
    }

    // 单边丢失时中线是补线结果，不再用它估计曲率；保守降至安全速度。
    if((image_process_left_edge_ok == 0U) || (image_process_right_edge_ok == 0U))
    {
        speed_planner_raw_target_pulse = speed_planner_get_safe_target();
        return;
    }

    bend_full = speed_planner_config.bend_full_px;
    steer_full = speed_planner_config.steer_full_deg;
    if(bend_full < 1.0f)
    {
        bend_full = 1.0f;
    }
    if(steer_full < 1.0f)
    {
        steer_full = 1.0f;
    }

    curve_raw = (float)speed_planner_bend_px / bend_full;
    curve_raw += speed_planner_config.steer_weight
        * speed_planner_absf(servomotor_control_angle_command - SERVOMOTOR_CONTROL_CENTER_ANGLE)
        / steer_full;
    curve_raw = speed_planner_limitf(curve_raw, 0.0f, 1.0f);

    filter_current = speed_planner_limitf(
        (float)speed_planner_config.curve_filter_current, 0.0f, 100.0f);
    speed_planner_curve = (curve_raw * filter_current
        + speed_planner_curve * (100.0f - filter_current)) / 100.0f;

    curve_min = (float)speed_planner_limit_target(speed_planner_config.curve_min_target);
    target = (float)speed_planner_get_straight_target()
        - ((float)speed_planner_get_straight_target() - curve_min) * speed_planner_curve;
    speed_planner_raw_target_pulse = speed_planner_limit_target(
        speed_planner_float_to_int16(target));
}

void speed_planner_10ms_task(void)
{
    int16 raw_target;
    int16 step;

    if(speed_planner_running == 0U)
    {
        return;
    }

    raw_target = speed_planner_raw_target_pulse;
    if(speed_planner_config.enabled != 0U)
    {
        if(image_process_frame_age_ms >= speed_planner_config.image_stop_age_ms)
        {
            // 图像长期停滞时立即给零目标，由现有速度 PI 主动刹住车轮。
            speed_planner_target_pulse = 0;
            speed_decision_set_base_target(0);
            return;
        }
        if(image_process_frame_age_ms >= speed_planner_config.image_safe_age_ms)
        {
            raw_target = speed_planner_get_safe_target();
        }
    }

    raw_target = speed_planner_limit_target(raw_target);
    if(raw_target > speed_planner_target_pulse)
    {
        step = speed_planner_limit_step(speed_planner_config.up_step_per_10ms);
        speed_planner_target_pulse += step;
        if(speed_planner_target_pulse > raw_target)
        {
            speed_planner_target_pulse = raw_target;
        }
    }
    else if(raw_target < speed_planner_target_pulse)
    {
        step = speed_planner_limit_step(speed_planner_config.down_step_per_10ms);
        speed_planner_target_pulse -= step;
        if(speed_planner_target_pulse < raw_target)
        {
            speed_planner_target_pulse = raw_target;
        }
    }

    speed_decision_set_base_target(speed_planner_target_pulse);
}
