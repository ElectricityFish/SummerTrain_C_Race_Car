#include "zf_common_headfile.h"
#include "Control.h"
#include "Image_Process.h"
#include "Kfilter.h"
#include "MPU6050.h"
#include "Motor.h"
#include "SpeedControl.h"
#include "SpeedDecision.h"
#include "ServoMotor.h"
#include "FS-A8S.h"


volatile Common_State common_state;
volatile uint8 car_go_command;
volatile uint8 car_protection_reason;
volatile uint8 wireless_control_enabled;

Servo_PID_t servo_pid;
volatile bool servo_control_enabled;

static uint8 car_protection_active_reason;
static uint8 wireless_control_enabled_last;
static volatile uint8 car_zebra_pass_count;
static volatile bool car_zebra_straight_active;
static bool car_zebra_event_latched;
static uint8 car_zebra_absent_frames;
static bool car_race_finished_latched;

#define WIRELESS_SWITCH_LOW_MAX             (1250U)
#define WIRELESS_SWITCH_HIGH_MIN            (1750U)
#define WIRELESS_MOTOR_CHANNEL_MIN          (1000U)
#define WIRELESS_MOTOR_NEGATIVE_END         (1480U)
#define WIRELESS_MOTOR_POSITIVE_START       (1520U)
#define WIRELESS_MOTOR_CHANNEL_MAX          (2000U)
#define WIRELESS_MOTOR_MAX_DUTY             (2500)
#define WIRELESS_STEER_CHANNEL_MIN          (1000U)
#define WIRELESS_STEER_LEFT_END             (1485U)
#define WIRELESS_STEER_RIGHT_START          (1515U)
#define WIRELESS_STEER_CHANNEL_MAX          (2000U)
#define CAR_ZEBRA_REARM_ABSENT_FRAMES       (3U)

static float control_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void car_state_stop_actuators(void)
{
	speed_decision_stop();
	speed_control_set_closed_loop_enabled(false);
	speed_control_debug_stop();
    motor_set_duty(0, 0);
    servo_control_set_enabled(false);
    servomotor_disable();
}

static void car_state_hold_zero_speed(void)
{
	// IDLE/PROTECT 保持速度环工作，以零目标主动抑制车轮转动。
	speed_decision_stop();
	speed_control_debug_stop();
	speed_control_set_closed_loop_target(0, 0);
	speed_control_set_closed_loop_enabled(true);
	motor_set_duty(0, 0);
	servo_control_set_enabled(false);
	servomotor_disable();
}

static bool wireless_control_ch5_permitted(void)
{
    return (fs_a8s_channel_data.channel[4] >= WIRELESS_SWITCH_HIGH_MIN);
}

bool wireless_control_actuators_permitted(void)
{
    return (wireless_control_enabled != 0U)
        && fs_a8s_is_online()
        && wireless_control_ch5_permitted();
}

static void car_state_apply(Common_State next_state)
{
    if(common_state == next_state)
    {
        return;
    }

    common_state = next_state;
    if(next_state == COMMON_STATE_RUNNING)
    {
		// 每次从 IDLE 重新发车都开始一场新比赛，并重新统计起点/终点斑马线。
		car_zebra_pass_count = 0U;
		car_zebra_straight_active = false;
		car_zebra_event_latched = false;
		car_zebra_absent_frames = 0U;
		car_race_finished_latched = false;
		speed_control_debug_stop();
		speed_control_set_closed_loop_enabled(true);
		speed_decision_apply(
			speed_running_target_pulse,
			SERVO_CONTROL_DIRECTION * servo_pid.Out);
        servo_control_set_enabled(true);
    }
    else if((next_state == COMMON_STATE_IDLE) || (next_state == COMMON_STATE_PROTECT))
    {
		car_zebra_straight_active = false;
		car_state_hold_zero_speed();
    }
    else
    {
        car_state_stop_actuators();
    }
}

static void car_state_process_base_command(void)
{
    if(car_go_command == 0U)
    {
        // 完赛产生的零速命令被确认后，下一次 RunCmd=1 才会开始一场新比赛。
        car_race_finished_latched = false;
        if(common_state == COMMON_STATE_RUNNING)
        {
            car_state_apply(COMMON_STATE_IDLE);
        }
        else if((common_state == COMMON_STATE_PROTECT)
            && (car_protection_active_reason == CAR_PROTECTION_REASON_NONE))
        {
            // Protect 退出必须由人工把 RunCmd 置 0 确认；不会自动恢复运行。
            car_protection_reason = CAR_PROTECTION_REASON_NONE;
            car_state_apply(COMMON_STATE_IDLE);
        }
    }
    else if((common_state == COMMON_STATE_IDLE)
        && (car_protection_active_reason == CAR_PROTECTION_REASON_NONE)
        && !car_race_finished_latched)
    {
        car_state_apply(COMMON_STATE_RUNNING);
    }
}

static int16 wireless_control_get_motor_duty(uint16 channel_value)
{
    int32 duty;

    if(channel_value <= WIRELESS_MOTOR_CHANNEL_MIN)
    {
        return -WIRELESS_MOTOR_MAX_DUTY;
    }
    if(channel_value < WIRELESS_MOTOR_NEGATIVE_END)
    {
        duty = -((int32)(WIRELESS_MOTOR_NEGATIVE_END - channel_value)
            * WIRELESS_MOTOR_MAX_DUTY
            / (WIRELESS_MOTOR_NEGATIVE_END - WIRELESS_MOTOR_CHANNEL_MIN));
        return (int16)duty;
    }
    if(channel_value <= WIRELESS_MOTOR_POSITIVE_START)
    {
        return 0;
    }
    if(channel_value < WIRELESS_MOTOR_CHANNEL_MAX)
    {
        duty = (int32)(channel_value - WIRELESS_MOTOR_POSITIVE_START)
            * WIRELESS_MOTOR_MAX_DUTY
            / (WIRELESS_MOTOR_CHANNEL_MAX - WIRELESS_MOTOR_POSITIVE_START);
        return (int16)duty;
    }
    return WIRELESS_MOTOR_MAX_DUTY;
}

static float wireless_control_get_steering_angle(uint16 channel_value)
{
    float ratio;

    if(channel_value <= WIRELESS_STEER_CHANNEL_MIN)
    {
        return SERVOMOTOR_CONTROL_LEFT_MAX_ANGLE;
    }
    if(channel_value < WIRELESS_STEER_LEFT_END)
    {
        ratio = (float)(channel_value - WIRELESS_STEER_CHANNEL_MIN)
            / (float)(WIRELESS_STEER_LEFT_END - WIRELESS_STEER_CHANNEL_MIN);
        return SERVOMOTOR_CONTROL_LEFT_MAX_ANGLE
            + ratio * (SERVOMOTOR_CONTROL_CENTER_ANGLE - SERVOMOTOR_CONTROL_LEFT_MAX_ANGLE);
    }
    if(channel_value <= WIRELESS_STEER_RIGHT_START)
    {
        return SERVOMOTOR_CONTROL_CENTER_ANGLE;
    }
    if(channel_value < WIRELESS_STEER_CHANNEL_MAX)
    {
        ratio = (float)(channel_value - WIRELESS_STEER_RIGHT_START)
            / (float)(WIRELESS_STEER_CHANNEL_MAX - WIRELESS_STEER_RIGHT_START);
        return SERVOMOTOR_CONTROL_CENTER_ANGLE
            + ratio * (SERVOMOTOR_CONTROL_RIGHT_MAX_ANGLE - SERVOMOTOR_CONTROL_CENTER_ANGLE);
    }
    return SERVOMOTOR_CONTROL_RIGHT_MAX_ANGLE;
}

static void wireless_control_process_state(void)
{
    uint16 channel_6 = fs_a8s_channel_data.channel[5];

    if(!wireless_control_actuators_permitted())
    {
        car_go_command = 0U;
        car_state_stop_actuators();
        if(common_state != COMMON_STATE_PROTECT)
        {
            car_state_apply(COMMON_STATE_IDLE);
        }
        return;
    }

    if(channel_6 <= WIRELESS_SWITCH_LOW_MAX)
    {
        car_go_command = 0U;
        if(common_state == COMMON_STATE_PLAY)
        {
            car_state_apply(COMMON_STATE_IDLE);
        }
        car_state_process_base_command();
    }
    else if(channel_6 < WIRELESS_SWITCH_HIGH_MIN)
    {
        car_go_command = 1U;
        if(common_state == COMMON_STATE_PLAY)
        {
            car_state_apply(COMMON_STATE_IDLE);
        }
        car_state_process_base_command();
    }
    else if((common_state != COMMON_STATE_PROTECT)
        && (car_protection_active_reason == CAR_PROTECTION_REASON_NONE)
        && !car_race_finished_latched)
    {
        car_go_command = 0U;
        car_state_apply(COMMON_STATE_PLAY);
    }
}

static void car_state_enter_protect(uint8 reason)
{
    car_protection_reason |= reason;
    car_state_apply(COMMON_STATE_PROTECT);
}

static void servo_control_reset_pid(void)
{
	servo_pid.Actual = 0.0f;
	servo_pid.Out = 0.0f;
	servo_pid.Error0 = 0.0f;
	servo_pid.Error1 = 0.0f;
	servo_pid.ErrorInt = 0.0f;
	servo_pid.YawRate = 0.0f;
	servo_pid.KpNow = servo_pid.KpMin;
}

static void car_race_finish(void)
{
    // 正常完赛不是故障：立即进入 IDLE 的零速闭环，并阻止无线运行挡在释放前重新发车。
    car_race_finished_latched = true;
    car_go_command = 0U;
    car_zebra_straight_active = false;
    car_state_apply(COMMON_STATE_IDLE);
}

void car_race_process_zebra(bool zebra_detected)
{
    if(common_state != COMMON_STATE_RUNNING)
    {
        car_zebra_straight_active = false;
        car_zebra_event_latched = false;
        car_zebra_absent_frames = 0U;
        return;
    }

    if(zebra_detected)
    {
        car_zebra_straight_active = true;
        car_zebra_absent_frames = 0U;

        // 斑马线上立即回正，并绕过差速分配，保持左右轮相同目标速度。
        servo_control_reset_pid();
        servomotor_set_angle(SERVOMOTOR_CONTROL_CENTER_ANGLE);
        speed_control_set_closed_loop_target(
            speed_running_target_pulse,
            speed_running_target_pulse);

        if(car_zebra_event_latched)
        {
            return;
        }

        car_zebra_event_latched = true;
        if(car_zebra_pass_count < 2U)
        {
            car_zebra_pass_count++;
        }

        if(car_zebra_pass_count >= 2U)
        {
            car_race_finish();
        }
        return;
    }

    car_zebra_straight_active = false;
    if(car_zebra_event_latched)
    {
        if(car_zebra_absent_frames < CAR_ZEBRA_REARM_ABSENT_FRAMES)
        {
            car_zebra_absent_frames++;
        }
        if(car_zebra_absent_frames >= CAR_ZEBRA_REARM_ABSENT_FRAMES)
        {
            car_zebra_event_latched = false;
            car_zebra_absent_frames = 0U;
        }
    }
}

bool car_race_zebra_straight_is_active(void)
{
    return car_zebra_straight_active;
}

uint8 car_race_get_zebra_pass_count(void)
{
    return car_zebra_pass_count;
}

void control_init(void)
{
    memset(&servo_pid, 0, sizeof(servo_pid));

    common_state = COMMON_STATE_IDLE;
    car_go_command = 0U;
    car_protection_reason = CAR_PROTECTION_REASON_NONE;
    wireless_control_enabled = 0U;
    wireless_control_enabled_last = 0U;
    car_protection_active_reason = CAR_PROTECTION_REASON_NONE;
    car_zebra_pass_count = 0U;
    car_zebra_straight_active = false;
    car_zebra_event_latched = false;
    car_zebra_absent_frames = 0U;
    car_race_finished_latched = false;

    // PID 的 Target/Actual 单位均为图像列坐标，Out 的单位为上层逻辑转角（度）。
    servo_pid.Target = MT9V03X_W / 2.0f + SERVO_CONTROL_IMAGE_CENTER_OFFSET;
	servo_pid.KpMin = 0.35f;
	servo_pid.KpMax = 0.75f;
    servo_pid.ErrorFull = 35.0f;
    servo_pid.Ki = 0.0f;
    servo_pid.Kd = 0.3f;
    // 学长代码的 Kd2=0.25 作用于 gyro_raw*0.01；折算到 deg/s 后约为 0.036，先取 0.04 起调。
    // 左转横摆角速度为正，PID 中的 -Kd2*YawRate 会给出右转修正，形成负反馈。
    servo_pid.Kd2 = 0.05f;
    // PID 不再重复限制舵机行程；最终角度由 servomotor_set_angle() 按安装边界裁剪。
    servo_pid.OutMax = 55.0f;
    servo_pid.OutMin = -55.0f;

    servo_control_reset_pid();
    servo_control_enabled = true;
    servomotor_set_angle(SERVOMOTOR_CONTROL_CENTER_ANGLE);
    car_state_hold_zero_speed();
}

void car_state_command_task(void)
{
    if(wireless_control_enabled != wireless_control_enabled_last)
    {
        wireless_control_enabled_last = wireless_control_enabled;
        car_go_command = 0U;
        if(common_state != COMMON_STATE_PROTECT)
        {
            car_state_apply(COMMON_STATE_IDLE);
        }
    }

    if(wireless_control_enabled != 0U)
    {
        wireless_control_process_state();
        return;
    }

    car_state_process_base_command();
}

void car_protection_check_attitude(void)
{
    // IDLE 下不做保护触发，也不保留上一次运行留下的实时故障状态。
    if(common_state == COMMON_STATE_IDLE)
    {
        car_protection_active_reason &= (uint8)~CAR_PROTECTION_REASON_ATTITUDE;
        return;
    }

    if((control_absf(pitch) > CAR_PROTECTION_ANGLE_LIMIT_DEG)
        || (control_absf(roll) > CAR_PROTECTION_ANGLE_LIMIT_DEG))
    {
        car_protection_active_reason |= CAR_PROTECTION_REASON_ATTITUDE;
        if(common_state == COMMON_STATE_RUNNING || common_state == COMMON_STATE_PLAY)
        {
            car_state_enter_protect(CAR_PROTECTION_REASON_ATTITUDE);
        }
    }
    else
    {
        car_protection_active_reason &= (uint8)~CAR_PROTECTION_REASON_ATTITUDE;
    }
}

void car_protection_trigger_out_of_bounds(void)
{
    // 出界是一次性锁存事件，不加入 active_reason，也不根据后续图像自动解除。
    if(common_state == COMMON_STATE_RUNNING || common_state == COMMON_STATE_PLAY)
    {
        car_state_enter_protect(CAR_PROTECTION_REASON_OUT_OF_BOUNDS);
    }
}

void wireless_control_play_task(void)
{
    int16 motor_duty;

    if((common_state != COMMON_STATE_PLAY) || !wireless_control_actuators_permitted())
    {
        return;
    }

    motor_duty = wireless_control_get_motor_duty(fs_a8s_channel_data.channel[2]);
    motor_set_duty(motor_duty, motor_duty);
    servomotor_set_angle(wireless_control_get_steering_angle(fs_a8s_channel_data.channel[0]));
}

void servo_control_set_enabled(bool enabled)
{
    if(!enabled)
    {
        servo_control_reset_pid();
        servomotor_set_angle(SERVOMOTOR_CONTROL_CENTER_ANGLE);
    }
    else if(!servo_control_enabled)
    {
        // 再次启用时不保留停用前的微分和积分状态，避免舵机突跳。
        servo_control_reset_pid();
    }

    servo_control_enabled = enabled;
}

void servo_control(void)
{
    float control_angle;

    if(!servo_control_enabled || (common_state != COMMON_STATE_RUNNING))
    {
        return;
    }

    if(car_zebra_straight_active)
    {
        servo_control_reset_pid();
        servomotor_set_angle(SERVOMOTOR_CONTROL_CENTER_ANGLE);
        return;
    }

    // servo_pid_up_date 内部使用 Error = Target - Actual，并按 |Error| 动态计算 KpNow。
    // 赛道中线位于图像右侧时，输出为负，配合本车 90 度中位对应右转。
    servo_pid.Target = MT9V03X_W / 2.0f + SERVO_CONTROL_IMAGE_CENTER_OFFSET;
    servo_pid.Actual = (float)image_process_get_final_mid();
    // Z 轴是本车横摆轴；直接使用最近一次采样并换算为 deg/s，不经过姿态解算中的量化。
    servo_pid.YawRate = mpu6050_gyro_transition(mpu6050_gyro_z_data);
    servo_pid_up_date(&servo_pid);

    // PID 输出是相对中位的角度修正量；底层接口需要以 90 度为中位的绝对逻辑角度。
    control_angle = SERVOMOTOR_CONTROL_CENTER_ANGLE
        + SERVO_CONTROL_DIRECTION * servo_pid.Out;
    servomotor_set_angle(control_angle);
}
