#include "zf_common_headfile.h"
#include "Control.h"
#include "Image_Process.h"
#include "Kfilter.h"
#include "Motor.h"
#include "ServoMotor.h"
#include "FS-A8S.h"


volatile Common_State common_state;
volatile uint8 car_go_command;
volatile uint8 car_protection_reason;
volatile uint8 wireless_control_enabled;

volatile bool servo_control_enabled;

static uint8 car_protection_active_reason;
static uint8 wireless_control_enabled_last;

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

static float control_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void car_state_stop_actuators(void)
{
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
        servo_control_set_enabled(true);
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
        && (car_protection_active_reason == CAR_PROTECTION_REASON_NONE))
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
        && (car_protection_active_reason == CAR_PROTECTION_REASON_NONE))
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
	servo_pid_reset();
	yaw_rate_pid_reset();
}

void control_init(void)
{
    PID_init();

    common_state = COMMON_STATE_IDLE;
    car_go_command = 0U;
    car_protection_reason = CAR_PROTECTION_REASON_NONE;
    wireless_control_enabled = 0U;
    wireless_control_enabled_last = 0U;
    car_protection_active_reason = CAR_PROTECTION_REASON_NONE;

    // 图像外环的 Target/Actual 单位为图像列坐标，Out 为目标横摆角速度（°/s）。
    servo_pid.Target = MT9V03X_W / 2.0f + SERVO_CONTROL_IMAGE_CENTER_OFFSET;

    servo_control_reset_pid();
    servo_control_enabled = true;
    servomotor_set_angle(SERVOMOTOR_CONTROL_CENTER_ANGLE);
    car_state_stop_actuators();
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

    // 图像外环使用 Error = Target - Actual，左转为正，输出目标横摆角速度（°/s）。
    servo_pid.Target = MT9V03X_W / 2.0f + SERVO_CONTROL_IMAGE_CENTER_OFFSET;
    servo_pid.Actual = (float)image_process_get_final_mid();
    servo_pid_up_date(&servo_pid);

	// 横摆角速度内环直接读取 Kfilter 中处理后的反馈值；左转为正、右转为负。
	yaw_rate_pid.Target = servo_pid.Out;
	yaw_rate_pid.Actual = filtered_yaw_rate;
	yaw_rate_pid_up_date(&yaw_rate_pid);

    // 角速度内环输出是相对中位的舵机修正量；底层接口完成安装行程裁剪。
    control_angle = SERVOMOTOR_CONTROL_CENTER_ANGLE
        + SERVO_CONTROL_DIRECTION * yaw_rate_pid.Out;
    servomotor_set_angle(control_angle);
}
