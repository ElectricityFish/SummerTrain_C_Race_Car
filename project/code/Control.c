#include "zf_common_headfile.h"
#include "Control.h"
#include "Image_Process.h"
#include "Kfilter.h"
#include "Motor.h"
#include "ServoMotor.h"


volatile Common_State common_state;
volatile uint8 car_go_command;
volatile uint8 car_protection_reason;

PID_t servo_pid;
volatile bool servo_control_enabled;

static uint8 car_protection_active_reason;

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

static void car_state_enter_protect(uint8 reason)
{
    car_protection_reason |= reason;
    car_state_apply(COMMON_STATE_PROTECT);
}

static void servo_control_reset_pid(void)
{
    servo_pid.Actual = 0.0f;
    servo_pid.Actual1 = 0.0f;
    servo_pid.Out = 0.0f;
    servo_pid.Error0 = 0.0f;
    servo_pid.Error1 = 0.0f;
    servo_pid.ErrorInt = 0.0f;
}

void control_init(void)
{
    memset(&servo_pid, 0, sizeof(servo_pid));

    common_state = COMMON_STATE_IDLE;
    car_go_command = 0U;
    car_protection_reason = CAR_PROTECTION_REASON_NONE;
    car_protection_active_reason = CAR_PROTECTION_REASON_NONE;

    // PID 的 Target/Actual 单位均为图像列坐标，Out 的单位为上层逻辑转角（度）。
    servo_pid.Target = MT9V03X_W / 2.0f + SERVO_CONTROL_IMAGE_CENTER_OFFSET;
    servo_pid.Kp = 2.0f;
    servo_pid.Ki = 0.0f;
    servo_pid.Kd = 1.00f;
    servo_pid.OutMax = SERVO_CONTROL_OUTPUT_LIMIT;
    servo_pid.OutMin = -SERVO_CONTROL_OUTPUT_LIMIT;
    servo_pid.OutOffset = 0.0f;

    servo_control_reset_pid();
    servo_control_enabled = true;
    servomotor_set_angle(SERVOMOTOR_CONTROL_CENTER_ANGLE);
    car_state_stop_actuators();
}

void car_state_command_task(void)
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
        if(common_state == COMMON_STATE_RUNNING)
        {
            car_state_enter_protect(CAR_PROTECTION_REASON_ATTITUDE);
        }
    }
    else
    {
        car_protection_active_reason &= (uint8)~CAR_PROTECTION_REASON_ATTITUDE;
    }
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

    // PID_Update 内部使用 Error = Target - Actual。
    // 赛道中线位于图像右侧时，输出为负，配合本车 90 度中位对应右转。
    servo_pid.Target = MT9V03X_W / 2.0f + SERVO_CONTROL_IMAGE_CENTER_OFFSET;
    servo_pid.Actual = (float)image_process_get_final_mid();
    PID_Update(&servo_pid);

    // PID 输出是相对中位的角度修正量；底层接口需要以 90 度为中位的绝对逻辑角度。
    control_angle = SERVOMOTOR_CONTROL_CENTER_ANGLE
        + SERVO_CONTROL_DIRECTION * (servo_pid.Out + servo_pid.OutOffset);
    servomotor_set_angle(control_angle);
}
