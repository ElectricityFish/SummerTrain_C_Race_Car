#include "zf_common_headfile.h"
#include "Control.h"
#include "Image_Process.h"
#include "ServoMotor.h"

PID_t servo_pid;
bool servo_control_enabled;

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

    // PID 的 Target/Actual 单位均为图像列坐标，Out 的单位为上层逻辑转角（度）。
    servo_pid.Target = MT9V03X_W / 2.0f + SERVO_CONTROL_IMAGE_CENTER_OFFSET;
    servo_pid.Kp = 5.0f;
    servo_pid.Ki = 0.0f;
    servo_pid.Kd = 2.00f;
    servo_pid.OutMax = SERVO_CONTROL_OUTPUT_LIMIT;
    servo_pid.OutMin = -SERVO_CONTROL_OUTPUT_LIMIT;
    servo_pid.OutOffset = 0.0f;

    servo_control_reset_pid();
    servo_control_enabled = true;
    servomotor_set_angle(SERVOMOTOR_CONTROL_CENTER_ANGLE);
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

    if(!servo_control_enabled)
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
