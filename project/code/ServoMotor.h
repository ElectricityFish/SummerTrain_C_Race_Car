#ifndef __SERVOMOTOR_H
#define __SERVOMOTOR_H

#include "zf_driver_pwm.h"

//SU400使用A15，对应TIM2通道1
#define SERVOMOTOR_PWM_PIN             (TIM2_PWM_CH1_A15)

//当前使用285HzPWM响应更快
#define SERVOMOTOR_PWM_FREQ            (285U)

//SU400官方标注中位脉宽为1520us，先使用中位前后各500us的保守范围
#define SERVOMOTOR_MIN_PULSE_US        (1020U)
#define SERVOMOTOR_MID_PULSE_US        (1520U)
#define SERVOMOTOR_MAX_PULSE_US        (2020U)

//底层角度到PWM脉宽的完整换算范围，仅供驱动内部使用
#define SERVOMOTOR_MIN_ANGLE           (0.0f)
#define SERVOMOTOR_MID_ANGLE           (90.0f)
#define SERVOMOTOR_MAX_ANGLE           (180.0f)

//实车安全行程：实际输入72.5度为右极限，97.5度为直行，122.5度为左极限。
//继续增大转角会使前轮卡到底盘，因此上层统一限制在中心前后25度。
//为了让上层仍然以90度表示直行，驱动内部会自动给输入角度增加7.5度
#define SERVOMOTOR_ANGLE_OFFSET                  (7.5f)
#define SERVOMOTOR_CONTROL_RIGHT_MAX_ANGLE       (65.0f)
#define SERVOMOTOR_CONTROL_CENTER_ANGLE          (90.0f)
#define SERVOMOTOR_CONTROL_LEFT_MAX_ANGLE        (115.0f)

//初始化舵机PWM，并让车轮转到上层定义的90度直行位置
void servomotor_init(void);

// 最近一次经过机械安全边界限幅后实际下发的上层逻辑角度，供状态观察与调试。
extern volatile float servomotor_control_angle_command;

//控制车辆转向角度：
//65度为向右最大，90度为直行，115度为向左最大，超出范围会自动限制
void servomotor_set_angle(float angle);

// 停止输出舵机控制脉冲，使舵机不再保持当前位置。
void servomotor_disable(void);

#endif
