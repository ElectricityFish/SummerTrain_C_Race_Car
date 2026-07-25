#ifndef __SERVOMOTOR_H
#define __SERVOMOTOR_H

#include "zf_driver_pwm.h"

//SU400使用A15，对应TIM2通道1
#define SERVOMOTOR_PWM_PIN             (TIM2_PWM_CH1_A15)

//当前使用50Hz普通PWM控制；如需切换数字舵机高速模式，可单独改为285Hz测试
#define SERVOMOTOR_PWM_FREQ            (50U)

//SU400官方标注中位脉宽为1520us，先使用中位前后各500us的保守范围
#define SERVOMOTOR_MIN_PULSE_US        (1020U)
#define SERVOMOTOR_MID_PULSE_US        (1520U)
#define SERVOMOTOR_MAX_PULSE_US        (2020U)

//底层角度到PWM脉宽的完整换算范围，仅供驱动内部使用
#define SERVOMOTOR_MIN_ANGLE           (0.0f)
#define SERVOMOTOR_MID_ANGLE           (90.0f)
#define SERVOMOTOR_MAX_ANGLE           (180.0f)

//实车标定结果：实际输入70度为右极限，97.5度为直行，125度为左极限
//为了让上层仍然以90度表示直行，驱动内部会自动给输入角度增加7.5度
#define SERVOMOTOR_ANGLE_OFFSET                  (7.5f)
#define SERVOMOTOR_CONTROL_RIGHT_MAX_ANGLE       (62.5f)
#define SERVOMOTOR_CONTROL_CENTER_ANGLE          (90.0f)
#define SERVOMOTOR_CONTROL_LEFT_MAX_ANGLE        (117.5f)

//初始化舵机PWM，并让车轮转到上层定义的90度直行位置
void servomotor_init(void);

//控制车辆转向角度：
//62.5度为向右最大，90度为直行，117.5度为向左最大，超出范围会自动限制
void servomotor_set_angle(float angle);

#endif
