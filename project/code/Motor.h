#ifndef __MOTOR_H
#define __MOTOR_H

#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"

//两路DRV8701E电机接口：A1/A3输出PWM，A0/A2输出方向
#define MOTOR_PWM_FREQ                    (17000U)

#define MOTOR_RIGHT_PWM_PIN                (TIM5_PWM_CH2_A1)
#define MOTOR_RIGHT_DIR_PIN                (A0)
#define MOTOR_LEFT_PWM_PIN                 (TIM5_PWM_CH4_A3)
#define MOTOR_LEFT_DIR_PIN                 (A2)

//正占空比时方向引脚的电平。若实车测试某一路反向，只修改对应宏即可。
#define MOTOR_RIGHT_POSITIVE_DIR_LEVEL     (GPIO_HIGH)
#define MOTOR_LEFT_POSITIVE_DIR_LEVEL      (GPIO_HIGH)

//初始化两路17kHz PWM和方向引脚，初始状态为停止
void motor_init(void);

extern volatile int16 motor_left_duty_command;
extern volatile int16 motor_right_duty_command;

//设置左右电机占空比，范围为-10000～10000。
//正负号决定转向，绝对值决定占空比；0为停止，超出范围会自动限幅。
void motor_set_duty(int16 left_duty, int16 right_duty);

#endif
