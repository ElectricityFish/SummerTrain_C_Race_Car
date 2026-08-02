#include "Motor.h"

//取得与“正方向”相反的GPIO电平
static uint8 motor_reverse_level(uint8 positive_level)
{
	return (positive_level == GPIO_HIGH) ? GPIO_LOW : GPIO_HIGH;
}

volatile int16 motor_left_duty_command = 0;
volatile int16 motor_right_duty_command = 0;

//设置单路电机：符号控制方向，绝对值控制PWM占空比
static void motor_set_single(
	int16 duty,
	pwm_channel_enum pwm_pin,
	gpio_pin_enum dir_pin,
	uint8 positive_dir_level)
{
	int32 signed_duty = duty;
	uint32 pwm_duty;

	//先限幅，随后再取绝对值，避免int16最小值取反溢出
	if(signed_duty > PWM_DUTY_MAX)
	{
		signed_duty = PWM_DUTY_MAX;
	}
	if(signed_duty < -PWM_DUTY_MAX)
	{
		signed_duty = -PWM_DUTY_MAX;
	}

	if(signed_duty > 0)
	{
		gpio_set_level(dir_pin, positive_dir_level);
		pwm_duty = (uint32)signed_duty;
	}
	else if(signed_duty < 0)
	{
		gpio_set_level(dir_pin, motor_reverse_level(positive_dir_level));
		pwm_duty = (uint32)(-signed_duty);
	}
	else
	{
		//PWM为0时电机停止，方向脚置低使上电初始状态明确
		gpio_set_level(dir_pin, GPIO_LOW);
		pwm_duty = 0;
	}

	pwm_set_duty(pwm_pin, pwm_duty);
}

//初始化两路DRV8701E电机接口
void motor_init(void)
{
	//方向引脚使用普通推挽输出，低电平作为初始方向
	gpio_init(MOTOR_RIGHT_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
	gpio_init(MOTOR_LEFT_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

	//A1和A3同属TIM5，频率必须一致，初始化时PWM占空比为0
	pwm_init(MOTOR_RIGHT_PWM_PIN, MOTOR_PWM_FREQ, 0);
	pwm_init(MOTOR_LEFT_PWM_PIN, MOTOR_PWM_FREQ, 0);
}

//调用接口统一使用“左、右”顺序，硬件通道映射只保留在本模块内。
void motor_set_duty(int16 left_duty, int16 right_duty)
{
	motor_left_duty_command = left_duty;
	motor_right_duty_command = right_duty;

	motor_set_single(
		right_duty,
		MOTOR_RIGHT_PWM_PIN,
		MOTOR_RIGHT_DIR_PIN,
		MOTOR_RIGHT_POSITIVE_DIR_LEVEL);

	motor_set_single(
		left_duty,
		MOTOR_LEFT_PWM_PIN,
		MOTOR_LEFT_DIR_PIN,
		MOTOR_LEFT_POSITIVE_DIR_LEVEL);
}
