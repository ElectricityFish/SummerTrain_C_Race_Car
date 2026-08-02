#include "Encoder.h"

//左轮编码器：B4/B5，对应 TIM3 的 CH1/CH2。
#define ENCODER_LEFT_TIMER        (TIM3_ENCODER)
#define ENCODER_LEFT_CH1_PIN      (TIM3_ENCODER_CH1_B4)
#define ENCODER_LEFT_CH2_PIN      (TIM3_ENCODER_CH2_B5)

//右轮编码器：B6/B7，对应 TIM4 的 CH1/CH2。
#define ENCODER_RIGHT_TIMER        (TIM4_ENCODER)
#define ENCODER_RIGHT_CH1_PIN      (TIM4_ENCODER_CH1_B6)
#define ENCODER_RIGHT_CH2_PIN      (TIM4_ENCODER_CH2_B7)


volatile int16 encoder_left_pulse = 0;
volatile int16 encoder_right_pulse = 0;


//初始化两路硬件正交编码器，并从零开始计数。
void encoder_init(void)
{
	encoder_quad_init(ENCODER_LEFT_TIMER, ENCODER_LEFT_CH1_PIN, ENCODER_LEFT_CH2_PIN);
	encoder_quad_init(ENCODER_RIGHT_TIMER, ENCODER_RIGHT_CH1_PIN, ENCODER_RIGHT_CH2_PIN);

	encoder_clear_count(ENCODER_LEFT_TIMER);
	encoder_clear_count(ENCODER_RIGHT_TIMER);
}

//读取左轮编码器在两次调用之间产生的脉冲数，读取后立即清零。
int16 encoder_left_get_pulse(void)
{
	int16 pulse = encoder_get_count(ENCODER_LEFT_TIMER);

	encoder_clear_count(ENCODER_LEFT_TIMER);
	return pulse;
}

//读取右轮编码器在两次调用之间产生的脉冲数，读取后立即清零。
int16 encoder_right_get_pulse(void)
{
	int16 pulse = -encoder_get_count(ENCODER_RIGHT_TIMER);	//接线方向相反，取负后前进为正

	encoder_clear_count(ENCODER_RIGHT_TIMER);
	return pulse;
}
