#include "Encoder.h"

//编码器1：B4/B5，对应TIM3的CH1/CH2。
#define ENCODER_1_TIMER        (TIM3_ENCODER)
#define ENCODER_1_CH1_PIN      (TIM3_ENCODER_CH1_B4)
#define ENCODER_1_CH2_PIN      (TIM3_ENCODER_CH2_B5)

//编码器2：B6/B7，对应TIM4的CH1/CH2。
#define ENCODER_2_TIMER        (TIM4_ENCODER)
#define ENCODER_2_CH1_PIN      (TIM4_ENCODER_CH1_B6)
#define ENCODER_2_CH2_PIN      (TIM4_ENCODER_CH2_B7)


volatile int16 encoder1 = 0;		//左
volatile int16 encoder2 = 0;		//右


//初始化两路硬件正交编码器，并从零开始计数。
void encoder_init(void)
{
	encoder_quad_init(ENCODER_1_TIMER, ENCODER_1_CH1_PIN, ENCODER_1_CH2_PIN);
	encoder_quad_init(ENCODER_2_TIMER, ENCODER_2_CH1_PIN, ENCODER_2_CH2_PIN);

	encoder_clear_count(ENCODER_1_TIMER);
	encoder_clear_count(ENCODER_2_TIMER);
}

//读取编码器1在两次调用之间产生的脉冲数，读取后立即清零，适合定时速度计算。
int16 encoder_1_get_pulse(void)
{
	int16 pulse = encoder_get_count(ENCODER_1_TIMER);

	encoder_clear_count(ENCODER_1_TIMER);
	return pulse;
}

//读取编码器2在两次调用之间产生的脉冲数，读取后立即清零，适合定时速度计算。
int16 encoder_2_get_pulse(void)
{
	int16 pulse = -encoder_get_count(ENCODER_2_TIMER);	//这里有个负号，使轮胎向前转时脉冲为正

	encoder_clear_count(ENCODER_2_TIMER);
	return pulse;
}
