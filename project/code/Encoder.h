#ifndef __ENCODER_H
#define __ENCODER_H

#include "zf_driver_encoder.h"


//由 TIM6 中断每 10 ms 更新。数值为该周期内的原始脉冲增量，前进方向为正。
//后续速度闭环只能使用左右语义，不能再按硬件定时器编号绑定。
extern volatile int16 encoder_left_pulse;
extern volatile int16 encoder_right_pulse;


//两路电机编码器均使用正交解码模式：左编码器为 B4/B5，右编码器为 B6/B7。
void encoder_init(void);

//读取左编码器自上次读取以来的增量脉冲，并自动清零计数器。
//返回值带方向：正负方向由A、B两相信号的接线顺序决定。
int16 encoder_left_get_pulse(void);

//读取右编码器自上次读取以来的增量脉冲，并自动清零计数器。
//返回值带方向：正负方向由A、B两相信号的接线顺序决定。
int16 encoder_right_get_pulse(void);

#endif
