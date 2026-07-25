#ifndef __ENCODER_H
#define __ENCODER_H

#include "zf_driver_encoder.h"


//由TIM6中断周期更新，主循环和菜单读取时必须保留volatile属性。
extern volatile int16 encoder1;
extern volatile int16 encoder2;


//两路电机编码器均使用正交解码模式：编码器1为B4/B5，编码器2为B6/B7。
void encoder_init(void);

//读取编码器1自上次读取以来的增量脉冲，并自动清零计数器。
//返回值带方向：正负方向由A、B两相信号的接线顺序决定。
int16 encoder_1_get_pulse(void);

//读取编码器2自上次读取以来的增量脉冲，并自动清零计数器。
//返回值带方向：正负方向由A、B两相信号的接线顺序决定。
int16 encoder_2_get_pulse(void);

#endif
