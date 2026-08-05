#ifndef __SPEED_DECISION_H__
#define __SPEED_DECISION_H__

#include "zf_common_typedef.h"

// 舵机输出差速参数：先扣除输出死区，再经非线性饱和公式计算内轮减速比例。
extern volatile uint8 steering_differential_enabled;
extern volatile float steering_differential_deadband;
extern volatile float steering_differential_gain;
extern volatile float steering_differential_max_ratio;
// 外轮加速量 = 内轮减速量 * OuterScale；设为 0 可关闭外轮加速。
extern volatile float steering_differential_outer_scale;

// 最近一次 RUNNING 决策结果，供菜单与串口观察。
extern volatile int16 speed_decision_base_target_pulse;
extern volatile uint8 speed_decision_differential_active;
extern volatile float speed_decision_steering_demand;
extern volatile float speed_decision_effective_demand;
extern volatile float speed_decision_differential_ratio;
extern volatile float speed_decision_inner_reduce_pulse;
extern volatile float speed_decision_outer_plus_pulse;
extern volatile int16 speed_decision_left_target_pulse;
extern volatile int16 speed_decision_right_target_pulse;

void speed_decision_init(void);

// 仅由 RUNNING 的 10 ms 速度链调用：有符号舵机控制需求同时决定差速方向和强度。
// 正值表示左转，负值表示右转；内轮减速，外轮可按 OuterScale 比例加速。
void speed_decision_apply(int16 base_target, float signed_steering_demand);

// 离开 RUNNING 时清除差速内部状态；零速目标由上层状态机明确下发。
void speed_decision_stop(void);

#endif
