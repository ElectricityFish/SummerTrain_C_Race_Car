#ifndef __SPEED_DECISION_H__
#define __SPEED_DECISION_H__

#include "zf_common_typedef.h"

// 阿克曼差速开关和几何模型强度。Gain=1 表示完整几何比例。
extern volatile uint8 ackermann_enabled;
extern volatile float ackermann_gain;

// 最近一次 RUNNING 决策结果，供菜单与串口观察。
extern volatile int16 speed_decision_base_target_pulse;
extern volatile float ackermann_servo_delta_deg;
extern volatile float ackermann_wheel_delta_deg;
extern volatile float ackermann_differential_ratio;

void speed_decision_init(void);

// 仅由 RUNNING 的 10 ms 速度链调用：基础目标 -> 阿克曼分配 -> 左右轮目标。
void speed_decision_apply(int16 base_target);

// 离开 RUNNING 时清除差速内部状态；零速目标由上层状态机明确下发。
void speed_decision_stop(void);

#endif
