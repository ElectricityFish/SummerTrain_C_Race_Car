#ifndef __SPEED_DECISION_H__
#define __SPEED_DECISION_H__

#include "zf_common_typedef.h"

// 阿克曼只保留开关，几何增益固定为 1.0。
extern volatile uint8 ackermann_enabled;

// 经验残差参数均为归一化比例：满舵时按 RunTarget 的相应比例修正原始目标。
extern volatile uint8 empirical_differential_enabled;
extern volatile float empirical_reduce_max_ratio;
extern volatile float empirical_plus_max_ratio;
extern volatile float differential_inner_min_ratio;
extern volatile float differential_outer_max_ratio;

// 最近一次 RUNNING 决策结果，供菜单与串口观察。
extern volatile int16 speed_decision_base_target_pulse;
extern volatile uint8 speed_decision_differential_active;
extern volatile float ackermann_servo_delta_deg;
extern volatile float ackermann_wheel_delta_deg;
extern volatile float ackermann_differential_ratio;
extern volatile float ackermann_pixel_error_abs;
extern volatile float empirical_steer_ratio;
extern volatile float empirical_inner_reduce_pulse;
extern volatile float empirical_outer_plus_pulse;
extern volatile float speed_decision_left_raw_target_pulse;
extern volatile float speed_decision_right_raw_target_pulse;
extern volatile int16 speed_decision_left_target_pulse;
extern volatile int16 speed_decision_right_target_pulse;

void speed_decision_init(void);

// 仅由 RUNNING 的 10 ms 速度链调用：RunTarget 与舵机像素误差
// -> 10 像素硬死区 -> 阿克曼几何差速 -> 经验残差 -> 内外轮目标限幅。
void speed_decision_apply(int16 base_target, float steering_error_px);

// 离开 RUNNING 时清除差速内部状态；零速目标由上层状态机明确下发。
void speed_decision_stop(void);

#endif
