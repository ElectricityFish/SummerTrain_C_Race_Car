#ifndef __SPEED_DECISION_H__
#define __SPEED_DECISION_H__

#include "zf_common_typedef.h"

// 阿克曼差速开关和几何模型强度。Gain=1 表示完整几何比例。
extern volatile uint8 ackermann_enabled;
extern volatile float ackermann_gain;
extern volatile float ackermann_error_off_px;
extern volatile float ackermann_error_full_px;

// 最近一次 RUNNING 决策结果，供菜单与串口观察。
extern volatile int16 speed_decision_base_target_pulse;
extern volatile float ackermann_servo_delta_deg;
extern volatile float ackermann_wheel_delta_deg;
extern volatile float ackermann_differential_ratio;
extern volatile float ackermann_pixel_error_abs;
extern volatile float ackermann_mix;

void speed_decision_init(void);

// 仅由 RUNNING 的 10 ms 速度链调用：基础目标与舵机像素误差
// -> 直道死区/平滑介入 -> 阿克曼分配 -> 左右轮目标。
void speed_decision_apply(int16 base_target, float steering_error_px);

// 离开 RUNNING 时清除差速内部状态；零速目标由上层状态机明确下发。
void speed_decision_stop(void);

#endif
