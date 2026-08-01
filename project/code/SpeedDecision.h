#ifndef __SPEED_DECISION_H__
#define __SPEED_DECISION_H__

#include "zf_common_typedef.h"

// 正式速度决策的基础目标单位为“10 ms 内的编码器脉冲数”。
#define SPEED_DECISION_RUNNING_BASE_TARGET_PULSE    (220)

// 阿克曼辅助差速开关和强度。0 表示关闭差速；1 表示完整几何模型。
extern volatile uint8 ackermann_enabled;
extern volatile float ackermann_gain;

// 便于菜单/串口观察的最新基础目标与查表结果。
extern volatile int16 speed_decision_base_target_pulse;
extern volatile float ackermann_servo_delta_deg;
extern volatile float ackermann_wheel_delta_deg;

void speed_decision_init(void);

// 设置 RUNNING 或 PLAY 的基础速度。正反转跨零时清 PI 积分，避免反向突跳。
void speed_decision_set_base_target(int16 base_target);

// 退出正式运行时只清决策状态；不改动 IDLE 独立调试的左右目标。
void speed_decision_stop(void);

// 每 10 ms 在速度 PI 之前调用：最终舵机逻辑指令 -> 阿克曼分配 -> 左右目标。
void speed_decision_10ms_task(void);

#endif
