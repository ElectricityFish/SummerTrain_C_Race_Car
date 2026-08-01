#ifndef __SPEED_CONTROL_H__
#define __SPEED_CONTROL_H__

#include "zf_common_typedef.h"

// 速度环的单位统一为“10 ms 内的编码器脉冲数”。独立调试与正式运行共用
// 同一组左右 PI；正式速度决策和后续阿克曼分配只需更新左右 TargetPulse。
typedef struct
{
    int16 TargetPulse;
    int16 ActualPulse;
    float Kp;                 // 菜单系数，内部计算时会乘以 10
    float Ki;                 // 菜单系数，内部计算时会乘以 10
    float Error;
    float ErrorInt;
    float POut;
    float IOut;
    int16 Out;
    int16 OutMax;
} Speed_PID_t;

extern volatile Speed_PID_t speed_left_pid;
extern volatile Speed_PID_t speed_right_pid;

// 临时单轮调试开关。Enable 表示进入调试保护逻辑，Run 才允许输出电机 PWM。
extern volatile uint8 speed_debug_enabled;
extern volatile uint8 speed_debug_run;
extern volatile uint8 speed_debug_left_enabled;
extern volatile uint8 speed_debug_right_enabled;

// 初始化左右独立 PI 参数和调试安全开关。上电后默认不会输出电机 PWM。
void speed_control_init(void);

// 每 10 ms 在读取最新左右原始编码器脉冲后调用。该函数不直接写 PWM。
void speed_control_10ms_task(int16 left_pulse, int16 right_pulse);

// 正式速度环的启停与目标设置。启停、以及 Common_State 发生切换时均清 PI
// 的积分和输出，避免上一种运行状态带着积分进入下一种状态。
void speed_control_set_closed_loop_enabled(bool enabled);
void speed_control_set_closed_loop_target(int16 left_target, int16 right_target);
bool speed_control_closed_loop_is_active(void);
void speed_control_get_closed_loop_duty(int16 *left_duty, int16 *right_duty);
void speed_control_reset_pid(void);

// 调试模式是否已经满足 Enable、Run 和至少一路轮子启用条件。
bool speed_control_debug_is_active(void);

// 读取调试模式应当下发的左右 PWM；非活动状态始终返回 0。
void speed_control_debug_get_duty(int16 *left_duty, int16 *right_duty);

// 退出调试运行并清除 PI 的积分和输出；保留已调好的目标、Kp、Ki 和输出上限。
void speed_control_debug_stop(void);

#endif
