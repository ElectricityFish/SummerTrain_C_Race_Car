#ifndef __SPEED_CONTROL_H__
#define __SPEED_CONTROL_H__

#include "zf_common_typedef.h"

// RunTarget 是差速分配前的基准速度，菜单范围固定为 0~400。
#define SPEED_CONTROL_RUN_TARGET_MAX       (400)

// 差速后的左右目标内部硬上限。OutMax 最高为 2.0，因此预留到 800。
#define SPEED_CONTROL_TARGET_ABS_MAX       (800)

// 速度单位统一为“10 ms 内的编码器脉冲数”。
typedef struct
{
	int16 TargetPulse;
	int16 ActualPulse;
	float Kp;                 // 菜单系数，内部计算时乘以 10
	float Ki;                 // 直接作为积分系数使用，不做额外缩放
	float Error;
	float ErrorInt;
	float POut;
	float IOut;
	int16 Out;
	int16 OutMax;
} Speed_PID_t;

extern volatile Speed_PID_t speed_left_pid;
extern volatile Speed_PID_t speed_right_pid;

// RUNNING 状态下左右轮共同使用的固定目标，单位为 pulse/10 ms。
extern volatile int16 speed_running_target_pulse;

// IDLE 状态下的单轮闭环调试开关。
extern volatile uint8 speed_debug_enabled;
extern volatile uint8 speed_debug_run;
extern volatile uint8 speed_debug_left_enabled;
extern volatile uint8 speed_debug_right_enabled;

void speed_control_init(void);
void speed_control_10ms_task(int16 left_pulse, int16 right_pulse);

void speed_control_set_closed_loop_enabled(bool enabled);
void speed_control_set_closed_loop_target(int16 left_target, int16 right_target);
bool speed_control_closed_loop_is_active(void);
void speed_control_get_closed_loop_duty(int16 *left_duty, int16 *right_duty);
void speed_control_reset_pid(void);

bool speed_control_debug_is_active(void);
void speed_control_debug_get_duty(int16 *left_duty, int16 *right_duty);
void speed_control_debug_stop(void);

#endif
