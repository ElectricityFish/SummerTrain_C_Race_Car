#ifndef __SPEED_PLANNER_H__
#define __SPEED_PLANNER_H__

#include "zf_common_typedef.h"

// 第一阶段只处理直道与弯道：图像中线曲率决定基础速度，左右轮差速仍由
// SpeedDecision 中的阿克曼模型完成。速度单位为 10 ms 内的编码器脉冲数。
typedef struct
{
    uint8 enabled;
    int16 start_target;
    int16 straight_target;
    int16 curve_min_target;
    int16 safe_target;
    float bend_full_px;
    float steer_full_deg;
    float steer_weight;
    uint8 curve_filter_current;
    int16 up_step_per_10ms;
    int16 down_step_per_10ms;
    uint16 image_safe_age_ms;
    uint16 image_stop_age_ms;
} Speed_Planner_Config_t;

extern volatile Speed_Planner_Config_t speed_planner_config;

// 以下量供菜单观察。BendPx 是三点中线二阶差分的绝对值；
// Curve 是 0.0~1.0 的滤波后曲率系数。
extern volatile uint16 speed_planner_bend_px;
extern volatile float speed_planner_curve;
extern volatile int16 speed_planner_raw_target_pulse;
extern volatile int16 speed_planner_target_pulse;

void speed_planner_init(void);

// 仅在 RUNNING 进入和退出时调用。start 会立即给出保守起步速度，后续
// 由 10 ms 斜坡平滑过渡到图像规划速度。
void speed_planner_start(void);
void speed_planner_stop(void);

// 在 image_process_frame() 和本帧 servo_control() 完成后调用；只读取已
// 完整生成的中线，避免在 10 ms 中断中与图像处理并发访问数组。
void speed_planner_update_from_image(void);

// 每 10 ms、且在 speed_decision_10ms_task() 之前调用。
void speed_planner_10ms_task(void);

#endif
