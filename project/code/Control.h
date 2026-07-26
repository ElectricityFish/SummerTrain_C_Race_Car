#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "PID.h"

// 图像列坐标的目标偏置：正值表示目标中线向图像右侧移动。
#define SERVO_CONTROL_IMAGE_CENTER_OFFSET     (0.0f)

// 首次闭环测试的相对转角限幅。确认方向、机械行程和参数后再逐步增大。
#define SERVO_CONTROL_OUTPUT_LIMIT             (20.0f)

// 1.0f：图像中线右移时输出右转；若实车方向相反，改为 -1.0f。
#define SERVO_CONTROL_DIRECTION                (1.0f)

extern PID_t servo_pid;
extern bool servo_control_enabled;

// 初始化视觉舵机 PID；应在摄像头、图像处理和舵机底层初始化完成后调用。
void control_init(void);

// 设置视觉舵机闭环的启停。关闭时清除 PID 状态并回正。
void servo_control_set_enabled(bool enabled);

// 每 10 ms 调用一次：图像中线 -> PID 相对角度 -> 舵机逻辑角度 -> PWM。
void servo_control(void);

#endif
