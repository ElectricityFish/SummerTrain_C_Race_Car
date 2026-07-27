#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "PID.h"

// 小车全局状态。只有状态机接口可以改变状态，避免菜单或业务逻辑绕过保护。
typedef enum
{
	COMMON_STATE_IDLE = 0,
	COMMON_STATE_RUNNING,
	COMMON_STATE_PROTECT
} Common_State;

// 保护原因可以组合并锁存，供菜单显示和故障定位使用。
#define CAR_PROTECTION_REASON_NONE       (0U)
#define CAR_PROTECTION_REASON_ATTITUDE   (1U << 0)

#define CAR_PROTECTION_ANGLE_LIMIT_DEG       (50.0f)

// common_state 由主循环和定时中断共同访问，必须使用 volatile。
extern volatile Common_State common_state;
extern volatile uint8 car_go_command;             // CarGo/RunCmd：0 停车，1 请求发车
extern volatile uint8 car_protection_reason;      // 已锁存的保护原因位图


// 图像列坐标的目标偏置：正值表示目标中线向图像右侧移动。
#define SERVO_CONTROL_IMAGE_CENTER_OFFSET     (0.0f)

// 首次闭环测试的相对转角限幅。确认方向、机械行程和参数后再逐步增大。
#define SERVO_CONTROL_OUTPUT_LIMIT             (20.0f)

// 1.0f：图像中线右移时输出右转；若实车方向相反，改为 -1.0f。
#define SERVO_CONTROL_DIRECTION                (1.0f)

extern PID_t servo_pid;
extern volatile bool servo_control_enabled;

// 初始化视觉舵机 PID；应在摄像头、图像处理和舵机底层初始化完成后调用。
void control_init(void);

// 在主循环调用，处理菜单的发车/停车请求。Protect 只能在故障消失且 RunCmd=0 时退回 IDLE。
void car_state_command_task(void);

// 在姿态解算完成后调用；仅 RUNNING 状态命中条件时进入 Protect。
void car_protection_check_attitude(void);

// 设置视觉舵机闭环的启停。关闭时清除 PID 状态并回正。
void servo_control_set_enabled(bool enabled);

// 每 10 ms 调用一次：图像中线 -> PID 相对角度 -> 舵机逻辑角度 -> PWM。
void servo_control(void);

#endif
