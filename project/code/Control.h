#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "PID.h"

// 小车全局状态。只有状态机接口可以改变状态，避免菜单或业务逻辑绕过保护。
typedef enum
{
	COMMON_STATE_IDLE = 0,
	COMMON_STATE_RUNNING,
	COMMON_STATE_PLAY,
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
extern volatile uint8 wireless_control_enabled;   // Wireless_Control/Enable：0 关闭，1 开启


// 图像列坐标的目标偏置：正值表示目标中线向图像右侧移动。
#define SERVO_CONTROL_IMAGE_CENTER_OFFSET     (0.0f)

// 1.0f：图像中线右移时输出右转；若实车方向相反，改为 -1.0f。
#define SERVO_CONTROL_DIRECTION                (1.0f)

// 小 S 安全直切状态下，最终舵机相对中位最多修正 2 度，避免 PID 微分或横摆项再次放大打角。
#define SERVO_CONTROL_SMALL_S_MAX_CORRECTION   (2.0f)

extern Servo_PID_t servo_pid;
extern volatile bool servo_control_enabled;

// 初始化视觉舵机 PID；应在摄像头、图像处理和舵机底层初始化完成后调用。
void control_init(void);

// 在主循环调用，处理 Base_Control 或 Wireless_Control 的状态请求。
// Protect 只能在故障消失且状态请求为 IDLE 时退回 IDLE。
void car_state_command_task(void);

// 无线总使能是否允许电机与舵机动作；供 1ms 执行器任务作最高优先级急停判断。
bool wireless_control_actuators_permitted(void);

// PLAY 状态下按 CH1/CH3 刷新舵机和双电机输出；由 20ms 执行器任务调用。
void wireless_control_play_task(void);

// 在姿态解算完成后调用；RUNNING 与 PLAY 状态命中条件时进入 Protect。
void car_protection_check_attitude(void);

// 设置视觉舵机闭环的启停。关闭时清除 PID 状态并回正。
void servo_control_set_enabled(bool enabled);

// 当前每 20 ms 调用一次：图像中线与横摆角速度 -> 转向控制量 -> 舵机逻辑角度 -> PWM。
void servo_control(void);

#endif
