#ifndef __PID_H
#define __PID_H
#include "zf_driver_pwm.h"


// Generic PID structure.
typedef struct {
	float Target;
	float Actual;
	float Actual1;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	float OutMax;
	float OutMin;
	float OutOffset;
} PID_t;	

// Generic PID update function.
void PID_Update(PID_t *p);

// 舵机视觉控制专用 PID。KpNow 由误差大小动态计算，单位为“舵机角度/图像像素”。
// 保留 Ki、Kd 及其状态，便于后续在此专用控制器上继续加入积分或微分。
typedef struct {
	float Target;
	float Actual;
	float Out;

	float KpMin;
	float KpMax;
	float KpNow;
	float ErrorFull;
	float Ki;
	float Kd;
	float Kd2;            // 偏航角速度阻尼增益，单位为舵机角度/(°/s)
	float YawRateDps;     // 当前去零偏、低通后的偏航角速度，单位 °/s
	float Kd2Out;         // Kd2 对舵机输出的贡献，便于菜单观察

	float Error0;
	float Error1;
	float ErrorInt;

	float OutMax;
	float OutMin;
} Servo_PID_t;

// 舵机专用更新：Kp 随 |Target - Actual| 按二次曲线由 KpMin 过渡到 KpMax。
void servo_pid_up_date(Servo_PID_t *p);


#endif

