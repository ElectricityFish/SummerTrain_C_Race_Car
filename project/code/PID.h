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
// Kd 对图像误差作差分，Kd2 对陀螺仪横摆角速度作阻尼反馈。
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
	// 横摆角速度阻尼系数；YawRate 单位为 deg/s，因此 Kd2 的量纲为 s。
	float Kd2;
	// X 轴陀螺仪直接换算得到的横摆角速度：左转为正，右转为负。
	float YawRate;

	float Error0;
	float Error1;
	float ErrorInt;

	float OutMax;
	float OutMin;
} Servo_PID_t;

// 舵机专用更新：Kp 随 |Target - Actual| 按二次曲线由 KpMin 过渡到 KpMax。
void servo_pid_up_date(Servo_PID_t *p);


#endif

