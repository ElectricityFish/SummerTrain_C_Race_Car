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

// 图像外环：根据图像中线误差生成目标横摆角速度。
// KpNow 单位为“°/s / 像素”，Out 单位为“°/s”。
typedef struct {
	float Target;
	float Actual;
	float Out;

	float KpMin;
	float KpMax;
	float KpNow;
	float ErrorFull;

	float Error0;

	float OutMax;
	float OutMin;
} Servo_PID_t;

// 横摆角速度内环：Target/Actual/Error 单位均为 °/s，Out 单位为舵机相对修正角度（°）。
typedef struct {
	float Target;
	float Actual;
	float Out;

	float Kp;
	float Ki;
	float Kd;

	float Error0;
	float Error1;
	float ErrorInt;

	float GyroBias;
	float FilterAlpha;
	float Deadband;
	float OutMax;
	float OutMin;
} Yaw_Rate_PID_t;

extern Servo_PID_t servo_pid;
extern Yaw_Rate_PID_t yaw_rate_pid;

// 初始化所有视觉转向 PID 的结构体、初始参数与状态。由 control_init() 调用一次。
void PID_init(void);

// 清空运行状态，不改变 PID_init() 中设定的参数。
void servo_pid_reset(void);
void yaw_rate_pid_reset(void);

// 图像外环更新：Kp 随 |Target - Actual| 按二次曲线由 KpMin 过渡到 KpMax。
void servo_pid_up_date(Servo_PID_t *p);

// 角速度内环更新：调用前需将 Actual 赋为 filtered_yaw_rate，单位为 °/s。
void yaw_rate_pid_up_date(Yaw_Rate_PID_t *p);


#endif

