#ifndef __PID_H
#define __PID_H
#include "zf_driver_pwm.h"


//PID�ṹ������
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
	float OutOffset;	//���ƫ��ֵ
} PID_t;	

// 使用结构体中的 Target、Actual 和 PID 参数，计算并更新 Out。
void PID_Update(PID_t *p);


#endif

