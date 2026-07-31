#ifndef __KFILTER_H
#define __KFILTER_H

#include "zf_common_typedef.h"

//卡尔曼滤波器的状态量。Pitch和Roll必须各自使用一个独立实例。
typedef struct
{
	float Q_angle;		//角度过程噪声协方差
	float Q_bias;		//陀螺仪零偏过程噪声协方差
	float R_measure;	//加速度计角度测量噪声协方差
	float angle;		//滤波后的角度
	float bias;		//估计出的陀螺仪零偏
	float rate;		//去除零偏后的角速度
	float P[2][2];	//误差协方差矩阵
} KalmanFilter;

//Get_Angle()的调用周期，单位为秒。若实际调用周期改变，必须同步修改此宏。
#define KFILTER_SAMPLE_DT			(0.01f)
#define KFILTER_OFFSET_SAMPLE_COUNT	(100U)
#define KFILTER_YAW_RATE_CALIBRATION_COUNT (100U)

extern KalmanFilter KF;			//Pitch滤波器
extern KalmanFilter KF_Roll;	//Roll滤波器

//这三个变量在TIM6中断内更新，主循环菜单读取时需要使用volatile。
extern volatile float yaw;
extern volatile float pitch;
extern volatile float roll;
extern volatile float yaw_rate_dps;  // 去零偏、低通后的偏航角速度，单位 °/s
extern float pitch_raw;		//未减安装偏置的Pitch，用于静止标定
extern float roll_raw;		//未减安装偏置的Roll，用于静止标定
extern float Offset;			//Pitch安装偏置
extern float RollOffset;		//Roll安装偏置

void Kalman_Init(KalmanFilter *kf, float Q_angle, float Q_bias, float R_measure);
float calculatePitchAngle(float ax, float ay, float az, float gy, float dt, KalmanFilter *kf);
float calculateRollAngle(float ax, float ay, float az, float gx, float dt, KalmanFilter *kf);
float getAccelAngle(float ax, float ay, float az);
float getAccelRollAngle(float ax, float ay, float az);

//Flag=0：清除本次标定累加；Flag=1：采集一个静止角度样本。
//连续采集KFILTER_OFFSET_SAMPLE_COUNT次后，自动将平均值写入Offset。
void GetOffset(float *Offset, float Angle, uint8 Flag);

//初始化Pitch/Roll双卡尔曼滤波器和姿态变量。上电后调用一次。
void kfilter_init(void);

//读取MPU6050并完成Yaw、Pitch、Roll解算。必须每10ms调用一次。
void Get_Angle(void);

#endif
