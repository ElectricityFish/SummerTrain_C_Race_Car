#include "Kfilter.h"

#include <math.h>

#include "MPU6050.h"
#include "Promopt.h"

#define KFILTER_Q_ANGLE		(0.001f)
#define KFILTER_Q_BIAS		(0.003f)
#define KFILTER_R_MEASURE	(0.03f)
#define KFILTER_RAD_TO_DEG	(57.2957795f)

KalmanFilter KF;
KalmanFilter KF_Roll;

volatile float yaw = 0.0f;
volatile float pitch = 0.0f;
volatile float roll = 0.0f;
float pitch_raw = 0.0f;
float roll_raw = 0.0f;
float Offset = 0.0f;
float RollOffset = 0.0f;

static float gyro_yaw = 0.0f;
static bool kfilter_first_update = true;

//初始化一个卡尔曼滤波器的状态和噪声参数。
void Kalman_Init(KalmanFilter *kf, float Q_angle, float Q_bias, float R_measure)
{
	if(kf == NULL)
	{
		return;
	}

	kf->Q_angle = Q_angle;
	kf->Q_bias = Q_bias;
	kf->R_measure = R_measure;
	kf->angle = 0.0f;
	kf->bias = 0.0f;
	kf->rate = 0.0f;
	kf->P[0][0] = 0.0f;
	kf->P[0][1] = 0.0f;
	kf->P[1][0] = 0.0f;
	kf->P[1][1] = 0.0f;
}

//单轴一维卡尔曼融合：加速度计提供角度观测，陀螺仪提供角速度预测。
static float Kalman_Calculate(KalmanFilter *kf, float new_angle, float new_rate, float dt)
{
	float innovation_covariance;
	float gain_angle;
	float gain_bias;
	float innovation;
	float p00;
	float p01;

	if(kf == NULL || dt <= 0.0f)
	{
		return 0.0f;
	}

	kf->rate = new_rate - kf->bias;
	kf->angle += dt * kf->rate;

	kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
	kf->P[0][1] -= dt * kf->P[1][1];
	kf->P[1][0] -= dt * kf->P[1][1];
	kf->P[1][1] += kf->Q_bias * dt;

	innovation_covariance = kf->P[0][0] + kf->R_measure;
	if(innovation_covariance <= 0.0f)
	{
		return kf->angle;
	}
	gain_angle = kf->P[0][0] / innovation_covariance;
	gain_bias = kf->P[1][0] / innovation_covariance;

	innovation = new_angle - kf->angle;
	kf->angle += gain_angle * innovation;
	kf->bias += gain_bias * innovation;

	p00 = kf->P[0][0];
	p01 = kf->P[0][1];
	kf->P[0][0] -= gain_angle * p00;
	kf->P[0][1] -= gain_angle * p01;
	kf->P[1][0] -= gain_bias * p00;
	kf->P[1][1] -= gain_bias * p01;

	return kf->angle;
}

//由加速度计计算Pitch：绕Y轴转动时主要表现为X轴重力分量变化。
float getAccelAngle(float ax, float ay, float az)
{
	return -atan2f(ax, sqrtf(ay * ay + az * az)) * KFILTER_RAD_TO_DEG;
}

//由加速度计计算Roll：绕X轴转动时主要表现为Y轴重力分量变化。
float getAccelRollAngle(float ax, float ay, float az)
{
	return atan2f(ay, sqrtf(ax * ax + az * az)) * KFILTER_RAD_TO_DEG;
}

//Pitch使用GY（绕Y轴角速度）和加速度计Pitch角进行融合。
float calculatePitchAngle(float ax, float ay, float az, float gy, float dt, KalmanFilter *kf)
{
	float gyro_y_dps = mpu6050_gyro_transition((int16)gy);

	return Kalman_Calculate(kf, getAccelAngle(ax, ay, az), gyro_y_dps, dt);
}

//Roll使用GX（绕X轴角速度）和加速度计Roll角进行融合。
float calculateRollAngle(float ax, float ay, float az, float gx, float dt, KalmanFilter *kf)
{
	float gyro_x_dps = mpu6050_gyro_transition((int16)gx);

	return Kalman_Calculate(kf, getAccelRollAngle(ax, ay, az), gyro_x_dps, dt);
}

//静止标定平均值采集。一次标定只对应一个Offset变量；Pitch和Roll请依次单独标定。
void GetOffset(float *offset, float angle, uint8 flag)
{
	static float angle_sum = 0.0f;
	static uint16 sample_count = 0;

	if(offset == NULL)
	{
		return;
	}

	if(flag == 0U)
	{
		angle_sum = 0.0f;
		sample_count = 0;
		return;
	}

	angle_sum += angle;
	sample_count++;
	if(sample_count >= KFILTER_OFFSET_SAMPLE_COUNT)
	{
		*offset = angle_sum / (float)sample_count;
		angle_sum = 0.0f;
		sample_count = 0;

		//标定完成提醒。promopt_tick()每1ms使计数减1，因此20约为20ms提示音。
		promopt_count = 20U;
	}
}

//初始化双轴卡尔曼滤波器。Pitch和Roll的状态必须相互独立。
void kfilter_init(void)
{
	Kalman_Init(&KF, KFILTER_Q_ANGLE, KFILTER_Q_BIAS, KFILTER_R_MEASURE);
	Kalman_Init(&KF_Roll, KFILTER_Q_ANGLE, KFILTER_Q_BIAS, KFILTER_R_MEASURE);
	yaw = 0.0f;
	pitch = 0.0f;
	roll = 0.0f;
	pitch_raw = 0.0f;
	roll_raw = 0.0f;
	gyro_yaw = 0.0f;
	Offset = 0.0f;
	RollOffset = 0.0f;
	kfilter_first_update = true;
}

//将原始数据按100为步长截断，保留原工程的简易加速度计/陀螺仪滤波方式。
static int16 kfilter_raw_quantize(int16 raw_value)
{
	return (int16)((raw_value / 100) * 100);
}

//每10ms读取一次MPU6050并完成姿态解算；调用周期变化时必须修改KFILTER_SAMPLE_DT。
void Get_Angle(void)
{
	int16 ax;
	int16 ay;
	int16 az;
	int16 gx;
	int16 gy;
	int16 gz;

	mpu6050_get_data();

	ax = kfilter_raw_quantize(mpu6050_accel_x);
	ay = kfilter_raw_quantize(mpu6050_accel_y);
	az = kfilter_raw_quantize(mpu6050_accel_z);
	gx = kfilter_raw_quantize(mpu6050_gyro_x_data);
	gy = kfilter_raw_quantize(mpu6050_gyro_y_data);
	gz = kfilter_raw_quantize(mpu6050_gyro_z_data);

	//首次用加速度计角度初始化，避免滤波器从0度缓慢收敛。
	if(kfilter_first_update)
	{
		KF.angle = getAccelAngle((float)ax, (float)ay, (float)az);
		KF_Roll.angle = getAccelRollAngle((float)ax, (float)ay, (float)az);
		kfilter_first_update = false;
	}

	gyro_yaw += mpu6050_gyro_transition(gz) * KFILTER_SAMPLE_DT;
	yaw = gyro_yaw;

	pitch_raw = calculatePitchAngle((float)ax, (float)ay, (float)az, (float)gy, KFILTER_SAMPLE_DT, &KF);
	roll_raw = calculateRollAngle((float)ax, (float)ay, (float)az, (float)gx, KFILTER_SAMPLE_DT, &KF_Roll);
	pitch = pitch_raw - Offset;
	roll = roll_raw - RollOffset;
}
