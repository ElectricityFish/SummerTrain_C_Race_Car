#include "MPU6050.h"

int16 mpu6050_accel_x = 0;
int16 mpu6050_accel_y = 0;
int16 mpu6050_accel_z = 0;
int16 mpu6050_gyro_x_data = 0;
int16 mpu6050_gyro_y_data = 0;
int16 mpu6050_gyro_z_data = 0;

//初始化厂商MPU6050驱动。驱动内部会初始化B13/B15软件IIC并完成传感器自检。
uint8 mpu6050_module_init(void)
{
	return mpu6050_init();
}

//读取厂商驱动的六轴原始数据，并复制到本模块的全局变量供上层直接使用。
void mpu6050_get_data(void)
{
	mpu6050_get_acc();
	mpu6050_get_gyro();

	mpu6050_accel_x = mpu6050_acc_x;
	mpu6050_accel_y = mpu6050_acc_y;
	mpu6050_accel_z = mpu6050_acc_z;
	mpu6050_gyro_x_data = mpu6050_gyro_x;
	mpu6050_gyro_y_data = mpu6050_gyro_y;
	mpu6050_gyro_z_data = mpu6050_gyro_z;
}

