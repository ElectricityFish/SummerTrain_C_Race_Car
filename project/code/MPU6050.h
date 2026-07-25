#ifndef __MPU6050_H
#define __MPU6050_H

#include "zf_device_mpu6050.h"

//三轴加速度计原始数据。当前厂商配置为±8g量程，换算为g时除以4096。
extern int16 mpu6050_accel_x;
extern int16 mpu6050_accel_y;
extern int16 mpu6050_accel_z;

//三轴陀螺仪原始数据。当前厂商配置为±2000°/s量程，换算为°/s时除以16.4。
extern int16 mpu6050_gyro_x_data;
extern int16 mpu6050_gyro_y_data;
extern int16 mpu6050_gyro_z_data;

//初始化MPU6050。厂商默认使用软件IIC：B13为SCL，B15为SDA；返回0表示成功。
uint8 mpu6050_module_init(void);

//读取一次三轴加速度计和三轴陀螺仪原始数据，并更新以上六个全局变量。
void mpu6050_get_data(void);

#endif
