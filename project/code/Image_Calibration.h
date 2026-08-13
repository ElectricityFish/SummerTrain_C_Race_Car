#ifndef __IMAGE_CALIBRATION_H__
#define __IMAGE_CALIBRATION_H__

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"

// 2026-08-13 实车标定。前后位置以摄像头相对前轴为正负参考。
#define IMAGE_CAMERA_HEIGHT_MM             (285)
#define IMAGE_CAMERA_OFFSET_FROM_FRONT_MM  (-30)
#define IMAGE_VEHICLE_WHEELBASE_MM         (200U)
#define IMAGE_VEHICLE_TRACK_X100_MM        (15475U)
#define IMAGE_ROAD_WIDTH_MM                (450U)
#define IMAGE_CALIBRATED_CENTER_COL        (95U)

// 每行到前轮中心轴线的地面距离，单位 mm。远端只用于置信度和前瞻描述。
extern const uint16 image_row_distance_mm[MT9V03X_H];

// 45 cm 直道在每个图像行的期望像素宽度。
extern const uint8 image_expected_width_px[MT9V03X_H];

#endif
