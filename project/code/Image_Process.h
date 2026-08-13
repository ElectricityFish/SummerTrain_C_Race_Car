#ifndef __IMAGE_PROCESS_H__
#define __IMAGE_PROCESS_H__

#include "Image.h"
#include "Image_Track_V2.h"

// 图像处理的所有运行时参数。可直接绑定到菜单进行在线调节。
typedef struct
{
    uint8 reference_rows;          // 底部参考区域的高度，单位：行
    uint8 reference_cols;          // 底部参考区域的宽度，单位：列
    uint8 black_threshold;         // 判定为黑色的最低灰度门限
    uint8 white_min_scale;         // 动态白色下限比例，10 表示 1.0 倍参考灰度
    uint8 white_max_scale;         // 动态白色上限比例，10 表示 1.0 倍参考灰度
    uint8 contrast_threshold;      // 黑白边沿的归一化对比度门限
    uint8 contrast_offset;         // 对比像素间隔，单位：像素
    uint8 search_range;            // 相邻两行边线搜索范围，单位：像素
    uint8 weight_center_row;       // 加权中线最关注的图像行
    uint8 weight_span;             // 加权区域半宽，单位：行
    uint8 weight_peak;             // 加权区域中心的最大权重
    uint8 mid_filter_current;      // 最终中线中当前帧占比，范围 0~100
} Image_Process_Config;

// 十字状态：连续确认后会启用两拐点到底部两角的边线补线，并参与中线控制。
typedef enum
{
    IMAGE_CROSS_STATE_NONE = 0,
    IMAGE_CROSS_STATE_CANDIDATE,
    IMAGE_CROSS_STATE_DETECTED,
} image_cross_state_enum;

extern Image_Process_Config image_process_config;

// 每行的处理结果，供显示、调试和后续特殊元素识别使用。
extern uint16 image_left_edge[MT9V03X_H];
extern uint16 image_right_edge[MT9V03X_H];
extern uint8 image_mid_line[MT9V03X_H];
extern bool image_left_edge_valid[MT9V03X_H];
extern bool image_right_edge_valid[MT9V03X_H];

// 初始化图像处理状态和默认参数。
void image_process_init(void);

// 处理 image_get_buffer() 指向的一帧灰度图。应在 image_take_new_frame() 返回 true 后调用。
void image_process_frame(void);

// 在 IPS200 上显示灰度图，并叠加参考列、左右边线和中线。
void image_process_display(void);

// 取走一帧新的处理结果通知，适合用于控制显示刷新频率。
bool image_process_take_new_result(void);

uint8 image_process_get_final_mid(void);
uint8 image_process_get_reference_col(void);
uint8 image_process_get_reference_gray(void);
uint8 image_process_get_white_min(void);
uint8 image_process_get_white_max(void);

// 第一阶段 V2 路径结果。默认影子运行；CONTROL_ENABLE=1 时接管 final_mid。
const image_track_v2_result_t *image_process_get_v2_result(void);

// 返回最底行白色占比，以及该行白色占比是否低于出界门限。
uint8 image_process_get_bottom_white_ratio(void);
bool image_process_is_out_of_bounds(void);

// 斑马线由底部多行规则黑白条纹确认，并在图像层优先于出界判定。
bool image_process_is_zebra_detected(void);

// 返回当前十字识别状态，以及用于补线控制和调试显示的左上、右上拐点。
image_cross_state_enum image_process_get_cross_state(void);
bool image_process_get_cross_corners(
    uint8 *left_col,
    uint8 *left_row,
    uint8 *right_col,
    uint8 *right_row);

#endif
