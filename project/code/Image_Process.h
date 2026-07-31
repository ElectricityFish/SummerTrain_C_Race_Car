#ifndef __IMAGE_PROCESS_H__
#define __IMAGE_PROCESS_H__

#include "Image.h"

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
    uint8 single_edge_target_bias; // 普通单边循线时的目标列补偿，范围 0~10 像素
} Image_Process_Config;

extern Image_Process_Config image_process_config;

// 每行的处理结果，供显示、调试和后续特殊元素识别使用。
extern uint16 image_left_edge[MT9V03X_H];
extern uint16 image_right_edge[MT9V03X_H];
extern uint8 image_mid_line[MT9V03X_H];

// 当前中线生成模式。单边模式用于某一侧边线在特殊元素中短暂丢失时维持循线。
#define IMAGE_TRACK_MODE_BOTH          (0U)
#define IMAGE_TRACK_MODE_FOLLOW_LEFT   (1U)
#define IMAGE_TRACK_MODE_FOLLOW_RIGHT  (2U)

// 十字圆环状态。CROSS_FOLLOW 表示正在绕环，CROSS_EXIT 表示已识别出环十字标志后的过渡帧。
#define IMAGE_CROSS_STATE_NORMAL        (0U)
#define IMAGE_CROSS_STATE_FOLLOW        (1U)
#define IMAGE_CROSS_STATE_EXIT          (2U)

// 圆环方向。右转环在当前相机安装下以左侧稳定边为主跟随；左转使用镜像的右侧稳定边。
#define IMAGE_CROSS_DIRECTION_NONE      (0U)
#define IMAGE_CROSS_DIRECTION_LEFT      (1U)
#define IMAGE_CROSS_DIRECTION_RIGHT     (2U)

// 供菜单观察的边线可靠性与中线模式：1 表示该边线在当前控制权重区域内可靠。
extern uint8 image_process_track_mode;
extern uint8 image_process_left_edge_ok;
extern uint8 image_process_right_edge_ok;
extern uint8 image_process_cross_state;
extern uint8 image_process_cross_direction;
extern uint8 image_process_cross_feature;

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

#endif
