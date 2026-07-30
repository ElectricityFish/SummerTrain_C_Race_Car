#include "zf_common_headfile.h"
#include "Image_Process.h"

#define IMAGE_PROCESS_WEIGHT_BASE     (1U)
#define IMAGE_PROCESS_DISPLAY_WIDTH   (240U)
#define IMAGE_PROCESS_DISPLAY_HEIGHT  (153U)
#define IMAGE_PROCESS_EDGE_OK_PERCENT (70U)
#define IMAGE_PROCESS_TRACK_LOSS_FRAMES (3U)
#define IMAGE_PROCESS_TRACK_RECOVER_FRAMES (5U)

Image_Process_Config image_process_config;

uint16 image_left_edge[MT9V03X_H];
uint16 image_right_edge[MT9V03X_H];
uint8 image_mid_line[MT9V03X_H];
uint8 image_process_track_mode;
uint8 image_process_left_edge_ok;
uint8 image_process_right_edge_ok;

static uint8 image_reference_col;
static uint8 image_reference_gray;
static uint8 image_white_min;
static uint8 image_white_max;
static uint8 image_final_mid;
static uint8 image_last_final_mid;
static bool image_has_last_mid;
static bool image_new_result;
// 直道标定得到的逐行半赛道宽度（188 x 120 原始图像坐标）。
// 第 20~98 行由“图片/直道图像/1.png”按当前寻边参数实测；两端仅作受限外推。
// 单边循线只读取此表，不会随赛道、圆环或异常帧改变标定值。
static const uint8 image_lane_half_width[MT9V03X_H] =
{
     0U,  0U,  0U,  0U,  0U,  0U,  0U,  0U,  0U,  0U,
     0U,  0U,  0U,  0U,  1U,  2U,  3U,  4U,  5U,  6U,
     8U,  8U,  9U, 10U, 11U, 12U, 13U, 14U, 15U, 16U,
    17U, 18U, 19U, 20U, 21U, 23U, 24U, 25U, 26U, 27U,
    28U, 29U, 30U, 31U, 32U, 33U, 34U, 35U, 36U, 37U,
    38U, 39U, 40U, 41U, 42U, 44U, 45U, 46U, 47U, 48U,
    49U, 50U, 51U, 53U, 54U, 55U, 56U, 57U, 58U, 59U,
    60U, 62U, 63U, 64U, 65U, 66U, 67U, 68U, 69U, 70U,
    71U, 72U, 73U, 75U, 76U, 77U, 78U, 79U, 80U, 81U,
    82U, 83U, 84U, 85U, 86U, 87U, 88U, 89U, 90U, 91U,
    92U, 93U, 93U, 93U, 93U, 93U, 93U, 93U, 93U, 93U,
    93U, 93U, 93U, 93U, 93U, 93U, 93U, 93U, 93U, 93U
};
static uint8 image_left_lost_frames;
static uint8 image_right_lost_frames;
static uint8 image_both_recovered_frames;

static uint8 image_process_limit_u8(int32 value, uint8 lower, uint8 upper)
{
    if(value < lower)
    {
        return lower;
    }
    if(value > upper)
    {
        return upper;
    }
    return (uint8)value;
}

static uint8 image_process_abs_diff(uint8 value_a, uint8 value_b)
{
    return (value_a >= value_b) ? (value_a - value_b) : (value_b - value_a);
}

static uint8 image_process_contrast(uint8 inside, uint8 outside)
{
    uint16 sum = (uint16)inside + outside;
    int16 difference;

    if(sum == 0U)
    {
        return 0U;
    }

    difference = (int16)inside - outside;
    if(difference <= 0)
    {
        return 0U;
    }
    return (uint8)(((int32)difference * 200) / sum);
}

static uint8 image_process_row_weight(uint8 row)
{
    uint8 center = image_process_limit_u8(image_process_config.weight_center_row, 0U, MT9V03X_H - 1U);
    uint8 span = image_process_limit_u8(image_process_config.weight_span, 1U, MT9V03X_H - 1U);
    uint8 peak = image_process_limit_u8(image_process_config.weight_peak, IMAGE_PROCESS_WEIGHT_BASE, 100U);
    uint8 distance = image_process_abs_diff(row, center);

    if(distance >= span)
    {
        return IMAGE_PROCESS_WEIGHT_BASE;
    }

    return (uint8)(IMAGE_PROCESS_WEIGHT_BASE
        + ((uint16)(peak - IMAGE_PROCESS_WEIGHT_BASE) * (span - distance)) / span);
}

// 左右边线搜索失败时分别返回 0 和图像最右列；这些值不能作为真实边线参与中线计算。
static bool image_process_left_edge_is_valid(uint16 edge)
{
    return (edge > 0U) && (edge < MT9V03X_W - 1U);
}

static bool image_process_right_edge_is_valid(uint16 edge)
{
    return (edge > 0U) && (edge < MT9V03X_W - 1U);
}

static bool image_process_row_has_both_edges(uint16 row)
{
    return image_process_left_edge_is_valid(image_left_edge[row])
        && image_process_right_edge_is_valid(image_right_edge[row])
        && (image_right_edge[row] > image_left_edge[row]);
}

// 只统计当前加权区域，避免图像最远端的偶发边线丢失频繁切换模式。
static void image_process_update_edge_reliability(void)
{
    uint16 row;
    uint32 total_weight = 0U;
    uint32 left_weight = 0U;
    uint32 right_weight = 0U;

    for(row = 0U; row < MT9V03X_H; row++)
    {
        uint8 weight = image_process_row_weight((uint8)row);

        if(weight <= IMAGE_PROCESS_WEIGHT_BASE)
        {
            continue;
        }

        total_weight += weight;
        if(image_process_left_edge_is_valid(image_left_edge[row]))
        {
            left_weight += weight;
        }
        if(image_process_right_edge_is_valid(image_right_edge[row]))
        {
            right_weight += weight;
        }
    }

    // 若用户把 WeightPeak 设为 1，整个画面权重相同，退回到全行统计。
    if(total_weight == 0U)
    {
        for(row = 0U; row < MT9V03X_H; row++)
        {
            total_weight++;
            if(image_process_left_edge_is_valid(image_left_edge[row]))
            {
                left_weight++;
            }
            if(image_process_right_edge_is_valid(image_right_edge[row]))
            {
                right_weight++;
            }
        }
    }

    image_process_left_edge_ok = (left_weight * 100U >= total_weight * IMAGE_PROCESS_EDGE_OK_PERCENT) ? 1U : 0U;
    image_process_right_edge_ok = (right_weight * 100U >= total_weight * IMAGE_PROCESS_EDGE_OK_PERCENT) ? 1U : 0U;
}

static void image_process_set_track_mode(uint8 mode)
{
    image_process_track_mode = mode;
    image_left_lost_frames = 0U;
    image_right_lost_frames = 0U;
    image_both_recovered_frames = 0U;
}

// 第一版仅在单侧连续失效时做兜底切换；圆环入口、出口的专用状态机后续再建立。
static void image_process_update_track_mode(void)
{
    if(image_process_track_mode == IMAGE_TRACK_MODE_BOTH)
    {
        if((image_process_left_edge_ok == 0U) && (image_process_right_edge_ok != 0U))
        {
            image_left_lost_frames++;
            image_right_lost_frames = 0U;
            if(image_left_lost_frames >= IMAGE_PROCESS_TRACK_LOSS_FRAMES)
            {
                image_process_set_track_mode(IMAGE_TRACK_MODE_FOLLOW_RIGHT);
            }
        }
        else if((image_process_left_edge_ok != 0U) && (image_process_right_edge_ok == 0U))
        {
            image_right_lost_frames++;
            image_left_lost_frames = 0U;
            if(image_right_lost_frames >= IMAGE_PROCESS_TRACK_LOSS_FRAMES)
            {
                image_process_set_track_mode(IMAGE_TRACK_MODE_FOLLOW_LEFT);
            }
        }
        else
        {
            image_left_lost_frames = 0U;
            image_right_lost_frames = 0U;
        }
        return;
    }

    if((image_process_left_edge_ok != 0U) && (image_process_right_edge_ok != 0U))
    {
        image_both_recovered_frames++;
        if(image_both_recovered_frames >= IMAGE_PROCESS_TRACK_RECOVER_FRAMES)
        {
            image_process_set_track_mode(IMAGE_TRACK_MODE_BOTH);
        }
        return;
    }

    image_both_recovered_frames = 0U;
    if((image_process_track_mode == IMAGE_TRACK_MODE_FOLLOW_LEFT)
        && (image_process_left_edge_ok == 0U)
        && (image_process_right_edge_ok != 0U))
    {
        image_left_lost_frames++;
        if(image_left_lost_frames >= IMAGE_PROCESS_TRACK_LOSS_FRAMES)
        {
            image_process_set_track_mode(IMAGE_TRACK_MODE_FOLLOW_RIGHT);
        }
    }
    else if((image_process_track_mode == IMAGE_TRACK_MODE_FOLLOW_RIGHT)
        && (image_process_left_edge_ok != 0U)
        && (image_process_right_edge_ok == 0U))
    {
        image_right_lost_frames++;
        if(image_right_lost_frames >= IMAGE_PROCESS_TRACK_LOSS_FRAMES)
        {
            image_process_set_track_mode(IMAGE_TRACK_MODE_FOLLOW_LEFT);
        }
    }
}

static uint8 image_process_get_mid_from_edges(uint16 row)
{
    bool left_valid = image_process_left_edge_is_valid(image_left_edge[row]);
    bool right_valid = image_process_right_edge_is_valid(image_right_edge[row]);

    if((image_process_track_mode == IMAGE_TRACK_MODE_BOTH) && image_process_row_has_both_edges(row))
    {
        return (uint8)((image_left_edge[row] + image_right_edge[row]) / 2U);
    }
    if((image_process_track_mode == IMAGE_TRACK_MODE_FOLLOW_LEFT) && left_valid)
    {
        return image_process_limit_u8((int32)image_left_edge[row] + image_lane_half_width[row], 0U, MT9V03X_W - 1U);
    }
    if((image_process_track_mode == IMAGE_TRACK_MODE_FOLLOW_RIGHT) && right_valid)
    {
        return image_process_limit_u8((int32)image_right_edge[row] - image_lane_half_width[row], 0U, MT9V03X_W - 1U);
    }

    // 切换确认前也不要将失效边线与有效边线直接平均；优先使用仍可靠的一侧。
    if(left_valid)
    {
        return image_process_limit_u8((int32)image_left_edge[row] + image_lane_half_width[row], 0U, MT9V03X_W - 1U);
    }
    if(right_valid)
    {
        return image_process_limit_u8((int32)image_right_edge[row] - image_lane_half_width[row], 0U, MT9V03X_W - 1U);
    }

    // 两侧均不可用时保留上帧该行结果，避免把 0/最右列突变直接传入 PID。
    return image_mid_line[row];
}

static void image_process_calculate_threshold(const uint8 image[][MT9V03X_W])
{
    uint8 rows = image_process_limit_u8(image_process_config.reference_rows, 1U, MT9V03X_H);
    uint8 cols = image_process_limit_u8(image_process_config.reference_cols, 1U, MT9V03X_W);
    uint8 min_scale = image_process_limit_u8(image_process_config.white_min_scale, 1U, 20U);
    uint8 max_scale = image_process_limit_u8(image_process_config.white_max_scale, min_scale, 25U);
    uint16 start_col = (MT9V03X_W - cols) / 2U;
    uint16 end_col = start_col + cols;
    uint16 start_row = MT9V03X_H - rows;
    uint32 sum = 0U;
    uint16 count = 0U;
    uint16 row;
    uint16 col;

    for(row = start_row; row < MT9V03X_H; row++)
    {
        for(col = start_col; col < end_col; col++)
        {
            sum += image[row][col];
            count++;
        }
    }

    image_reference_gray = (uint8)(sum / count);
    if(image_reference_gray < image_process_config.black_threshold)
    {
        image_reference_gray = image_process_config.black_threshold;
    }

    image_white_min = image_process_limit_u8(
        ((uint16)image_reference_gray * min_scale) / 10U,
        image_process_config.black_threshold,
        255U);
    image_white_max = image_process_limit_u8(
        ((uint16)image_reference_gray * max_scale) / 10U,
        image_white_min,
        255U);
}

// 返回从图像底部连续向上保持白色的距离。数值越大，说明该列越像赛道内部。
static uint8 image_process_get_white_run(const uint8 image[][MT9V03X_W], uint8 col)
{
    int16 row;
    uint8 offset = image_process_limit_u8(image_process_config.contrast_offset, 1U, 8U);
    uint8 threshold = image_process_config.contrast_threshold;

    for(row = MT9V03X_H - 1; row >= (int16)offset; row--)
    {
        uint8 current = image[row][col];
        uint8 upper = image[row - offset][col];

        if(current < image_white_min)
        {
            return (uint8)(MT9V03X_H - 1 - row);
        }
        if(upper > image_white_max)
        {
            continue;
        }
        if(image_process_contrast(current, upper) > threshold)
        {
            return (uint8)(MT9V03X_H - 1 - row);
        }
    }

    return MT9V03X_H - 1U;
}

static void image_process_find_reference_col(const uint8 image[][MT9V03X_W])
{
    uint8 offset = image_process_limit_u8(image_process_config.contrast_offset, 1U, 8U);
    uint8 center = MT9V03X_W / 2U;
    uint8 best_col = center;
    uint8 best_run = 0U;
    uint16 col;

    for(col = 0U; col < MT9V03X_W; col += offset)
    {
        uint8 run;

        if(image[MT9V03X_H - 1U][col] < image_white_min)
        {
            continue;
        }

        run = image_process_get_white_run(image, (uint8)col);
        if((run > best_run)
            || ((run == best_run) && (image_process_abs_diff((uint8)col, center)
                < image_process_abs_diff(best_col, center))))
        {
            best_run = run;
            best_col = (uint8)col;
        }
    }

    image_reference_col = best_col;
}

static uint16 image_process_find_left_edge(const uint8 image[][MT9V03X_W], uint8 row, int16 start, int16 end)
{
    uint8 offset = image_process_limit_u8(image_process_config.contrast_offset, 1U, 8U);
    uint8 threshold = image_process_config.contrast_threshold;
    int16 col;

    start = image_process_limit_u8(start, offset, MT9V03X_W - 1U);
    end = image_process_limit_u8(end, offset, start);

    for(col = start; col >= end; col--)
    {
        uint8 inside = image[row][col];
        uint8 outside = image[row][col - offset];

        if(inside < image_white_min)
        {
            return (uint16)col;
        }
        if(outside > image_white_max)
        {
            continue;
        }
        if(image_process_contrast(inside, outside) > threshold)
        {
            return (uint16)(col - offset);
        }
    }

    return 0U;
}

static uint16 image_process_find_right_edge(const uint8 image[][MT9V03X_W], uint8 row, int16 start, int16 end)
{
    uint8 offset = image_process_limit_u8(image_process_config.contrast_offset, 1U, 8U);
    uint8 threshold = image_process_config.contrast_threshold;
    int16 col;

    start = image_process_limit_u8(start, 0U, MT9V03X_W - 1U - offset);
    end = image_process_limit_u8(end, start, MT9V03X_W - 1U - offset);

    for(col = start; col <= end; col++)
    {
        uint8 inside = image[row][col];
        uint8 outside = image[row][col + offset];

        if(inside < image_white_min)
        {
            return (uint16)col;
        }
        if(outside > image_white_max)
        {
            continue;
        }
        if(image_process_contrast(inside, outside) > threshold)
        {
            return (uint16)(col + offset);
        }
    }

    return MT9V03X_W - 1U;
}

static void image_process_track_edges(const uint8 image[][MT9V03X_W])
{
    int16 left_start = image_reference_col;
    int16 right_start = image_reference_col;
    int16 row;
    uint8 range = image_process_limit_u8(image_process_config.search_range, 1U, MT9V03X_W / 2U);

    for(row = MT9V03X_H - 1; row >= 0; row--)
    {
        uint16 left_edge;
        uint16 right_edge;

        left_edge = image_process_find_left_edge(image, (uint8)row, left_start, 0);
        if((left_edge == 0U) && (left_start != image_reference_col))
        {
            left_edge = image_process_find_left_edge(image, (uint8)row, image_reference_col, 0);
        }

        right_edge = image_process_find_right_edge(image, (uint8)row, right_start, MT9V03X_W - 1U);
        if((right_edge == MT9V03X_W - 1U) && (right_start != image_reference_col))
        {
            right_edge = image_process_find_right_edge(image, (uint8)row, image_reference_col, MT9V03X_W - 1U);
        }

        image_left_edge[row] = left_edge;
        image_right_edge[row] = right_edge;

        left_start = image_process_limit_u8((int16)left_edge + range, 0U, MT9V03X_W - 1U);
        right_start = image_process_limit_u8((int16)right_edge - range, 0U, MT9V03X_W - 1U);
    }
}

static void image_process_calculate_mid(void)
{
    uint32 weighted_sum = 0U;
    uint16 weight_sum = 0U;
    uint16 row;
    uint8 current_weight = image_process_limit_u8(image_process_config.mid_filter_current, 0U, 100U);
    uint8 current_mid;

    for(row = 0U; row < MT9V03X_H; row++)
    {
        uint8 weight = image_process_row_weight((uint8)row);

        image_mid_line[row] = image_process_get_mid_from_edges(row);
        weighted_sum += (uint32)image_mid_line[row] * weight;
        weight_sum += weight;
    }

    current_mid = (uint8)(weighted_sum / weight_sum);
    if(!image_has_last_mid)
    {
        image_final_mid = current_mid;
        image_has_last_mid = true;
    }
    else
    {
        image_final_mid = (uint8)(((uint16)current_mid * current_weight
            + (uint16)image_last_final_mid * (100U - current_weight) + 50U) / 100U);
    }
    image_last_final_mid = image_final_mid;
}

void image_process_init(void)
{
    uint16 row;

    image_process_config.reference_rows = 5U;
    image_process_config.reference_cols = 80U;
    image_process_config.black_threshold = 50U;
    image_process_config.white_min_scale = 7U;
    image_process_config.white_max_scale = 13U;
    image_process_config.contrast_threshold = 20U;
    image_process_config.contrast_offset = 3U;
    image_process_config.search_range = 10U;
    image_process_config.weight_center_row = 72U;
    image_process_config.weight_span = 35U;
    image_process_config.weight_peak = 20U;
    image_process_config.mid_filter_current = 80U;

    memset(image_left_edge, 0, sizeof(image_left_edge));
    memset(image_right_edge, 0, sizeof(image_right_edge));
    for(row = 0U; row < MT9V03X_H; row++)
    {
        image_mid_line[row] = MT9V03X_W / 2U;
    }
    image_process_track_mode = IMAGE_TRACK_MODE_BOTH;
    image_process_left_edge_ok = 0U;
    image_process_right_edge_ok = 0U;
    image_left_lost_frames = 0U;
    image_right_lost_frames = 0U;
    image_both_recovered_frames = 0U;
    image_reference_col = MT9V03X_W / 2U;
    image_reference_gray = 0U;
    image_white_min = 0U;
    image_white_max = 0U;
    image_final_mid = MT9V03X_W / 2U;
    image_last_final_mid = image_final_mid;
    image_has_last_mid = false;
    image_new_result = false;
}

void image_process_frame(void)
{
    const uint8 (*image)[MT9V03X_W] = (const uint8 (*)[MT9V03X_W])image_get_buffer();

    image_process_calculate_threshold(image);
    image_process_find_reference_col(image);
    image_process_track_edges(image);
    image_process_update_edge_reliability();
    image_process_update_track_mode();
    image_process_calculate_mid();
    image_new_result = true;
}

void image_process_display(void)
{
    const uint8 *image = image_get_buffer();
    uint16 row;

    ips200_show_gray_image(0U, 0U, image, MT9V03X_W, MT9V03X_H,
        IMAGE_PROCESS_DISPLAY_WIDTH, IMAGE_PROCESS_DISPLAY_HEIGHT, 0U);

    for(row = 0U; row < MT9V03X_H; row++)
    {
        uint16 y = (row * IMAGE_PROCESS_DISPLAY_HEIGHT) / MT9V03X_H;
        uint16 left_x = (image_left_edge[row] * IMAGE_PROCESS_DISPLAY_WIDTH) / MT9V03X_W;
        uint16 right_x = (image_right_edge[row] * IMAGE_PROCESS_DISPLAY_WIDTH) / MT9V03X_W;
        uint16 mid_x = (image_mid_line[row] * IMAGE_PROCESS_DISPLAY_WIDTH) / MT9V03X_W;
        uint16 ref_x = (image_reference_col * IMAGE_PROCESS_DISPLAY_WIDTH) / MT9V03X_W;

        ips200_draw_point(left_x, y, RGB565_RED);
        ips200_draw_point(right_x, y, RGB565_BLUE);
        ips200_draw_point(mid_x, y, RGB565_GREEN);
        ips200_draw_point(ref_x, y, RGB565_YELLOW);
    }

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0U, 160U, "MID:");
    ips200_show_uint(40U, 160U, image_final_mid, 3U);
    ips200_show_string(88U, 160U, "REF:");
    ips200_show_uint(128U, 160U, image_reference_col, 3U);
    ips200_show_string(0U, 176U, "M:");
    ips200_show_uint(16U, 176U, image_process_track_mode, 1U);
    ips200_show_string(40U, 176U, "L:");
    ips200_show_uint(56U, 176U, image_process_left_edge_ok, 1U);
    ips200_show_string(80U, 176U, "R:");
    ips200_show_uint(96U, 176U, image_process_right_edge_ok, 1U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 192U, "R:RED B:BLUE G:MID");
    ips200_show_string(0U, 208U, "M:MODE L/R:EDGE KEY4");
}

bool image_process_take_new_result(void)
{
    if(!image_new_result)
    {
        return false;
    }

    image_new_result = false;
    return true;
}

uint8 image_process_get_final_mid(void)
{
    return image_final_mid;
}

uint8 image_process_get_reference_col(void)
{
    return image_reference_col;
}

uint8 image_process_get_reference_gray(void)
{
    return image_reference_gray;
}

uint8 image_process_get_white_min(void)
{
    return image_white_min;
}

uint8 image_process_get_white_max(void)
{
    return image_white_max;
}
