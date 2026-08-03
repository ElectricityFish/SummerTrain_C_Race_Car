#include "zf_common_headfile.h"
#include "Image_Process.h"

#define IMAGE_PROCESS_WEIGHT_BASE     (1U)
#define IMAGE_PROCESS_DISPLAY_WIDTH   (240U)
#define IMAGE_PROCESS_DISPLAY_HEIGHT  (153U)
#define IMAGE_PROCESS_SMALL_S_BANDS   (4U)
#define IMAGE_PROCESS_SMALL_S_VALID_PERCENT (60U)
#define IMAGE_PROCESS_SMALL_S_ENTER_FRAMES  (2U)
#define IMAGE_PROCESS_SMALL_S_EXIT_FRAMES   (3U)

Image_Process_Config image_process_config;

uint16 image_left_edge[MT9V03X_H];
uint16 image_right_edge[MT9V03X_H];
uint8 image_mid_line[MT9V03X_H];
uint8 image_small_s_active;

static uint8 image_reference_col;
static uint8 image_reference_gray;
static uint8 image_white_min;
static uint8 image_white_max;
static uint8 image_final_mid;
static uint8 image_last_final_mid;
static bool image_has_last_mid;
static bool image_new_result;
static uint8 image_small_s_detect_count;
static uint8 image_small_s_miss_count;

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

// 统计指定纵向观察带内双边都有效时的平均中点。边线落在图像边界表示该边未真正找到。
static bool image_process_get_dual_edge_band_mid(uint8 start_percent, uint8 end_percent, uint8 *band_mid)
{
    uint16 start_row = ((uint16)MT9V03X_H * start_percent) / 100U;
    uint16 end_row = ((uint16)MT9V03X_H * end_percent) / 100U;
    uint32 mid_sum = 0U;
    uint16 valid_count = 0U;
    uint16 row_count;
    uint16 row;

    if(end_row > MT9V03X_H)
    {
        end_row = MT9V03X_H;
    }
    if(end_row <= start_row)
    {
        return false;
    }

    row_count = end_row - start_row;
    for(row = start_row; row < end_row; row++)
    {
        if((image_left_edge[row] > 0U)
            && (image_right_edge[row] < MT9V03X_W - 1U))
        {
            mid_sum += (image_left_edge[row] + image_right_edge[row]) / 2U;
            valid_count++;
        }
    }

    if((valid_count == 0U)
        || ((uint32)valid_count * 100U
            < (uint32)row_count * IMAGE_PROCESS_SMALL_S_VALID_PERCENT))
    {
        return false;
    }

    *band_mid = (uint8)((mid_sum + valid_count / 2U) / valid_count);
    return true;
}

// 连续小 S 的中线会沿图像纵向连续两次反向，而普通直道/单弯通常不会。
static bool image_process_detect_small_s(void)
{
    // 对 120 行图像约对应 14~29、35~49、55~69、75~95 行。
    static const uint8 band_start_percent[IMAGE_PROCESS_SMALL_S_BANDS] = {12U, 29U, 46U, 63U};
    static const uint8 band_end_percent[IMAGE_PROCESS_SMALL_S_BANDS] = {25U, 42U, 59U, 80U};
    uint8 band_mid[IMAGE_PROCESS_SMALL_S_BANDS];
    uint8 min_mid;
    uint8 max_mid;
    uint8 min_swing;
    uint8 max_swing;
    int8 direction[IMAGE_PROCESS_SMALL_S_BANDS - 1U];
    uint8 index;

    if(image_process_config.small_s_enable == 0U)
    {
        return false;
    }

    for(index = 0U; index < IMAGE_PROCESS_SMALL_S_BANDS; index++)
    {
        if(!image_process_get_dual_edge_band_mid(
            band_start_percent[index], band_end_percent[index], &band_mid[index]))
        {
            return false;
        }
    }

    min_swing = image_process_limit_u8(
        image_process_config.small_s_min_swing, 1U, MT9V03X_W / 2U);
    max_swing = image_process_limit_u8(
        image_process_config.small_s_max_swing, min_swing, MT9V03X_W - 1U);

    min_mid = band_mid[0];
    max_mid = band_mid[0];
    for(index = 1U; index < IMAGE_PROCESS_SMALL_S_BANDS; index++)
    {
        int16 delta = (int16)band_mid[index] - band_mid[index - 1U];

        if(band_mid[index] < min_mid)
        {
            min_mid = band_mid[index];
        }
        if(band_mid[index] > max_mid)
        {
            max_mid = band_mid[index];
        }

        if(delta >= (int16)min_swing)
        {
            direction[index - 1U] = 1;
        }
        else if(delta <= -(int16)min_swing)
        {
            direction[index - 1U] = -1;
        }
        else
        {
            return false;
        }
    }

    if((uint8)(max_mid - min_mid) > max_swing)
    {
        return false;
    }

    return (direction[0] != direction[1]) && (direction[1] != direction[2]);
}

static void image_process_update_small_s(void)
{
    bool detected = image_process_detect_small_s();

    if(image_process_config.small_s_enable == 0U)
    {
        image_small_s_active = 0U;
        image_small_s_detect_count = 0U;
        image_small_s_miss_count = 0U;
        return;
    }

    if(detected)
    {
        image_small_s_miss_count = 0U;
        if(image_small_s_active == 0U)
        {
            if(image_small_s_detect_count < IMAGE_PROCESS_SMALL_S_ENTER_FRAMES)
            {
                image_small_s_detect_count++;
            }
            if(image_small_s_detect_count >= IMAGE_PROCESS_SMALL_S_ENTER_FRAMES)
            {
                image_small_s_active = 1U;
            }
        }
    }
    else
    {
        image_small_s_detect_count = 0U;
        if(image_small_s_active != 0U)
        {
            if(image_small_s_miss_count < IMAGE_PROCESS_SMALL_S_EXIT_FRAMES)
            {
                image_small_s_miss_count++;
            }
            if(image_small_s_miss_count >= IMAGE_PROCESS_SMALL_S_EXIT_FRAMES)
            {
                image_small_s_active = 0U;
            }
        }
    }
}

static void image_process_calculate_mid(void)
{
    uint32 weighted_sum = 0U;
    uint16 weight_sum = 0U;
    uint16 row;
    uint8 current_weight = image_process_limit_u8(image_process_config.mid_filter_current, 0U, 100U);
    uint8 current_mid;
    uint8 small_s_lower = 0U;
    uint8 small_s_upper = MT9V03X_W - 1U;

    for(row = 0U; row < MT9V03X_H; row++)
    {
        uint8 weight = image_process_row_weight((uint8)row);

        image_mid_line[row] = (uint8)((image_left_edge[row] + image_right_edge[row]) / 2U);
        weighted_sum += (uint32)image_mid_line[row] * weight;
        weight_sum += weight;
    }

    current_mid = (uint8)(weighted_sum / weight_sum);
    if(image_small_s_active != 0U)
    {
        uint8 error_limit = image_process_limit_u8(
            image_process_config.small_s_error_limit, 0U, MT9V03X_W / 2U);

        small_s_lower = image_process_limit_u8(
            (int32)(MT9V03X_W / 2U) - error_limit, 0U, MT9V03X_W - 1U);
        small_s_upper = image_process_limit_u8(
            (int32)(MT9V03X_W / 2U) + error_limit, 0U, MT9V03X_W - 1U);
        current_mid = image_process_limit_u8(current_mid, small_s_lower, small_s_upper);
    }
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
    // 再限一次，避免进入小 S 的第一帧仍残留上一帧滤波值造成大打角。
    if(image_small_s_active != 0U)
    {
        image_final_mid = image_process_limit_u8(image_final_mid, small_s_lower, small_s_upper);
    }
    image_last_final_mid = image_final_mid;
}

void image_process_init(void)
{
    image_process_config.reference_rows = 5U;
    image_process_config.reference_cols = 80U;
    image_process_config.black_threshold = 50U;
    image_process_config.white_min_scale = 7U;
    image_process_config.white_max_scale = 13U;
    image_process_config.contrast_threshold = 20U;
    image_process_config.contrast_offset = 3U;
    image_process_config.search_range = 10U;
    image_process_config.weight_center_row = 85U;
    image_process_config.weight_span = 35U;
    image_process_config.weight_peak = 20U;
    image_process_config.mid_filter_current = 80U;
    image_process_config.small_s_enable = 1U;
    image_process_config.small_s_min_swing = 5U;
    image_process_config.small_s_max_swing = 28U;
    image_process_config.small_s_error_limit = 2U;

    memset(image_left_edge, 0, sizeof(image_left_edge));
    memset(image_right_edge, 0, sizeof(image_right_edge));
    memset(image_mid_line, 0, sizeof(image_mid_line));
    image_reference_col = MT9V03X_W / 2U;
    image_reference_gray = 0U;
    image_white_min = 0U;
    image_white_max = 0U;
    image_final_mid = MT9V03X_W / 2U;
    image_last_final_mid = image_final_mid;
    image_has_last_mid = false;
    image_new_result = false;
    image_small_s_active = 0U;
    image_small_s_detect_count = 0U;
    image_small_s_miss_count = 0U;
}

void image_process_frame(void)
{
    const uint8 (*image)[MT9V03X_W] = (const uint8 (*)[MT9V03X_W])image_get_buffer();

    image_process_calculate_threshold(image);
    image_process_find_reference_col(image);
    image_process_track_edges(image);
    image_process_update_small_s();
    image_process_calculate_mid();
    image_new_result = true;
    image_process_finish_handler();
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
    ips200_show_string(176U, 160U, "S:");
    ips200_show_uint(216U, 160U, image_small_s_active, 1U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 176U, "R:RED B:BLUE G:MID");
    ips200_show_string(0U, 192U, "Y:REF  KEY4:BACK");
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
