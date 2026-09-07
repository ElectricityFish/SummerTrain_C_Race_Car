#include "zf_common_headfile.h"
#include "Image_Process.h"

#define IMAGE_PROCESS_WEIGHT_BASE     (1U)
#define IMAGE_PROCESS_DISPLAY_WIDTH   (240U)
#define IMAGE_PROCESS_DISPLAY_HEIGHT  (153U)

// 出界检测逐行统计图像底部5行；只有5行白色占比都不足才累计异常帧。
// 动态白色门限在出界时会随暗背景下降，因此保留绝对灰度下限。
#define IMAGE_OUT_BOUND_WHITE_GRAY_MIN       (100U)
#define IMAGE_OUT_BOUND_SAMPLE_ROWS           (5U)
#define IMAGE_OUT_BOUND_WHITE_RATIO_MIN      (15U)
#define IMAGE_OUT_BOUND_CONFIRM_FRAMES        (5U)

// 斑马线使用底部三条横向采样线识别重复黑白条纹，并优先于出界判定。
#define IMAGE_ZEBRA_SAMPLE_ROWS               (3U)
#define IMAGE_ZEBRA_SAMPLE_ROW_STEP           (5U)
#define IMAGE_ZEBRA_VALID_ROWS_MIN            (1U)
#define IMAGE_ZEBRA_WHITE_RATIO_MIN           (25U)
#define IMAGE_ZEBRA_WHITE_RATIO_MAX           (75U)
#define IMAGE_ZEBRA_TRANSITIONS_MIN           (5U)
#define IMAGE_ZEBRA_RUN_WIDTH_MIN             (4U)
#define IMAGE_ZEBRA_BLACK_RUNS_MIN            (3U)
#define IMAGE_ZEBRA_WHITE_RUNS_MIN            (3U)
#define IMAGE_ZEBRA_FILTER_RADIUS             (2U)
#define IMAGE_ZEBRA_HOLD_MISSED_FRAMES        (2U)

// 仅在当前约40 cm前瞻附近持续出现同向单边线时，给最终转向中线增加小幅弯内偏置。
#define IMAGE_CURVE_SINGLE_EDGE_ROW_TOP       (55U)
#define IMAGE_CURVE_SINGLE_EDGE_ROW_BOTTOM    (80U)
#define IMAGE_CURVE_SINGLE_EDGE_ROWS_MIN      (18U)
#define IMAGE_CURVE_INNER_BIAS_MAX            (10U)

// 十字识别与补线参数。只有连续确认后，补线结果才会参与中线控制。
#define IMAGE_CROSS_ROI_TOP                 (12U)
#define IMAGE_CROSS_ROI_BOTTOM              (75U)
#define IMAGE_CROSS_CORNER_SEARCH_TOP       (30U)
#define IMAGE_CROSS_CORNER_SEARCH_BOTTOM    (65U)
#define IMAGE_CROSS_CONTEXT_ROWS             (7U)
#define IMAGE_CROSS_ABOVE_INVALID_MIN        (2U)
#define IMAGE_CROSS_BELOW_VALID_MIN          (4U)
#define IMAGE_CROSS_CORNER_SIDE_MARGIN       (5U)
#define IMAGE_CROSS_CORNER_MIN_GAP           (18U)
#define IMAGE_CROSS_CORNER_MAX_GAP           (130U)
#define IMAGE_CROSS_LANE_ROW_MAX_GAP         (150U)
#define IMAGE_CROSS_CORNER_MAX_ROW_DIFF      (18U)
#define IMAGE_CROSS_MID_MAX_OFFSET           (36U)
#define IMAGE_CROSS_TANGENT_SEARCH_TOP       (30U)
#define IMAGE_CROSS_TANGENT_SEARCH_BOTTOM    (72U)
#define IMAGE_CROSS_TANGENT_HALF_WINDOW       (3U)
#define IMAGE_CROSS_TANGENT_MAX_STEP         (10U)
#define IMAGE_CROSS_TANGENT_EDGE_MARGIN       (2U)
#define IMAGE_CROSS_SLOPE_SCALE              (256)
#define IMAGE_CROSS_FULL_WIDTH_TOP           (42U)
#define IMAGE_CROSS_FULL_WIDTH_BOTTOM        (95U)
#define IMAGE_CROSS_FULL_WIDTH_WHITE_MIN     (MT9V03X_W - 12U)
#define IMAGE_CROSS_EDGE_SAMPLE_COLS          (8U)
#define IMAGE_CROSS_EDGE_WHITE_MIN            (7U)
#define IMAGE_CROSS_FULL_WIDTH_ROWS           (3U)
#define IMAGE_CROSS_CONFIRM_FRAMES           (2U)
#define IMAGE_CROSS_HOLD_MISSED_FRAMES       (2U)

Image_Process_Config image_process_config;

uint16 image_left_edge[MT9V03X_H];
uint16 image_right_edge[MT9V03X_H];
uint8 image_mid_line[MT9V03X_H];
bool image_left_edge_valid[MT9V03X_H];
bool image_right_edge_valid[MT9V03X_H];

static uint8 image_reference_col;
static uint8 image_reference_gray;
static uint8 image_white_min;
static uint8 image_white_max;
static uint8 image_final_mid;
static uint8 image_last_final_mid;
static bool image_has_last_mid;
static bool image_new_result;
static uint8 image_bottom_white_ratio;
static bool image_out_of_bounds;
static uint8 image_out_of_bounds_confirm_count;
static bool image_zebra_detected;
static uint8 image_zebra_missed_count;
static image_cross_state_enum image_cross_state;
static uint8 image_cross_left_col;
static uint8 image_cross_left_row;
static uint8 image_cross_right_col;
static uint8 image_cross_right_row;
static bool image_cross_corners_valid;
static uint8 image_cross_confirm_count;
static uint8 image_cross_missed_count;

// 独立逐行扫描的临时结果，避免十字检测受底部向上跟踪路径的影响。
static uint16 image_cross_scan_left[MT9V03X_H];
static uint16 image_cross_scan_right[MT9V03X_H];
static bool image_cross_scan_left_valid[MT9V03X_H];
static bool image_cross_scan_right_valid[MT9V03X_H];
static uint8 image_cross_scan_seed_col;

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

static uint8 image_process_get_feature_white_threshold(void)
{
    return (image_white_min < IMAGE_OUT_BOUND_WHITE_GRAY_MIN)
        ? IMAGE_OUT_BOUND_WHITE_GRAY_MIN
        : image_white_min;
}

// 对横向5像素窗口作多数表决，消除单像素和双像素噪点后再统计条纹。
static bool image_process_zebra_pixel_is_white(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    uint16 col,
    uint8 white_threshold)
{
    uint16 start_col = (col > IMAGE_ZEBRA_FILTER_RADIUS)
        ? col - IMAGE_ZEBRA_FILTER_RADIUS
        : 0U;
    uint16 end_col = col + IMAGE_ZEBRA_FILTER_RADIUS;
    uint8 white_count = 0U;
    uint8 sample_count = 0U;
    uint16 sample_col;

    if(end_col >= MT9V03X_W)
    {
        end_col = MT9V03X_W - 1U;
    }

    for(sample_col = start_col; sample_col <= end_col; sample_col++)
    {
        if(image[row][sample_col] >= white_threshold)
        {
            white_count++;
        }
        sample_count++;
    }

    return ((uint16)white_count * 2U >= (uint16)sample_count + 1U);
}

static bool image_process_zebra_row_is_valid(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    uint8 white_threshold)
{
    bool last_white = image_process_zebra_pixel_is_white(image, row, 0U, white_threshold);
    uint16 white_count = last_white ? 1U : 0U;
    uint16 run_width = 1U;
    uint8 transition_count = 0U;
    uint8 black_run_count = 0U;
    uint8 white_run_count = 0U;
    uint16 col;

    for(col = 1U; col < MT9V03X_W; col++)
    {
        bool current_white = image_process_zebra_pixel_is_white(
            image,
            row,
            col,
            white_threshold);

        if(current_white)
        {
            white_count++;
        }

        if(current_white == last_white)
        {
            run_width++;
            continue;
        }

        transition_count++;
        if(run_width >= IMAGE_ZEBRA_RUN_WIDTH_MIN)
        {
            if(last_white)
            {
                white_run_count++;
            }
            else
            {
                black_run_count++;
            }
        }
        last_white = current_white;
        run_width = 1U;
    }

    if(run_width >= IMAGE_ZEBRA_RUN_WIDTH_MIN)
    {
        if(last_white)
        {
            white_run_count++;
        }
        else
        {
            black_run_count++;
        }
    }

    return ((uint32)white_count * 100U
            >= (uint32)MT9V03X_W * IMAGE_ZEBRA_WHITE_RATIO_MIN)
        && ((uint32)white_count * 100U
            <= (uint32)MT9V03X_W * IMAGE_ZEBRA_WHITE_RATIO_MAX)
        && (transition_count >= IMAGE_ZEBRA_TRANSITIONS_MIN)
        && (black_run_count >= IMAGE_ZEBRA_BLACK_RUNS_MIN)
        && (white_run_count >= IMAGE_ZEBRA_WHITE_RUNS_MIN);
}

static void image_process_detect_zebra(const uint8 image[][MT9V03X_W])
{
    uint8 white_threshold = image_process_get_feature_white_threshold();
    uint8 valid_rows = 0U;
    uint8 sample;

    for(sample = 0U; sample < IMAGE_ZEBRA_SAMPLE_ROWS; sample++)
    {
        uint8 row = MT9V03X_H - 1U - sample * IMAGE_ZEBRA_SAMPLE_ROW_STEP;

        if(image_process_zebra_row_is_valid(image, row, white_threshold))
        {
            valid_rows++;
        }
    }

    if(valid_rows >= IMAGE_ZEBRA_VALID_ROWS_MIN)
    {
        image_zebra_detected = true;
        image_zebra_missed_count = 0U;
    }
    else if(image_zebra_detected
        && image_zebra_missed_count < IMAGE_ZEBRA_HOLD_MISSED_FRAMES)
    {
        // 短暂漏检时继续保持斑马线优先，避免同一条斑马线中途误触发出界保护。
        image_zebra_missed_count++;
    }
    else
    {
        image_zebra_detected = false;
        image_zebra_missed_count = 0U;
    }
}

// 底部5行全部白色不足并连续多帧出现时才确认离开赛道。
// 显示值取5行中的最大白色占比，判定使用交叉相乘避免除法误差。
static void image_process_detect_out_of_bounds(
    const uint8 image[][MT9V03X_W],
    bool monitor_enabled)
{
    uint8 white_threshold = image_process_get_feature_white_threshold();
    uint8 maximum_white_ratio = 0U;
    bool all_rows_below_threshold = true;
    uint8 sample;
    uint16 col;

    for(sample = 0U; sample < IMAGE_OUT_BOUND_SAMPLE_ROWS; sample++)
    {
        uint8 row = MT9V03X_H - 1U - sample;
        uint16 white_count = 0U;
        uint8 white_ratio;

        for(col = 0U; col < MT9V03X_W; col++)
        {
            if(image[row][col] >= white_threshold)
            {
                white_count++;
            }
        }

        white_ratio = (uint8)(((uint32)white_count * 100U) / MT9V03X_W);
        if(white_ratio > maximum_white_ratio)
        {
            maximum_white_ratio = white_ratio;
        }
        if((uint32)white_count * 100U
            >= (uint32)MT9V03X_W * IMAGE_OUT_BOUND_WHITE_RATIO_MIN)
        {
            all_rows_below_threshold = false;
        }
    }

    image_bottom_white_ratio = maximum_white_ratio;
    if(!monitor_enabled)
    {
        image_process_reset_out_of_bounds();
        return;
    }
    if(image_zebra_detected || !all_rows_below_threshold)
    {
        // 斑马线优先级最高；任意一行恢复正常也会打断连续出界计数。
        image_process_reset_out_of_bounds();
        return;
    }

    if(image_out_of_bounds_confirm_count < IMAGE_OUT_BOUND_CONFIRM_FRAMES)
    {
        image_out_of_bounds_confirm_count++;
    }
    image_out_of_bounds =
        (image_out_of_bounds_confirm_count >= IMAGE_OUT_BOUND_CONFIRM_FRAMES);
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

static uint16 image_process_find_left_edge(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    int16 start,
    int16 end,
    bool *found)
{
    uint8 offset = image_process_limit_u8(image_process_config.contrast_offset, 1U, 8U);
    uint8 threshold = image_process_config.contrast_threshold;
    int16 col;

    if(found != NULL)
    {
        *found = false;
    }

    start = image_process_limit_u8(start, offset, MT9V03X_W - 1U);
    end = image_process_limit_u8(end, offset, start);

    for(col = start; col >= end; col--)
    {
        uint8 inside = image[row][col];
        uint8 outside = image[row][col - offset];

        if(inside < image_white_min)
        {
            if(found != NULL)
            {
                *found = true;
            }
            return (uint16)col;
        }
        if(outside > image_white_max)
        {
            continue;
        }
        if(image_process_contrast(inside, outside) > threshold)
        {
            if(found != NULL)
            {
                *found = true;
            }
            return (uint16)(col - offset);
        }
    }

    return 0U;
}

static uint16 image_process_find_right_edge(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    int16 start,
    int16 end,
    bool *found)
{
    uint8 offset = image_process_limit_u8(image_process_config.contrast_offset, 1U, 8U);
    uint8 threshold = image_process_config.contrast_threshold;
    int16 col;

    if(found != NULL)
    {
        *found = false;
    }

    start = image_process_limit_u8(start, 0U, MT9V03X_W - 1U - offset);
    end = image_process_limit_u8(end, start, MT9V03X_W - 1U - offset);

    for(col = start; col <= end; col++)
    {
        uint8 inside = image[row][col];
        uint8 outside = image[row][col + offset];

        if(inside < image_white_min)
        {
            if(found != NULL)
            {
                *found = true;
            }
            return (uint16)col;
        }
        if(outside > image_white_max)
        {
            continue;
        }
        if(image_process_contrast(inside, outside) > threshold)
        {
            if(found != NULL)
            {
                *found = true;
            }
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
        bool left_valid;
        bool right_valid;

        left_edge = image_process_find_left_edge(image, (uint8)row, left_start, 0, &left_valid);
        // 保持原巡线行为：只要局部搜索落到图像边界，就从参考列再搜索一次。
        if((left_edge == 0U) && (left_start != image_reference_col))
        {
            left_edge = image_process_find_left_edge(
                image,
                (uint8)row,
                image_reference_col,
                0,
                &left_valid);
        }

        right_edge = image_process_find_right_edge(
            image,
            (uint8)row,
            right_start,
            MT9V03X_W - 1U,
            &right_valid);
        if((right_edge == MT9V03X_W - 1U) && (right_start != image_reference_col))
        {
            right_edge = image_process_find_right_edge(
                image,
                (uint8)row,
                image_reference_col,
                MT9V03X_W - 1U,
                &right_valid);
        }

        image_left_edge[row] = left_edge;
        image_right_edge[row] = right_edge;
        image_left_edge_valid[row] = left_valid;
        image_right_edge_valid[row] = right_valid;

        left_start = image_process_limit_u8((int16)left_edge + range, 0U, MT9V03X_W - 1U);
        right_start = image_process_limit_u8((int16)right_edge - range, 0U, MT9V03X_W - 1U);
    }
}

// 横向宽白带结束后，纵向赛道会重新形成一段可同时看到左右边界的过渡带。
static bool image_process_cross_lane_row_valid(uint8 row)
{
    uint16 left_col = image_cross_scan_left[row];
    uint16 right_col = image_cross_scan_right[row];
    uint16 gap;

    if(!image_cross_scan_left_valid[row] || !image_cross_scan_right_valid[row]
        || left_col + IMAGE_CROSS_CORNER_SIDE_MARGIN >= image_cross_scan_seed_col
        || right_col <= image_cross_scan_seed_col + IMAGE_CROSS_CORNER_SIDE_MARGIN
        || left_col >= right_col)
    {
        return false;
    }

    gap = right_col - left_col;
    return gap >= IMAGE_CROSS_CORNER_MIN_GAP && gap <= IMAGE_CROSS_LANE_ROW_MAX_GAP;
}

// 十字横道会在图像中下部形成连续的全宽白带；普通赛道只会在最底部因透视变宽。
static bool image_process_has_cross_white_band(const uint8 image[][MT9V03X_W])
{
    uint8 consecutive_rows = 0U;
    uint16 row;

    for(row = IMAGE_CROSS_FULL_WIDTH_TOP; row <= IMAGE_CROSS_FULL_WIDTH_BOTTOM; row++)
    {
        uint16 white_count = 0U;
        uint8 left_white = 0U;
        uint8 right_white = 0U;
        uint16 col;

        for(col = 0U; col < MT9V03X_W; col++)
        {
            if(image[row][col] >= image_white_min)
            {
                white_count++;
                if(col < IMAGE_CROSS_EDGE_SAMPLE_COLS)
                {
                    left_white++;
                }
                if(col >= MT9V03X_W - IMAGE_CROSS_EDGE_SAMPLE_COLS)
                {
                    right_white++;
                }
            }
        }

        if(white_count >= IMAGE_CROSS_FULL_WIDTH_WHITE_MIN
            && left_white >= IMAGE_CROSS_EDGE_WHITE_MIN
            && right_white >= IMAGE_CROSS_EDGE_WHITE_MIN)
        {
            consecutive_rows++;
            if(consecutive_rows >= IMAGE_CROSS_FULL_WIDTH_ROWS)
            {
                return true;
            }
        }
        else
        {
            consecutive_rows = 0U;
        }
    }

    return false;
}

// 在确认存在十字过渡带后，选择与“候选点到底角补线”方向最接近的轮廓切点。
static bool image_process_find_cross_corner(
    bool left_side,
    uint8 *corner_col,
    uint8 *corner_row)
{
    const uint16 *edge = left_side ? image_cross_scan_left : image_cross_scan_right;
    const bool *edge_valid = left_side
        ? image_cross_scan_left_valid
        : image_cross_scan_right_valid;
    int32 bottom_col = left_side ? 0 : (MT9V03X_W - 1U);
    uint16 best_score = 0xFFFFU;
    uint8 best_col = 0U;
    uint8 best_row = 0U;
    uint16 row;
    bool transition_found = false;
    bool tangent_found = false;

    // 原过渡带条件只负责证明这里存在十字拐角，不再用横坐标极值决定拐点。
    for(row = IMAGE_CROSS_CORNER_SEARCH_TOP; row <= IMAGE_CROSS_CORNER_SEARCH_BOTTOM; row++)
    {
        uint8 above_invalid = 0U;
        uint8 below_valid = 0U;
        uint16 sample;

        if(!image_process_cross_lane_row_valid((uint8)row))
        {
            continue;
        }

        for(sample = row - IMAGE_CROSS_CONTEXT_ROWS; sample < row; sample++)
        {
            if(!image_process_cross_lane_row_valid((uint8)sample))
            {
                above_invalid++;
            }
        }
        for(sample = row;
            sample <= row + IMAGE_CROSS_CONTEXT_ROWS && sample <= IMAGE_CROSS_ROI_BOTTOM;
            sample++)
        {
            if(image_process_cross_lane_row_valid((uint8)sample))
            {
                below_valid++;
            }
        }

        if(above_invalid < IMAGE_CROSS_ABOVE_INVALID_MIN
            || below_valid < IMAGE_CROSS_BELOW_VALID_MIN)
        {
            continue;
        }

        transition_found = true;
        break;
    }

    if(!transition_found)
    {
        return false;
    }

    for(row = IMAGE_CROSS_TANGENT_SEARCH_TOP; row <= IMAGE_CROSS_TANGENT_SEARCH_BOTTOM; row++)
    {
        int32 local_slope;
        int32 repair_slope;
        int32 slope_difference;
        uint16 score;
        uint16 sample;
        bool continuous = true;

        for(sample = row - IMAGE_CROSS_TANGENT_HALF_WINDOW;
            sample <= row + IMAGE_CROSS_TANGENT_HALF_WINDOW;
            sample++)
        {
            if(!edge_valid[sample]
                || edge[sample] <= IMAGE_CROSS_TANGENT_EDGE_MARGIN
                || edge[sample] + IMAGE_CROSS_TANGENT_EDGE_MARGIN >= MT9V03X_W)
            {
                continuous = false;
                break;
            }
        }
        if(!continuous)
        {
            continue;
        }

        for(sample = row - IMAGE_CROSS_TANGENT_HALF_WINDOW;
            sample < row + IMAGE_CROSS_TANGENT_HALF_WINDOW;
            sample++)
        {
            int32 step = (int32)edge[sample + 1U] - edge[sample];

            if(step < 0)
            {
                step = -step;
            }
            if(step > IMAGE_CROSS_TANGENT_MAX_STEP)
            {
                continuous = false;
                break;
            }
        }
        if(!continuous)
        {
            continue;
        }

        local_slope = ((int32)edge[row + IMAGE_CROSS_TANGENT_HALF_WINDOW]
            - edge[row - IMAGE_CROSS_TANGENT_HALF_WINDOW])
            * IMAGE_CROSS_SLOPE_SCALE
            / (int32)(2U * IMAGE_CROSS_TANGENT_HALF_WINDOW);
        repair_slope = (bottom_col - edge[row]) * IMAGE_CROSS_SLOPE_SCALE
            / (int32)((MT9V03X_H - 1U) - row);
        slope_difference = local_slope - repair_slope;
        if(slope_difference < 0)
        {
            slope_difference = -slope_difference;
        }
        score = (uint16)slope_difference;

        if(!tangent_found || score < best_score)
        {
            tangent_found = true;
            best_score = score;
            best_col = (uint8)edge[row];
            best_row = (uint8)row;
        }
    }

    if(!tangent_found)
    {
        return false;
    }

    *corner_col = best_col;
    *corner_row = best_row;
    return true;
}

static bool image_process_find_cross_pair(
    const uint8 image[][MT9V03X_W],
    uint8 seed_col,
    uint8 *left_col,
    uint8 *left_row,
    uint8 *right_col,
    uint8 *right_row)
{
    uint16 row;
    bool left_found;
    bool right_found;

    image_cross_scan_seed_col = seed_col;
    // 每一行都从同一个种子列重新扫描，避免普通边线跟踪在横向支路上越走越远。
    for(row = IMAGE_CROSS_ROI_TOP; row <= IMAGE_CROSS_ROI_BOTTOM; row++)
    {
        image_cross_scan_left[row] = image_process_find_left_edge(
            image,
            (uint8)row,
            seed_col,
            0,
            &image_cross_scan_left_valid[row]);
        image_cross_scan_right[row] = image_process_find_right_edge(
            image,
            (uint8)row,
            seed_col,
            MT9V03X_W - 1U,
            &image_cross_scan_right_valid[row]);
    }

    left_found = image_process_find_cross_corner(true, left_col, left_row);
    right_found = image_process_find_cross_corner(false, right_col, right_row);

    if(left_found && right_found && *left_col < *right_col)
    {
        uint8 row_difference = image_process_abs_diff(*left_row, *right_row);
        uint16 gap = (uint16)*right_col - *left_col;
        uint8 corner_mid = (uint8)(((uint16)*left_col + *right_col) / 2U);

        if(gap >= IMAGE_CROSS_CORNER_MIN_GAP
            && gap <= IMAGE_CROSS_CORNER_MAX_GAP
            && row_difference <= IMAGE_CROSS_CORNER_MAX_ROW_DIFF
            && image_process_abs_diff(corner_mid, seed_col)
                <= IMAGE_CROSS_MID_MAX_OFFSET)
        {
            return true;
        }
    }

    return false;
}

static void image_process_detect_cross(const uint8 image[][MT9V03X_W])
{
    uint8 left_col = 0U;
    uint8 left_row = 0U;
    uint8 right_col = 0U;
    uint8 right_row = 0U;
    bool pair_valid;

    pair_valid = false;
    if(image_process_has_cross_white_band(image))
    {
        // 优先沿用当前参考列，使已经验证准确的正入、偏左姿态保持原角点。
        pair_valid = image_process_find_cross_pair(
            image,
            image_reference_col,
            &left_col,
            &left_row,
            &right_col,
            &right_row);

        // 斜入时参考列可能落入左/右支路；主搜索失败后用固定画面中心兜底。
        if(!pair_valid && image_reference_col != MT9V03X_W / 2U)
        {
            pair_valid = image_process_find_cross_pair(
                image,
                MT9V03X_W / 2U,
                &left_col,
                &left_row,
                &right_col,
                &right_row);
        }
    }

    if(pair_valid)
    {
        image_cross_left_col = left_col;
        image_cross_left_row = left_row;
        image_cross_right_col = right_col;
        image_cross_right_row = right_row;
        image_cross_corners_valid = true;
        image_cross_missed_count = 0U;
        if(image_cross_confirm_count < IMAGE_CROSS_CONFIRM_FRAMES)
        {
            image_cross_confirm_count++;
        }
        image_cross_state = (image_cross_confirm_count >= IMAGE_CROSS_CONFIRM_FRAMES)
            ? IMAGE_CROSS_STATE_DETECTED
            : IMAGE_CROSS_STATE_CANDIDATE;
    }
    else if(image_cross_state == IMAGE_CROSS_STATE_DETECTED
        && image_cross_missed_count < IMAGE_CROSS_HOLD_MISSED_FRAMES)
    {
        // 短暂丢失时保留已确认的角点，防止调试标记一帧一闪。
        image_cross_missed_count++;
    }
    else
    {
        image_cross_state = IMAGE_CROSS_STATE_NONE;
        image_cross_corners_valid = false;
        image_cross_confirm_count = 0U;
        image_cross_missed_count = 0U;
    }
}

static uint16 image_process_cross_interpolate_edge(
    uint8 corner_col,
    uint8 corner_row,
    uint16 bottom_col,
    uint8 row)
{
    int32 row_span = (int32)(MT9V03X_H - 1U) - corner_row;
    int32 row_offset = (int32)row - corner_row;
    int32 col_span = (int32)bottom_col - corner_col;
    int32 col = corner_col;

    if(row_span > 0)
    {
        col += (col_span * row_offset) / row_span;
    }
    return image_process_limit_u8(col, 0U, MT9V03X_W - 1U);
}

// 学长方案：两个上拐点分别连接图像左下角和右下角，恢复十字中的纵向走廊。
static void image_process_apply_cross_repair(void)
{
    uint8 start_row;
    uint16 row;

    if(image_cross_state != IMAGE_CROSS_STATE_DETECTED || !image_cross_corners_valid)
    {
        return;
    }

    // 两条补线必须从同一行开始参与中线，避免两角高度不同时只修补单侧。
    start_row = (image_cross_left_row > image_cross_right_row)
        ? image_cross_left_row
        : image_cross_right_row;

    for(row = start_row; row < MT9V03X_H; row++)
    {
        image_left_edge[row] = image_process_cross_interpolate_edge(
            image_cross_left_col,
            image_cross_left_row,
            0U,
            (uint8)row);
        image_right_edge[row] = image_process_cross_interpolate_edge(
            image_cross_right_col,
            image_cross_right_row,
            MT9V03X_W - 1U,
            (uint8)row);
        image_left_edge_valid[row] = true;
        image_right_edge_valid[row] = true;
    }
}

// 返回 1 表示只见左边线的右弯，-1 表示只见右边线的左弯，0 表示不启用弯内偏置。
static int8 image_process_get_curve_inner_direction(void)
{
    uint8 left_only_rows = 0U;
    uint8 right_only_rows = 0U;
    uint16 row;

    // 特殊元素保持各自原有控制逻辑，不让普通弯道偏置介入。
    if(image_cross_state != IMAGE_CROSS_STATE_NONE || image_zebra_detected)
    {
        return 0;
    }

    for(row = IMAGE_CURVE_SINGLE_EDGE_ROW_TOP;
        row <= IMAGE_CURVE_SINGLE_EDGE_ROW_BOTTOM && row < MT9V03X_H;
        row++)
    {
        if(image_left_edge_valid[row] && !image_right_edge_valid[row])
        {
            left_only_rows++;
        }
        else if(!image_left_edge_valid[row] && image_right_edge_valid[row])
        {
            right_only_rows++;
        }
    }

    if(left_only_rows >= IMAGE_CURVE_SINGLE_EDGE_ROWS_MIN
        && left_only_rows > right_only_rows)
    {
        return 1;
    }
    if(right_only_rows >= IMAGE_CURVE_SINGLE_EDGE_ROWS_MIN
        && right_only_rows > left_only_rows)
    {
        return -1;
    }
    return 0;
}

static void image_process_calculate_mid(void)
{
    uint32 weighted_sum = 0U;
    uint16 weight_sum = 0U;
    uint16 row;
    uint8 current_weight = image_process_limit_u8(image_process_config.mid_filter_current, 0U, 100U);
    uint8 curve_inner_bias = image_process_limit_u8(
        image_process_config.curve_inner_bias,
        0U,
        IMAGE_CURVE_INNER_BIAS_MAX);
    int8 curve_inner_direction;
    uint8 current_mid;

    for(row = 0U; row < MT9V03X_H; row++)
    {
        uint8 weight = image_process_row_weight((uint8)row);

        image_mid_line[row] = (uint8)((image_left_edge[row] + image_right_edge[row]) / 2U);
        weighted_sum += (uint32)image_mid_line[row] * weight;
        weight_sum += weight;
    }

    current_mid = (uint8)(weighted_sum / weight_sum);
    curve_inner_direction = image_process_get_curve_inner_direction();
    current_mid = image_process_limit_u8(
        (int32)current_mid + (int32)curve_inner_direction * curve_inner_bias,
        0U,
        MT9V03X_W - 1U);
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
    image_process_config.reference_rows = 5U;
    image_process_config.reference_cols = 80U;
    image_process_config.black_threshold = 50U;
    image_process_config.white_min_scale = 7U;
    image_process_config.white_max_scale = 13U;
    image_process_config.contrast_threshold = 20U;
    image_process_config.contrast_offset = 3U;
    image_process_config.search_range = 10U;
    image_process_config.weight_center_row = 68U;
    image_process_config.weight_span = 35U;
    image_process_config.weight_peak = 20U;
    image_process_config.mid_filter_current = 80U;
    image_process_config.curve_inner_bias = 10U;

    memset(image_left_edge, 0, sizeof(image_left_edge));
    memset(image_right_edge, 0, sizeof(image_right_edge));
    memset(image_mid_line, 0, sizeof(image_mid_line));
    memset(image_left_edge_valid, 0, sizeof(image_left_edge_valid));
    memset(image_right_edge_valid, 0, sizeof(image_right_edge_valid));
    memset(image_cross_scan_left_valid, 0, sizeof(image_cross_scan_left_valid));
    memset(image_cross_scan_right_valid, 0, sizeof(image_cross_scan_right_valid));
    image_reference_col = MT9V03X_W / 2U;
    image_reference_gray = 0U;
    image_white_min = 0U;
    image_white_max = 0U;
    image_final_mid = MT9V03X_W / 2U;
    image_last_final_mid = image_final_mid;
    image_has_last_mid = false;
    image_new_result = false;
    image_bottom_white_ratio = 100U;
    image_out_of_bounds = false;
    image_out_of_bounds_confirm_count = 0U;
    image_zebra_detected = false;
    image_zebra_missed_count = 0U;
    image_cross_state = IMAGE_CROSS_STATE_NONE;
    image_cross_left_col = 0U;
    image_cross_left_row = 0U;
    image_cross_right_col = 0U;
    image_cross_right_row = 0U;
    image_cross_corners_valid = false;
    image_cross_confirm_count = 0U;
    image_cross_missed_count = 0U;
    image_cross_scan_seed_col = MT9V03X_W / 2U;
}

void image_process_frame(bool out_of_bounds_monitor_enabled)
{
    const uint8 (*image)[MT9V03X_W] = (const uint8 (*)[MT9V03X_W])image_get_buffer();

    image_process_calculate_threshold(image);
    image_process_detect_zebra(image);
    image_process_detect_out_of_bounds(image, out_of_bounds_monitor_enabled);
    image_process_find_reference_col(image);
    image_process_track_edges(image);
    image_process_detect_cross(image);
    image_process_apply_cross_repair();
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

        if(image_left_edge_valid[row])
        {
            ips200_draw_point(left_x, y, RGB565_RED);
        }
        if(image_right_edge_valid[row])
        {
            ips200_draw_point(right_x, y, RGB565_BLUE);
        }
        ips200_draw_point(mid_x, y, RGB565_GREEN);
        ips200_draw_point(ref_x, y, RGB565_YELLOW);
    }

    if(image_cross_corners_valid)
    {
        uint16 left_x = ((uint16)image_cross_left_col * IMAGE_PROCESS_DISPLAY_WIDTH) / MT9V03X_W;
        uint16 left_y = ((uint16)image_cross_left_row * IMAGE_PROCESS_DISPLAY_HEIGHT) / MT9V03X_H;
        uint16 right_x = ((uint16)image_cross_right_col * IMAGE_PROCESS_DISPLAY_WIDTH) / MT9V03X_W;
        uint16 right_y = ((uint16)image_cross_right_row * IMAGE_PROCESS_DISPLAY_HEIGHT) / MT9V03X_H;

        ips200_draw_line(left_x - 3U, left_y, left_x + 3U, left_y, RGB565_MAGENTA);
        ips200_draw_line(left_x, left_y - 3U, left_x, left_y + 3U, RGB565_MAGENTA);
        ips200_draw_line(right_x - 3U, right_y, right_x + 3U, right_y, RGB565_CYAN);
        ips200_draw_line(right_x, right_y - 3U, right_x, right_y + 3U, RGB565_CYAN);
    }

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0U, 160U, "MID:");
    ips200_show_uint(40U, 160U, image_final_mid, 3U);
    ips200_show_string(88U, 160U, "REF:");
    ips200_show_uint(128U, 160U, image_reference_col, 3U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 176U, "R:RED B:BLUE G:MID");
    ips200_show_string(0U, 192U, "Y:REF  KEY4:BACK");
    ips200_show_string(0U, 208U, "CROSS:");
    if(image_cross_state == IMAGE_CROSS_STATE_DETECTED)
    {
        ips200_show_string(48U, 208U, "YES ");
    }
    else if(image_cross_state == IMAGE_CROSS_STATE_CANDIDATE)
    {
        ips200_show_string(48U, 208U, "CAND");
    }
    else
    {
        ips200_show_string(48U, 208U, "NONE");
    }

    ips200_show_string(0U, 224U, "BOT:");
    ips200_show_uint(40U, 224U, image_bottom_white_ratio, 3U);
    ips200_show_string(64U, 224U, "%");
    ips200_show_string(72U, 224U, "OUT:");
    ips200_show_string(112U, 224U, image_out_of_bounds ? "YES" : "NO ");
    ips200_show_string(0U, 240U, "ZEBRA:");
    ips200_show_string(56U, 240U, image_zebra_detected ? "YES" : "NO ");
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

uint8 image_process_get_bottom_white_ratio(void)
{
    return image_bottom_white_ratio;
}

bool image_process_is_out_of_bounds(void)
{
    return image_out_of_bounds;
}

void image_process_reset_out_of_bounds(void)
{
    image_out_of_bounds = false;
    image_out_of_bounds_confirm_count = 0U;
}

bool image_process_is_zebra_detected(void)
{
    return image_zebra_detected;
}

image_cross_state_enum image_process_get_cross_state(void)
{
    return image_cross_state;
}

bool image_process_get_cross_corners(
    uint8 *left_col,
    uint8 *left_row,
    uint8 *right_col,
    uint8 *right_row)
{
    if(!image_cross_corners_valid)
    {
        return false;
    }

    if(left_col != NULL)
    {
        *left_col = image_cross_left_col;
    }
    if(left_row != NULL)
    {
        *left_row = image_cross_left_row;
    }
    if(right_col != NULL)
    {
        *right_col = image_cross_right_col;
    }
    if(right_row != NULL)
    {
        *right_row = image_cross_right_row;
    }
    return true;
}
