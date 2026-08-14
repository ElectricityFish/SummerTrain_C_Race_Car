#include "zf_common_headfile.h"
#include "Image_Process.h"
#include "Image_Perspective_Table.h"

#if (MT9V03X_W != 188) || (MT9V03X_H != 120)
#error "Image_Perspective_Table.h is calibrated only for MT9V03X 188x120 output."
#endif

#if IMAGE_PROCESS_MAX_POINTS > 255U
#error "Image point counts use uint8 and must not exceed 255."
#endif

#define IMAGE_PROCESS_DISPLAY_WIDTH        (240U)
#define IMAGE_PROCESS_DISPLAY_HEIGHT       (153U)
#define IMAGE_LOCAL_BLOCK_HALF             (2)
#define IMAGE_SEED_ROW_BOTTOM              (116)
#define IMAGE_SEED_ROW_TOP                 (70)
#define IMAGE_SEED_ROW_STEP                (3)
#define IMAGE_SEED_CENTER_COL              (MT9V03X_W / 2U)
#define IMAGE_SEED_SAMPLE_ROW_TOP          (70U)
#define IMAGE_SEED_SAMPLE_ROW_BOTTOM       (90U)
#define IMAGE_SEED_SIDE_SAMPLE_WIDTH       (16U)
#define IMAGE_TRACE_MIN_MARGIN             (3)
#define IMAGE_CENTER_RESAMPLE_STEP         (2U)
#define IMAGE_TARGET_REACH_TOLERANCE_CM    (4U)
// 必须保持为有符号数：边界切向量 dx/dy 可能为负。
// 若这里保留 U 后缀，C 的通常算术转换会把负方向量转成巨大无符号数，
// 使右边界生成的中心线被饱和到最右列 187（最终 MID 恰好变成 140）。
#define IMAGE_TRACK_HALF_WIDTH_UNITS       ((int32)((225U + IMAGE_PERSPECTIVE_GRID_MM / 2U) / IMAGE_PERSPECTIVE_GRID_MM))

static const int8 image_trace_forward[4][2] =
{
    {-2, 0}, {0, 2}, {2, 0}, {0, -2}
};
static const int8 image_trace_left[4][2] =
{
    {-2, -2}, {-2, 2}, {2, 2}, {2, -2}
};
static const int8 image_trace_right[4][2] =
{
    {-2, 2}, {2, 2}, {2, -2}, {-2, -2}
};

Image_Process_Config image_process_config;

static Image_Track_Point image_left_border[IMAGE_PROCESS_MAX_POINTS];
static Image_Track_Point image_right_border[IMAGE_PROCESS_MAX_POINTS];
static Image_Track_Point image_centerline_image[IMAGE_PROCESS_MAX_POINTS];
static const uint8 *image_source_frame;
static Image_Bird_Point image_left_bird[IMAGE_PROCESS_MAX_POINTS];
static Image_Bird_Point image_right_bird[IMAGE_PROCESS_MAX_POINTS];
static Image_Bird_Point image_centerline_bird[IMAGE_PROCESS_MAX_POINTS];
static Image_Bird_Point image_resample_scratch[IMAGE_PROCESS_MAX_POINTS];

static uint8 image_left_border_count;
static uint8 image_right_border_count;
static uint8 image_left_bird_count;
static uint8 image_right_bird_count;
static uint8 image_centerline_count;
static uint8 image_left_seed_threshold;
static uint8 image_right_seed_threshold;
static bool image_seed_threshold_initialized;
static Image_Track_Point image_target_point;
static bool image_target_point_valid;
static uint8 image_last_final_mid;
static bool image_has_last_mid;
static bool image_new_result;
static bool image_cycle_counter_ready;
static Image_Process_Result image_process_result;
static volatile uint8 image_published_mid;
static volatile bool image_published_mid_valid;

static uint8 image_limit_u8(int32 value, uint8 lower, uint8 upper)
{
    if(value < (int32)lower)
    {
        return lower;
    }
    if(value > (int32)upper)
    {
        return upper;
    }
    return (uint8)value;
}

static void image_sanitize_config(void)
{
    uint8 maximum_lookahead_cm = (uint8)(
        (IMAGE_PERSPECTIVE_NEAR_X * IMAGE_PERSPECTIVE_GRID_MM) / 10U);

    if(image_process_config.min_border_points < 3U)
    {
        image_process_config.min_border_points = 3U;
    }
    if(image_process_config.min_border_points > IMAGE_PROCESS_MAX_POINTS)
    {
        image_process_config.min_border_points = IMAGE_PROCESS_MAX_POINTS;
    }
    if(image_process_config.resample_step == 0U)
    {
        image_process_config.resample_step = 1U;
    }
    if(image_process_config.lookahead_cm == 0U)
    {
        image_process_config.lookahead_cm = 1U;
    }
    if(image_process_config.lookahead_cm > maximum_lookahead_cm)
    {
        image_process_config.lookahead_cm = maximum_lookahead_cm;
    }
    if(image_process_config.target_gain_percent > 100U)
    {
        image_process_config.target_gain_percent = 100U;
    }
    if(image_process_config.target_filter_current > 100U)
    {
        image_process_config.target_filter_current = 100U;
    }
}

static uint16 image_sqrt_u32(uint32 value)
{
    uint32 result = 0U;
    uint32 bit = 1UL << 30;

    while(bit > value)
    {
        bit >>= 2;
    }
    while(bit != 0U)
    {
        if(value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (uint16)result;
}

static uint8 image_local_threshold(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    uint8 col)
{
    uint16 total = 0U;
    int16 sample_row;
    int16 sample_col;
    uint8 mean;

    for(sample_row = (int16)row - IMAGE_LOCAL_BLOCK_HALF;
        sample_row <= (int16)row + IMAGE_LOCAL_BLOCK_HALF;
        sample_row++)
    {
        for(sample_col = (int16)col - IMAGE_LOCAL_BLOCK_HALF;
            sample_col <= (int16)col + IMAGE_LOCAL_BLOCK_HALF;
            sample_col++)
        {
            total += image[sample_row][sample_col];
        }
    }

    mean = (uint8)(total / 25U);
    if(mean <= image_process_config.local_threshold_offset)
    {
        return 0U;
    }
    return (uint8)(mean - image_process_config.local_threshold_offset);
}

static bool image_seed_edge_has_contrast(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    uint8 col)
{
    uint8 left = image[row][col - 2U];
    uint8 right = image[row][col + 2U];
    uint16 difference = (left >= right) ? (left - right) : (right - left);
    uint16 sum = (uint16)left + right;

    if(difference < image_process_config.start_contrast_min || difference == 0U)
    {
        return false;
    }
    return (sum / difference <= 7U);
}

static uint8 image_region_average(
    const uint8 image[][MT9V03X_W],
    uint8 row0,
    uint8 row1,
    uint8 col0,
    uint8 col1)
{
    uint32 total = 0U;
    uint16 count = 0U;
    uint16 row;
    uint16 col;

    for(row = row0; row <= row1; row++)
    {
        for(col = col0; col <= col1; col++)
        {
            total += image[row][col];
            count++;
        }
    }
    return (uint8)(total / count);
}

static void image_update_seed_thresholds(const uint8 image[][MT9V03X_W])
{
    uint8 white = image_region_average(
        image,
        IMAGE_SEED_SAMPLE_ROW_TOP,
        IMAGE_SEED_SAMPLE_ROW_BOTTOM,
        IMAGE_SEED_CENTER_COL - 8U,
        IMAGE_SEED_CENTER_COL + 8U);
    uint8 left_black = image_region_average(
        image,
        IMAGE_SEED_SAMPLE_ROW_TOP,
        IMAGE_SEED_SAMPLE_ROW_BOTTOM,
        3U,
        3U + IMAGE_SEED_SIDE_SAMPLE_WIDTH - 1U);
    uint8 right_black = image_region_average(
        image,
        IMAGE_SEED_SAMPLE_ROW_TOP,
        IMAGE_SEED_SAMPLE_ROW_BOTTOM,
        MT9V03X_W - 3U - IMAGE_SEED_SIDE_SAMPLE_WIDTH,
        MT9V03X_W - 4U);
    uint8 left_target;
    uint8 right_target;

    left_target = (white > left_black + image_process_config.start_contrast_min)
        ? (uint8)(((uint16)white + left_black) / 2U)
        : image_left_seed_threshold;
    right_target = (white > right_black + image_process_config.start_contrast_min)
        ? (uint8)(((uint16)white + right_black) / 2U)
        : image_right_seed_threshold;

    if(!image_seed_threshold_initialized)
    {
        image_left_seed_threshold = left_target;
        image_right_seed_threshold = right_target;
        image_seed_threshold_initialized = true;
    }
    else
    {
        image_left_seed_threshold = (uint8)(((uint16)image_left_seed_threshold + left_target) / 2U);
        image_right_seed_threshold = (uint8)(((uint16)image_right_seed_threshold + right_target) / 2U);
    }
}

static bool image_find_left_seed(
    const uint8 image[][MT9V03X_W],
    Image_Track_Point *seed)
{
    int16 row;

    for(row = IMAGE_SEED_ROW_BOTTOM; row >= IMAGE_SEED_ROW_TOP; row -= IMAGE_SEED_ROW_STEP)
    {
        int16 left = IMAGE_TRACE_MIN_MARGIN;
        int16 right = IMAGE_SEED_CENTER_COL;

        while(left < right)
        {
            int16 middle = (left + right) >> 1;
            int16 col8 = middle - 8;
            int16 col25 = middle - 25;

            if(col8 < IMAGE_TRACE_MIN_MARGIN) col8 = IMAGE_TRACE_MIN_MARGIN;
            if(col25 < IMAGE_TRACE_MIN_MARGIN) col25 = IMAGE_TRACE_MIN_MARGIN;
            if(image[row][middle] < image_left_seed_threshold
                && image[row][col8] < image_left_seed_threshold
                && image[row][col25] < image_left_seed_threshold)
            {
                left = middle + 1;
            }
            else
            {
                right = middle;
            }
        }

        if(left >= IMAGE_TRACE_MIN_MARGIN
            && left < MT9V03X_W - IMAGE_TRACE_MIN_MARGIN
            && image[row][left + 2] >= image_left_seed_threshold
            && image_seed_edge_has_contrast(image, (uint8)row, (uint8)left))
        {
            seed->row = (uint8)row;
            seed->col = (uint8)(left + 1);
            return true;
        }
    }
    return false;
}

static bool image_find_right_seed(
    const uint8 image[][MT9V03X_W],
    Image_Track_Point *seed)
{
    int16 row;

    for(row = IMAGE_SEED_ROW_BOTTOM; row >= IMAGE_SEED_ROW_TOP; row -= IMAGE_SEED_ROW_STEP)
    {
        int16 left = IMAGE_SEED_CENTER_COL;
        int16 right = MT9V03X_W - 1 - IMAGE_TRACE_MIN_MARGIN;

        while(left < right)
        {
            int16 middle = (left + right + 1) >> 1;
            int16 col8 = middle + 8;
            int16 col25 = middle + 25;

            if(col8 > MT9V03X_W - 1 - IMAGE_TRACE_MIN_MARGIN)
                col8 = MT9V03X_W - 1 - IMAGE_TRACE_MIN_MARGIN;
            if(col25 > MT9V03X_W - 1 - IMAGE_TRACE_MIN_MARGIN)
                col25 = MT9V03X_W - 1 - IMAGE_TRACE_MIN_MARGIN;
            if(image[row][middle] < image_right_seed_threshold
                && image[row][col8] < image_right_seed_threshold
                && image[row][col25] < image_right_seed_threshold)
            {
                right = middle - 1;
            }
            else
            {
                left = middle;
            }
        }

        if(right >= IMAGE_TRACE_MIN_MARGIN
            && right < MT9V03X_W - IMAGE_TRACE_MIN_MARGIN
            && image[row][right - 2] >= image_right_seed_threshold
            && image_seed_edge_has_contrast(image, (uint8)row, (uint8)right))
        {
            seed->row = (uint8)row;
            seed->col = (uint8)(right - 1);
            return true;
        }
    }
    return false;
}

static uint8 image_trace_border(
    const uint8 image[][MT9V03X_W],
    Image_Track_Point seed,
    bool trace_left,
    Image_Track_Point output[IMAGE_PROCESS_MAX_POINTS])
{
    uint8 row = seed.row;
    uint8 col = seed.col;
    uint8 direction = 0U;
    uint8 turns = 0U;
    uint8 update_count = 0U;
    uint8 point_count = 0U;
    uint8 threshold = image_local_threshold(image, row, col);

    while(row > IMAGE_TRACE_MIN_MARGIN
        && row < MT9V03X_H - IMAGE_TRACE_MIN_MARGIN
        && col > IMAGE_TRACE_MIN_MARGIN
        && col < MT9V03X_W - IMAGE_TRACE_MIN_MARGIN
        && point_count < IMAGE_PROCESS_MAX_POINTS
        && turns < 4U)
    {
        int16 forward_row;
        int16 forward_col;
        int16 side_row;
        int16 side_col;
        const int8 (*side_direction)[2] = trace_left ? image_trace_left : image_trace_right;

        update_count++;
        if(update_count >= 2U)
        {
            threshold = image_local_threshold(image, row, col);
            update_count = 0U;
        }

        forward_row = (int16)row + image_trace_forward[direction][0];
        forward_col = (int16)col + image_trace_forward[direction][1];
        side_row = (int16)row + side_direction[direction][0];
        side_col = (int16)col + side_direction[direction][1];

        if(image[forward_row][forward_col] < threshold)
        {
            direction = trace_left ? ((direction + 1U) & 3U) : ((direction + 3U) & 3U);
            turns++;
        }
        else if(image[side_row][side_col] < threshold)
        {
            row = (uint8)forward_row;
            col = (uint8)forward_col;
            output[point_count].row = row;
            output[point_count].col = col;
            point_count++;
            turns = 0U;
        }
        else
        {
            row = (uint8)side_row;
            col = (uint8)side_col;
            output[point_count].row = row;
            output[point_count].col = col;
            point_count++;
            turns = 0U;
            direction = trace_left ? ((direction + 3U) & 3U) : ((direction + 1U) & 3U);
        }

        if(row == seed.row && col == seed.col && turns == 0U)
        {
            break;
        }
    }
    return point_count;
}

static void image_transform_border(
    const Image_Track_Point input[IMAGE_PROCESS_MAX_POINTS],
    uint8 count,
    Image_Bird_Point output[IMAGE_PROCESS_MAX_POINTS])
{
    uint8 index;

    for(index = 0U; index < count; index++)
    {
        uint32 lut_index = ((uint32)input[index].row * MT9V03X_W
            + input[index].col) * 2U;

        output[index].x = image_perspective_lut[lut_index];
        output[index].y = image_perspective_lut[lut_index + 1U];
    }
}

static void image_filter_bird_line(Image_Bird_Point line[IMAGE_PROCESS_MAX_POINTS], uint8 count)
{
    uint8 index;

    for(index = 1U; index < count; index++)
    {
        line[index].x = (uint8)(((uint16)line[index - 1U].x + line[index].x) / 2U);
        line[index].y = (uint8)(((uint16)line[index - 1U].y + line[index].y) / 2U);
    }
}

static uint8 image_resample_bird_line(
    Image_Bird_Point line[IMAGE_PROCESS_MAX_POINTS],
    uint8 count,
    uint8 distance)
{
    int16 remain = 0;
    uint8 output_count = 0U;
    uint8 index;

    if(count < 2U || distance == 0U)
    {
        return 0U;
    }

    for(index = 0U; index + 1U < count; index++)
    {
        int16 x0 = line[index].x;
        int16 y0 = line[index].y;
        int16 dx = (int16)line[index + 1U].x - x0;
        int16 dy = (int16)line[index + 1U].y - y0;
        uint16 length = image_sqrt_u32((uint32)(dx * dx + dy * dy));

        if(length == 0U)
        {
            continue;
        }
        while(output_count < IMAGE_PROCESS_MAX_POINTS)
        {
            int32 tx;
            int32 ty;

            if(remain >= (int16)length)
            {
                remain -= (int16)length;
                break;
            }
            tx = (int32)x0 * length + (int32)dx * remain;
            ty = (int32)y0 * length + (int32)dy * remain;
            image_resample_scratch[output_count].x = image_limit_u8(tx / length, 0U, MT9V03X_H - 1U);
            image_resample_scratch[output_count].y = image_limit_u8(ty / length, 0U, MT9V03X_W - 1U);
            output_count++;
            remain += distance;
        }
    }

    if(output_count == 0U)
    {
        return 0U;
    }
    memcpy(line, image_resample_scratch, (uint32)output_count * sizeof(Image_Bird_Point));
    return output_count;
}

static uint8 image_centerline_from_border(
    const Image_Bird_Point border[IMAGE_PROCESS_MAX_POINTS],
    uint8 count,
    bool from_left,
    Image_Bird_Point center[IMAGE_PROCESS_MAX_POINTS])
{
    uint8 index;

    if(count < 3U)
    {
        return 0U;
    }
    for(index = 1U; index < count; index++)
    {
        uint8 index0 = (index > 2U) ? (index - 2U) : 0U;
        uint8 index1 = (index + 2U < count) ? (index + 2U) : (count - 1U);
        int16 dx = (int16)border[index1].x - border[index0].x;
        int16 dy = (int16)border[index1].y - border[index0].y;
        uint16 length = image_sqrt_u32((uint32)(dx * dx + dy * dy));
        int32 center_x;
        int32 center_y;

        if(length == 0U)
        {
            center[index] = border[index];
            continue;
        }
        if(from_left)
        {
            center_x = (int32)border[index].x + ((int32)dy * IMAGE_TRACK_HALF_WIDTH_UNITS) / length;
            center_y = (int32)border[index].y - ((int32)dx * IMAGE_TRACK_HALF_WIDTH_UNITS) / length;
        }
        else
        {
            center_x = (int32)border[index].x - ((int32)dy * IMAGE_TRACK_HALF_WIDTH_UNITS) / length;
            center_y = (int32)border[index].y + ((int32)dx * IMAGE_TRACK_HALF_WIDTH_UNITS) / length;
        }
        center[index].x = image_limit_u8(center_x, 0U, MT9V03X_H - 1U);
        center[index].y = image_limit_u8(center_y, 0U, MT9V03X_W - 1U);
    }
    center[0] = center[1];
    return count;
}

static Image_Track_Point image_bird_to_image(Image_Bird_Point point)
{
    Image_Track_Point result;
    uint8 row = image_perspective_source_row[point.x];
    int16 lateral_units = (int16)point.y - (int16)IMAGE_PERSPECTIVE_CENTER_COL;
    int32 lateral_mm = (int32)lateral_units * (int32)IMAGE_PERSPECTIVE_GRID_MM;
    int32 lateral_q8 = lateral_mm * (int32)image_perspective_lane_width_q8[row]
        / (int32)IMAGE_PERSPECTIVE_TRACK_WIDTH_MM;
    int32 col_q8 = image_perspective_lane_center_q8[row]
        + lateral_q8;

    result.row = row;
    result.col = image_limit_u8((col_q8 + 128) / 256, 0U, MT9V03X_W - 1U);
    return result;
}

static uint8 image_find_target_index(uint8 lookahead_cm)
{
    uint16 lookahead_mm = (uint16)lookahead_cm * 10U;
    uint8 target_x = (lookahead_mm >= IMAGE_PERSPECTIVE_NEAR_X * IMAGE_PERSPECTIVE_GRID_MM)
        ? 0U
        : (uint8)(IMAGE_PERSPECTIVE_NEAR_X - lookahead_mm / IMAGE_PERSPECTIVE_GRID_MM);
    uint8 best_index = 0U;
    uint8 best_difference = 255U;
    uint8 index;

    for(index = 0U; index < image_centerline_count; index++)
    {
        uint8 difference = (image_centerline_bird[index].x >= target_x)
            ? (image_centerline_bird[index].x - target_x)
            : (target_x - image_centerline_bird[index].x);
        if(difference < best_difference)
        {
            best_difference = difference;
            best_index = index;
        }
    }
    return best_index;
}

static void image_clear_result(void)
{
    image_left_border_count = 0U;
    image_right_border_count = 0U;
    image_left_bird_count = 0U;
    image_right_bird_count = 0U;
    image_centerline_count = 0U;
    image_target_point_valid = false;
    image_process_result.source_frame_valid = false;
    image_process_result.centerline_valid = false;
    image_process_result.target_reached = false;
    image_process_result.confidence = 0U;
    image_process_result.valid_distance_cm = 0U;
    image_process_result.target_distance_cm = 0U;
    image_process_result.left_border_count = 0U;
    image_process_result.right_border_count = 0U;
    image_process_result.centerline_count = 0U;
    image_process_result.selected_side = IMAGE_TRACK_SIDE_NONE;
}

static void image_finish_frame(uint32 cycle_start, bool frame_processed)
{
    if(image_cycle_counter_ready && system_clock >= 1000000U)
    {
        uint32 elapsed_cycles = DWT->CYCCNT - cycle_start;
        uint32 elapsed_us = elapsed_cycles / (system_clock / 1000000U);

        image_process_result.process_time_us = (elapsed_us > 0xFFFFU)
            ? 0xFFFFU
            : (uint16)elapsed_us;
    }
    else
    {
        image_process_result.process_time_us = 0U;
    }
    image_new_result = true;
    if(frame_processed)
    {
        image_process_finish_handler();
    }
}

void image_process_init(void)
{
    image_process_config.local_threshold_offset = 6U;
    image_process_config.start_contrast_min = 15U;
    image_process_config.min_border_points = 12U;
    image_process_config.resample_step = 3U;
    image_process_config.lookahead_cm = 75U;
    image_process_config.target_gain_percent = 50U;
    image_process_config.target_filter_current = 80U;

    memset(image_left_border, 0, sizeof(image_left_border));
    memset(image_right_border, 0, sizeof(image_right_border));
    memset(image_centerline_image, 0, sizeof(image_centerline_image));
    image_source_frame = NULL;
    memset(image_left_bird, 0, sizeof(image_left_bird));
    memset(image_right_bird, 0, sizeof(image_right_bird));
    memset(image_centerline_bird, 0, sizeof(image_centerline_bird));
    memset(&image_process_result, 0, sizeof(image_process_result));
    image_left_seed_threshold = 128U;
    image_right_seed_threshold = 128U;
    image_seed_threshold_initialized = false;
    image_last_final_mid = IMAGE_PERSPECTIVE_CENTER_COL;
    image_has_last_mid = false;
    image_published_mid = IMAGE_PERSPECTIVE_CENTER_COL;
    image_published_mid_valid = false;
    image_new_result = false;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    image_cycle_counter_ready = ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U)
        && ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
    image_process_result.final_mid = IMAGE_PERSPECTIVE_CENTER_COL;
    image_clear_result();
}

void image_process_frame(void)
{
    const uint8 (*image)[MT9V03X_W];
    const uint8 *latest_frame;
    Image_Track_Point left_seed;
    Image_Track_Point right_seed;
    bool left_seed_valid;
    bool right_seed_valid;
    bool left_valid;
    bool right_valid;
    uint8 target_index;
    uint8 index;
    uint8 raw_mid;
    uint8 current_weight;
    uint16 target_distance_difference_cm;
    int32 target_offset;
    uint8 minimum_x = IMAGE_PERSPECTIVE_NEAR_X;
    uint8 chosen_count = 0U;
    uint32 cycle_start = image_cycle_counter_ready ? DWT->CYCCNT : 0U;

    image_clear_result();
    image_sanitize_config();
    image_release_frame(image_source_frame);
    image_source_frame = NULL;
    latest_frame = image_acquire_latest_frame();
    if(latest_frame == NULL)
    {
        image_process_result.final_mid = image_last_final_mid;
        image_published_mid_valid = false;
        image_finish_frame(cycle_start, false);
        return;
    }
    image_source_frame = latest_frame;
    image = (const uint8 (*)[MT9V03X_W])image_source_frame;
    image_process_result.source_frame_valid = true;
    image_update_seed_thresholds(image);
    left_seed_valid = image_find_left_seed(image, &left_seed);
    right_seed_valid = image_find_right_seed(image, &right_seed);

    if(left_seed_valid)
    {
        image_left_border_count = image_trace_border(
            image, left_seed, true, image_left_border);
    }
    if(right_seed_valid)
    {
        image_right_border_count = image_trace_border(
            image, right_seed, false, image_right_border);
    }

    left_valid = image_left_border_count >= image_process_config.min_border_points;
    right_valid = image_right_border_count >= image_process_config.min_border_points;
    if(left_valid)
    {
        image_transform_border(image_left_border, image_left_border_count, image_left_bird);
        image_left_bird_count = image_left_border_count;
        image_filter_bird_line(image_left_bird, image_left_bird_count);
        image_left_bird_count = image_resample_bird_line(
            image_left_bird, image_left_bird_count, image_process_config.resample_step);
        left_valid = image_left_bird_count >= image_process_config.min_border_points;
    }
    if(right_valid)
    {
        image_transform_border(image_right_border, image_right_border_count, image_right_bird);
        image_right_bird_count = image_right_border_count;
        image_filter_bird_line(image_right_bird, image_right_bird_count);
        image_right_bird_count = image_resample_bird_line(
            image_right_bird, image_right_bird_count, image_process_config.resample_step);
        right_valid = image_right_bird_count >= image_process_config.min_border_points;
    }

    if(left_valid && (!right_valid || image_left_bird_count >= image_right_bird_count))
    {
        image_centerline_count = image_centerline_from_border(
            image_left_bird, image_left_bird_count, true, image_centerline_bird);
        image_process_result.selected_side = IMAGE_TRACK_SIDE_LEFT;
        chosen_count = image_left_bird_count;
    }
    else if(right_valid)
    {
        image_centerline_count = image_centerline_from_border(
            image_right_bird, image_right_bird_count, false, image_centerline_bird);
        image_process_result.selected_side = IMAGE_TRACK_SIDE_RIGHT;
        chosen_count = image_right_bird_count;
    }
    else
    {
        image_process_result.final_mid = image_last_final_mid;
        image_process_result.left_border_count = image_left_border_count;
        image_process_result.right_border_count = image_right_border_count;
        image_published_mid_valid = false;
        image_finish_frame(cycle_start, true);
        return;
    }

    image_filter_bird_line(image_centerline_bird, image_centerline_count);
    image_centerline_count = image_resample_bird_line(
        image_centerline_bird, image_centerline_count, IMAGE_CENTER_RESAMPLE_STEP);
    if(image_centerline_count < image_process_config.min_border_points)
    {
        image_process_result.final_mid = image_last_final_mid;
        image_process_result.left_border_count = image_left_border_count;
        image_process_result.right_border_count = image_right_border_count;
        image_published_mid_valid = false;
        image_finish_frame(cycle_start, true);
        return;
    }

    for(index = 0U; index < image_centerline_count; index++)
    {
        image_centerline_image[index] = image_bird_to_image(image_centerline_bird[index]);
        if(image_centerline_bird[index].x < minimum_x)
        {
            minimum_x = image_centerline_bird[index].x;
        }
    }
    image_process_result.valid_distance_cm =
        (uint16)(((uint16)(IMAGE_PERSPECTIVE_NEAR_X - minimum_x) * IMAGE_PERSPECTIVE_GRID_MM) / 10U);

    target_index = image_find_target_index(image_process_config.lookahead_cm);
    image_target_point = image_centerline_image[target_index];
    image_target_point_valid = true;
    image_process_result.target_distance_cm = (uint16)(
        ((uint16)(IMAGE_PERSPECTIVE_NEAR_X - image_centerline_bird[target_index].x)
            * IMAGE_PERSPECTIVE_GRID_MM + 5U) / 10U);
    target_distance_difference_cm =
        (image_process_result.target_distance_cm >= image_process_config.lookahead_cm)
        ? (image_process_result.target_distance_cm - image_process_config.lookahead_cm)
        : (image_process_config.lookahead_cm - image_process_result.target_distance_cm);
    image_process_result.target_reached =
        target_distance_difference_cm <= IMAGE_TARGET_REACH_TOLERANCE_CM;
    target_offset = (int32)image_target_point.col - (int32)IMAGE_PERSPECTIVE_CENTER_COL;
    raw_mid = image_limit_u8(
        (int32)IMAGE_PERSPECTIVE_CENTER_COL
            + target_offset * (int32)image_process_config.target_gain_percent / 100,
        0U,
        MT9V03X_W - 1U);
    current_weight = image_process_config.target_filter_current;
    if(current_weight > 100U) current_weight = 100U;
    if(!image_has_last_mid)
    {
        image_process_result.final_mid = raw_mid;
        image_has_last_mid = true;
    }
    else
    {
        image_process_result.final_mid = (uint8)(
            ((uint16)raw_mid * current_weight
                + (uint16)image_last_final_mid * (100U - current_weight)
                + 50U) / 100U);
    }
    image_last_final_mid = image_process_result.final_mid;
    image_published_mid = image_process_result.final_mid;
    image_published_mid_valid = true;
    image_process_result.centerline_valid = true;
    image_process_result.left_border_count = image_left_border_count;
    image_process_result.right_border_count = image_right_border_count;
    image_process_result.centerline_count = image_centerline_count;
    image_process_result.confidence = image_limit_u8((uint32)chosen_count * 100U / 45U, 0U, 100U);
    if(image_process_result.valid_distance_cm < image_process_config.lookahead_cm)
    {
        image_process_result.confidence = (uint8)(
            (uint16)image_process_result.confidence
            * image_process_result.valid_distance_cm
            / image_process_config.lookahead_cm);
    }

    image_finish_frame(cycle_start, true);
}

void image_process_display(void)
{
    const uint8 *image = image_source_frame;
    uint8 index;

    if(image == NULL)
    {
        return;
    }

    ips200_show_gray_image(0U, 0U, image, MT9V03X_W, MT9V03X_H,
        IMAGE_PROCESS_DISPLAY_WIDTH, IMAGE_PROCESS_DISPLAY_HEIGHT, 0U);

    for(index = 0U; index < image_left_border_count; index++)
    {
        ips200_draw_point(
            (uint16)image_left_border[index].col * IMAGE_PROCESS_DISPLAY_WIDTH / MT9V03X_W,
            (uint16)image_left_border[index].row * IMAGE_PROCESS_DISPLAY_HEIGHT / MT9V03X_H,
            RGB565_RED);
    }
    for(index = 0U; index < image_right_border_count; index++)
    {
        ips200_draw_point(
            (uint16)image_right_border[index].col * IMAGE_PROCESS_DISPLAY_WIDTH / MT9V03X_W,
            (uint16)image_right_border[index].row * IMAGE_PROCESS_DISPLAY_HEIGHT / MT9V03X_H,
            RGB565_BLUE);
    }
    for(index = 0U; index < image_centerline_count; index++)
    {
        ips200_draw_point(
            (uint16)image_centerline_image[index].col * IMAGE_PROCESS_DISPLAY_WIDTH / MT9V03X_W,
            (uint16)image_centerline_image[index].row * IMAGE_PROCESS_DISPLAY_HEIGHT / MT9V03X_H,
            RGB565_GREEN);
    }
    if(image_target_point_valid)
    {
        uint16 x = (uint16)image_target_point.col * IMAGE_PROCESS_DISPLAY_WIDTH / MT9V03X_W;
        uint16 y = (uint16)image_target_point.row * IMAGE_PROCESS_DISPLAY_HEIGHT / MT9V03X_H;
        uint16 x_end = (x + 3U < IMAGE_PROCESS_DISPLAY_WIDTH)
            ? (x + 3U) : (IMAGE_PROCESS_DISPLAY_WIDTH - 1U);
        uint16 y_end = (y + 3U < IMAGE_PROCESS_DISPLAY_HEIGHT)
            ? (y + 3U) : (IMAGE_PROCESS_DISPLAY_HEIGHT - 1U);

        ips200_draw_line((x >= 3U) ? x - 3U : 0U, y, x_end, y, RGB565_YELLOW);
        ips200_draw_line(x, (y >= 3U) ? y - 3U : 0U, x, y_end, RGB565_YELLOW);
    }

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0U, 160U, "MID:");
    ips200_show_uint(40U, 160U, image_process_result.final_mid, 3U);
    ips200_show_string(88U, 160U, "VALID:");
    ips200_show_string(144U, 160U, image_process_result.centerline_valid ? "YES" : "NO ");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 176U, "L:");
    ips200_show_uint(16U, 176U, image_left_border_count, 3U);
    ips200_show_string(48U, 176U, "R:");
    ips200_show_uint(64U, 176U, image_right_border_count, 3U);
    ips200_show_string(96U, 176U, "C:");
    ips200_show_uint(112U, 176U, image_centerline_count, 3U);
    ips200_show_string(0U, 192U, "DIST:");
    ips200_show_uint(48U, 192U, image_process_result.valid_distance_cm, 3U);
    ips200_show_string(80U, 192U, "cm CONF:");
    ips200_show_uint(152U, 192U, image_process_result.confidence, 3U);
    ips200_show_string(0U, 208U, "TGT:");
    ips200_show_uint(32U, 208U, image_process_result.target_distance_cm, 3U);
    ips200_show_string(64U, 208U, "cm REACH:");
    ips200_show_string(144U, 208U, image_process_result.target_reached ? "YES" : "NO ");
    ips200_show_string(0U, 224U, "Y:TARGET KEY4:BACK");
    ips200_show_string(0U, 240U, "TIME:");
    ips200_show_uint(48U, 240U, image_process_result.process_time_us, 4U);
    ips200_show_string(88U, 240U, "us");
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

const Image_Process_Result *image_process_get_result(void)
{
    return &image_process_result;
}

bool image_process_get_steering_mid(uint8 *mid)
{
    if(mid == NULL || !image_published_mid_valid)
    {
        return false;
    }
    *mid = image_published_mid;
    return true;
}

const uint8 *image_process_get_source_frame(void)
{
    return image_process_result.source_frame_valid
        ? image_source_frame
        : NULL;
}

const Image_Track_Point *image_process_get_left_border(uint8 *count)
{
    if(count != NULL) *count = image_left_border_count;
    return image_left_border;
}

const Image_Track_Point *image_process_get_right_border(uint8 *count)
{
    if(count != NULL) *count = image_right_border_count;
    return image_right_border;
}

const Image_Track_Point *image_process_get_centerline_image(uint8 *count)
{
    if(count != NULL) *count = image_centerline_count;
    return image_centerline_image;
}

const Image_Bird_Point *image_process_get_centerline_bird(uint8 *count)
{
    if(count != NULL) *count = image_centerline_count;
    return image_centerline_bird;
}

bool image_process_get_target_point(Image_Track_Point *point)
{
    if(!image_target_point_valid)
    {
        return false;
    }
    if(point != NULL)
    {
        *point = image_target_point;
    }
    return true;
}
