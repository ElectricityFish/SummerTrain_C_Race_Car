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
#define IMAGE_NEAR_REACH_TOLERANCE_CM      (4U)
#define IMAGE_WHEELBASE_MM                  (200U)
#define IMAGE_WHEELBASE_UNITS               ((int32)((IMAGE_WHEELBASE_MM + IMAGE_PERSPECTIVE_GRID_MM / 2U) / IMAGE_PERSPECTIVE_GRID_MM))
#define IMAGE_PURE_PURSUIT_MAX_ANGLE_X10    (300)
#define IMAGE_PURE_PURSUIT_DEADBAND_X10     (10)
#define IMAGE_STEERING_COMMAND_LIMIT_X10    (250)
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
static Image_Track_Point image_steering_target_point;
static bool image_steering_target_point_valid;
static int16 image_last_steering_command_x10;
static bool image_has_last_steering_command;
static bool image_new_result;
static bool image_cycle_counter_ready;
static Image_Process_Result image_process_result;
static volatile int16 image_published_steering_angle_x10;
static volatile bool image_published_steering_valid;

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
    uint8 maximum_near_cm = (uint8)(
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
    if(image_process_config.steering_near_cm == 0U)
    {
        image_process_config.steering_near_cm = 1U;
    }
    if(image_process_config.steering_near_cm > maximum_near_cm)
    {
        image_process_config.steering_near_cm = maximum_near_cm;
    }
    if(image_process_config.steering_gain_percent > 300U)
    {
        image_process_config.steering_gain_percent = 300U;
    }
    if(image_process_config.steering_filter_current > 100U)
    {
        image_process_config.steering_filter_current = 100U;
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

static uint16 image_bird_segment_length_mm(Image_Bird_Point first, Image_Bird_Point second)
{
    int32 dx_mm = ((int32)second.x - (int32)first.x) * IMAGE_PERSPECTIVE_GRID_MM;
    int32 dy_mm = ((int32)second.y - (int32)first.y) * IMAGE_PERSPECTIVE_GRID_MM;

    return image_sqrt_u32((uint32)(dx_mm * dx_mm + dy_mm * dy_mm));
}

// HFK atan2_int 的定点查表版本；返回0.1度，并在线性插值后限制到正负30度。
static int16 image_atan_ratio_x10(int32 numerator, int32 denominator)
{
    static const uint16 tangent_x65536[31] =
    {
        0U, 1143U, 2288U, 3434U, 4582U, 5732U,
        6887U, 8046U, 9212U, 10383U, 11562U, 12749U,
        13945U, 15151U, 16368U, 17597U, 18840U, 20097U,
        21369U, 22657U, 23962U, 25286U, 26628U, 27989U,
        29371U, 30775U, 32203U, 33654U, 35132U, 36637U,
        37837U
    };
    int32 sign;
    int32 absolute_numerator;
    int32 scaled_numerator;
    int32 degree;

    if(numerator == 0 || denominator <= 0)
    {
        return 0;
    }
    sign = (numerator > 0) ? 1 : -1;
    absolute_numerator = (numerator > 0) ? numerator : -numerator;
    scaled_numerator = absolute_numerator * 65536;
    if(scaled_numerator >= denominator * tangent_x65536[30])
    {
        return (int16)(sign * IMAGE_PURE_PURSUIT_MAX_ANGLE_X10);
    }

    for(degree = 29; degree >= 0; degree--)
    {
        int32 lower = denominator * tangent_x65536[degree];

        if(scaled_numerator >= lower)
        {
            int32 upper = denominator * tangent_x65536[degree + 1];
            int32 fraction_x10 = (scaled_numerator - lower) * 10
                / (upper - lower);
            return (int16)(sign * (degree * 10 + fraction_x10));
        }
    }
    return 0;
}

static int16 image_calculate_pure_pursuit_angle_x10(Image_Bird_Point target)
{
    // 鸟瞰坐标以图像下方向为 x 增大、右方向为 y 增大。
    // 标定距离以当前前轮轴为原点；HFK/自行车模型以后轮轴为原点，故前向距离加一轴距。
    int32 forward_units = (int32)IMAGE_PERSPECTIVE_NEAR_X - (int32)target.x
        + IMAGE_WHEELBASE_UNITS;
    int32 lateral_units = (int32)IMAGE_PERSPECTIVE_CENTER_COL - (int32)target.y;
    int32 numerator = 2 * IMAGE_WHEELBASE_UNITS * lateral_units;
    int32 denominator = forward_units * forward_units + lateral_units * lateral_units;
    int16 angle_x10 = image_atan_ratio_x10(numerator, denominator);

    // HFK原函数以1度为分辨率，小于1度时输出0；保留该死区以抑制直道标定量化抖动。
    return (angle_x10 > -IMAGE_PURE_PURSUIT_DEADBAND_X10
        && angle_x10 < IMAGE_PURE_PURSUIT_DEADBAND_X10) ? 0 : angle_x10;
}

static int16 image_scale_and_limit_steering_x10(int16 angle_x10, uint16 gain_percent)
{
    int32 scaled = (int32)angle_x10 * gain_percent;

    scaled = (scaled >= 0) ? (scaled + 50) / 100 : (scaled - 50) / 100;
    if(scaled > IMAGE_STEERING_COMMAND_LIMIT_X10)
    {
        scaled = IMAGE_STEERING_COMMAND_LIMIT_X10;
    }
    if(scaled < -IMAGE_STEERING_COMMAND_LIMIT_X10)
    {
        scaled = -IMAGE_STEERING_COMMAND_LIMIT_X10;
    }
    return (int16)scaled;
}

static uint8 image_find_steering_target_index(
    uint8 steering_near_cm,
    uint16 *actual_distance_mm,
    uint16 *centerline_length_mm)
{
    // 前瞻从前轮轴中心开始沿中心线累计弧长。弯中横向延伸也必须计入距离，
    // 不能再用鸟瞰 x 坐标差代替路径长度。
    const Image_Bird_Point vehicle_origin = {
        IMAGE_PERSPECTIVE_NEAR_X,
        IMAGE_PERSPECTIVE_CENTER_COL
    };
    uint16 requested_mm = (uint16)steering_near_cm * 10U;
    uint16 accumulated_mm = image_bird_segment_length_mm(
        vehicle_origin, image_centerline_bird[0]);
    uint8 best_index = 0U;
    uint16 best_distance_mm = accumulated_mm;
    uint16 best_difference_mm = (accumulated_mm >= requested_mm)
        ? (accumulated_mm - requested_mm)
        : (requested_mm - accumulated_mm);
    uint8 index;

    for(index = 1U; index < image_centerline_count; index++)
    {
        uint16 difference_mm;

        accumulated_mm += image_bird_segment_length_mm(
            image_centerline_bird[index - 1U], image_centerline_bird[index]);
        difference_mm = (accumulated_mm >= requested_mm)
            ? (accumulated_mm - requested_mm)
            : (requested_mm - accumulated_mm);
        if(difference_mm < best_difference_mm)
        {
            best_difference_mm = difference_mm;
            best_distance_mm = accumulated_mm;
            best_index = index;
        }
    }

    if(actual_distance_mm != NULL)
    {
        *actual_distance_mm = best_distance_mm;
    }
    if(centerline_length_mm != NULL)
    {
        *centerline_length_mm = accumulated_mm;
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
    image_steering_target_point_valid = false;
    image_process_result.source_frame_valid = false;
    image_process_result.centerline_valid = false;
    image_process_result.pure_pursuit_angle_x10 = 0;
    image_process_result.steering_command_x10 = 0;
    image_process_result.steering_near_reached = false;
    image_process_result.confidence = 0U;
    image_process_result.centerline_length_cm = 0U;
    image_process_result.steering_near_actual_cm = 0U;
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
    image_process_config.steering_near_cm = 55U;
    image_process_config.steering_gain_percent = 150U;
    image_process_config.steering_filter_current = 80U;

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
    image_last_steering_command_x10 = 0;
    image_has_last_steering_command = false;
    image_published_steering_angle_x10 = 0;
    image_published_steering_valid = false;
    image_new_result = false;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    image_cycle_counter_ready = ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U)
        && ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
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
    int16 pure_pursuit_angle_x10;
    int16 raw_steering_command_x10;
    int32 filtered_steering_x10;
    uint8 current_weight;
    uint16 steering_near_difference_cm;
    uint16 steering_near_actual_mm;
    uint16 centerline_length_mm;
    uint8 chosen_count = 0U;
    uint32 cycle_start = image_cycle_counter_ready ? DWT->CYCCNT : 0U;

    image_clear_result();
    image_sanitize_config();
    image_release_frame(image_source_frame);
    image_source_frame = NULL;
    latest_frame = image_acquire_latest_frame();
    if(latest_frame == NULL)
    {
        image_published_steering_valid = false;
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
        image_process_result.left_border_count = image_left_border_count;
        image_process_result.right_border_count = image_right_border_count;
        image_published_steering_valid = false;
        image_finish_frame(cycle_start, true);
        return;
    }

    image_filter_bird_line(image_centerline_bird, image_centerline_count);
    image_centerline_count = image_resample_bird_line(
        image_centerline_bird, image_centerline_count, IMAGE_CENTER_RESAMPLE_STEP);
    if(image_centerline_count < image_process_config.min_border_points)
    {
        image_process_result.left_border_count = image_left_border_count;
        image_process_result.right_border_count = image_right_border_count;
        image_published_steering_valid = false;
        image_finish_frame(cycle_start, true);
        return;
    }

    for(index = 0U; index < image_centerline_count; index++)
    {
        image_centerline_image[index] = image_bird_to_image(image_centerline_bird[index]);
    }

    target_index = image_find_steering_target_index(
        image_process_config.steering_near_cm,
        &steering_near_actual_mm,
        &centerline_length_mm);
    image_steering_target_point = image_centerline_image[target_index];
    image_steering_target_point_valid = true;
    image_process_result.centerline_length_cm = (centerline_length_mm + 5U) / 10U;
    image_process_result.steering_near_actual_cm = (steering_near_actual_mm + 5U) / 10U;
    steering_near_difference_cm =
        (image_process_result.steering_near_actual_cm >= image_process_config.steering_near_cm)
        ? (image_process_result.steering_near_actual_cm - image_process_config.steering_near_cm)
        : (image_process_config.steering_near_cm - image_process_result.steering_near_actual_cm);
    image_process_result.steering_near_reached =
        steering_near_difference_cm <= IMAGE_NEAR_REACH_TOLERANCE_CM;
    pure_pursuit_angle_x10 = image_calculate_pure_pursuit_angle_x10(
        image_centerline_bird[target_index]);
    raw_steering_command_x10 = image_scale_and_limit_steering_x10(
        pure_pursuit_angle_x10, image_process_config.steering_gain_percent);
    image_process_result.pure_pursuit_angle_x10 = pure_pursuit_angle_x10;
    current_weight = image_process_config.steering_filter_current;
    if(current_weight > 100U) current_weight = 100U;
    if(!image_has_last_steering_command)
    {
        image_process_result.steering_command_x10 = raw_steering_command_x10;
        image_has_last_steering_command = true;
    }
    else
    {
        filtered_steering_x10 = (int32)raw_steering_command_x10 * current_weight
            + (int32)image_last_steering_command_x10 * (100U - current_weight);
        filtered_steering_x10 = (filtered_steering_x10 >= 0)
            ? (filtered_steering_x10 + 50) / 100
            : (filtered_steering_x10 - 50) / 100;
        image_process_result.steering_command_x10 = (int16)filtered_steering_x10;
    }
    image_last_steering_command_x10 = image_process_result.steering_command_x10;
    image_published_steering_angle_x10 = image_process_result.steering_command_x10;
    image_published_steering_valid = true;
    image_process_result.centerline_valid = true;
    image_process_result.left_border_count = image_left_border_count;
    image_process_result.right_border_count = image_right_border_count;
    image_process_result.centerline_count = image_centerline_count;
    image_process_result.confidence = image_limit_u8((uint32)chosen_count * 100U / 45U, 0U, 100U);
    if(image_process_result.centerline_length_cm < image_process_config.steering_near_cm)
    {
        image_process_result.confidence = (uint8)(
            (uint16)image_process_result.confidence
            * image_process_result.centerline_length_cm
            / image_process_config.steering_near_cm);
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
    if(image_steering_target_point_valid)
    {
        uint16 x = (uint16)image_steering_target_point.col * IMAGE_PROCESS_DISPLAY_WIDTH / MT9V03X_W;
        uint16 y = (uint16)image_steering_target_point.row * IMAGE_PROCESS_DISPLAY_HEIGHT / MT9V03X_H;
        uint16 x_end = (x + 3U < IMAGE_PROCESS_DISPLAY_WIDTH)
            ? (x + 3U) : (IMAGE_PROCESS_DISPLAY_WIDTH - 1U);
        uint16 y_end = (y + 3U < IMAGE_PROCESS_DISPLAY_HEIGHT)
            ? (y + 3U) : (IMAGE_PROCESS_DISPLAY_HEIGHT - 1U);

        ips200_draw_line((x >= 3U) ? x - 3U : 0U, y, x_end, y, RGB565_YELLOW);
        ips200_draw_line(x, (y >= 3U) ? y - 3U : 0U, x, y_end, RGB565_YELLOW);
    }

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0U, 160U, "PP:");
    ips200_show_int(24U, 160U, image_process_result.pure_pursuit_angle_x10, 4U);
    ips200_show_string(72U, 160U, "CMD:");
    ips200_show_int(104U, 160U, image_process_result.steering_command_x10, 4U);
    ips200_show_string(160U, 160U, "V:");
    ips200_show_string(176U, 160U, image_process_result.centerline_valid ? "YES" : "NO ");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 176U, "L:");
    ips200_show_uint(16U, 176U, image_left_border_count, 3U);
    ips200_show_string(48U, 176U, "R:");
    ips200_show_uint(64U, 176U, image_right_border_count, 3U);
    ips200_show_string(96U, 176U, "C:");
    ips200_show_uint(112U, 176U, image_centerline_count, 3U);
    ips200_show_string(0U, 192U, "PATH:");
    ips200_show_uint(48U, 192U, image_process_result.centerline_length_cm, 3U);
    ips200_show_string(80U, 192U, "cm CONF:");
    ips200_show_uint(152U, 192U, image_process_result.confidence, 3U);
    ips200_show_string(0U, 208U, "NEAR:");
    ips200_show_uint(48U, 208U, image_process_result.steering_near_actual_cm, 3U);
    ips200_show_string(80U, 208U, "cm HIT:");
    ips200_show_string(144U, 208U, image_process_result.steering_near_reached ? "YES" : "NO ");
    ips200_show_string(0U, 224U, "SET:");
    ips200_show_uint(32U, 224U, image_process_config.steering_near_cm, 3U);
    ips200_show_string(64U, 224U, "cm Y:NEAR K4:BACK");
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

bool image_process_get_steering_angle_x10(int16 *angle_x10)
{
    if(angle_x10 == NULL || !image_published_steering_valid)
    {
        return false;
    }
    *angle_x10 = image_published_steering_angle_x10;
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

bool image_process_get_steering_target_point(Image_Track_Point *point)
{
    if(!image_steering_target_point_valid)
    {
        return false;
    }
    if(point != NULL)
    {
        *point = image_steering_target_point;
    }
    return true;
}
