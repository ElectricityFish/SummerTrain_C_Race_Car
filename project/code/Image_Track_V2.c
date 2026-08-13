#include "zf_common_headfile.h"
#include "Image_Track_V2.h"

#include <string.h>

#define IMAGE_TRACK_V2_BINARY_ROW_BYTES       ((MT9V03X_W + 7U) / 8U)
#define IMAGE_TRACK_V2_HISTOGRAM_BINS          (64U)
#define IMAGE_TRACK_V2_HISTOGRAM_SHIFT          (2U)
#define IMAGE_TRACK_V2_SAMPLE_STEP              (2U)
#define IMAGE_TRACK_V2_THRESHOLD_STEP_MAX        (6U)
#define IMAGE_TRACK_V2_EDGE_GRAY_MARGIN          (4U)
#define IMAGE_TRACK_V2_EDGE_CONTRAST_MIN        (12U)
#define IMAGE_TRACK_V2_SEARCH_RADIUS            (10U)
#define IMAGE_TRACK_V2_SEARCH_RADIUS_WIDE       (18U)
#define IMAGE_TRACK_V2_PREDICT_ROWS_MAX          (3U)
#define IMAGE_TRACK_V2_SEED_ROW_TOP             (82U)
#define IMAGE_TRACK_V2_SEED_ROW_BOTTOM          (92U)
#define IMAGE_TRACK_V2_SEED_ROW_STEP             (2U)
#define IMAGE_TRACK_V2_FAR_LOW_CONF_ROW          (30U)

static uint8 image_track_v2_binary[MT9V03X_H][IMAGE_TRACK_V2_BINARY_ROW_BYTES];
static image_track_v2_result_t image_track_v2_result;
static uint8 image_track_v2_last_threshold;
static uint8 image_track_v2_last_final_mid;
static uint8 image_track_v2_last_seed_center;
static bool image_track_v2_has_threshold;

static uint8 image_track_v2_limit_u8(int16 value, uint8 lower, uint8 upper)
{
    if(value < (int16)lower)
    {
        return lower;
    }
    if(value > (int16)upper)
    {
        return upper;
    }
    return (uint8)value;
}

static uint8 image_track_v2_abs_diff_u8(uint8 a, uint8 b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static uint8 image_track_v2_binary_is_white(uint8 row, uint8 col)
{
    return (uint8)((image_track_v2_binary[row][col >> 3U] >> (col & 7U)) & 1U);
}

static void image_track_v2_timer_init(void)
{
#if defined(DWT) && defined(CoreDebug) && defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static uint32 image_track_v2_timer_get(void)
{
#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_CYCCNTENA_Msk)
    return DWT->CYCCNT;
#else
    return 0U;
#endif
}

static uint16 image_track_v2_cycles_to_us(uint32 cycles)
{
    uint32 cycles_per_us;
    uint32 time_us;

    if(system_clock < 1000000U)
    {
        return 0U;
    }

    cycles_per_us = system_clock / 1000000U;
    time_us = cycles / cycles_per_us;
    return (time_us > 65535U) ? 65535U : (uint16)time_us;
}

static uint8 image_track_v2_calculate_threshold(
    const uint8 image[][MT9V03X_W],
    uint8 *contrast_span)
{
    uint16 histogram[IMAGE_TRACK_V2_HISTOGRAM_BINS];
    uint32 total_count = 0U;
    uint32 total_sum = 0U;
    uint32 background_count = 0U;
    uint32 background_sum = 0U;
    uint32 best_score = 0U;
    uint8 best_bin = 31U;
    uint8 best_background_mean = 0U;
    uint8 best_foreground_mean = 63U;
    uint16 row;
    uint16 col;
    uint8 bin;
    uint8 threshold;

    memset(histogram, 0, sizeof(histogram));
    for(row = 8U; row < MT9V03X_H; row += IMAGE_TRACK_V2_SAMPLE_STEP)
    {
        for(col = 0U; col < MT9V03X_W; col += IMAGE_TRACK_V2_SAMPLE_STEP)
        {
            bin = image[row][col] >> IMAGE_TRACK_V2_HISTOGRAM_SHIFT;
            histogram[bin]++;
            total_count++;
            total_sum += bin;
        }
    }

    for(bin = 0U; bin < IMAGE_TRACK_V2_HISTOGRAM_BINS - 1U; bin++)
    {
        uint32 foreground_count;
        uint8 background_mean;
        uint8 foreground_mean;
        uint8 mean_difference;
        uint32 balanced_weight;
        uint32 score;

        background_count += histogram[bin];
        background_sum += (uint32)histogram[bin] * bin;
        foreground_count = total_count - background_count;
        if(background_count == 0U || foreground_count == 0U)
        {
            continue;
        }

        background_mean = (uint8)(background_sum / background_count);
        foreground_mean = (uint8)((total_sum - background_sum) / foreground_count);
        mean_difference = (foreground_mean > background_mean)
            ? (foreground_mean - background_mean) : 0U;
        balanced_weight = (background_count * foreground_count) / total_count;
        score = balanced_weight * mean_difference * mean_difference;

        if(score > best_score)
        {
            best_score = score;
            best_bin = bin;
            best_background_mean = background_mean;
            best_foreground_mean = foreground_mean;
        }
    }

    *contrast_span = (uint8)((best_foreground_mean - best_background_mean)
        << IMAGE_TRACK_V2_HISTOGRAM_SHIFT);
    threshold = (uint8)((best_bin << IMAGE_TRACK_V2_HISTOGRAM_SHIFT) + 2U);

    if(image_track_v2_has_threshold)
    {
        uint8 lower = (image_track_v2_last_threshold > IMAGE_TRACK_V2_THRESHOLD_STEP_MAX)
            ? (image_track_v2_last_threshold - IMAGE_TRACK_V2_THRESHOLD_STEP_MAX) : 0U;
        uint8 upper = (image_track_v2_last_threshold < 255U - IMAGE_TRACK_V2_THRESHOLD_STEP_MAX)
            ? (image_track_v2_last_threshold + IMAGE_TRACK_V2_THRESHOLD_STEP_MAX) : 255U;

        threshold = image_track_v2_limit_u8(threshold, lower, upper);
    }

    image_track_v2_last_threshold = threshold;
    image_track_v2_has_threshold = true;
    return threshold;
}

static void image_track_v2_build_binary(
    const uint8 image[][MT9V03X_W],
    uint8 threshold)
{
    uint16 row;
    uint16 col;

    memset(image_track_v2_binary, 0, sizeof(image_track_v2_binary));
    for(row = 0U; row < MT9V03X_H; row++)
    {
        for(col = 0U; col < MT9V03X_W; col++)
        {
            if(image[row][col] >= threshold)
            {
                image_track_v2_binary[row][col >> 3U] |= (uint8)(1U << (col & 7U));
            }
        }
    }
}

static bool image_track_v2_left_candidate(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    uint8 col,
    uint8 threshold)
{
    uint8 white_limit = (threshold <= 255U - IMAGE_TRACK_V2_EDGE_GRAY_MARGIN)
        ? (threshold + IMAGE_TRACK_V2_EDGE_GRAY_MARGIN) : 255U;
    uint8 black_limit = (threshold >= IMAGE_TRACK_V2_EDGE_GRAY_MARGIN)
        ? (threshold - IMAGE_TRACK_V2_EDGE_GRAY_MARGIN) : 0U;
    uint8 white_votes = 0U;
    uint8 black_votes = 0U;
    uint16 inside_sum = 0U;
    uint16 outside_sum = 0U;
    uint8 offset;

    if(col < 3U || col > MT9V03X_W - 3U)
    {
        return false;
    }

    for(offset = 0U; offset < 3U; offset++)
    {
        uint8 inside = image[row][col + offset];
        uint8 outside = image[row][col - 1U - offset];
        inside_sum += inside;
        outside_sum += outside;
        if(inside >= white_limit)
        {
            white_votes++;
        }
        if(outside <= black_limit)
        {
            black_votes++;
        }
    }

    return (white_votes >= 2U && black_votes >= 2U
        && inside_sum >= outside_sum + 3U * IMAGE_TRACK_V2_EDGE_CONTRAST_MIN);
}

static bool image_track_v2_right_candidate(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    uint8 col,
    uint8 threshold)
{
    uint8 white_limit = (threshold <= 255U - IMAGE_TRACK_V2_EDGE_GRAY_MARGIN)
        ? (threshold + IMAGE_TRACK_V2_EDGE_GRAY_MARGIN) : 255U;
    uint8 black_limit = (threshold >= IMAGE_TRACK_V2_EDGE_GRAY_MARGIN)
        ? (threshold - IMAGE_TRACK_V2_EDGE_GRAY_MARGIN) : 0U;
    uint8 white_votes = 0U;
    uint8 black_votes = 0U;
    uint16 inside_sum = 0U;
    uint16 outside_sum = 0U;
    uint8 offset;

    if(col < 2U || col > MT9V03X_W - 4U)
    {
        return false;
    }

    for(offset = 0U; offset < 3U; offset++)
    {
        uint8 inside = image[row][col - offset];
        uint8 outside = image[row][col + 1U + offset];
        inside_sum += inside;
        outside_sum += outside;
        if(inside >= white_limit)
        {
            white_votes++;
        }
        if(outside <= black_limit)
        {
            black_votes++;
        }
    }

    return (white_votes >= 2U && black_votes >= 2U
        && inside_sum >= outside_sum + 3U * IMAGE_TRACK_V2_EDGE_CONTRAST_MIN);
}

static bool image_track_v2_find_left_edge(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    int16 predicted,
    uint8 radius,
    uint8 threshold,
    uint8 *edge)
{
    uint8 distance;

    for(distance = 0U; distance <= radius; distance++)
    {
        int16 candidate = predicted - distance;
        if(candidate >= 3 && candidate <= MT9V03X_W - 3
            && image_track_v2_left_candidate(image, row, (uint8)candidate, threshold))
        {
            *edge = (uint8)candidate;
            return true;
        }

        candidate = predicted + distance;
        if(distance != 0U && candidate >= 3 && candidate <= MT9V03X_W - 3
            && image_track_v2_left_candidate(image, row, (uint8)candidate, threshold))
        {
            *edge = (uint8)candidate;
            return true;
        }
    }
    return false;
}

static bool image_track_v2_find_right_edge(
    const uint8 image[][MT9V03X_W],
    uint8 row,
    int16 predicted,
    uint8 radius,
    uint8 threshold,
    uint8 *edge)
{
    uint8 distance;

    for(distance = 0U; distance <= radius; distance++)
    {
        int16 candidate = predicted + distance;
        if(candidate >= 2 && candidate <= MT9V03X_W - 4
            && image_track_v2_right_candidate(image, row, (uint8)candidate, threshold))
        {
            *edge = (uint8)candidate;
            return true;
        }

        candidate = predicted - distance;
        if(distance != 0U && candidate >= 2 && candidate <= MT9V03X_W - 4
            && image_track_v2_right_candidate(image, row, (uint8)candidate, threshold))
        {
            *edge = (uint8)candidate;
            return true;
        }
    }
    return false;
}

static bool image_track_v2_find_seed(
    uint8 *seed_row,
    uint8 *seed_left,
    uint8 *seed_right,
    uint8 *seed_center)
{
    uint16 best_score = 65535U;
    uint8 row;
    bool found = false;

    for(row = IMAGE_TRACK_V2_SEED_ROW_TOP;
        row <= IMAGE_TRACK_V2_SEED_ROW_BOTTOM;
        row += IMAGE_TRACK_V2_SEED_ROW_STEP)
    {
        uint16 col = 0U;
        uint8 expected_width = image_expected_width_px[row];

        while(col < MT9V03X_W)
        {
            uint16 start;
            uint16 end;
            uint16 width;
            int16 center;
            uint16 score;

            while(col < MT9V03X_W
                && !image_track_v2_binary_is_white(row, (uint8)col))
            {
                col++;
            }
            if(col >= MT9V03X_W)
            {
                break;
            }

            start = col;
            while(col < MT9V03X_W
                && image_track_v2_binary_is_white(row, (uint8)col))
            {
                col++;
            }
            end = col - 1U;
            width = end - start + 1U;

            if(width < ((uint16)expected_width * 45U) / 100U
                || width > ((uint16)expected_width * 170U) / 100U
                || (start <= 2U && end >= MT9V03X_W - 3U))
            {
                continue;
            }

            if(start == 0U)
            {
                center = (int16)end - expected_width / 2;
            }
            else if(end == MT9V03X_W - 1U)
            {
                center = (int16)start + expected_width / 2;
            }
            else
            {
                center = (int16)((start + end) / 2U);
            }

            score = (uint16)image_track_v2_abs_diff_u8(
                image_track_v2_limit_u8(center, 0U, MT9V03X_W - 1U),
                image_track_v2_last_seed_center) * 3U;
            score += (width >= expected_width) ? (width - expected_width)
                : (expected_width - width);
            if(image_track_v2_last_seed_center < start
                || image_track_v2_last_seed_center > end)
            {
                score += 20U;
            }

            if(score < best_score)
            {
                best_score = score;
                *seed_row = row;
                *seed_left = (uint8)start;
                *seed_right = (uint8)end;
                *seed_center = image_track_v2_limit_u8(center, 0U, MT9V03X_W - 1U);
                found = true;
            }
        }
    }
    return found;
}

static uint8 image_track_v2_apply_quality_scale(uint8 row, uint8 confidence)
{
    if(row < IMAGE_TRACK_V2_FAR_LOW_CONF_ROW)
    {
        confidence = (uint8)(((uint16)confidence * 3U) / 4U);
    }
    if(image_track_v2_result.contrast_span < 24U)
    {
        confidence /= 2U;
    }
    else if(image_track_v2_result.contrast_span < 40U)
    {
        confidence = (uint8)(((uint16)confidence * 3U) / 4U);
    }
    return confidence;
}

static void image_track_v2_store_seed(
    uint8 row,
    uint8 left,
    uint8 right,
    uint8 center)
{
    uint8 expected_width = image_expected_width_px[row];
    bool left_valid = (left > 2U);
    bool right_valid = (right < MT9V03X_W - 3U);
    uint8 confidence;

    image_track_v2_result.center_line[row] = center;
    image_track_v2_result.left_valid[row] = left_valid;
    image_track_v2_result.right_valid[row] = right_valid;

    if(left_valid && right_valid)
    {
        uint8 width = right - left + 1U;
        uint8 width_error = image_track_v2_abs_diff_u8(width, expected_width);
        confidence = (width_error >= expected_width / 2U)
            ? 65U : (uint8)(100U - ((uint16)width_error * 60U) / expected_width);
        image_track_v2_result.left_edge[row] = left;
        image_track_v2_result.right_edge[row] = right;
        image_track_v2_result.source[row] = IMAGE_TRACK_SOURCE_BOTH_MEASURED;
    }
    else if(left_valid)
    {
        image_track_v2_result.left_edge[row] = left;
        image_track_v2_result.right_edge[row] = image_track_v2_limit_u8(
            (int16)left + expected_width, 0U, MT9V03X_W - 1U);
        image_track_v2_result.center_line[row] = image_track_v2_limit_u8(
            (int16)left + expected_width / 2, 0U, MT9V03X_W - 1U);
        image_track_v2_result.source[row] = IMAGE_TRACK_SOURCE_LEFT_ONLY;
        confidence = 62U;
    }
    else
    {
        image_track_v2_result.right_edge[row] = right;
        image_track_v2_result.left_edge[row] = image_track_v2_limit_u8(
            (int16)right - expected_width, 0U, MT9V03X_W - 1U);
        image_track_v2_result.center_line[row] = image_track_v2_limit_u8(
            (int16)right - expected_width / 2, 0U, MT9V03X_W - 1U);
        image_track_v2_result.source[row] = IMAGE_TRACK_SOURCE_RIGHT_ONLY;
        confidence = 62U;
    }
    image_track_v2_result.confidence[row] = image_track_v2_apply_quality_scale(row, confidence);
}

static void image_track_v2_track_direction(
    const uint8 image[][MT9V03X_W],
    int16 start_row,
    int16 end_row,
    int16 step,
    uint8 seed_row)
{
    int16 previous_center = image_track_v2_result.center_line[seed_row];
    int16 previous_previous_center = previous_center;
    uint8 previous_confidence = image_track_v2_result.confidence[seed_row];
    uint8 missed_rows = 0U;
    int16 row;

    for(row = start_row; row != end_row; row += step)
    {
        uint8 expected_width = image_expected_width_px[row];
        int16 center_delta = previous_center - previous_previous_center;
        int16 predicted_center;
        int16 predicted_left;
        int16 predicted_right;
        uint8 left_edge = 0U;
        uint8 right_edge = MT9V03X_W - 1U;
        bool left_found;
        bool right_found;
        bool used_wide_search = false;
        uint8 confidence = 0U;

        if(center_delta > 6)
        {
            center_delta = 6;
        }
        else if(center_delta < -6)
        {
            center_delta = -6;
        }
        predicted_center = previous_center + center_delta;
        predicted_center = image_track_v2_limit_u8(
            predicted_center, 0U, MT9V03X_W - 1U);
        predicted_left = predicted_center - expected_width / 2;
        predicted_right = predicted_center + expected_width / 2;

        left_found = image_track_v2_find_left_edge(
            image, (uint8)row, predicted_left, IMAGE_TRACK_V2_SEARCH_RADIUS,
            image_track_v2_result.threshold, &left_edge);
        right_found = image_track_v2_find_right_edge(
            image, (uint8)row, predicted_right, IMAGE_TRACK_V2_SEARCH_RADIUS,
            image_track_v2_result.threshold, &right_edge);

        if(!left_found)
        {
            left_found = image_track_v2_find_left_edge(
                image, (uint8)row, predicted_left, IMAGE_TRACK_V2_SEARCH_RADIUS_WIDE,
                image_track_v2_result.threshold, &left_edge);
            used_wide_search = left_found;
        }
        if(!right_found)
        {
            right_found = image_track_v2_find_right_edge(
                image, (uint8)row, predicted_right, IMAGE_TRACK_V2_SEARCH_RADIUS_WIDE,
                image_track_v2_result.threshold, &right_edge);
            used_wide_search = used_wide_search || right_found;
        }

        if(left_found && right_found)
        {
            uint16 measured_width = (right_edge >= left_edge)
                ? (uint16)(right_edge - left_edge + 1U) : 0U;
            uint16 width_min = ((uint16)expected_width * 60U) / 100U;
            uint16 width_max = ((uint16)expected_width * 150U) / 100U;

            if(measured_width < width_min || measured_width > width_max)
            {
                uint8 left_error = image_track_v2_abs_diff_u8(
                    left_edge, image_track_v2_limit_u8(predicted_left, 0U, MT9V03X_W - 1U));
                uint8 right_error = image_track_v2_abs_diff_u8(
                    right_edge, image_track_v2_limit_u8(predicted_right, 0U, MT9V03X_W - 1U));
                if(left_error <= right_error)
                {
                    right_found = false;
                }
                else
                {
                    left_found = false;
                }
            }
        }

        if(left_found && right_found)
        {
            uint8 measured_width = right_edge - left_edge + 1U;
            uint8 width_error = image_track_v2_abs_diff_u8(measured_width, expected_width);
            int16 center = ((int16)left_edge + right_edge) / 2;

            confidence = (uint8)(100U - (((uint16)width_error * 60U) / expected_width));
            if(used_wide_search && confidence > 15U)
            {
                confidence -= 15U;
            }
            image_track_v2_result.left_edge[row] = left_edge;
            image_track_v2_result.right_edge[row] = right_edge;
            image_track_v2_result.center_line[row] = (uint8)center;
            image_track_v2_result.left_valid[row] = true;
            image_track_v2_result.right_valid[row] = true;
            image_track_v2_result.source[row] = IMAGE_TRACK_SOURCE_BOTH_MEASURED;
            missed_rows = 0U;
        }
        else if(left_found)
        {
            image_track_v2_result.left_edge[row] = left_edge;
            image_track_v2_result.right_edge[row] = image_track_v2_limit_u8(
                (int16)left_edge + expected_width, 0U, MT9V03X_W - 1U);
            image_track_v2_result.center_line[row] = image_track_v2_limit_u8(
                (int16)left_edge + expected_width / 2, 0U, MT9V03X_W - 1U);
            image_track_v2_result.left_valid[row] = true;
            image_track_v2_result.source[row] = IMAGE_TRACK_SOURCE_LEFT_ONLY;
            confidence = used_wide_search ? 50U : 62U;
            missed_rows = 0U;
        }
        else if(right_found)
        {
            image_track_v2_result.right_edge[row] = right_edge;
            image_track_v2_result.left_edge[row] = image_track_v2_limit_u8(
                (int16)right_edge - expected_width, 0U, MT9V03X_W - 1U);
            image_track_v2_result.center_line[row] = image_track_v2_limit_u8(
                (int16)right_edge - expected_width / 2, 0U, MT9V03X_W - 1U);
            image_track_v2_result.right_valid[row] = true;
            image_track_v2_result.source[row] = IMAGE_TRACK_SOURCE_RIGHT_ONLY;
            confidence = used_wide_search ? 50U : 62U;
            missed_rows = 0U;
        }
        else
        {
            missed_rows++;
            if(missed_rows > IMAGE_TRACK_V2_PREDICT_ROWS_MAX)
            {
                break;
            }

            image_track_v2_result.center_line[row] = (uint8)predicted_center;
            image_track_v2_result.left_edge[row] = image_track_v2_limit_u8(
                predicted_left, 0U, MT9V03X_W - 1U);
            image_track_v2_result.right_edge[row] = image_track_v2_limit_u8(
                predicted_right, 0U, MT9V03X_W - 1U);
            image_track_v2_result.source[row] = IMAGE_TRACK_SOURCE_SHORT_PREDICTED;
            confidence = (uint8)(((uint16)previous_confidence * 2U) / 3U);
            if(confidence > 35U)
            {
                confidence = 35U;
            }
        }

        confidence = image_track_v2_apply_quality_scale((uint8)row, confidence);
        image_track_v2_result.confidence[row] = confidence;
        previous_previous_center = previous_center;
        previous_center = image_track_v2_result.center_line[row];
        previous_confidence = confidence;
    }
}

static void image_track_v2_finalize_result(void)
{
    static const uint8 anchor_rows[IMAGE_TRACK_V2_ANCHOR_COUNT] =
        {90U, 66U, 47U, 35U, 26U};
    static const uint8 anchor_weights[IMAGE_TRACK_V2_ANCHOR_COUNT] =
        {10U, 25U, 30U, 25U, 10U};
    uint16 frame_confidence_sum = 0U;
    int32 error_sum = 0;
    uint32 control_weight_sum = 0U;
    uint8 anchor;
    uint8 row;
    int16 current_mid;
    int16 far_row;
    uint8 weak_far_rows = 0U;

    image_track_v2_result.valid_far_distance_mm = 0U;
    for(row = 0U; row < MT9V03X_H; row++)
    {
        uint8 source = image_track_v2_result.source[row];
        if(source == IMAGE_TRACK_SOURCE_BOTH_MEASURED)
        {
            image_track_v2_result.measured_row_count++;
        }
        else if(source == IMAGE_TRACK_SOURCE_LEFT_ONLY
            || source == IMAGE_TRACK_SOURCE_RIGHT_ONLY)
        {
            image_track_v2_result.rebuilt_row_count++;
        }

    }

    // “最远有效距离”必须从道路种子连续向远端成立，不能越过一段弱跟踪后
    // 又把背景中偶然满足宽度的白块算成数米前瞻。
    for(far_row = image_track_v2_result.seed_row; far_row >= 0; far_row--)
    {
        uint8 source = image_track_v2_result.source[far_row];
        uint8 confidence = image_track_v2_result.confidence[far_row];
        bool measured_or_rebuilt = (source == IMAGE_TRACK_SOURCE_BOTH_MEASURED
            || source == IMAGE_TRACK_SOURCE_LEFT_ONLY
            || source == IMAGE_TRACK_SOURCE_RIGHT_ONLY);

        if(measured_or_rebuilt && confidence >= 55U)
        {
            weak_far_rows = 0U;
            image_track_v2_result.valid_far_distance_mm = image_row_distance_mm[far_row];
        }
        else if((measured_or_rebuilt && confidence >= 40U)
            || source == IMAGE_TRACK_SOURCE_SHORT_PREDICTED)
        {
            weak_far_rows++;
            if(weak_far_rows > IMAGE_TRACK_V2_PREDICT_ROWS_MAX)
            {
                break;
            }
            image_track_v2_result.valid_far_distance_mm = image_row_distance_mm[far_row];
        }
        else
        {
            break;
        }
    }

    for(anchor = 0U; anchor < IMAGE_TRACK_V2_ANCHOR_COUNT; anchor++)
    {
        uint8 anchor_row = anchor_rows[anchor];
        uint8 confidence = image_track_v2_result.confidence[anchor_row];
        uint32 control_weight = (uint32)anchor_weights[anchor] * confidence;

        image_track_v2_result.anchor_center[anchor] =
            image_track_v2_result.center_line[anchor_row];
        image_track_v2_result.anchor_confidence[anchor] = confidence;
        frame_confidence_sum += (uint16)anchor_weights[anchor] * confidence;

        if(image_track_v2_result.source[anchor_row] == IMAGE_TRACK_SOURCE_SHORT_PREDICTED)
        {
            control_weight /= 2U;
        }
        if(image_track_v2_result.source[anchor_row] != IMAGE_TRACK_SOURCE_INVALID)
        {
            error_sum += ((int16)image_track_v2_result.center_line[anchor_row]
                - IMAGE_CALIBRATED_CENTER_COL) * (int32)control_weight;
            control_weight_sum += control_weight;
        }
    }

    image_track_v2_result.frame_confidence = (uint8)(frame_confidence_sum / 100U);
    if(control_weight_sum == 0U)
    {
        current_mid = image_track_v2_last_final_mid;
    }
    else if(error_sum >= 0)
    {
        current_mid = IMAGE_CALIBRATED_CENTER_COL
            + (int16)((error_sum + (int32)control_weight_sum / 2) / (int32)control_weight_sum);
    }
    else
    {
        current_mid = IMAGE_CALIBRATED_CENTER_COL
            + (int16)((error_sum - (int32)control_weight_sum / 2) / (int32)control_weight_sum);
    }

    current_mid = image_track_v2_limit_u8(current_mid, 0U, MT9V03X_W - 1U);
    image_track_v2_result.final_mid = (uint8)(((uint16)current_mid * 3U
        + image_track_v2_last_final_mid + 2U) / 4U);
    image_track_v2_last_final_mid = image_track_v2_result.final_mid;
}

void image_track_v2_init(void)
{
    memset(image_track_v2_binary, 0, sizeof(image_track_v2_binary));
    memset(&image_track_v2_result, 0, sizeof(image_track_v2_result));
    image_track_v2_last_threshold = 128U;
    image_track_v2_last_final_mid = IMAGE_CALIBRATED_CENTER_COL;
    image_track_v2_last_seed_center = IMAGE_CALIBRATED_CENTER_COL;
    image_track_v2_has_threshold = false;
    image_track_v2_timer_init();
}

void image_track_v2_process(const uint8 image[][MT9V03X_W])
{
    uint32 start_cycles = image_track_v2_timer_get();
    uint8 seed_row = 0U;
    uint8 seed_left = 0U;
    uint8 seed_right = MT9V03X_W - 1U;
    uint8 seed_center = IMAGE_CALIBRATED_CENTER_COL;
    uint8 row;

    memset(&image_track_v2_result, 0, sizeof(image_track_v2_result));
    for(row = 0U; row < MT9V03X_H; row++)
    {
        image_track_v2_result.left_edge[row] = IMAGE_CALIBRATED_CENTER_COL;
        image_track_v2_result.right_edge[row] = IMAGE_CALIBRATED_CENTER_COL;
        image_track_v2_result.center_line[row] = IMAGE_CALIBRATED_CENTER_COL;
    }

    image_track_v2_result.threshold = image_track_v2_calculate_threshold(
        image, &image_track_v2_result.contrast_span);
    image_track_v2_build_binary(image, image_track_v2_result.threshold);

    if(image_track_v2_find_seed(&seed_row, &seed_left, &seed_right, &seed_center))
    {
        image_track_v2_result.seed_row = seed_row;
        image_track_v2_store_seed(seed_row, seed_left, seed_right, seed_center);
        image_track_v2_last_seed_center = image_track_v2_result.center_line[seed_row];

        if(seed_row > 0U)
        {
            image_track_v2_track_direction(
                image, (int16)seed_row - 1, -1, -1, seed_row);
        }
        if(seed_row < MT9V03X_H - 1U)
        {
            image_track_v2_track_direction(
                image, (int16)seed_row + 1, MT9V03X_H, 1, seed_row);
        }
    }

    image_track_v2_finalize_result();
    image_track_v2_result.process_time_us = image_track_v2_cycles_to_us(
        image_track_v2_timer_get() - start_cycles);
}

const image_track_v2_result_t *image_track_v2_get_result(void)
{
    return &image_track_v2_result;
}
