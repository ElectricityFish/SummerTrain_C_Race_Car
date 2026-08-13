#ifndef __IMAGE_TRACK_V2_H__
#define __IMAGE_TRACK_V2_H__

#include "Image.h"
#include "Image_Calibration.h"

// 首次上车保持 0：V2 只做影子运行和叠加显示。验证通过后改为 1 接管 final_mid。
#define IMAGE_TRACK_V2_CONTROL_ENABLE       (1U)
#define IMAGE_TRACK_V2_DISPLAY_ENABLE       (1U)
#define IMAGE_TRACK_V2_CONTROL_CONFIDENCE_MIN (15U)

#define IMAGE_TRACK_V2_ANCHOR_COUNT         (5U)

typedef enum
{
    IMAGE_TRACK_SOURCE_INVALID = 0,
    IMAGE_TRACK_SOURCE_BOTH_MEASURED,
    IMAGE_TRACK_SOURCE_LEFT_ONLY,
    IMAGE_TRACK_SOURCE_RIGHT_ONLY,
    IMAGE_TRACK_SOURCE_SHORT_PREDICTED,
} image_track_source_enum;

typedef enum
{
    IMAGE_TRACK_ANCHOR_25CM = 0,
    IMAGE_TRACK_ANCHOR_40CM,
    IMAGE_TRACK_ANCHOR_60CM,
    IMAGE_TRACK_ANCHOR_80CM,
    IMAGE_TRACK_ANCHOR_100CM,
} image_track_anchor_enum;

typedef struct
{
    uint8 left_edge[MT9V03X_H];
    uint8 right_edge[MT9V03X_H];
    uint8 center_line[MT9V03X_H];
    uint8 left_valid[MT9V03X_H];
    uint8 right_valid[MT9V03X_H];
    uint8 source[MT9V03X_H];
    uint8 confidence[MT9V03X_H];

    uint8 anchor_center[IMAGE_TRACK_V2_ANCHOR_COUNT];
    uint8 anchor_confidence[IMAGE_TRACK_V2_ANCHOR_COUNT];

    uint8 threshold;
    uint8 contrast_span;
    uint8 seed_row;
    uint8 frame_confidence;
    uint8 final_mid;
    uint8 measured_row_count;
    uint8 rebuilt_row_count;
    uint16 valid_far_distance_mm;
    uint16 process_time_us;
} image_track_v2_result_t;

void image_track_v2_init(void);
void image_track_v2_process(const uint8 image[][MT9V03X_W]);
const image_track_v2_result_t *image_track_v2_get_result(void);

#endif
