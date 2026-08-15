#ifndef __IMAGE_PROCESS_H__
#define __IMAGE_PROCESS_H__

#include "Image.h"

#define IMAGE_PROCESS_MAX_POINTS (100U)

typedef struct
{
    uint8 row;
    uint8 col;
} Image_Track_Point;

typedef struct
{
    uint8 x;
    uint8 y;
} Image_Bird_Point;

typedef enum
{
    IMAGE_TRACK_SIDE_NONE = 0,
    IMAGE_TRACK_SIDE_LEFT,
    IMAGE_TRACK_SIDE_RIGHT,
} image_track_side_enum;

// HFK point-tracing parameters. These are the only runtime vision parameters.
typedef struct
{
    uint8 local_threshold_offset;  // Local 5x5 mean minus this value.
    uint8 start_contrast_min;      // Minimum gray difference across a seed edge.
    uint8 min_border_points;       // Minimum connected points for a valid border.
    uint8 resample_step;           // Bird-view equal-distance sample step.
    uint8 steering_near_cm;        // Steering near-lookahead measured along the centerline.
    uint16 steering_gain_percent;  // Pure-pursuit angle to servo-command scale.
    uint8 steering_filter_current; // Current-frame share in steering command, 0..100.
} Image_Process_Config;

typedef struct
{
    int16 pure_pursuit_angle_x10;
    int16 steering_command_x10;
    bool source_frame_valid;
    bool centerline_valid;
    bool steering_near_reached;
    uint8 confidence;
    uint16 centerline_length_cm;
    uint16 steering_near_actual_cm;
    uint8 left_border_count;
    uint8 right_border_count;
    uint8 centerline_count;
    image_track_side_enum selected_side;
    uint16 process_time_us;
} Image_Process_Result;

extern Image_Process_Config image_process_config;

void image_process_init(void);
void image_process_frame(void);
void image_process_display(void);
bool image_process_take_new_result(void);

const Image_Process_Result *image_process_get_result(void);
bool image_process_get_steering_angle_x10(int16 *angle_x10);
const uint8 *image_process_get_source_frame(void);

const Image_Track_Point *image_process_get_left_border(uint8 *count);
const Image_Track_Point *image_process_get_right_border(uint8 *count);
const Image_Track_Point *image_process_get_centerline_image(uint8 *count);
const Image_Bird_Point *image_process_get_centerline_bird(uint8 *count);
bool image_process_get_steering_target_point(Image_Track_Point *point);

#endif
