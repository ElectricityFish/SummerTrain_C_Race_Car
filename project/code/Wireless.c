#include "Wireless.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "zf_device_mt9v03x.h"
#include "Image_Process.h"

#define WIRELESS_IMAGE_VOFA_ORIGIN_CHANNEL_ID (0U)
#define WIRELESS_IMAGE_VOFA_PROCESSED_CHANNEL_ID (1U)
#define WIRELESS_IMAGE_VOFA_GRAYSCALE8_FORMAT (24U)
#define WIRELESS_IMAGE_VOFA_RGB565_FORMAT      (7U)
#define WIRELESS_IMAGE_VOFA_HEADER_SIZE       (64U)
#define WIRELESS_IMAGE_RTS_TIMEOUT_MS          (8000U)

#define WIRELESS_IMAGE_RGB565_RED              (0xF800U)
#define WIRELESS_IMAGE_RGB565_BLUE             (0x001FU)
#define WIRELESS_IMAGE_RGB565_GREEN            (0x07E0U)
#define WIRELESS_IMAGE_RGB565_YELLOW           (0xFFE0U)
#define WIRELESS_IMAGE_RGB565_MAGENTA          (0xF81FU)
#define WIRELESS_IMAGE_RGB565_CYAN             (0x07FFU)

volatile uint8 wireless_image_send_status = WIRELESS_IMAGE_SEND_NOT_SENT;

// 必须先保存快照：摄像头 DMA 会持续改写 mt9v03x_image，不能在约 2 秒的串口发送期间直接读取它。
static uint8 wireless_image_snapshot[MT9V03X_IMAGE_SIZE];
// 处理图逐行生成、逐行发送，避免再占用 45120 字节的 RGB565 全帧缓冲。
static uint16 wireless_image_processed_line[MT9V03X_W];
static volatile uint8 wireless_image_send_type = WIRELESS_IMAGE_SEND_ORIGIN;
static bool wireless_image_snapshot_ready = false;

//提供和printf相同的可变参数调用方式，底层复用厂商无线串口的RTS流控发送函数。
int wireless_uart_printf(const char *format, ...)
{
	static char send_buffer[WIRELESS_PRINTF_BUFFER_SIZE];
	va_list args;
	int format_length;
	uint32 remaining_length;

	if(format == NULL)
	{
		return -1;
	}

	va_start(args, format);
	format_length = vsnprintf(send_buffer, sizeof(send_buffer), format, args);
	va_end(args);

	//vsnprintf返回值大于等于缓冲区长度，说明内容被截断；本函数不发送不完整数据。
	if(format_length < 0 || format_length >= (int)sizeof(send_buffer))
	{
		return -1;
	}

	remaining_length = wireless_uart_send_buffer((const uint8 *)send_buffer, (uint32)format_length);
	if(remaining_length != 0U)
	{
		return -1;
	}

	return format_length;
}

// 此函数可在按键中断中调用，只修改一个 8 位状态标志，不进行耗时操作。
void wireless_image_request_send(wireless_image_send_type_enum send_type)
{
    if(wireless_image_send_status != WIRELESS_IMAGE_SEND_SENDING)
    {
        wireless_image_send_type = (uint8)send_type;
        wireless_image_snapshot_ready = false;
        wireless_image_send_status = WIRELESS_IMAGE_SEND_SENDING;
    }
}

// 在 DMA 完成后立即保存原图，保证后续图像处理和长时间串口发送都不会读到被 DMA 改写的像素。
void wireless_image_capture_task(bool is_idle, const uint8 *image_addr, uint16 image_width, uint16 image_height)
{
    uint32 image_size;

    if(wireless_image_send_status != WIRELESS_IMAGE_SEND_SENDING
        || wireless_image_snapshot_ready || !is_idle)
    {
        return;
    }

    image_size = (uint32)image_width * image_height;
    if(image_addr == NULL || image_width != MT9V03X_W || image_height != MT9V03X_H
        || image_size != MT9V03X_IMAGE_SIZE)
    {
        wireless_image_send_status = WIRELESS_IMAGE_SEND_FAILED;
        return;
    }

    // image_addr 指向刚完成 DMA 的一帧。复制完成后，后续 DMA 改写不会影响本次发送。
    memcpy(wireless_image_snapshot, image_addr, image_size);
    wireless_image_snapshot_ready = true;
}

static bool wireless_image_send_header(uint8 channel_id, uint32 image_size, uint16 image_width, uint16 image_height, uint8 image_format)
{
    char image_header[WIRELESS_IMAGE_VOFA_HEADER_SIZE];
    int header_length;

    header_length = snprintf(
        image_header,
        sizeof(image_header),
        "image:%u,%lu,%u,%u,%u\n",
        (unsigned int)channel_id,
        (unsigned long)image_size,
        (unsigned int)image_width,
        (unsigned int)image_height,
        (unsigned int)image_format);
    if(header_length <= 0 || header_length >= (int)sizeof(image_header))
    {
        return false;
    }
    return (wireless_uart_send_buffer_timeout(
        (const uint8 *)image_header,
        (uint32)header_length,
        WIRELESS_IMAGE_RTS_TIMEOUT_MS) == 0U);
}

static uint16 wireless_image_gray_to_rgb565(uint8 gray)
{
    return (uint16)(((uint16)(gray & 0xF8U) << 8)
        | ((uint16)(gray & 0xFCU) << 3)
        | ((uint16)gray >> 3));
}

static void wireless_image_draw_cross_marker(
    uint16 row,
    uint8 marker_col,
    uint8 marker_row,
    uint16 color)
{
    uint16 row_difference = (row >= marker_row) ? (row - marker_row) : (marker_row - row);

    if(row_difference > 2U)
    {
        return;
    }

    wireless_image_processed_line[marker_col] = color;
    if(row_difference == 0U)
    {
        uint16 start_col = (marker_col >= 2U) ? (marker_col - 2U) : 0U;
        uint16 end_col = marker_col + 2U;
        uint16 col;

        if(end_col >= MT9V03X_W)
        {
            end_col = MT9V03X_W - 1U;
        }
        for(col = start_col; col <= end_col; col++)
        {
            wireless_image_processed_line[col] = color;
        }
    }
}

static bool wireless_image_send_origin(uint8 channel_id)
{
    if(!wireless_image_send_header(
        channel_id,
        MT9V03X_IMAGE_SIZE,
        MT9V03X_W,
        MT9V03X_H,
        WIRELESS_IMAGE_VOFA_GRAYSCALE8_FORMAT))
    {
        return false;
    }

    return (wireless_uart_send_buffer_timeout(
        wireless_image_snapshot,
        MT9V03X_IMAGE_SIZE,
        WIRELESS_IMAGE_RTS_TIMEOUT_MS) == 0U);
}

static bool wireless_image_send_processed(uint8 channel_id)
{
    uint16 row;
    uint32 image_size = (uint32)MT9V03X_IMAGE_SIZE * sizeof(uint16);
#if IMAGE_TRACK_V2_DISPLAY_ENABLE
    const image_track_v2_result_t *v2_result = image_process_get_v2_result();
    uint8 reference_col = IMAGE_CALIBRATED_CENTER_COL;
#else
    uint8 reference_col = image_process_get_reference_col();
#endif
    uint8 cross_left_col = 0U;
    uint8 cross_left_row = 0U;
    uint8 cross_right_col = 0U;
    uint8 cross_right_row = 0U;
    bool cross_corners_valid = image_process_get_cross_corners(
        &cross_left_col,
        &cross_left_row,
        &cross_right_col,
        &cross_right_row);
    image_cross_state_enum cross_state = image_process_get_cross_state();

    if(!wireless_image_send_header(
        channel_id,
        image_size,
        MT9V03X_W,
        MT9V03X_H,
        WIRELESS_IMAGE_VOFA_RGB565_FORMAT))
    {
        return false;
    }

    for(row = 0U; row < MT9V03X_H; row++)
    {
        uint16 col;
#if IMAGE_TRACK_V2_DISPLAY_ENABLE
        uint16 left_edge = v2_result->left_edge[row];
        uint16 right_edge = v2_result->right_edge[row];
        uint16 mid_line = v2_result->center_line[row];
#else
        uint16 left_edge = image_left_edge[row];
        uint16 right_edge = image_right_edge[row];
        uint16 mid_line = image_mid_line[row];
#endif

        for(col = 0U; col < MT9V03X_W; col++)
        {
            wireless_image_processed_line[col] = wireless_image_gray_to_rgb565(
                wireless_image_snapshot[row * MT9V03X_W + col]);
        }

        // 先画标定中心，后画路径，避免直道中线被参考线覆盖。
        if(reference_col < MT9V03X_W)
        {
            wireless_image_processed_line[reference_col] = WIRELESS_IMAGE_RGB565_CYAN;
        }

        // 与 image_process_display() 保持相同的绘制语义。
#if IMAGE_TRACK_V2_DISPLAY_ENABLE
        if(v2_result->left_valid[row] && left_edge < MT9V03X_W)
        {
            wireless_image_processed_line[left_edge] = WIRELESS_IMAGE_RGB565_RED;
        }
        if(v2_result->right_valid[row] && right_edge < MT9V03X_W)
        {
            wireless_image_processed_line[right_edge] = WIRELESS_IMAGE_RGB565_BLUE;
        }
        if(mid_line < MT9V03X_W)
        {
            if(v2_result->source[row] == IMAGE_TRACK_SOURCE_BOTH_MEASURED)
            {
                wireless_image_processed_line[mid_line] = WIRELESS_IMAGE_RGB565_GREEN;
            }
            else if(v2_result->source[row] == IMAGE_TRACK_SOURCE_LEFT_ONLY
                || v2_result->source[row] == IMAGE_TRACK_SOURCE_RIGHT_ONLY)
            {
                wireless_image_processed_line[mid_line] = WIRELESS_IMAGE_RGB565_YELLOW;
            }
            else if(v2_result->source[row] == IMAGE_TRACK_SOURCE_SHORT_PREDICTED)
            {
                wireless_image_processed_line[mid_line] = WIRELESS_IMAGE_RGB565_MAGENTA;
            }
        }
#else
        if(image_left_edge_valid[row] && left_edge < MT9V03X_W)
        {
            wireless_image_processed_line[left_edge] = WIRELESS_IMAGE_RGB565_RED;
        }
        if(image_right_edge_valid[row] && right_edge < MT9V03X_W)
        {
            wireless_image_processed_line[right_edge] = WIRELESS_IMAGE_RGB565_BLUE;
        }
        if(mid_line < MT9V03X_W)
        {
            wireless_image_processed_line[mid_line] = WIRELESS_IMAGE_RGB565_GREEN;
        }
#endif
        if(cross_corners_valid)
        {
            wireless_image_draw_cross_marker(
                row,
                cross_left_col,
                cross_left_row,
                WIRELESS_IMAGE_RGB565_MAGENTA);
            wireless_image_draw_cross_marker(
                row,
                cross_right_col,
                cross_right_row,
                WIRELESS_IMAGE_RGB565_CYAN);
        }

        // 左上角状态条：黄色表示单帧候选，绿色表示连续两帧确认。
        if(row >= 2U && row <= 4U && cross_state != IMAGE_CROSS_STATE_NONE)
        {
            uint16 status_col;
            uint16 status_color = (cross_state == IMAGE_CROSS_STATE_DETECTED)
                ? WIRELESS_IMAGE_RGB565_GREEN
                : WIRELESS_IMAGE_RGB565_YELLOW;

            for(status_col = 2U; status_col <= 7U; status_col++)
            {
                wireless_image_processed_line[status_col] = status_color;
            }
        }

        if(wireless_uart_send_buffer_timeout(
            (const uint8 *)wireless_image_processed_line,
            (uint32)MT9V03X_W * sizeof(uint16),
            WIRELESS_IMAGE_RTS_TIMEOUT_MS) != 0U)
        {
            return false;
        }
    }

    return true;
}

// 发送 FireWater 图片前导帧和图片数据。必须在 image_process_frame() 后调用，才能发送叠加结果。
void wireless_image_send_task(void)
{
    bool send_success;

    if(wireless_image_send_status != WIRELESS_IMAGE_SEND_SENDING || !wireless_image_snapshot_ready)
    {
        return;
    }

    if(wireless_image_send_type == WIRELESS_IMAGE_SEND_PROCESSED)
    {
        send_success = wireless_image_send_processed(WIRELESS_IMAGE_VOFA_PROCESSED_CHANNEL_ID);
    }
    else if(wireless_image_send_type == WIRELESS_IMAGE_SEND_BOTH)
    {
        // 两张图都由同一份 wireless_image_snapshot 生成，保证原图与处理结果一一对应。
        send_success = wireless_image_send_origin(WIRELESS_IMAGE_VOFA_ORIGIN_CHANNEL_ID)
            && wireless_image_send_processed(WIRELESS_IMAGE_VOFA_PROCESSED_CHANNEL_ID);
    }
    else
    {
        send_success = wireless_image_send_origin(WIRELESS_IMAGE_VOFA_ORIGIN_CHANNEL_ID);
    }

    wireless_image_send_status = send_success ? WIRELESS_IMAGE_SEND_SUCCESS : WIRELESS_IMAGE_SEND_FAILED;
    wireless_image_snapshot_ready = false;
}
